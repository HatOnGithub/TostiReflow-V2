#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <SparkFun_ADS1219.h>

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>

#include <ArduinoOTA.h>

#include "config.h"
#include "secret.h"

// Wireless
const char* ssid = SSID;
const char* password = PSWD;

WebServer server(80);
String header;

uint32_t last_ota_time = 0;

// Display
TFT_eSPI tft = TFT_eSPI();
ulong refreshrate = REFRESH_RATE, refreshTime, lastRefresh = 0;

// Touchscreen
//XPT2046_Touchscreen ts(TOUCH_CS_PIN, TOUCH_IRQ_PIN);
TS_Point startPoint, lastPoint;
bool isTouched = false, wasTouched = false;
ulong pollingRate = POLLING_RATE, lastPoll = 0, pollingTime = 1000 / pollingRate;

// ADS and Temperature
ulong lastTempSample = 0, sampleRate= TEMP_SAMPLE_RATE, sampleTime = 1000 / sampleRate;
double tempSamplesTop[TEMP_SAMPLES], tempSamplesBottom[TEMP_SAMPLES];
int tempIndex = 0;
double avgTempTop = 0.0, avgTempBottom = 0.0;

SfeADS1219ArdI2C ads;

// PID variables



// --------------------Function Definitions-------------------------

void SetupOTA();
void SetupWebServer();
void HandleTouch();
void SampleTemperatures();
void GetTemperature(int from);
double ADCtoResistance(double adcIn, double adcOut, double Rref);
double ResistanceToTemperature(double Rntc, double R0, double T0, double B);
// ----------------------------------------------------------------


void setup(){

  // =============== Setup pins ===============
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_B, OUTPUT);
  pinMode(RELAY_F, OUTPUT);
  pinMode(RELAY_T, OUTPUT);

  // ============== Startup Sound ==============
  analogWriteResolution(12);
  analogWriteFrequency(500);

  analogWrite(BUZZER_PIN, 2048);
  delay(100);
  
  analogWriteFrequency(1000);
  delay(100);

  analogWriteFrequency(2000);
  delay(100);
  
  analogWrite(BUZZER_PIN, 0);
  analogWriteFrequency(5000);


  // ====== Start communication interfaces ======
  Serial0.begin(115200);
  SPI.begin(SPI_CLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial0.println("Setup started");

  // =========== Setup TFT display ==============
  Serial0.println("TFT init");
  tft.begin();
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(1);
  tft.setFreeFont( &FreeSans9pt7b );
  tft.setTextSize(1);
  tft.setCursor(5 ,20);
  tft.setTextColor(TFT_WHITE);
  tft.println(DEVICE_NAME);
  tft.println("Firmware v" FIRMWARE_VERSION);
  tft.println("by JTD Chen");
  tft.println("TFT and Touchscreen initialized");

  delay(1000);

  tft.println("Auxilary ADC initializing...");

  while (!ads.begin(Wire, 0x44)) {
    tft.println("Failed to communicate with ADS1219.");
    delay(1000);
  }

  ads.setVoltageReference(ADS1219_VREF_EXTERNAL);
  ads.setGain(ADS1219_GAIN_1);

  tft.println("ADC initialized");
  delay(1000);


  // ======== Setup Webserver ==========

  SetupWebServer();

  // =========== Setup OTA ==============
  SetupOTA();

  // =========== Finalize ==============
  tft.println("Setup complete");
  delay(1000);

  
  tft.setTextColor(TFT_BLACK);
  tft.fillScreen(TFT_WHITE);
}

void loop(){
  ArduinoOTA.handle();
  HandleTouch();
  SampleTemperatures();

}

void SetupWebServer(){
  tft.println("Wifi and Webserver initializing...");

  WiFi.begin(ssid, password);
  
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial0.println("Connection Failed! Rebooting...");
    tft.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  IPAddress IP = WiFi.localIP();
  Serial0.print("AP IP address: ");
  Serial0.println(IP);
  tft.print("IP Address: ");
  tft.println(IP);

  server.begin();

  if (!MDNS.begin("tostireflow")) {
    tft.println("Error setting up MDNS responder!");
  } else {
    tft.println("mDNS responder started");
  }

  tft.println("Wifi and Webserver initialized");
  delay(1000);

}

void SetupOTA(){

  ArduinoOTA.setHostname("TostiReflow V2");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      tft.fillScreen(TFT_BLACK);
      tft.setCursor(160 , 100);
      tft.setTextColor(TFT_WHITE);
      tft.println("Start updating " + type);
      tft.drawRect(120, 160, 240, 20, TFT_WHITE);
    })
    .onEnd([]() {
      
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      if (millis() - last_ota_time > 500) {
        tft.fillRect(120, 160, (int)(240.0f * ((float)progress / (float)total)), 20, TFT_WHITE);

        last_ota_time = millis();
      }
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        tft.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        tft.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        tft.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        tft.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        tft.println("End Failed");
      }
    });

  ArduinoOTA.begin();
}

void HandleTouch(){
  if (millis() - lastPoll < pollingTime) return;
  lastPoll = millis();

  wasTouched = isTouched;
  uint16_t x, y;
  isTouched = tft.getTouch(&x, &y);

  tft.fillRect(0,0, 480, 20, TFT_BLACK);
  tft.setCursor(5 ,15);
  tft.setTextColor(TFT_WHITE);
  tft.print("TOP:");
  tft.print(avgTempTop, 1);
  tft.print("C, BOTTOM:");
  tft.print(avgTempBottom, 1);
  tft.print("C");

  if (isTouched) {
      
    lastPoint.x = SCREEN_WIDTH - x;
    lastPoint.y = SCREEN_HEIGHT - y;

    if (!wasTouched) {
      startPoint = lastPoint;
      tft.fillScreen(TFT_RED);

    }

    tft.fillSmoothCircle(lastPoint.x, lastPoint.y, 2, TFT_WHITE);

  } else if (wasTouched) {
    tft.fillScreen(TFT_WHITE);
  }
  else{
    lastPoint.x = -1;
    lastPoint.y = -1;
  }
}

void SampleTemperatures(){

  if (millis() - lastTempSample < sampleTime) return;
  lastTempSample = millis();

  GetTemperature(TOP);
  GetTemperature(BOTTOM);

  tempIndex = (tempIndex + 1) % TEMP_SAMPLES;

  double top = 0.0, bottom = 0.0;
  for (int i = 0; i < TEMP_SAMPLES; i++) {
    top += tempSamplesTop[i];
    bottom += tempSamplesBottom[i];
  }

  avgTempTop = top / TEMP_SAMPLES;
  avgTempBottom = bottom / TEMP_SAMPLES;
}

void GetTemperature(int from){
  if (from == TOP) {
    while(!ads.setInputMultiplexer(ADS1219_CONFIG_MUX_DIFF_P2_N3));
  } else if (from == BOTTOM) {
    while(!ads.setInputMultiplexer(ADS1219_CONFIG_MUX_DIFF_P0_N1));
  } else {
    tft.setTextColor(TFT_RED);
    tft.println("GetTemperature: Invalid 'from' argument");
    delay(1000);
    return;
  }

  ads.startSync();
  while (!ads.dataReady());
  if (!ads.readConversion()) {
    tft.setTextColor(TFT_RED);
    tft.println("Failed to read conversion from ADS1219");
    delay(1000);
    return;
  }

  float adcOut = ads.getConversionMillivolts(3300.0); // 3.3V Vref
  if (from == TOP) adcOut *= -1; // Invert bottom reading due to wiring

  double resistance = ADCtoResistance(adcOut, 3300.0, R_REF);
  double temperature = ResistanceToTemperature(resistance, R_0, T_0, R_NTC_B);

  if (from == TOP) {
    tempSamplesTop[tempIndex] = temperature;
  } else if (from == BOTTOM) {
    tempSamplesBottom[tempIndex] = temperature;
  }
}

// Converts the 2 24-bit ADC values to resistance in ohms
double ADCtoResistance(double adcIn, double adcOut, double Rref){
  return abs(Rref / ((adcIn/adcOut) - 1));
}

// Converts resistance in ohms to temperature in degrees Celsius
double ResistanceToTemperature(double Rntc, double R0, double T0, double B){
  return (1/((log(Rntc/R0)/B) + (1/(T0+273.15)))) - 273.15;
}
