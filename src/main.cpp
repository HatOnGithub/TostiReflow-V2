#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <SparkFun_ADS1219.h>
#include <PID_v1.h>
#include "esp_ota_ops.h"

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>

#include <ArduinoOTA.h>

#include "config.h"
#include "secret.h"

// debug
bool safemode = false;

// Wireless
const char* ssid = SSID;
const char* password = PSWD;

WebServer server(80);
String header;

uint32_t last_ota_time = 0;

// Display
TFT_eSPI tft = TFT_eSPI();
ulong refreshrate = REFRESH_RATE, refreshTime = 1000 / refreshrate, lastRefresh = 0;
// Display buffer using sprite
TFT_eSprite buffer = TFT_eSprite(&tft);


// Touchscreen
uint16_t startX = 0, startY = 0, lastX = 0, lastY = 0;
bool isTouched = false, wasTouched = false;
ulong pollingRate = POLLING_RATE, lastPoll = 0, pollingTime = 1000 / pollingRate;

// ADS and Temperature
ulong lastTempSample = 0, sampleRate= TEMP_SAMPLE_RATE, sampleTime = 1000 / sampleRate;
double tempSamplesTop[TEMP_SAMPLES], tempSamplesBottom[TEMP_SAMPLES];
int tempIndex = 0;
double avgTempTop = 0.0, avgTempBottom = 0.0;

SfeADS1219ArdI2C ads;

// PID variables
double InputT = 0, OutputT = 0, SetpointT = 0.0;
double InputB = 0, OutputB = 0, SetpointB = 0.0;
double Kp = 0.0, Ki = 0.0, Kd = 0.0;
PID PIDT(&InputT, &OutputT, &SetpointT, Kp, Ki, Kd, DIRECT);
PID PIDB(&InputB, &OutputB, &SetpointB, Kp, Ki, Kd, DIRECT);

// PWM variables
unsigned long lastPeriod = 0; // last time the PWM signal was updated
unsigned long dutyCycleStep = PWM_PERIOD / PWM_STEPS; // how much to increase the duty cycle each step

// --------------------Function Definitions-------------------------

void SetupOTA();
void SetupWebServer();
void SetupPID();
void HandleTouch();
void SampleTemperatures();
void DrawUI();
void GetTemperature(int from);
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
  SPI.setFrequency(40000000); // 40 MHz
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial0.println("Setup started");

  // =========== Setup TFT display ==============
  Serial0.println("TFT init");
  tft.begin();
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(1);
  tft.setFreeFont( &FreeSans9pt7b );
  tft.setTextSize(1);
  tft.setCursor(0 ,20);
  tft.setTextColor(TFT_WHITE);
  tft.println("Firmware v" FIRMWARE_VERSION);
  tft.println("TFT, Buffer, and Touchscreen initialized");
  
  buffer.createSprite(SCREEN_WIDTH, SCREEN_HEIGHT);
  buffer.setFreeFont( &FreeSans9pt7b );

  delay(500);

  uint16_t x, y;
  safemode |= tft.getTouch(&x, &y);

  // ========== Setup PID ============
  Serial0.println("PID init");
  tft.println("PID initializing...");
  SetupPID();
  tft.println("PID initialized");

  delay(500);
  safemode |= tft.getTouch(&x, &y);

  // ========== Setup ADS1219 ============
  Serial0.println("Auxilary ADC init");
  tft.println("Auxilary ADC initializing...");

  while (!ads.begin(Wire, 0x44)) {
    tft.println("Failed to communicate with ADS1219.");
    delay(500);
  }

  ads.setVoltageReference(ADS1219_VREF_EXTERNAL);
  ads.setGain(ADS1219_GAIN_1);

  tft.println("ADC initialized");

  delay(500);
  safemode |= tft.getTouch(&x, &y);

  // ======== Setup Webserver ==========
  SetupWebServer();

  delay(500);
  safemode |= tft.getTouch(&x, &y);

  // =========== Setup OTA ==============
  SetupOTA();

  delay(500);
  safemode |= tft.getTouch(&x, &y);

  // =========== Finalize ==============

  tft.println("Setup complete");
  delay(500);
  safemode |= tft.getTouch(&x, &y);

  tft.setTextColor(TFT_BLACK);
  tft.fillScreen(TFT_WHITE);

  if (safemode) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(150 , 100);
    tft.println("SAFE MODE");
    
    tft.setTextSize(1);
    tft.setCursor(110 , 150);
    tft.println("OTA Active, listening for updates...");
    tft.setCursor(130 , 180);
    tft.println("Restart to leave safe mode");

    analogWriteFrequency(1000);

    analogWrite(BUZZER_PIN, 2048);
    delay(2000);
    analogWrite(BUZZER_PIN, 0);

    analogWriteFrequency(5000);
  }
}

void loop(){
  ArduinoOTA.handle();

  if (safemode) return;

  HandleTouch();
  SampleTemperatures();
  DrawUI();

}

void SetupWebServer(){
  Serial0.println("Wifi and Webserver init");
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

}

void SetupOTA(){
  Serial0.println("OTA init");
  tft.println("OTA handler initializing...");
  ArduinoOTA.setHostname("TostiReflow V2");
  ArduinoOTA.setMdnsEnabled(true);

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

  tft.println("OTA handler initialized");
}

// This function sets up the PID controller
void SetupPID(){
  SetpointT = 0;
  SetpointB = 0;

  // tell the PID to range between 0 and the full window size
  PIDT.SetOutputLimits(0, 1);
  PIDB.SetOutputLimits(0, 1);

  PIDT.SetSampleTime(PID_SAMPLE_TIME);
  PIDB.SetSampleTime(PID_SAMPLE_TIME);

  // turn the PID on
  PIDT.SetMode(AUTOMATIC);
  PIDT.SetIntegralBounds(-10, 10); // set integral bounds to prevent windup

  PIDB.SetMode(AUTOMATIC);
  PIDB.SetIntegralBounds(-10, 10); // set integral bounds to prevent windup

  OutputT = 0; // initialize OutputT to 0
  OutputB = 0; // initialize OutputB to 0
}

void DrawUI(){
  if (millis() - lastRefresh < refreshTime) return;
  lastRefresh = millis();

  // Use buffer to draw the UI

  // Background
  buffer.fillSprite(TFT_WHITE);

  // Top bar with temperatures
  buffer.fillRect(0,0, SCREEN_WIDTH, 20, TFT_BLACK);
  buffer.setCursor(5 ,15);
  buffer.setTextColor(TFT_WHITE, TFT_BLACK, true);
  buffer.print("TostiReflow V2 - v" FIRMWARE_VERSION);
  buffer.print(" | ");
  buffer.print("Top: ");
  if (avgTempTop < 30.0) buffer.setTextColor(TFT_CYAN, TFT_BLACK, true);
  else if (avgTempTop < 60.0) buffer.setTextColor(TFT_GREEN, TFT_BLACK, true);
  else if (avgTempTop < 150.0) buffer.setTextColor(TFT_YELLOW, TFT_BLACK, true);
  else buffer.setTextColor(TFT_RED, TFT_BLACK, true);

  buffer.print(avgTempTop, 1);  

  buffer.setTextColor(TFT_WHITE, TFT_BLACK, true);
  buffer.print("C | Bottom: ");

  if (avgTempBottom < 30.0) buffer.setTextColor(TFT_CYAN, TFT_BLACK, true);
  else if (avgTempBottom < 60.0) buffer.setTextColor(TFT_GREEN, TFT_BLACK, true);
  else if (avgTempBottom < 150.0) buffer.setTextColor(TFT_YELLOW, TFT_BLACK, true);
  else buffer.setTextColor(TFT_RED, TFT_BLACK, true);
  buffer.print(avgTempBottom, 1);

  buffer.setTextColor(TFT_WHITE, TFT_BLACK, true);
  buffer.print("C");

  // Draw page, TODO

  // write the buffer to the screen
  buffer.pushSprite(0,0);

}

// Handles touchscreen input and updates touch state variables
void HandleTouch(){
  if (millis() - lastPoll < pollingTime) return;
  lastPoll = millis();

  wasTouched = isTouched;
  uint16_t x, y;
  isTouched = tft.getTouch(&x, &y);

  if (isTouched) {
      
    lastX = SCREEN_WIDTH - x;
    lastY = SCREEN_HEIGHT - y;

    // Rising edge
    if (!wasTouched) {
      startX = lastX;
      startY = lastY;
    }

  }
  // Falling edge
  else if (wasTouched) {

  }
  // Not touched
  else{
    lastX = -1;
    lastY = -1;
  }
}

// Samples temperature from both thermistors and updates the average values
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

// Reads temperature from the specified thermistor
// 'from' should be either TOP or BOTTOM
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

  double resistance = abs(R_REF / ((adcOut / 3300.0) - 1));
  double temperature = (1/((log(resistance/R_0)/R_NTC_B) + (1/(T_0+273.15)))) - 273.15;

  if (from == TOP) {
    tempSamplesTop[tempIndex] = temperature;
  } else if (from == BOTTOM) {
    tempSamplesBottom[tempIndex] = temperature;
  }
}
