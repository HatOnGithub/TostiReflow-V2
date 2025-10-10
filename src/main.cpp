#include <Arduino.h>
#include <Esp.h>
#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <SparkFun_ADS1219.h>
#include <PID_v1.h>
#include "esp_ota_ops.h"
#include "LittleFS.h"
#include <EEPROM.h>

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>

#include <ArduinoOTA.h>

#include "pages.h"

#include "config.h"
#include "secret.h"


/*
===============================================================================================
                                    Function Prototypes
===============================================================================================
*/

void SetupOTA();
void SetupWiFi();
void SetupWebServer();
void SetupPID();
void SetupFS();
void HandleTouch();
void SampleTemperatures();
void SlowPWM();
void DrawUI();
void GetTemperature(int from);

void OnConnect();

void DisplayError(String message);
void DisplaySafeMode();

/*
===============================================================================================
                                        Class Declarations
===============================================================================================
*/

// Abstract profile step class
class ProfileStep;

// ------------- Profile steps for combined heating of top and bottom heater ------------
// Combined steps will always have the convection fan on due to the desire to equally heat both sides
// Linear ramp step
class LinearStep;
// AFAP step -> may cause overshoot
class InstantStep;

// ----------- Profile steps for independent heating of top and bottom heater ------------
// Independent linear ramp step
class BiLinearStep;
// Independent AFAP step -> may cause overshoot
class BiInstantStep;

Page* currentPage = nullptr; // pointer to the current page being displayed

HomePage homePage;
ProfilePage profilePage;
MonitorPage monitorPage;
SettingsPage settingsPage;
PIDSettingsPage pidSettingsPage;
NetworkSettingsPage networkSettingsPage;
WiFiSettingsPage wifiSettingsPage;

/*
===============================================================================================
                                          Variables
===============================================================================================
*/
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
unsigned long refreshrate = REFRESH_RATE, refreshTime = 1000 / refreshrate, lastRefresh = 0;

// Display buffer using sprites
TFT_eSprite buffer = TFT_eSprite(&tft);
TFT_eSprite screen = TFT_eSprite(&buffer);

// Touchscreen
uint16_t startX = 0, startY = 0, lastX = 0, lastY = 0;
bool isTouched = false, wasTouched = false;
unsigned long pollingRate = POLLING_RATE, lastPoll = 0, pollingTime = 1000 / pollingRate;

// ADS and Temperature
unsigned long lastTempSample = 0, sampleRate= TEMP_SAMPLE_RATE, sampleTime = 1000 / sampleRate;
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

// Control variables
unsigned long lastPeriod = 0; // last time the PWM signal was updated
unsigned long dutyCycleStep = PWM_PERIOD / PWM_STEPS; // how much to increase the duty cycle each step
bool fanState = false; // state of the convection fan

// Profile Variables
String profiles[MAX_PROFILES]; // populated at startup
String profileName = "";
uint16_t profileIndex = 0, profileLength = 0;
unsigned long profileStartTime = 0;
unsigned long profileEstimatedDuration = 0;

ProfileStep* profileSteps[MAX_PROFILE_STEPS]; // populate at profile load
uint16_t currentStepIndex = 0, totalSteps = 0;
ProfileStep* currentStep = nullptr; // populate at profile load, reset at profile end or cancel
bool profileLoaded = false, profileRunning = false;

/*
===============================================================================================
                                    Class definitions
===============================================================================================
*/

class ProfileStep {
  public:
    // Target temperature for this step
    double finalTempT; 
    double finalTempB;

    // Duration of this step in milliseconds, 
    // calculated by Init() based on this step's behavior and the previous step's final temperature
    unsigned long duration; 
    
    // given the final target temperature of the last step, start this step
    virtual void Init(double endTempT, double endTempB) = 0;

    // Reset any internal variables to prepare for a new profile run
    virtual void Start() = 0;

    // update the setpoints based on the elapsed time and this step's behavior
    virtual void Update(double* SetpointT, double* SetpointB) = 0;

    // Signal that the next step should start
    virtual bool IsComplete() = 0;

    // Draw itself on a graph sprite at position x,y with given step sizes (1 pixel = stepX in time, 1 pixel = stepY in temperature)
    virtual void DrawOnGraph(TFT_eSprite* graphSprite, uint16_t startX, uint16_t startY, double stepX, double stepY) = 0;
};



/*
===============================================================================================
                                        Setup & Loop
===============================================================================================
*/

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
  tft.println("TFT and Touchscreen initialized");

  safemode |= tft.getTouch(&lastX, &lastY); // read initial touch state to check for safe mode

  // ========== Setup WiFi ============
  SetupWiFi();
  
  safemode |= tft.getTouch(&lastX, &lastY); // read initial touch state to check for safe mode
  delay(500);

  // =========== Setup OTA ==============
  SetupOTA();

  safemode |= tft.getTouch(&lastX, &lastY); // read initial touch state to check for safe mode
  delay(500);

  // ========== Check for safe mode ============
  // As the bare essentials have been set up, check for touchscreen input to enter safe mode
  if (safemode == true) {
    DisplaySafeMode();
    Serial0.println("Safe mode activated due to touchscreen input at startup");
    return;
  }

  delay(500);
  tft.println("No input detected, continuing setup...");

  // ========== Setup PID ============
  Serial0.println("PID init");
  tft.println("PID initializing...");
  SetupPID();

  delay(500);

  // ========== Setup ADS1219 ============
  Serial0.println("Auxilary ADC init");
  tft.println("Auxilary ADC initializing...");

  while (!ads.begin(Wire, 0x44)) {
    tft.println("Failed to communicate with ADS1219.");
    delay(500);
  }

  ads.setVoltageReference(ADS1219_VREF_EXTERNAL);
  ads.setGain(ADS1219_GAIN_1);

  delay(500);

  // ======== Setup File System ==========
  SetupFS();

  delay(500);
  // ======== Setup Webserver ==========

  SetupWebServer();

  delay(500);

  // ====== Setup Sprite Buffers =======
  Serial0.println("Sprite buffers init");
  tft.println("Sprite buffers initializing...");
  buffer.createSprite(SCREEN_WIDTH, SCREEN_HEIGHT);
  buffer.setFreeFont( &FreeSans9pt7b );
  buffer.setAttribute(PSRAM_ENABLE, true); // enable psram for the sprite buffer

  screen.createSprite(DRAW_WIDTH, DRAW_HEIGHT);
  screen.setFreeFont( &FreeSans9pt7b );
  screen.setAttribute(PSRAM_ENABLE, true); // enable psram for the sprite buffer

  // =========== Finalize ==============

  tft.println("Setup complete");
  delay(500);
}

void loop(){
  ArduinoOTA.handle();

  if (safemode) {
    // Ensure everything is off
    digitalWrite(RELAY_B, LOW);
    digitalWrite(RELAY_T, LOW);
    digitalWrite(RELAY_F, LOW);
    return;
  }

  HandleTouch();
  SampleTemperatures();
  SlowPWM();
  DrawUI();

}

/*
===============================================================================================
                                    Setup Functions
===============================================================================================
*/

void SetupWiFi(){
  
  Serial0.println("Wifi init");
  tft.println("Wifi initializing...");

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
}

void SetupWebServer(){
  Serial0.println("Webserver init");
  tft.println("Webserver initializing...");
  server.begin();

  if (!MDNS.begin("tostireflow")) {
    tft.println("Error setting up MDNS responder!");
  } else {
    tft.println("mDNS responder started");
  }

  server.serveStatic("/static", LittleFS, "/static");
}

// This functions mounts LittleFS
void SetupFS() {
  Serial0.println("File system init");
  tft.println("File system initializing...");
  if (!LittleFS.begin()) {
    Serial0.println("LittleFS Mount Failed");
    tft.println("LittleFS Mount Failed");
    return;
  }

  Serial0.println("LittleFS Mounted Successfully");
  Serial0.println("File System Storage:");
  Serial0.print(LittleFS.usedBytes());
  Serial0.print(" / ");
  Serial0.print(LittleFS.totalBytes());
  Serial0.print(" (" + String((float)LittleFS.usedBytes() / (float)LittleFS.totalBytes() * 100, 2) + "%)");
  Serial0.println(" bytes used");
  tft.print("Storage: ");
  tft.print(LittleFS.usedBytes());
  tft.print(" / ");
  tft.print(LittleFS.totalBytes());
  tft.print(" (");
  tft.print((float)LittleFS.usedBytes() / (float)LittleFS.totalBytes() * 100, 2);
  tft.println("%) bytes used");
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

  tft.println("OTA handler initialized, check for safe mode...");
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

/*
===============================================================================================
                                    Main Loop Functions
===============================================================================================
*/


void DrawUI(){
  if (millis() - lastRefresh < refreshTime) return;
  lastRefresh = millis();

  // Use buffer to draw the UI

  // Background
  buffer.fillSprite(TFT_WHITE);

  // Top bar with temperatures
  buffer.fillRect(0,0, SCREEN_WIDTH, 25, TFT_BLACK);
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

  buffer.fillRect(0, DRAW_TOP, DRAW_LEFT, DRAW_HEIGHT, TFT_BLACK); // left margin

  buffer.drawRect(0, DRAW_TOP, DRAW_LEFT, DRAW_LEFT, TFT_YELLOW);
  // fan indicator
  if (fanState)
    buffer.fillRect(2, DRAW_TOP+2, DRAW_LEFT - 4, DRAW_LEFT - 4, TFT_YELLOW);
  
  // PID output indicator
  int halfRemainingHeight = (SCREEN_HEIGHT - DRAW_TOP - DRAW_LEFT) / 2; // half of the remaining height in the box

  buffer.drawRect(0, DRAW_TOP + DRAW_LEFT, DRAW_LEFT, halfRemainingHeight, TFT_BLUE);
  int pidHeightT = (int)(OutputT * halfRemainingHeight);
  buffer.fillRect(2, DRAW_TOP + DRAW_LEFT + halfRemainingHeight - pidHeightT, DRAW_LEFT - 2, pidHeightT, TFT_BLUE);

  buffer.drawRect(0, DRAW_TOP + DRAW_LEFT + halfRemainingHeight, DRAW_LEFT, halfRemainingHeight, TFT_GREEN);
  int pidHeightB = (int)(OutputB * halfRemainingHeight);
  buffer.fillRect(2, DRAW_TOP + DRAW_LEFT + halfRemainingHeight + halfRemainingHeight - pidHeightB, DRAW_LEFT - 2, pidHeightB, TFT_GREEN);

  if (currentPage != nullptr) {
    currentPage->Draw();
  }

  screen.pushToSprite(&buffer, DRAW_LEFT, DRAW_TOP); // draw screen below top bar
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
      if (currentPage != nullptr) {
        currentPage->OnTouch(lastX, lastY);
      }
    }
    // Continuous touch
    else {
      if (currentPage != nullptr) {
        currentPage->UpdateTouch(lastX, lastY);
      }
    }
  }
  // Falling edge
  else if (wasTouched) {
    if (currentPage != nullptr) {
      currentPage->OnRelease();
    }
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
    DisplayError("GetTemperature: Invalid 'from' argument");
    return;
  }

  ads.startSync();
  while (!ads.dataReady());
  if (!ads.readConversion()) {
    DisplayError("Failed to read from ADS1219");
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

void SlowPWM(){

  digitalWrite(RELAY_F, fanState ? HIGH : LOW); // Fan always on when heating

  if (!profileRunning) {
    digitalWrite(RELAY_T, LOW);
    digitalWrite(RELAY_B, LOW);
    return;
  }

  if (millis() - lastPeriod >= (unsigned long)(OutputT * dutyCycleStep))
    digitalWrite(RELAY_T, LOW);

  if (millis() - lastPeriod >= (unsigned long)(OutputB * dutyCycleStep))
    digitalWrite(RELAY_B, LOW);

  if (millis() - lastPeriod < PWM_PERIOD) return;
  lastPeriod = millis();

  if (OutputT > 0) digitalWrite(RELAY_T, HIGH); // update PWM
  if (OutputB > 0) digitalWrite(RELAY_B, HIGH);
}

void DisplayError(String message) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_RED);
  tft.setTextSize(2);
  tft.setCursor(150 , 100);
  tft.println("ERROR");
  
  tft.setTextSize(1);
  tft.setCursor(50 , 150);
  tft.println(message);

  analogWriteFrequency(1000);

  analogWrite(BUZZER_PIN, 2048);
  delay(2000);
  analogWrite(BUZZER_PIN, 0);

  analogWriteFrequency(5000);

  safemode = true;
}

void DisplaySafeMode(){
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




/*
===============================================================================================
                                   Webpages and APIs
===============================================================================================
*/

void NotFound(){
  Serial.println("Not Found: " + server.uri());
  server.send(404, "text/plain", "Not Found");
}

void OnConnect(){
  fs::File file = LittleFS.open("/static/index.html", "r");
  if (!file) {
    Serial.println("Failed to open file for reading");
    server.send(404, "text/plain", "File not found");
    return;
  }
  server.streamFile(file, "text/html");
  file.close();
}