#pragma region Includes

#include <Arduino.h>
#include <Esp.h>
#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <SparkFun_ADS1219.h>
#include <PID_v1.h>
#include "LittleFS.h"
#include <EEPROM.h>
#include "time.h"

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include "esp_ota_ops.h"

#include "profile.h"
#include "config.h"

// Include WiFi credentials and OTA hash from secret.h
// To create your own, copy secret_template.h to secret.h and fill in your details
#include "secret.h"

#pragma endregion Includes

// ============================================================================================
//                                   Function Prototypes
// ============================================================================================

#pragma region FunctionPrototypes

// Setup functions
void SetupTFT();
void SetupOTA();
void SetupADS();
void SetupWiFi();
void SetupWebServer();
void SetupPID();
void SetupFS();
void SetupPages();
void SetupBuffers();
void SetupTasks();

// Main loop functions
void HandleTouch();
void SampleTemperatures();
void SlowPWM();
void DrawUI();
void GetTemperature(int from);

// Web server functions and API
void OnConnect();

// Debug functions
void DisplayError(String message);
void DisplaySafeMode();
void Beep();
void Beep(int duration);

#pragma endregion FunctionPrototypes

// ============================================================================================
//                                       Variables
// ============================================================================================

#pragma region Variables

SemaphoreHandle_t SPI_Mutex = xSemaphoreCreateMutex();

// debug
bool safemode = false, rollbackTouching = false, rollbackWasTouching = false;
unsigned long lastLoopStart = 0, lastLoopEnd = 0, loopDuration = 0, lastRollbackTouchCheck = 0, rollbackPressStart = 0;

// Wireless
const char* ssid = SSID;
const char* password = PSWD;
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600; // GMT +1
const int daylightOffset_sec = 3600;

bool validNTP = false;
unsigned long lastNTPCheck = -60000, NTPCheckInterval = 60000; // check every minute
tm timeinfo;

WebServer server(80);
String header;

uint32_t last_ota_time = 0;

// Display
TFT_eSPI tft = TFT_eSPI();
unsigned long refreshrate = REFRESH_RATE, refreshTime = 1000 / refreshrate, lastRefresh = 0;

// Display buffer using sprites
TFT_eSprite topBar = TFT_eSprite(&tft);
TFT_eSprite leftBar = TFT_eSprite(&tft);
TFT_eSprite buffer = TFT_eSprite(&tft);

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
Profile profiles[MAX_PROFILES]; // populated at startup
String profileNames[MAX_PROFILES]; // populated at startup
String profileName = "";
uint16_t profileIndex = 0, profileCount = 0;
unsigned long profileStartTime = 0;
unsigned long profileEstimatedDuration = 0;
bool profileLoaded = false, profileRunning = false;

#pragma endregion Variables

// ============================================================================================
//                                 Page class instances
// ============================================================================================

#pragma region PageClasses

class Page;

Page* currentPage = nullptr;

// Abstract page class
class Page {
  public:

    Page() {
      darkGrey = buffer.color565(80, 80, 80);
      darkerGrey = buffer.color565(60, 60, 60);
      darkestGrey = buffer.color565(30, 30, 30);
      highlightGrey = buffer.color565(100, 100, 100);
    }
    virtual void Update() = 0;
    virtual void OnTouch(uint16_t x, uint16_t y) = 0;
    virtual void UpdateTouch(uint16_t x, uint16_t y) = 0;
    virtual void OnRelease(uint16_t x, uint16_t y) = 0;
    void Push(){
        xSemaphoreTake(SPI_Mutex, portMAX_DELAY);
        buffer.pushSprite(DRAW_LEFT, DRAW_TOP);
        xSemaphoreGive(SPI_Mutex);
    }

    void Push(uint16_t x, uint16_t y, uint16_t w, uint16_t h){
        xSemaphoreTake(SPI_Mutex, portMAX_DELAY);
        buffer.pushSprite(DRAW_LEFT + x, DRAW_TOP + y, x, y, w, h);
        xSemaphoreGive(SPI_Mutex);
    }

    void SwitchTo(Page* newPage) {
      currentPage = newPage;
      newPage->firstDraw = true; // force redraw of the new page
    }
  protected:    


    uint32_t darkGrey, darkerGrey, darkestGrey, highlightGrey;
    bool firstDraw = true; // flag to indicate if it's the first time drawing the page
    unsigned long lastUpdate = 0;
};

class HomePage;
class ProfilePage;
class MonitorPage;
class SettingsPage;
class PIDSettingsPage;
class NetworkSettingsPage;
class WiFiSettingsPage;

// Main navigation page
class HomePage : public Page {
    public:
        HomePage() : Page() {}
        void LinkPages(ProfilePage* profilePage, SettingsPage* settingsPage, MonitorPage* monitorPage);
        void Update() override;
        void DrawStatic();
        void OnTouch(uint16_t x, uint16_t y) override;
        void UpdateTouch(uint16_t x, uint16_t y) override;
        void OnRelease(uint16_t x, uint16_t y) override;
    private:
        int padding = 5;
        int width = DRAW_WIDTH / 3;

        bool touchingProfile = false, touchingSettings = false, touchingMonitor = false;
        bool wasTouchingProfile = false, wasTouchingSettings = false, wasTouchingMonitor = false;

        uint16_t 
            monitorX = padding, monitorY =  200 + padding, 
            monitorWidth = 150 - (2 * padding), monitorHeight = 90 - (2 * padding),

            profileX = 150 + padding, profileY =  200 + padding, 
            profileWidth = 150 - (2 * padding), profileHeight = 90 - (2 * padding),

            settingsX = 300 + padding, settingsY = 200 + padding, 
            settingsWidth = 150 - (2 * padding), settingsHeight = 90 - (2 * padding);


        ProfilePage* linkedProfilePage = nullptr;
        SettingsPage* linkedSettingsPage = nullptr;
        MonitorPage* linkedMonitorPage = nullptr;
};

// Page for selecting and running profiles
class ProfilePage : public Page {
    public:
        ProfilePage() : Page() {}
        void LinkPages(HomePage* homePage);
        void Update() override;
        void DrawStatic();
        void OnTouch(uint16_t x, uint16_t y) override;
        void UpdateTouch(uint16_t x, uint16_t y) override;
        void OnRelease(uint16_t x, uint16_t y) override;
    private:
        HomePage* linkedHomePage = nullptr;
};

// Page for showing the running a profile and showing progress
class MonitorPage : public Page {
    public:
        MonitorPage() : Page() {}
        void LinkPages(HomePage* homePage);
        void Update() override;
        void DrawStatic();
        void OnTouch(uint16_t x, uint16_t y) override;
        void UpdateTouch(uint16_t x, uint16_t y) override;
        void OnRelease(uint16_t x, uint16_t y) override;
    private:
        HomePage* linkedHomePage = nullptr;
};

// Page for showing submenus for different settings
class SettingsPage : public Page {
    public:
        SettingsPage() : Page() {}
        void LinkPages(HomePage* homePage, String settingNames[], Page* subPages[], int numSubPages);
        void Update() override;
        void DrawStatic();
        void OnTouch(uint16_t x, uint16_t y) override;
        void UpdateTouch(uint16_t x, uint16_t y) override;
        void OnRelease(uint16_t x, uint16_t y) override;
    private:
        HomePage* linkedHomePage = nullptr;
        Page* subPages[SETTING_SUBPAGES]; // maximum of 10 subpages for now
        String settingNames[SETTING_SUBPAGES];
};

// Page for setting PID parameters Kp, Ki, and Kd, mainly for tuning
class PIDSettingsPage : public Page {
    public:
        PIDSettingsPage() : Page() {}
        void LinkPages(SettingsPage* settingsPage);
        void Update() override;
        void DrawStatic();
        void OnTouch(uint16_t x, uint16_t y) override;
        void UpdateTouch(uint16_t x, uint16_t y) override;
        void OnRelease(uint16_t x, uint16_t y) override;
    private:
        SettingsPage* linkedSettingsPage = nullptr;
};

// Page for setting WiFi SSID and Password (and possibly other network settings in the future like MQTT, BT, etc)
class NetworkSettingsPage : public Page {
    public:
        NetworkSettingsPage() : Page() {}
        void LinkPages(SettingsPage* settingsPage, String settingNames[], Page* subPages[], int numSubPages);
        void Update() override;
        void DrawStatic();
        void OnTouch(uint16_t x, uint16_t y) override;
        void UpdateTouch(uint16_t x, uint16_t y) override;
        void OnRelease(uint16_t x, uint16_t y) override;
    private:
        SettingsPage* linkedSettingsPage = nullptr;
        Page* subPages[NETWORK_SUBPAGES]; // maximum of 10 subpages for now
        String settingNames[NETWORK_SUBPAGES];
};

// Page for setting WiFi SSID and Password
class WiFiSettingsPage : public Page {
    public:
        WiFiSettingsPage() : Page() {}
        void LinkPages(NetworkSettingsPage* networkSettingsPage);
        void Update() override;
        void DrawStatic();
        void OnTouch(uint16_t x, uint16_t y) override;
        void UpdateTouch(uint16_t x, uint16_t y) override;
        void OnRelease(uint16_t x, uint16_t y) override;
    private:
        NetworkSettingsPage* linkedNetworkSettingsPage = nullptr;
};



HomePage homePage = HomePage();
ProfilePage profilePage = ProfilePage();
MonitorPage monitorPage = MonitorPage();
SettingsPage settingsPage = SettingsPage();
PIDSettingsPage pidSettingsPage = PIDSettingsPage();
NetworkSettingsPage networkSettingsPage = NetworkSettingsPage();
WiFiSettingsPage wifiSettingsPage = WiFiSettingsPage();

#pragma endregion PageClasses

// ============================================================================================
//                                      Task Setup
// ============================================================================================

#pragma region Tasks

TaskHandle_t WebServerTaskHandle;
TaskHandle_t DrawTaskHandle;
TaskHandle_t TouchTaskHandle;
TaskHandle_t TimeCriticalTasksHandle;
TaskHandle_t beepHandle;

void WebServerTask( void * parameter ){
  for(;;){
    server.handleClient();
    vTaskDelay( 1 / portTICK_PERIOD_MS);
  }
}

void DrawTask( void * parameter ){
  for(;;){
    DrawUI();
    vTaskDelay(  (1000 / REFRESH_RATE)  / portTICK_PERIOD_MS );
  }
}

void TouchTask( void * parameter ){
  for(;;){
    HandleTouch();
    vTaskDelay(  (1000 / POLLING_RATE)  / portTICK_PERIOD_MS);
  }
}

void TimeCriticalTasks( void * parameter ){
  for(;;){
    SampleTemperatures();
    SlowPWM();
    vTaskDelay( 10  / portTICK_PERIOD_MS);
  }
}

#pragma endregion Tasks

// ============================================================================================
//                                     Setup & Loop
// ============================================================================================

#pragma region SetupAndLoop

void setup(){

  // =============== Setup pins ===============
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_B, OUTPUT);
  pinMode(RELAY_F, OUTPUT);
  pinMode(RELAY_T, OUTPUT);

  // ============== Startup Sound ==============

  // the internal DAC and ADC are not used except for buzzer functions
  // temperature is measured by the ADS1219 over I2C
  // PID is done through manual PWM on the relays as the normal PWM frequencies is too high for AC solid state relays
  // so we can change the analog frequency without affecting anything else
  analogWriteResolution(12);

  analogWriteFrequency(2000);
  analogWrite(BUZZER_PIN, 2048);
  delay(100);
  
  analogWrite(BUZZER_PIN, 0);

  // ====== Start communication interfaces ======
  Serial0.begin(115200);
  SPI.begin(SPI_CLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  SPI.setFrequency(40000000); // 40 MHz
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial0.println("Setup started");

  // =========== Setup TFT display ==============
  SetupTFT();

  safemode |= tft.getTouch(&lastX, &lastY); // read initial touch state to check for safe mode

  // ========== Setup WiFi ============
  SetupWiFi();
  
  safemode |= tft.getTouch(&lastX, &lastY); // read initial touch state to check for safe mode

  // =========== Setup OTA ==============
  SetupOTA();

  safemode |= tft.getTouch(&lastX, &lastY); // read initial touch state to check for safe mode

  // ========== Check for safe mode ============
  // As the bare essentials have been set up, check for touchscreen input to enter safe mode
  if (safemode == true) {
    DisplaySafeMode();
    Serial0.println("Safe mode activated due to touchscreen input at startup");
    return;
  }

  tft.println("No input detected, continuing setup...");
  delay(1000); // wait a bit to allow the user to remove their finger

  // ========== Setup PID ============
  SetupPID();

  // ========== Setup ADS1219 ============
  SetupADS();

  // ======== Setup File System ==========
  SetupFS();

  // ======== Setup Webserver ==========
  SetupWebServer();

  // ======== Get the time ==========
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // ====== Setup Sprite Buffers =======
  SetupBuffers();

  // ========== Setup Pages ============
  SetupPages();

  // =========== Setup Tasks ==============

  // prevent SPI bus conflicts and a whole crash...
  xSemaphoreTake(SPI_Mutex, portMAX_DELAY);
  SetupTasks();

  // =========== Finalize ==============

  tft.println("Setup complete");
  tft.fillScreen(TFT_BLACK);
  
  // release SPI bus to the tasks
  xSemaphoreGive(SPI_Mutex);
}

void loop(){
  loopDuration = lastLoopEnd - lastLoopStart;
  lastLoopStart = millis();

  ArduinoOTA.handle();

  if (safemode) {
    // Ensure everything is off
    digitalWrite(RELAY_B, LOW);
    digitalWrite(RELAY_T, LOW);
    digitalWrite(RELAY_F, LOW);
    
    // Check for rollback touch
    if (millis() - lastRollbackTouchCheck > 100){
      lastRollbackTouchCheck = millis();
      rollbackTouching = tft.getTouch(&lastX, &lastY);
      rollbackWasTouching = rollbackTouching;

      if (rollbackTouching && !rollbackWasTouching){
        rollbackPressStart = millis();
      } 
      else if (rollbackWasTouching){
        if (millis() - rollbackPressStart >= 5000){
          // Rollback to last version
          tft.fillScreen(TFT_BLACK);
          tft.setCursor(160 , 100);
          tft.setTextColor(TFT_WHITE);
          tft.println("Rolling back to last version...");
          Beep();
          delay(2000);
          if (!esp_ota_mark_app_invalid_rollback_and_reboot()) {
            tft.println("Rollback failed!");

            Beep(1000);
            delay(2000);
            
            Beep(1000);
            delay(2000);

            Beep(1000);
            delay(2000);
          }
        }
      }
    }
    return;
  }

  if (currentPage != nullptr){
    currentPage->Update();
  }

  lastLoopEnd = millis();
}

#pragma endregion SetupAndLoop

// ============================================================================================
//                                    Setup Functions
// ============================================================================================

#pragma region SetupFunctions

void SetupTFT(){
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
}

void SetupWiFi(){
  
  Serial0.println("Wifi init");

  WiFi.begin(ssid, password);
  
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial0.println("Connection Failed! Rebooting...");
    tft.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  
  if (!MDNS.begin("tostireflow")) {
    tft.println("Error setting up MDNS responder!");
  } else {
    tft.println("mDNS responder started");
  }

  IPAddress IP = WiFi.localIP();
  Serial0.print("AP IP address: ");
  Serial0.println(IP);
  tft.print("IP Address: ");
  tft.println(IP);
}

void SetupWebServer(){
  Serial0.println("Webserver init");
  server.begin();

  // TODO: Add all webserver routes here
  server.serveStatic("/static", LittleFS, "/static");

  Serial0.println("Webserver initialized");
  tft.println("Webserver initialized");

}

// This functions mounts LittleFS
void SetupFS() {
  Serial0.println("File system init");
  tft.println("File system initializing...");
  if (!LittleFS.begin()) {
    Serial0.println("LittleFS Mount Failed");
    tft.println("LittleFS Mount Failed");
    delay(1000);
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

void SetupADS(){
  Serial0.println("Auxilary ADC init");

  while (!ads.begin(Wire, 0x44)) {
    tft.println("Failed to communicate with ADS1219.!");
    delay(1000);
  }
  ads.setVoltageReference(ADS1219_VREF_EXTERNAL);
  ads.setGain(ADS1219_GAIN_1);
}

void SetupOTA(){
  Serial0.println("OTA init");
  ArduinoOTA.setHostname("tostireflowOTA");
  ArduinoOTA.setPasswordHash(OTA_PASSWORD_HASH);
  ArduinoOTA.setMdnsEnabled(true);

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      if (!safemode){
        vTaskSuspend(DrawTaskHandle);
        vTaskSuspend(TouchTaskHandle);
        vTaskSuspend(TimeCriticalTasksHandle);
        vTaskSuspend(WebServerTaskHandle);
        digitalWrite(RELAY_B, LOW);
        digitalWrite(RELAY_T, LOW);
        digitalWrite(RELAY_F, LOW);
        safemode = true;
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

void SetupPages(){
  Serial0.println("Pages init");

  // link pages
  homePage.LinkPages(&profilePage, &settingsPage, &monitorPage);
  profilePage.LinkPages(&homePage);
  monitorPage.LinkPages(&homePage);
  settingsPage.LinkPages(&homePage, 
    new String[SETTING_SUBPAGES]{ "PID Settings", "Network Settings" }, 
    new Page*[SETTING_SUBPAGES]{&pidSettingsPage, &networkSettingsPage}, SETTING_SUBPAGES);
  pidSettingsPage.LinkPages(&settingsPage);
  networkSettingsPage.LinkPages(&settingsPage, 
    new String[NETWORK_SUBPAGES]{ "WiFi Settings" },
     new Page*[NETWORK_SUBPAGES]{&wifiSettingsPage}, NETWORK_SUBPAGES);
  wifiSettingsPage.LinkPages(&networkSettingsPage);

  // set initial page
  currentPage = &homePage;

  Serial0.println("Pages initialized");
  tft.println("Pages initialized");
}

// This function sets up the PID controller
// None of the functions have a failure state, so no error handling is needed
void SetupPID(){
  Serial0.println("PID init");

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

  ProfileStep::begin(&InputT, &InputB, &SetpointT, &SetpointB);

  Serial0.println("PID initialized");
  tft.println("PID initialized");
}

void SetupBuffers(){
  Serial0.println("Sprite buffers init");
  buffer.createSprite(DRAW_WIDTH, DRAW_HEIGHT);
  buffer.setFreeFont( &FreeSans9pt7b );
  buffer.setAttribute(PSRAM_ENABLE, true); // enable psram for the sprite buffer

  topBar.createSprite(SCREEN_WIDTH, DRAW_TOP);
  topBar.setFreeFont( &FreeSans9pt7b );
  topBar.setAttribute(PSRAM_ENABLE, true); // enable psram for the sprite buffer

  leftBar.createSprite(DRAW_LEFT, DRAW_HEIGHT);
  leftBar.setFreeFont( &FreeSans9pt7b );
  leftBar.setAttribute(PSRAM_ENABLE, true); // enable psram for the sprite buffer
  Serial0.println("Sprite buffers initialized");
  tft.println("Sprite buffers initialized");
}

void SetupTasks(){
  Serial0.println("Tasks init");

  xTaskCreatePinnedToCore(
    WebServerTask,          /* Function to implement the task */
    "Web Server Task",       /* Name of the task */
    10000,            /* Stack size in words */
    NULL,             /* Task input parameter */
    0,                /* Priority of the task */
    &WebServerTaskHandle,  /* Task handle. */
    1);               /* Core where the task should run */

  xTaskCreatePinnedToCore(
    DrawTask,          /* Function to implement the task */
    "Draw Task",       /* Name of the task */
    50000,            /* Stack size in words */
    NULL,             /* Task input parameter */
    1,                /* Priority of the task */
    &DrawTaskHandle,  /* Task handle. */
    1);               /* Core where the task should run */

  xTaskCreatePinnedToCore(
    TouchTask,          /* Function to implement the task */
    "Touch Task",       /* Name of the task */
    10000,            /* Stack size in words */
    NULL,             /* Task input parameter */
    2,                /* Priority of the task */
    &TouchTaskHandle,  /* Task handle. */
    1);               /* Core where the task should run */
              /* Core where the task should run */

  xTaskCreatePinnedToCore(
    TimeCriticalTasks,          /* Function to implement the task */
    "TimeCritical Task",       /* Name of the task */
    10000,            /* Stack size in words */
    NULL,             /* Task input parameter */
    3,                /* Priority of the task */
    &TimeCriticalTasksHandle,  /* Task handle. */
    0);  

  Serial0.println("Tasks initialized");
  tft.println("Tasks initialized");
}

#pragma endregion SetupFunctions

// ============================================================================================
//                                   Main Loop Functions
// ============================================================================================

#pragma region LoopFunctions

void DrawUI(){

  // Use buffer to draw the UI

  // Top bar with temperatures
  topBar.fillSprite(TFT_BLACK);
  topBar.setCursor(0 ,15);
  topBar.setTextColor(TFT_WHITE, TFT_BLACK, true);
  
  if (millis() - lastNTPCheck > NTPCheckInterval) {
    if (getLocalTime(&timeinfo)) {
      validNTP = true;
    } else {
      validNTP = false;
    }
    lastNTPCheck = millis();
  }

  if (validNTP) {
    if ((millis() - lastNTPCheck) % 1000 < 200) {
      topBar.print(&timeinfo, "%H:%M");
    }
    else{
      topBar.print(&timeinfo, "%H.%M");
    }
  } else {
    topBar.print("??:??");
  }

  topBar.setCursor(50, 15);
  topBar.print("TostiReflow V2 - v" FIRMWARE_VERSION);
  topBar.print(" | ");
  topBar.print("Top: ");
  if (avgTempTop < 30.0) topBar.setTextColor(TFT_CYAN, TFT_BLACK, true);
  else if (avgTempTop < 60.0) topBar.setTextColor(TFT_GREEN, TFT_BLACK, true);
  else if (avgTempTop < 150.0) topBar.setTextColor(TFT_YELLOW, TFT_BLACK, true);
  else topBar.setTextColor(TFT_RED, TFT_BLACK, true);

  topBar.print(avgTempTop, 1);

  topBar.setTextColor(TFT_WHITE, TFT_BLACK, true);
  topBar.print("C | Bottom: ");

  if (avgTempBottom < 30.0) topBar.setTextColor(TFT_CYAN, TFT_BLACK, true);
  else if (avgTempBottom < 60.0) topBar.setTextColor(TFT_GREEN, TFT_BLACK, true);
  else if (avgTempBottom < 150.0) topBar.setTextColor(TFT_YELLOW, TFT_BLACK, true);
  else topBar.setTextColor(TFT_RED, TFT_BLACK, true);
  topBar.print(avgTempBottom, 1);

  topBar.setTextColor(TFT_WHITE, TFT_BLACK, true);
  topBar.print("C");

  
  leftBar.fillSprite(TFT_BLACK);

  leftBar.drawRect(2,2, DRAW_LEFT - 4, DRAW_LEFT - 4, TFT_YELLOW);
  // fan indicator
  if (fanState)
    leftBar.fillRect(4, 4, DRAW_LEFT - 8, DRAW_LEFT - 8, TFT_YELLOW);
  
  // PID output indicator
  int halfRemainingHeight = DRAW_HEIGHT / 2; // half of the remaining height in the box

  leftBar.drawRect(2, DRAW_LEFT, DRAW_LEFT - 4, halfRemainingHeight - 2, TFT_BLUE);
  int pidHeightT = (int)(OutputT * halfRemainingHeight - 2);
  leftBar.fillRect(4, DRAW_LEFT + halfRemainingHeight - pidHeightT, DRAW_LEFT - 8, pidHeightT - 2, TFT_BLUE);

  leftBar.drawRect(2, DRAW_LEFT + halfRemainingHeight, DRAW_LEFT - 4, halfRemainingHeight, TFT_GREEN);
  int pidHeightB = (int)(OutputB * halfRemainingHeight - 2);
  leftBar.fillRect(4, DRAW_LEFT + halfRemainingHeight + halfRemainingHeight - pidHeightB, DRAW_LEFT - 8, pidHeightB, TFT_GREEN);
  
  if (xSemaphoreTake(SPI_Mutex, portMAX_DELAY) == pdTRUE) {
    leftBar.pushSprite(0, DRAW_TOP);
    topBar.pushSprite(0, 0);
    xSemaphoreGive(SPI_Mutex);
  }
}

// Handles touchscreen input and updates touch state variables
void HandleTouch(){

  wasTouched = isTouched;
  uint16_t x, y;
  
  if (xSemaphoreTake(SPI_Mutex, portMAX_DELAY) == pdTRUE) {
    isTouched = tft.getTouch(&x, &y);
    xSemaphoreGive(SPI_Mutex);
  }

  if (isTouched) {
      
    lastX = SCREEN_WIDTH - x;
    lastY = SCREEN_HEIGHT - y;

    // Rising edge
    if (!wasTouched) {
      startX = lastX;
      startY = lastY;

      Beep();
      
      if (currentPage != nullptr) {
        currentPage->OnTouch(lastX - DRAW_LEFT, lastY - DRAW_TOP);
      }
    }
    // Continuous touch
    else if (currentPage != nullptr) {
      currentPage->UpdateTouch(lastX - DRAW_LEFT, lastY - DRAW_TOP);
    }
    
  }
  // Falling edge
  else if (wasTouched) {
    if (currentPage != nullptr) {
      currentPage->OnRelease(lastX - DRAW_LEFT, lastY - DRAW_TOP);
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

  digitalWrite(RELAY_F, fanState ? HIGH : LOW);

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

void LoadProfiles(){
  // directory to scan
  String path = "/profiles";
  fs::File dir = LittleFS.open(path, "r");
  if (!dir || !dir.isDirectory()) {
    Serial0.println("Failed to open directory for reading");
    return;
  }

  int index = 0;
  fs::File file = dir.openNextFile();
  while (file) {
    String fileName = file.name();
    String prfName = fileName.substring(0, fileName.length() - 4); // remove .prf extension
    if (!fileName.endsWith(".prf")) {
      file = dir.openNextFile();
      continue; // skip non-profile files (Why are they here anyway?)
    }

    if (index < MAX_PROFILES) {
      profileNames[index] = prfName; 
      Profile prf = Profile();
      prf.name = prfName;
      profiles[index] = prf;


      if (profiles[index].LoadFromFile(file)) {
        Serial0.println("Loaded profile: " + prfName);
      } else {
        Serial0.println("Failed to load profile: " + prfName);
        profileNames[index] = "INVALID";
      }
      index++;
    } else {
      Serial0.println("Maximum number of profiles reached, skipping remaining files");
      break;
    }
  }
}

#pragma endregion LoopFunctions

// ============================================================================================
//                                  Touchscreen UI Pages
// ============================================================================================

#pragma region PageImplementations

// ============================================================================================
//                             Home Page - main navigation page
// ============================================================================================

#pragma region HomePage

void HomePage::LinkPages(ProfilePage *profilePage, SettingsPage *settingsPage, MonitorPage *monitorPage){
  // Link the pages together
  linkedProfilePage = profilePage;
  linkedSettingsPage = settingsPage;
  linkedMonitorPage = monitorPage;
}

void HomePage::Update() {
  // Nothing to update on the home page for now
  if (firstDraw){
    DrawStatic();
    firstDraw = false;
  }
}

void HomePage::DrawStatic(){
  // Draw static elements that don't change often

  buffer.fillSprite(darkestGrey);
  buffer.setTextColor(TFT_WHITE);

  // Draw the home page UI elements

  // Title
  buffer.setTextSize(2);
  buffer.setCursor(95, 115);
  buffer.println("Tosti Reflow V2");
  buffer.setTextSize(1);
  buffer.setCursor(145, 145);
  buffer.println("by JTD Chen, v" FIRMWARE_VERSION);

  // Buttons for navigation
  // Monitor Button
  buffer.fillRoundRect(monitorX, monitorY, monitorWidth, monitorHeight, 5, darkerGrey);
  buffer.setCursor(45, 255);
  buffer.println("Monitor");

  // Profile Button
  buffer.fillRoundRect(profileX, profileY, profileWidth, profileHeight, 5, darkerGrey);
  buffer.setCursor(200, 255);
  buffer.println("Profile");

  // Settings Button
  buffer.fillRoundRect(settingsX, settingsY, settingsWidth, settingsHeight, 5, darkerGrey);
  buffer.setCursor(340, 255);
  buffer.println("Settings");

  Push();
}

void HomePage::OnTouch(uint16_t x, uint16_t y) {
  // Check if touching any of the buttons
  wasTouchingProfile = (x >= profileX && x <= profileX + profileWidth && y >= profileY && y <= profileY + profileHeight);
  wasTouchingSettings = (x >= settingsX && x <= settingsX + settingsWidth && y >= settingsY && y <= settingsY + settingsHeight);
  wasTouchingMonitor = (x >= monitorX && x <= monitorX + monitorWidth && y >= monitorY && y <= monitorY + monitorHeight);

  if (wasTouchingProfile || wasTouchingSettings || wasTouchingMonitor) {
    // Redraw buttons with highlight
    if (wasTouchingProfile) {
      buffer.fillRoundRect(profileX, profileY, profileWidth, profileHeight, 5, highlightGrey);
      buffer.setCursor(200, 255);
      buffer.println("Profile");
      Push(profileX, profileY, profileWidth, profileHeight);
    } 
    if (wasTouchingSettings) {
      buffer.fillRoundRect(settingsX, settingsY, settingsWidth, settingsHeight, 5, highlightGrey);
      buffer.setCursor(340, 255);
      buffer.println("Settings");
      Push(settingsX, settingsY, settingsWidth, settingsHeight);
    } 
    if (wasTouchingMonitor) {
      buffer.fillRoundRect(monitorX, monitorY, monitorWidth, monitorHeight, 5, highlightGrey);
      buffer.setCursor(45, 255);
      buffer.println("Monitor");
      Push(monitorX, monitorY, monitorWidth, monitorHeight);
    } 
  }
}

void HomePage::UpdateTouch(uint16_t x, uint16_t y) {
  
}

void HomePage::OnRelease(uint16_t x, uint16_t y) {

  touchingProfile = (x >= profileX && x <= profileX + profileWidth && y >= profileY && y <= profileY + profileHeight);
  touchingSettings = (x >= settingsX && x <= settingsX + settingsWidth && y >= settingsY && y <= settingsY + settingsHeight);
  touchingMonitor = (x >= monitorX && x <= monitorX + monitorWidth && y >= monitorY && y <= monitorY + monitorHeight);

  // If touch is released within a button, navigate to that page
  if (!touchingProfile && !touchingSettings && !touchingMonitor) {

    if (wasTouchingProfile){
      // Redraw Profile button to normal
      buffer.fillRoundRect(profileX, profileY, profileWidth, profileHeight, 5, darkerGrey);
      buffer.setCursor(200, 255);
      buffer.println("Profile");
      Push(profileX, profileY, profileWidth, profileHeight);
    }

    if (wasTouchingMonitor){
      // Redraw Monitor button to normal
      buffer.fillRoundRect(monitorX, monitorY, monitorWidth, monitorHeight, 5, darkerGrey);
      buffer.setCursor(45, 255);
      buffer.println("Monitor");
      Push(monitorX, monitorY, monitorWidth, monitorHeight);
    }

    if (wasTouchingSettings){
      // Redraw Settings button to normal
      buffer.fillRoundRect(settingsX, settingsY, settingsWidth, settingsHeight, 5, darkerGrey);
      buffer.setCursor(340, 255);
      buffer.println("Settings");
      Push(settingsX, settingsY, settingsWidth, settingsHeight);
    }

    return; // Not touching any button
  }

  if (touchingProfile) {
    SwitchTo(linkedProfilePage);
  } else if (touchingSettings) {
    SwitchTo(linkedSettingsPage);
  } else if (touchingMonitor) {
    SwitchTo(linkedMonitorPage);
  }

  // Reset touch states
  touchingProfile = false;
  touchingSettings = false;
  touchingMonitor = false;
}

#pragma endregion HomePage

// ============================================================================================
//                      Profile Page - for selecting and running profiles
// ============================================================================================

#pragma region ProfilePage

void ProfilePage::LinkPages(HomePage* homePage) {
  // Link the pages together
  linkedHomePage = homePage;
}

void ProfilePage::Update() {
  // Update logic for ProfilePage
  if (firstDraw){
    DrawStatic();
    firstDraw = false;
  }
}

void ProfilePage::DrawStatic(){
  // Draw static elements that don't change often

  buffer.fillSprite(darkestGrey);
  buffer.setTextColor(TFT_WHITE);

  // Draw the profile page UI elements

  // Title
  buffer.setTextSize(2);
  buffer.setCursor(145, 130);
  buffer.println("Profile Page");
  buffer.setTextSize(1);
  buffer.setCursor(95, 160);
  buffer.println("Select and run profiles");

  // Back Button and Header Bar
  buffer.fillRect(60,0, DRAW_WIDTH - 60, 35, darkGrey);
  buffer.fillRoundRect(0, 0, 60, 35, 3, darkerGrey);
  buffer.setCursor(5, 20);
  buffer.println("Back");

  Push();
}

void ProfilePage::OnTouch(uint16_t x, uint16_t y) {
  // Handle touch start on ProfilePage
  if (x <= 60 && y <= 35) {
    SwitchTo(linkedHomePage);
  }
}

void ProfilePage::UpdateTouch(uint16_t x, uint16_t y) {
  // Handle touch move on ProfilePage
}

void ProfilePage::OnRelease(uint16_t x, uint16_t y) {
  // Handle touch release on ProfilePage
}

#pragma endregion ProfilePage

// ============================================================================================
//           Monitor Page - for showing the running a profile and showing progress
// ============================================================================================

#pragma region MonitorPage

void MonitorPage::LinkPages(HomePage* homePage) {
  // Link the pages together
  linkedHomePage = homePage;
}
void MonitorPage::Update() {
  // Update logic for MonitorPage
  if (firstDraw){
    DrawStatic();
    firstDraw = false;
  }
}

void MonitorPage::DrawStatic() {
  // Draw static elements that don't change often

  buffer.fillSprite(darkestGrey);
  buffer.setTextColor(TFT_WHITE);

  // Draw the monitor page UI elements

  // Title
  buffer.setTextSize(2);
  buffer.setCursor(140, 130);
  buffer.println("Monitor");
  buffer.setTextSize(1);
  buffer.setCursor(95, 160);
  buffer.println("Profile: None");

  
  // Back Button and Header Bar
  buffer.fillRect(60,0, DRAW_WIDTH - 60, 35, darkGrey);
  buffer.fillRoundRect(0, 0, 60, 35, 3, darkerGrey);
  buffer.setCursor(5, 20);
  buffer.println("Back");

  Push();
}

void MonitorPage::OnTouch(uint16_t x, uint16_t y) {
  if (x <= 60 && y <= 35) {
    SwitchTo(linkedHomePage);
  }
}

void MonitorPage::UpdateTouch(uint16_t x, uint16_t y) {
  // Handle touch move on MonitorPage
}

void MonitorPage::OnRelease(uint16_t x, uint16_t y) {
  // Handle touch release on MonitorPage
}

#pragma endregion MonitorPage

// ============================================================================================
//             Settings Page - for showing submenus for different settings
// ============================================================================================

#pragma region SettingsPage

void SettingsPage::LinkPages(HomePage* homePage, String settingNames[], Page* subPages[], int numSubPages) {
  // Link the pages together
  linkedHomePage = homePage;
  for (int i = 0; i < numSubPages; i++) {
    this->subPages[i] = subPages[i];
    this->settingNames[i] = settingNames[i];
  }
}

void SettingsPage::Update() {
  // Update logic for SettingsPage
  if (firstDraw){
    DrawStatic();
    firstDraw = false;
  }
}

void SettingsPage::DrawStatic(){
  // Draw static elements that don't change often

  buffer.fillSprite(darkestGrey);
  buffer.setTextColor(TFT_WHITE);

  // Draw the settings page UI elements

  // Title
  buffer.setTextSize(2);
  buffer.setCursor(130, 130);
  buffer.println("Settings");
  buffer.setTextSize(1);
  buffer.setCursor(95, 160);
  buffer.println("Configure system settings");

  // Back Button and Header Bar
  buffer.fillRect(60,0, DRAW_WIDTH - 60, 35, darkGrey);
  buffer.fillRoundRect(0, 0, 60, 35, 3, darkerGrey);
  buffer.setCursor(5, 20);
  buffer.println("Back");

  Push();
}

void SettingsPage::OnTouch(uint16_t x, uint16_t y) {
  // Handle touch start on SettingsPage
  if (x <= 60 && y <= 35) {
    SwitchTo(linkedHomePage);
  }
}

void SettingsPage::UpdateTouch(uint16_t x, uint16_t y) {
  // Handle touch move on SettingsPage
}

void SettingsPage::OnRelease(uint16_t x, uint16_t y) {
  // Handle touch release on SettingsPage
}

#pragma endregion SettingsPage

// ============================================================================================
//              PID Settings Page - for setting PID parameters and tuning
// ============================================================================================

#pragma region PIDSettingsPage

void PIDSettingsPage::LinkPages(SettingsPage* settingsPage) {
  // Link the pages together
  linkedSettingsPage = settingsPage;
}

void PIDSettingsPage::Update() {
  // Update logic for PIDSettingsPage
  if (firstDraw){
    DrawStatic();
    firstDraw = false;
  }
}

void PIDSettingsPage::DrawStatic(){
  // Draw static elements that don't change often

  buffer.fillSprite(darkestGrey);
  buffer.setTextColor(TFT_WHITE);

  // Draw the PID settings page UI elements

  // Title
  buffer.setTextSize(2);
  buffer.setCursor(120, 130);
  buffer.println("PID Settings");
  buffer.setTextSize(1);
  buffer.setCursor(95, 160);
  buffer.println("Set PID parameters");

  // Back Button and Header Bar
  buffer.fillRect(60,0, DRAW_WIDTH - 60, 35, darkGrey);
  buffer.fillRoundRect(0, 0, 60, 35, 3, darkerGrey);
  buffer.setCursor(5, 20);
  buffer.println("Back");

  Push();
}

void PIDSettingsPage::OnTouch(uint16_t x, uint16_t y) {
  // Handle touch start on PIDSettingsPage
}

void PIDSettingsPage::UpdateTouch(uint16_t x, uint16_t y) {
  // Handle touch move on PIDSettingsPage
}

void PIDSettingsPage::OnRelease(uint16_t x, uint16_t y) {
  // Handle touch release on PIDSettingsPage
}

#pragma endregion PIDSettingsPage

// ============================================================================================
//          Network Settings Page - Navigation for setting network related settings
// ============================================================================================

#pragma region NetworkSettingsPage

void NetworkSettingsPage::LinkPages(SettingsPage* settingsPage, String settingNames[], Page* subPages[], int numSubPages) {
  // Link the pages together
  linkedSettingsPage = settingsPage;
  for (int i = 0; i < numSubPages; i++) {
    this->subPages[i] = subPages[i];
    this->settingNames[i] = settingNames[i];
  }
}

void NetworkSettingsPage::Update() {
  // Update logic for NetworkSettingsPage
  if (firstDraw){
    DrawStatic();
    firstDraw = false;
  }
}

void NetworkSettingsPage::DrawStatic(){
  // Draw static elements that don't change often

  buffer.fillSprite(darkestGrey);
  buffer.setTextColor(TFT_WHITE);

  // Draw the network settings page UI elements

  // Title
  buffer.setTextSize(2);
  buffer.setCursor(110, 130);
  buffer.println("Network Settings");
  buffer.setTextSize(1);
  buffer.setCursor(95, 160);
  buffer.println("Configure network settings");

  // Back Button and Header Bar
  buffer.fillRect(60,0, DRAW_WIDTH - 60, 35, darkGrey);
  buffer.fillRoundRect(0, 0, 60, 35, 3, darkerGrey);
  buffer.setCursor(5, 20);
  buffer.println("Back");

  Push();
}

void NetworkSettingsPage::OnTouch(uint16_t x, uint16_t y) {
  // Handle touch start on NetworkSettingsPage
}

void NetworkSettingsPage::UpdateTouch(uint16_t x, uint16_t y) {
  // Handle touch move on NetworkSettingsPage
}

void NetworkSettingsPage::OnRelease(uint16_t x, uint16_t y) {
  // Handle touch release on NetworkSettingsPage
}

#pragma endregion NetworkSettingsPage

// ============================================================================================
//              WiFi Settings Page - for setting WiFi SSID and Password
// ============================================================================================

#pragma region WiFiSettingsPage

void WiFiSettingsPage::LinkPages(NetworkSettingsPage* networkSettingsPage) {
  // Link the pages together
  linkedNetworkSettingsPage = networkSettingsPage;
}

void WiFiSettingsPage::Update() {
  // Update logic for WiFiSettingsPage
  if (firstDraw){
    DrawStatic();
    firstDraw = false;
  }
}

void WiFiSettingsPage::DrawStatic(){
  // Draw static elements that don't change often

  buffer.fillSprite(darkestGrey);
  buffer.setTextColor(TFT_WHITE);

  // Draw the WiFi settings page UI elements

  // Title
  buffer.setTextSize(2);
  buffer.setCursor(120, 130);
  buffer.println("WiFi Settings");
  buffer.setTextSize(1);
  buffer.setCursor(95, 160);
  buffer.println("Set SSID and Password");

  // Back Button and Header Bar
  buffer.fillRect(60,0, DRAW_WIDTH - 60, 35, darkGrey);
  buffer.fillRoundRect(0, 0, 60, 35, 3, darkerGrey);
  buffer.setCursor(5, 20);
  buffer.println("Back");

  Push();
}

void WiFiSettingsPage::OnTouch(uint16_t x, uint16_t y) {
  // Handle touch start on WiFiSettingsPage
  
}

void WiFiSettingsPage::UpdateTouch(uint16_t x, uint16_t y) {
  // Handle touch move on WiFiSettingsPage
}

void WiFiSettingsPage::OnRelease(uint16_t x, uint16_t y) {
  // Handle touch release on WiFiSettingsPage
}

#pragma endregion WiFiSettingsPage

#pragma endregion PageImplementations

// ============================================================================================
//                                   Webpages and APIs
// ============================================================================================

#pragma region WebPagesAndAPIs

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

#pragma endregion WebPagesAndAPIs

// ============================================================================================
//                                    Debug Functions
// ============================================================================================

#pragma region DebugFunctions

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

  tft.fillRoundRect(10, 220, 460, 40, 10, TFT_DARKGREY);
  tft.setCursor(55, 235);
  tft.setTextColor(TFT_WHITE);
  tft.println("Hold for 5 seconds to roll back to last version");

  analogWriteFrequency(1000);

  analogWrite(BUZZER_PIN, 2048);
  delay(500);
  analogWrite(BUZZER_PIN, 0);
  
}

void Beep(){
  Beep(200);
}

void Beep(int duration){
  tone(BUZZER_PIN, 1000, duration); // 1kHz tone
}

#pragma endregion DebugFunctions
