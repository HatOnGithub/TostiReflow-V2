
/*
===============================================================================================
                                  Touchscreen UI Pages
===============================================================================================
*/

#include <TFT_eSPI.h>       // Hardware-specific library
#include "config.h"


// all pages inherit from the abstract Page class

class Page {
  public:
    virtual void Update() = 0;
    virtual void Draw() = 0;
    virtual void OnTouch(uint16_t x, uint16_t y) = 0;
    virtual void UpdateTouch(uint16_t x, uint16_t y) = 0;
    virtual void OnRelease() = 0;
};


// Main navigation page
class HomePage : public Page {
  public:
    void Update() override {
      // Update logic for HomePage
    }

    void Draw() override {
      
    }

    void OnTouch(uint16_t x, uint16_t y) override {
      // Handle touch start on HomePage
    }

    void UpdateTouch(uint16_t x, uint16_t y) override {
      // Handle touch move on HomePage
    }

    void OnRelease() override {
      // Handle touch release on HomePage
    }

};

// Page for selecting and running profiles
class ProfilePage : public Page {
  public:
    ProfilePage() {
      // Constructor logic for ProfilePage
    }
    void Update() override {
      // Update logic for ProfilePage
    }

    void Draw() override {
      // Drawing logic for ProfilePage
    }

    void OnTouch(uint16_t x, uint16_t y) override {
      // Handle touch start on ProfilePage
    }

    void UpdateTouch(uint16_t x, uint16_t y) override {
      // Handle touch move on ProfilePage
    }

    void OnRelease() override {
      // Handle touch release on ProfilePage
    }

};

// Page for showing the running a profile and showing progress
class MonitorPage : public Page {
  public:
    MonitorPage() {
      // Constructor logic for MonitorPage
    }
    void Update() override {
      // Update logic for MonitorPage
    }

    void Draw() override {
      // Drawing logic for MonitorPage
    }

    void OnTouch(uint16_t x, uint16_t y) override {
      // Handle touch start on MonitorPage
    }

    void UpdateTouch(uint16_t x, uint16_t y) override {
      // Handle touch move on MonitorPage
    }

    void OnRelease() override {
      // Handle touch release on MonitorPage
    }

};

// Page for showing submenus for different settings
class SettingsPage : public Page {
  public:
    SettingsPage() {
      // Constructor logic for SettingsPage
    }
    void Update() override {
      // Update logic for SettingsPage
    }

    void Draw() override {
      // Drawing logic for SettingsPage
    }

    void OnTouch(uint16_t x, uint16_t y) override {
      // Handle touch start on SettingsPage
    }

    void UpdateTouch(uint16_t x, uint16_t y) override {
      // Handle touch move on SettingsPage
    }

    void OnRelease() override {
      // Handle touch release on SettingsPage
    }

};

// Page for setting PID parameters Kp, Ki, and Kd, mainly for tuning
class PIDSettingsPage : public Page {
  public:
    PIDSettingsPage() {
      // Constructor logic for PIDSettingsPage
    }

    void Update() override {
      // Update logic for PIDSettingsPage
    }

    void Draw() override {
      // Drawing logic for PIDSettingsPage
    }

    void OnTouch(uint16_t x, uint16_t y) override {
      // Handle touch start on PIDSettingsPage
    }

    void UpdateTouch(uint16_t x, uint16_t y) override {
      // Handle touch move on PIDSettingsPage
    }

    void OnRelease() override {
      // Handle touch release on PIDSettingsPage
    }

};

// Page for setting WiFi SSID and Password (and possibly other network settings in the future like MQTT, BT, etc)
class NetworkSettingsPage : public Page {
  public:
    NetworkSettingsPage() {
      // Constructor logic for NetworkSettingsPage
    }

    void Update() override {
      // Update logic for NetworkSettingsPage
    }

    void Draw() override {
      // Drawing logic for NetworkSettingsPage
    }

    void OnTouch(uint16_t x, uint16_t y) override {
      // Handle touch start on NetworkSettingsPage
    }

    void UpdateTouch(uint16_t x, uint16_t y) override {
      // Handle touch move on NetworkSettingsPage
    }

    void OnRelease() override {
      // Handle touch release on NetworkSettingsPage
    }

};

// Page for setting WiFi SSID and Password
class WiFiSettingsPage : public Page {
  public:
    WiFiSettingsPage() {
      // Constructor logic for WiFiSettingsPage
    }

    void Update() override {
      // Update logic for WiFiSettingsPage
    }

    void Draw() override {
      // Drawing logic for WiFiSettingsPage
    }

    void OnTouch(uint16_t x, uint16_t y) override {
      // Handle touch start on WiFiSettingsPage
    }

    void UpdateTouch(uint16_t x, uint16_t y) override {
      // Handle touch move on WiFiSettingsPage
    }

    void OnRelease() override {
      // Handle touch release on WiFiSettingsPage
    }

};