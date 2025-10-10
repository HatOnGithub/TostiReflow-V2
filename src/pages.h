#include <Arduino.h>
#include <TFT_eSPI.h>


// Abstract page class
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
        void Update() override;
        void Draw() override;
        void OnTouch(uint16_t x, uint16_t y) override;
        void UpdateTouch(uint16_t x, uint16_t y) override;
        void OnRelease() override;
};

// Page for selecting and running profiles
class ProfilePage : public Page {
    public:
        void Update() override;
        void Draw() override;
        void OnTouch(uint16_t x, uint16_t y) override;
        void UpdateTouch(uint16_t x, uint16_t y) override;
        void OnRelease() override;
};

// Page for showing the running a profile and showing progress
class MonitorPage : public Page {
    public:
        void Update() override;
        void Draw() override;
        void OnTouch(uint16_t x, uint16_t y) override;
        void UpdateTouch(uint16_t x, uint16_t y) override;
        void OnRelease() override;
};

// Page for showing submenus for different settings
class SettingsPage : public Page {
    public:
        void Update() override;
        void Draw() override;
        void OnTouch(uint16_t x, uint16_t y) override;
        void UpdateTouch(uint16_t x, uint16_t y) override;
        void OnRelease() override;
};

// Page for setting PID parameters Kp, Ki, and Kd, mainly for tuning
class PIDSettingsPage : public Page {
    public:
        void Update() override;
        void Draw() override;
        void OnTouch(uint16_t x, uint16_t y) override;
        void UpdateTouch(uint16_t x, uint16_t y) override;
        void OnRelease() override;
};

// Page for setting WiFi SSID and Password (and possibly other network settings in the future like MQTT, BT, etc)
class NetworkSettingsPage : public Page {
    public:
        void Update() override;
        void Draw() override;
        void OnTouch(uint16_t x, uint16_t y) override;
        void UpdateTouch(uint16_t x, uint16_t y) override;
        void OnRelease() override;
};

// Page for setting WiFi SSID and Password
class WiFiSettingsPage : public Page {
    public:
        void Update() override;
        void Draw() override;
        void OnTouch(uint16_t x, uint16_t y) override;
        void UpdateTouch(uint16_t x, uint16_t y) override;
        void OnRelease() override;
};