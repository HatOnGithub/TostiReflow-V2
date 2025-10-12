#include <Arduino.h>
#include <TFT_eSPI.h>
#include "config.h"
#define PAGES_h

// Abstract page class
class Page {
  public:
    Page(TFT_eSprite* screen, Page** currentPage, SemaphoreHandle_t *spi_mutex) {
      this->screen = screen;
      this->currentPage = currentPage;
      this->spi_mutex = spi_mutex;
      darkerGrey = screen->color565(60, 60, 60);
      darkestGrey = screen->color565(30, 30, 30);
      highlightGrey = screen->color565(100, 100, 100);
    }
    virtual void Update() = 0;
    virtual void OnTouch(uint16_t x, uint16_t y) = 0;
    virtual void UpdateTouch(uint16_t x, uint16_t y) = 0;
    virtual void OnRelease(uint16_t x, uint16_t y) = 0;
    void Push(){
        xSemaphoreTake(*spi_mutex, portMAX_DELAY);
        screen->pushSprite(DRAW_LEFT, DRAW_TOP);
        xSemaphoreGive(*spi_mutex);
    }

    void Push(uint16_t x, uint16_t y, uint16_t w, uint16_t h){
        xSemaphoreTake(*spi_mutex, portMAX_DELAY);
        screen->pushSprite(DRAW_LEFT + x, DRAW_TOP + y, x, y, w, h);
        xSemaphoreGive(*spi_mutex);
    }

    void SwitchTo(Page* newPage) {
      *currentPage = newPage;
      newPage->firstDraw = true; // force redraw of the new page
    }
  protected:
    uint32_t darkerGrey, darkestGrey, highlightGrey;
    Page** currentPage; // pointer to the current page being displayed
    TFT_eSprite* screen;
    bool firstDraw = true; // flag to indicate if it's the first time drawing the page
    SemaphoreHandle_t *spi_mutex;
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
        HomePage(TFT_eSprite* screen, Page** currentPage, SemaphoreHandle_t *spi_mutex) : Page(screen, currentPage, spi_mutex) {}
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
        ProfilePage(TFT_eSprite* screen, Page** currentPage, SemaphoreHandle_t *spi_mutex) : Page(screen, currentPage, spi_mutex) {}
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
        MonitorPage(TFT_eSprite* screen, Page** currentPage, SemaphoreHandle_t *spi_mutex) : Page(screen, currentPage, spi_mutex) {}
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
        SettingsPage(TFT_eSprite* screen, Page** currentPage, SemaphoreHandle_t *spi_mutex) : Page(screen, currentPage, spi_mutex) {}
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
        PIDSettingsPage(TFT_eSprite* screen, Page** currentPage, SemaphoreHandle_t *spi_mutex) : Page(screen, currentPage, spi_mutex) {}
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
        NetworkSettingsPage(TFT_eSprite* screen, Page** currentPage, SemaphoreHandle_t *spi_mutex) : Page(screen, currentPage, spi_mutex) {}
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
        WiFiSettingsPage(TFT_eSprite* screen, Page** currentPage, SemaphoreHandle_t *spi_mutex) : Page(screen, currentPage, spi_mutex) {}
        void LinkPages(NetworkSettingsPage* networkSettingsPage);
        void Update() override;
        void DrawStatic();
        void OnTouch(uint16_t x, uint16_t y) override;
        void UpdateTouch(uint16_t x, uint16_t y) override;
        void OnRelease(uint16_t x, uint16_t y) override;
    private:
        NetworkSettingsPage* linkedNetworkSettingsPage = nullptr;
};