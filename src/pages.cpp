
/*
===============================================================================================
                                  Touchscreen UI Pages
===============================================================================================
*/

#include <TFT_eSPI.h>       // Hardware-specific library
#include "config.h"
#include "pages.h"


// all pages inherit from the abstract Page class

class Page;

// Forward declarations of all page classes for linking
class HomePage;
class ProfilePage;
class MonitorPage;
class SettingsPage;
class PIDSettingsPage;
class NetworkSettingsPage;
class WiFiSettingsPage;

// =========================================================================================================
//                                    Home Page - main navigation page
// =========================================================================================================

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

  screen->fillSprite(darkestGrey);
  screen->setTextColor(TFT_WHITE);

  // Draw the home page UI elements

  // Title
  screen->setTextSize(2);
  screen->setCursor(95, 115);
  screen->println("Tosti Reflow V2");
  screen->setTextSize(1);
  screen->setCursor(145, 145);
  screen->println("by JTD Chen, v" FIRMWARE_VERSION);

  // Buttons for navigation
  // Monitor Button
  screen->fillRoundRect(monitorX, monitorY, monitorWidth, monitorHeight, 5, darkerGrey);
  screen->setCursor(45, 255);
  screen->println("Monitor");

  // Profile Button
  screen->fillRoundRect(profileX, profileY, profileWidth, profileHeight, 5, darkerGrey);
  screen->setCursor(200, 255);
  screen->println("Profile");

  // Settings Button
  screen->fillRoundRect(settingsX, settingsY, settingsWidth, settingsHeight, 5, darkerGrey);
  screen->setCursor(340, 255);
  screen->println("Settings");

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
      screen->fillRoundRect(profileX, profileY, profileWidth, profileHeight, 5, highlightGrey);
      screen->setCursor(200, 255);
      screen->println("Profile");
      Push(profileX, profileY, profileWidth, profileHeight);
    } 
    if (wasTouchingSettings) {
      screen->fillRoundRect(settingsX, settingsY, settingsWidth, settingsHeight, 5, highlightGrey);
      screen->setCursor(340, 255);
      screen->println("Settings");
      Push(settingsX, settingsY, settingsWidth, settingsHeight);
    } 
    if (wasTouchingMonitor) {
      screen->fillRoundRect(monitorX, monitorY, monitorWidth, monitorHeight, 5, highlightGrey);
      screen->setCursor(45, 255);
      screen->println("Monitor");
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
      screen->fillRoundRect(profileX, profileY, profileWidth, profileHeight, 5, darkerGrey);
      screen->setCursor(200, 255);
      screen->println("Profile");
      Push(profileX, profileY, profileWidth, profileHeight);
    }

    if (wasTouchingMonitor){
      // Redraw Monitor button to normal
      screen->fillRoundRect(monitorX, monitorY, monitorWidth, monitorHeight, 5, darkerGrey);
      screen->setCursor(45, 255);
      screen->println("Monitor");
      Push(monitorX, monitorY, monitorWidth, monitorHeight);
    }

    if (wasTouchingSettings){
      // Redraw Settings button to normal
      screen->fillRoundRect(settingsX, settingsY, settingsWidth, settingsHeight, 5, darkerGrey);
      screen->setCursor(340, 255);
      screen->println("Settings");
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


// =========================================================================================================
//                            Profile Page - for selecting and running profiles
// =========================================================================================================
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

  screen->fillSprite(darkestGrey);
  screen->setTextColor(TFT_WHITE);

  // Draw the profile page UI elements

  // Title
  screen->setTextSize(2);
  screen->setCursor(145, 130);
  screen->println("Profile Page");
  screen->setTextSize(1);
  screen->setCursor(95, 160);
  screen->println("Select and run profiles");

  // Back Button and Header Bar
  screen->fillRect(60,0, DRAW_WIDTH - 60, 35, darkGrey);
  screen->fillRoundRect(0, 0, 60, 35, 3, darkerGrey);
  screen->setCursor(5, 20);
  screen->println("Back");

  Push();
}

void ProfilePage::OnTouch(uint16_t x, uint16_t y) {
  // Handle touch start on ProfilePage
  SwitchTo(linkedHomePage);
}

void ProfilePage::UpdateTouch(uint16_t x, uint16_t y) {
  // Handle touch move on ProfilePage
}

void ProfilePage::OnRelease(uint16_t x, uint16_t y) {
  // Handle touch release on ProfilePage
}



// =========================================================================================
//           Monitor Page - for showing the running a profile and showing progress
// =========================================================================================
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

  screen->fillSprite(darkestGrey);
  screen->setTextColor(TFT_WHITE);

  // Draw the monitor page UI elements

  // Title
  screen->setTextSize(2);
  screen->setCursor(140, 130);
  screen->println("Monitor");
  screen->setTextSize(1);
  screen->setCursor(95, 160);
  screen->println("Profile: None");

  
  // Back Button and Header Bar
  screen->fillRect(60,0, DRAW_WIDTH - 60, 35, darkGrey);
  screen->fillRoundRect(0, 0, 60, 35, 3, darkerGrey);
  screen->setCursor(5, 20);
  screen->println("Back");

  Push();
}

void MonitorPage::OnTouch(uint16_t x, uint16_t y) {
  SwitchTo(linkedHomePage);
}

void MonitorPage::UpdateTouch(uint16_t x, uint16_t y) {
  // Handle touch move on MonitorPage
}

void MonitorPage::OnRelease(uint16_t x, uint16_t y) {
  // Handle touch release on MonitorPage
}


// =========================================================================================
//             Settings Page - for showing submenus for different settings
// =========================================================================================

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

  screen->fillSprite(darkestGrey);
  screen->setTextColor(TFT_WHITE);

  // Draw the settings page UI elements

  // Title
  screen->setTextSize(2);
  screen->setCursor(130, 130);
  screen->println("Settings");
  screen->setTextSize(1);
  screen->setCursor(95, 160);
  screen->println("Configure system settings");

  // Back Button and Header Bar
  screen->fillRect(60,0, DRAW_WIDTH - 60, 35, darkGrey);
  screen->fillRoundRect(0, 0, 60, 35, 3, darkerGrey);
  screen->setCursor(5, 20);
  screen->println("Back");

  Push();
}

void SettingsPage::OnTouch(uint16_t x, uint16_t y) {
  // Handle touch start on SettingsPage
  SwitchTo(linkedHomePage);
}

void SettingsPage::UpdateTouch(uint16_t x, uint16_t y) {
  // Handle touch move on SettingsPage
}

void SettingsPage::OnRelease(uint16_t x, uint16_t y) {
  // Handle touch release on SettingsPage
}


// =========================================================================================
//              PID Settings Page - for setting PID parameters and tuning
// =========================================================================================

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

  screen->fillSprite(darkestGrey);
  screen->setTextColor(TFT_WHITE);

  // Draw the PID settings page UI elements

  // Title
  screen->setTextSize(2);
  screen->setCursor(120, 130);
  screen->println("PID Settings");
  screen->setTextSize(1);
  screen->setCursor(95, 160);
  screen->println("Set PID parameters");

  // Back Button and Header Bar
  screen->fillRect(60,0, DRAW_WIDTH - 60, 35, darkGrey);
  screen->fillRoundRect(0, 0, 60, 35, 3, darkerGrey);
  screen->setCursor(5, 20);
  screen->println("Back");

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


// =========================================================================================
//          Network Settings Page - Navigation for setting network related settings
// =========================================================================================

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

  screen->fillSprite(darkestGrey);
  screen->setTextColor(TFT_WHITE);

  // Draw the network settings page UI elements

  // Title
  screen->setTextSize(2);
  screen->setCursor(110, 130);
  screen->println("Network Settings");
  screen->setTextSize(1);
  screen->setCursor(95, 160);
  screen->println("Configure network settings");

  // Back Button and Header Bar
  screen->fillRect(60,0, DRAW_WIDTH - 60, 35, darkGrey);
  screen->fillRoundRect(0, 0, 60, 35, 3, darkerGrey);
  screen->setCursor(5, 20);
  screen->println("Back");

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

// =========================================================================================
//              WiFi Settings Page - for setting WiFi SSID and Password
// =========================================================================================

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

  screen->fillSprite(darkestGrey);
  screen->setTextColor(TFT_WHITE);

  // Draw the WiFi settings page UI elements

  // Title
  screen->setTextSize(2);
  screen->setCursor(120, 130);
  screen->println("WiFi Settings");
  screen->setTextSize(1);
  screen->setCursor(95, 160);
  screen->println("Set SSID and Password");

  // Back Button and Header Bar
  screen->fillRect(60,0, DRAW_WIDTH - 60, 35, darkGrey);
  screen->fillRoundRect(0, 0, 60, 35, 3, darkerGrey);
  screen->setCursor(5, 20);
  screen->println("Back");

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
