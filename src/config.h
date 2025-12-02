// Pin Assignments
#define TOUCH_IRQ_PIN 8
#define TOUCH_CS_PIN  9
#define DISPLAY_CS_PIN 10
#define SPI_MOSI_PIN 11
#define SPI_CLK_PIN 12
#define SPI_MISO_PIN 13
#define SPI_DC_PIN  14
#define SPI_RES_PIN 21
#define BACKLIGHT_PIN 3

#define I2C_SDA 4
#define I2C_SCL 5
#define DRDY_PIN 6

#define RELAY_T 15
#define RELAY_B 16
#define RELAY_F 17

#define BUZZER_PIN 18

// Settings
#define POLLING_RATE 50 // in Hz

#define REFRESH_RATE 20  // in Hz or FPS (same thing)

#define TEMP_SAMPLES 5 // Number of samples to average for temperature reading
#define TEMP_SAMPLE_RATE 20 // in Hz, how often to sample temperature for averaging

#define R_REF 10000 // Reference resistor value in ohms
#define R_0 100000 // Resistance of NTC at 25 degrees C
#define T_0 25 // Reference temperature of the NTC thermistor
#define R_NTC_B 3950 // B value of the NTC thermistors

#define PID_SAMPLE_TIME 1000 // in ms

#define PWM_PERIOD 500 // frequency of the PWM signal
#define PWM_STEPS 25

#define MAX_PROFILES 50 // Maximum number of profiles that can be stored
#define MAX_PROFILE_STEPS 100 // Maximum number of steps in a profile

// Information

#define FIRMWARE_VERSION "0.1.0"

#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 320

#define DRAW_TOP 30 // Height of the top bar
#define DRAW_LEFT 30 // Left margin for the drawable area

#define DRAW_WIDTH 450 // Width of the drawable area
#define DRAW_HEIGHT 290 // Height of the drawable area (excluding top bar)

#define GRAPH_WIDTH 280 // Width of the profile graph area
#define GRAPH_HEIGHT 155 // Height of the profile graph area

#define SETTING_SUBPAGES 2 // number of subpages in the settings page
#define NETWORK_SUBPAGES 1 // number of subpages in the network settings page

#define TOP 0
#define BOTTOM 1