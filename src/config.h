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
#define POLLING_RATE 100 // in Hz
#define REFRESH_RATE 50  // in Hz
#define TEMP_SAMPLES 5 // Number of samples to average for temperature reading
#define TEMP_SAMPLE_RATE 20 // in Hz, how often to sample temperature for averaging
#define R_REF 10000 // Reference resistor value in ohms
#define R_0 100000 // Resistance of NTC at 25 degrees C
#define T_0 25 // Reference temperature of the NTC thermistor
#define R_NTC_B 3950 // B value of the NTC thermistor

#define TOP 0
#define BOTTOM 1

// Information

#define FIRMWARE_VERSION "1.0.0"
#define DEVICE_NAME "TostiReflow V2"

#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 320
