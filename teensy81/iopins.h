#ifndef IOPINS_H
#define IOPINS_H

// ILI9341
#define TFT_SCLK        13
#define TFT_MOSI        12
#define TFT_MISO        11

#define TFT_TOUCH_CS    38
#define TFT_TOUCH_INT   37
#define TFT_DC          9
#define TFT_CS          10
#define TFT_RST         255  // 255 = unused, connected to 3.3V

// SD
#define SD_SCLK        13
#define SD_MOSI        12
#define SD_MISO        11  
#define SD_CS          23

// I2C keyboard
#define I2C_SCL_IO     19
#define I2C_SDA_IO     18


// Analog joystick (primary) for JOY2 and 5 extra buttons
#define PIN_JOY2_A1X    A1
#define PIN_JOY2_A2Y    A2
#define PIN_JOY2_BTN    17
#define PIN_KEY_USER1   3
#define PIN_KEY_USER2   4
#define PIN_KEY_USER3   255 // unmapped
#define PIN_KEY_USER4   255 // unmapped
#define PIN_KEY_ESCAPE  255 // unmapped



#endif




