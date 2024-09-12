// Setup for the Heltec Mesh Node T114
#define USER_SETUP_ID 74

// See SetupX_Template.h for all options available

#define TFT_WIDTH  135
#define TFT_HEIGHT 240
#define ST7789_DRIVER     // Configure all registers

#if !defined(_PINNUM)
#define _PINNUM(port, pin)    ((port)*32 + (pin))
#endif

#define TFT_MISO   -1
#define TFT_MOSI   _PINNUM(1,  9)
#define TFT_SCLK   _PINNUM(1,  8)
#define TFT_CS     _PINNUM(0, 11)
#define TFT_DC     _PINNUM(0, 12)
#define TFT_RST    _PINNUM(0,  2)
#define TFT_BL     _PINNUM(0, 15)

#undef  LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#undef  LOAD_FONT6
#undef  LOAD_FONT7
#undef  LOAD_FONT8
#undef  LOAD_GFXFF  // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts
#undef  SMOOTH_FONT

extern  SPIClass             SPI1;
#define TFT_SPI_PORT         SPI1
#define SPI_FREQUENCY        27000000
#define SPI_TOUCH_FREQUENCY  2500000
