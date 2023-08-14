// Setup for the Heltec Tracker S3

// See SetupX_Template.h for all options available

#define TFT_WIDTH  80
#define TFT_HEIGHT 160
#define ST7735_DRIVER
#define ST7735_GREENTAB160x80
#define TFT_RGB_ORDER TFT_BGR

#define USE_HSPI_PORT

#define TFT_MISO   -1
#define TFT_MOSI   42
#define TFT_SCLK   41
#define TFT_CS     38
#define TFT_DC     40
#define TFT_RST    39
#define TFT_BL     45 /* V1.0 PCB marking */
//#define TFT_BL   21 /* V1.1 PCB marking */

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#undef  LOAD_GFXFF  // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts
#undef  SMOOTH_FONT

#define SPI_FREQUENCY        27000000
#define SPI_TOUCH_FREQUENCY  2500000
