/*

  Hello_Adafruit_ILI9341.ino

  Demonstrates how to connect U8g2_for_Adafruit_GFX to Adafruit SSD1306 library.

  U8g2_for_Adafruit_GFX:
    - Use U8g2 fonts with Adafruit GFX
    - Support for UTF-8 in print statement
    - 90, 180 and 270 degree text direction
  
  List of all U8g2 fonts: https://github.com/olikraus/u8g2/wiki/fntlistall
      
*/
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "U8g2_for_Adafruit_GFX.h"

#define TFT_DC 9
#define TFT_CS 10

Adafruit_ILI9341 display = Adafruit_ILI9341(TFT_CS, TFT_DC);

U8G2_FOR_ADAFRUIT_GFX u8g2_for_adafruit_gfx;

void setup() {
  display.begin();
  display.setRotation(1);
  display.fillScreen(ILI9341_BLACK);
  u8g2_for_adafruit_gfx.begin(display);                 // connect u8g2 procedures to Adafruit GFX
}

unsigned long x = 0;

void loop() {  
  
  u8g2_for_adafruit_gfx.setFontMode(0);                 // use u8g2 none transparent mode
  u8g2_for_adafruit_gfx.setFontDirection(0);            // left to right (this is default)
  u8g2_for_adafruit_gfx.setForegroundColor(ILI9341_WHITE);      // apply Adafruit GFX color
  u8g2_for_adafruit_gfx.setFont(u8g2_font_helvR14_tf);  // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall
  u8g2_for_adafruit_gfx.setCursor(0,20);                // start writing at this position
  u8g2_for_adafruit_gfx.print("Hello World");
  u8g2_for_adafruit_gfx.setCursor(0,40);                // start writing at this position
  u8g2_for_adafruit_gfx.print("Umlaut ÄÖÜ");            // UTF-8 string with german umlaut chars

  u8g2_for_adafruit_gfx.setFont(u8g2_font_inb63_mn);  // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall
  u8g2_for_adafruit_gfx.setFontMode(0);                 // use u8g2 none transparent mode
  u8g2_for_adafruit_gfx.setCursor(0,220);                // start writing at this position
  u8g2_for_adafruit_gfx.print(x);            // UTF-8 string with german umlaut chars
  x++;
  delay(1000);
} 
