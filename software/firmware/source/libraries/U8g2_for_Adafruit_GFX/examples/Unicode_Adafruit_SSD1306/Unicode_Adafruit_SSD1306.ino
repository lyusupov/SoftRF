/*

  Hello_Adafruit_SSD1306.ino

  Demonstrates how to connect U8g2_for_Adafruit_GFX to Adafruit SSD1306 library.

  U8g2_for_Adafruit_GFX:
    - Use U8g2 fonts with Adafruit GFX
    - Support for UTF-8 in print statement
    - 90, 180 and 270 degree text direction
  
  List of all U8g2 fonts: https://github.com/olikraus/u8g2/wiki/fntlistall
      
*/
#include <Adafruit_SSD1306.h>
#include <U8g2_for_Adafruit_GFX.h>

Adafruit_SSD1306 display(/*MOSI*/ 11, /*CLK*/ 13, /*DC*/ 9, /*RESET*/ 8, /*CS*/ 10);
U8G2_FOR_ADAFRUIT_GFX u8g2_for_adafruit_gfx;

void setup() {
  display.begin(SSD1306_SWITCHCAPVCC);
  u8g2_for_adafruit_gfx.begin(display);                 // connect u8g2 procedures to Adafruit GFX
}

void loop() {  
  display.clearDisplay();                               // clear the graphcis buffer  
  u8g2_for_adafruit_gfx.setFontDirection(0);            // left to right (this is default)
  u8g2_for_adafruit_gfx.setForegroundColor(WHITE);      // apply Adafruit GFX color
  
  u8g2_for_adafruit_gfx.setFont(u8g2_font_siji_t_6x10);  // icon font
  u8g2_for_adafruit_gfx.setFontMode(1);                 // use u8g2 transparent mode (this is default)
  u8g2_for_adafruit_gfx.drawGlyph(0, 10, 0x0e200);  // Power Supply
  u8g2_for_adafruit_gfx.drawGlyph(12, 10, 0x0e201);  // Charging
  u8g2_for_adafruit_gfx.drawGlyph(24, 10, 0x0e10a);  // Right Arrow
  u8g2_for_adafruit_gfx.drawGlyph(36, 10, 0x0e24b);  // full Battery

  u8g2_for_adafruit_gfx.setFont(u8g2_font_7x13_te);  // extended font
  u8g2_for_adafruit_gfx.setFontMode(1);                 // use u8g2 transparent mode (this is default)
  u8g2_for_adafruit_gfx.setCursor(0,40);                // start writing at this position
  u8g2_for_adafruit_gfx.print("<Ȧǀʘ>");            // UTF-8 string: "<" 550 448 664 ">"
  display.display();                                    // make everything visible
  delay(2000);
} 

