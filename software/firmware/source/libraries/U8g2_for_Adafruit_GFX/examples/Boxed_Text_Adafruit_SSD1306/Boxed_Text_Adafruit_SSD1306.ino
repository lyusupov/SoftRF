/*

  Hello_Adafruit_SSD1306.ino

  Demonstrates how to draw a fitting box around a text.

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
  const char s[] = "gfx LCD";
  /* width and height of the text */
  int16_t height;
  int16_t width;
  /* desired position of the text */
  int16_t x = 4;
  int16_t y = 25;
  display.clearDisplay();                               // clear the graphcis buffer  
  u8g2_for_adafruit_gfx.setFont(u8g2_font_helvB14_tf);  // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall
  u8g2_for_adafruit_gfx.setFontMode(1);                 // use u8g2 transparent mode (this is default)
  u8g2_for_adafruit_gfx.setFontDirection(0);            // left to right (this is default)
  u8g2_for_adafruit_gfx.setForegroundColor(WHITE);      // apply Adafruit GFX color
  u8g2_for_adafruit_gfx.setCursor(x, y);                // start writing at this position
  u8g2_for_adafruit_gfx.print(s);
  /* calculate the size of the box into which the text will fit */
  height = u8g2_for_adafruit_gfx.getFontAscent() - u8g2_for_adafruit_gfx.getFontDescent();
  width = u8g2_for_adafruit_gfx.getUTF8Width(s);
  /* draw the box around the text*/
  display.drawRect(x, y-u8g2_for_adafruit_gfx.getFontAscent(), width, height, WHITE);
  display.display();                                    // make everything visible
  delay(2000);
} 
