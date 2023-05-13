// Feather Power Meter
//
// Small Feater-based power monitor using an INA219 breakout and 
// monochrome OLED display.
//
// Author: Tony DiCola (modded by ladyada)
//
// Released under a MIT license: http://opensource.org/licenses/MIT

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_INA219.h>

// Configure orientation of the display.
// 0 = none, 1 = 90 degrees clockwise, 2 = 180 degrees, 3 = 270 degrees CW
#define ROTATION     0

#define NEO_PIN 6

// Create NeoPixel, OLED and INA219 globals.
Adafruit_SSD1306 display;
Adafruit_INA219 ina219;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(144, NEO_PIN, NEO_GRB + NEO_KHZ800);

// Keep track of total time and milliamp measurements for milliamp-hour computation.
uint32_t total_sec = 0;
float total_mA = 0.0;

uint8_t counter = 0;
void pixel_show_and_powerupdate() {
  pixels.show();
  if (counter == 0) {
    update_power_display();
  } else {
    counter = (counter+1) % 10 ;
  }
}

void setup()   {
  pixels.begin();
  pixels.show();  // Initialize all pixels to 'off'
  
  // Setup the INA219.
  ina219.begin();
  // By default the INA219 will be calibrated with a range of 32V, 2A.
  // However uncomment one of the below to change the range.  A smaller
  // range can't measure as large of values but will measure with slightly
  // better precision.
  //ina219.setCalibration_32V_1A();
  //ina219.setCalibration_16V_400mA();
  
  // Setup the OLED display.
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Wire.setClock(400000);
  
  // Clear the display.
  display.clearDisplay();
  display.display();
  
  // Set rotation.
  display.setRotation(ROTATION);

  // Set text size and color.
  display.setTextSize(2);
  display.setTextColor(WHITE);
}

void loop() {
  // Some example procedures showing how to display to the pixels:
  colorWipe(pixels.Color(255, 0, 0), 50); // Red
  colorWipe(pixels.Color(0, 255, 0), 50); // Green
  colorWipe(pixels.Color(0, 0, 255), 50); // Blue
//colorWipe(pixels.Color(0, 0, 0, 255), 50); // White RGBW
  // Send a theater pixel chase in...
  theaterChase(pixels.Color(127, 127, 127), 50); // White
  theaterChase(pixels.Color(127, 0, 0), 50); // Red
  theaterChase(pixels.Color(0, 0, 127), 50); // Blue

  rainbow(20);
  rainbowCycle(20);
  theaterChaseRainbow(50);
}


// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<pixels.numPixels(); i++) {
    pixels.setPixelColor(i, c);
    pixel_show_and_powerupdate();
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<pixels.numPixels(); i++) {
      pixels.setPixelColor(i, Wheel((i+j) & 255));
    }
    pixel_show_and_powerupdate();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< pixels.numPixels(); i++) {
      pixels.setPixelColor(i, Wheel(((i * 256 / pixels.numPixels()) + j) & 255));
    }
    pixel_show_and_powerupdate();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < pixels.numPixels(); i=i+3) {
        pixels.setPixelColor(i+q, c);    //turn every third pixel on
      }
      pixel_show_and_powerupdate();

      delay(wait);

      for (uint16_t i=0; i < pixels.numPixels(); i=i+3) {
        pixels.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < pixels.numPixels(); i=i+3) {
        pixels.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      pixel_show_and_powerupdate();

      delay(wait);

      for (uint16_t i=0; i < pixels.numPixels(); i=i+3) {
        pixels.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}


void update_power_display() {
  // Read voltage and current from INA219.
  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  
  // Compute load voltage, power, and milliamp-hours.
  float loadvoltage = busvoltage + (shuntvoltage / 1000);  
  float power_mW = loadvoltage * current_mA;
  total_mA += current_mA;
  total_sec += 1;
  float total_mAH = total_mA / 3600.0;  
  
  // Update display.
  display.clearDisplay();
  display.setCursor(0,0);
  
  // Mode 0, display volts and amps.
  printSIValue(loadvoltage, "V:", 2, 10);
  display.setCursor(0, 16);
  printSIValue(current_mA/1000.0, "A:", 5, 10);
 
  display.display();
}

void printSIValue(float value, char* units, int precision, int maxWidth) {
  // Print a value in SI units with the units left justified and value right justified.
  // Will switch to milli prefix if value is below 1.
  
  // Add milli prefix if low value.
  if (fabs(value) < 1.0) {
    display.print('m');
    maxWidth -= 1;
    value *= 1000.0;
    precision = max(0, precision-3);
  }
  
  // Print units.
  display.print(units);
  maxWidth -= strlen(units);
  
  // Leave room for negative sign if value is negative.
  if (value < 0.0) {
    maxWidth -= 1;
  }
  
  // Find how many digits are in value.
  int digits = ceil(log10(fabs(value)));
  if (fabs(value) < 1.0) {
    digits = 1; // Leave room for 0 when value is below 0.
  }
  
  // Handle if not enough width to display value, just print dashes.
  if (digits > maxWidth) {
    // Fill width with dashes (and add extra dash for negative values);
    for (int i=0; i < maxWidth; ++i) {
      display.print('-');
    }
    if (value < 0.0) {
      display.print('-');
    }
    return;
  }
  
  // Compute actual precision for printed value based on space left after
  // printing digits and decimal point.  Clamp within 0 to desired precision.
  int actualPrecision = constrain(maxWidth-digits-1, 0, precision);
  
  // Compute how much padding to add to right justify.
  int padding = maxWidth-digits-1-actualPrecision;
  for (int i=0; i < padding; ++i) {
    display.print(' ');
  }
  
  // Finally, print the value!
  display.print(value, actualPrecision);
}


