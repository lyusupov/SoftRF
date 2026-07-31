// Basic demo for tap/doubletap readings from Adafruit LIS3DH

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
// Used for hardware & software SPI
#define LIS3DH_CS 10

// software SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// hardware SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

void setup(void) {
#ifndef ESP8266
  while (!Serial) yield();     // will pause Zero, Leonardo, etc until serial console opens
#endif

  Serial.begin(9600);
    Serial.println("Adafruit LIS3DH ADC Test!");
  
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1) yield();
  }
  Serial.println("LIS3DH found!");
  
  lis.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!
  
  Serial.print("Range = "); Serial.print(2 << lis.getRange());  
  Serial.println("G");
}


void loop() {
  int16_t adc;
  uint16_t volt;
  
  // read the ADCs
  adc = lis.readADC(1);
  volt = map(adc, -32512, 32512, 1800, 900);
  Serial.print("ADC1:\t"); Serial.print(adc); 
  Serial.print(" ("); Serial.print(volt); Serial.print(" mV)  ");
  
  adc = lis.readADC(2);
  volt = map(adc, -32512, 32512, 1800, 900);
  Serial.print("ADC2:\t"); Serial.print(adc); 
  Serial.print(" ("); Serial.print(volt); Serial.print(" mV)  ");

  adc = lis.readADC(3);
  volt = map(adc, -32512, 32512, 1800, 900);
  Serial.print("ADC3:\t"); Serial.print(adc); 
  Serial.print(" ("); Serial.print(volt); Serial.print(" mV)");

  Serial.println();
  delay(200);
}
