#include "ADXL362.h"

//TODO: set the correct pin for your setup
#define SPI_SLAVE_SELECT_PIN 7


ADXL362 acc(SPI_SLAVE_SELECT_PIN);

//TODO: comment out #define ADXL362_DEBUG at top of library .h file after code tested and working
void setup() {
  //TODO: adjust baud rate to your serial monitor setting
  Serial.begin(4800);
  while (!Serial);  

  check(acc.init());
  acc.printRegisters();

  Serial.print(F("\r\nCHIP REVISION : "));
  Serial.println(acc.getRevisionId());
  
  check(acc.activateMeasure());
  acc.printRegisters();
}

void loop() {
  //print xyz, just the highest 8 bits (saved 3 bytes spi transfer)
  MeasurementInMg xyzlow = acc.getXYZLowPower(ad_range_2G);
  printmeasurement(xyzlow);
  Serial.println(F(" (8bit)"));  

  //print full 12bit xyz
  MeasurementInMg xyz = acc.getXYZ(ad_range_2G);
  printmeasurement(xyz);

  //print temperature in celcius
  float temp = acc.getTemperature();
  Serial.print(F(" (12bit) temp : "));
  Serial.println(temp);

  delay(100);
}

void check(short code) {
  if (code <= 0) {
    Serial.print(F("\r\n********** ERROR: "));
    Serial.print(code);
    Serial.println(F(" **********"));
    if (code == -110)
      Serial.println(F("Device not connected? Otherwise check noisy power supply. Or wait a while till capacitors are empty: Softreset failed at first check for success"));
    else if (code >= -104 && code <= -102)
      Serial.println(F("Check power supply (decoupling, wire length). Wrong registry values read after reset/checkdevice."));
    delay(3000);
  } else {
    Serial.println(F("----------------------------------------"));
  }
}

void printmeasurement(MeasurementInMg m) {
  Serial.print(F(" x : "));
  Serial.print(m.x / 1000.0);
  Serial.print(F(" y : "));
  Serial.print(m.y / 1000.0);
  Serial.print(F(" z : "));
  Serial.print(m.z / 1000.0);
}
