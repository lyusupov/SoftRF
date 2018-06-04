// Runs at 160MHz CPU frequency and BAUD_RATE 115200.
// Runs at 80MHz CPU frequency and BAUD_RATE 57600.
// Connect pin 12 to 14.

#include <SoftwareSerial.h>

#define BAUD_RATE 115200

SoftwareSerial swSer(14, 12, false, 256);

void setup() {
  Serial.begin(BAUD_RATE);
  swSer.begin(BAUD_RATE);

  // ESP8266 internal cache RAM needs warm up - allow write and ISR to load
  swSer.write(static_cast<uint8_t>(0));
  Serial.println("\nSoftware serial test started");

  for (char ch = ' '; ch <= 'z'; ch++) {
    swSer.write(ch);
  }
  swSer.println("");
}

void loop() {
  while (swSer.available() > 0) {
    Serial.write(swSer.read());
    yield();
  }
  while (Serial.available() > 0) {
    swSer.write(Serial.read());
    yield();
  }

}
