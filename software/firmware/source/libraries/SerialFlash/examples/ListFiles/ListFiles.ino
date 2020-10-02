#include <SerialFlash.h>
#include <SPI.h>

const int FlashChipSelect = 13; // digital pin for flash chip CS pin

void setup() {
  Serial.begin(9600);

  // wait for Arduino Serial Monitor
  while (!Serial) ;
  delay(100);
  Serial.println("All Files on SPI Flash chip:");

  if (!SerialFlash.begin(FlashChipSelect)) {
    error("Unable to access SPI Flash chip");
  }

  SerialFlash.opendir();
  while (1) {
    char filename[64];
    uint32_t filesize;

    if (SerialFlash.readdir(filename, sizeof(filename), filesize)) {
      Serial.print("  ");
      Serial.print(filename);
      spaces(20 - strlen(filename));
      Serial.print("  ");
      Serial.print(filesize);
      Serial.print(" bytes");
      Serial.println();
    } else {
      break; // no more files
    }
  }
}

void spaces(int num) {
  for (int i=0; i < num; i++) {
    Serial.print(" ");
  }
}

void loop() {
}

void error(const char *message) {
  while (1) {
    Serial.println(message);
    delay(2500);
  }
}
