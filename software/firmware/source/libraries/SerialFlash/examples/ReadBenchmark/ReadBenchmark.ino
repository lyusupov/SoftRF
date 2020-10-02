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
    while (1) {
      Serial.println("Unable to access SPI Flash chip");
      delay(2500);
    }
  }

  SerialFlash.opendir();
  int filecount = 0;
  while (1) {
    char filename[64];
    uint32_t filesize;

    if (SerialFlash.readdir(filename, sizeof(filename), filesize)) {
      Serial.print("  ");
      Serial.print(filename);
      Serial.print(", ");
      Serial.print(filesize);
      Serial.print(" bytes");
      SerialFlashFile file = SerialFlash.open(filename);
      if (file) {
        unsigned long usbegin = micros();
        unsigned long n = filesize;
        char buffer[256];
        while (n > 0) {
          unsigned long rd = n;
          if (rd > sizeof(buffer)) rd = sizeof(buffer);
          file.read(buffer, rd);
          n = n - rd;
        }
        unsigned long usend = micros();
        Serial.print(", read in ");
        Serial.print(usend - usbegin);
        Serial.print(" us, speed = ");
        Serial.print((float)filesize * 1000.0 / (float)(usend - usbegin));
        Serial.println(" kbytes/sec");
        file.close();
      } else {
        Serial.println(" error reading this file!");
      }
      filecount = filecount + 1;
    } else {
      if (filecount == 0) {
        Serial.println("No files found in SerialFlash memory.");
      }
      break; // no more files
    }
  }
}

void loop() {
}

