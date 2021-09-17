/*
  uCDB benchmark sketch

  Write file bench.cdb ([0..5000000] divisible by 5) on SD card. Connect to Ardino board.
  Compile and upload sketch. Open Serial Monitor.

  Board: Arduino Uno
  SD card: SDHC 7.31Gb FAT32, sectorsPerCluster - 64
  SD chip select pin: 10
  Arduino IDE Serial Monitors settings: 115200 baud, no line ending.

  Created by Ioulianos Kakoulidis, 2021.
  Released into the public domain.
*/

//#define USE_SERIALFLASH_LIB
#ifdef USE_SERIALFLASH_LIB
#include <SerialFlash.h>
#include <SPI.h>
#define fat SerialFlash
#else
#include "SdFat.h"
#endif

#define TRACE_CDB
#include "uCDB.hpp"

#ifdef USE_SERIALFLASH_LIB
uCDB<SerialFlashChip, SerialFlashFile> ucdb(SerialFlash);
#else
SdFat fat;
uCDB<SdFat, File> ucdb(fat);
#endif

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (!fat.begin(10)) {
    Serial.println("SD card initialization failed!");
    while (true) {
      ;
    }
  }
}

void loop() {
  char str[16];
  long key;
  unsigned long startMillis;
  cdbResult rt;
  int br;

  Serial.println("Press any key to start test");
  while (!Serial.available()) {
    ;
  }

  if (ucdb.open("bench.cdb") != CDB_OK) {
    Serial.print("Invalid CDB: ");
    Serial.println("bench.cdb");
    return;
  }

  Serial.print("Total records number: ");
  Serial.println(ucdb.recordsNumber());

  Serial.println("Querying 1000 random keys from interval [0, 5000000)...");
  startMillis = millis();
  for (int i = 0; i < 1000; ++i) {
    key = random(5000000);
    sprintf(str, "%ld", key);
    rt = ucdb.findKey(str, strlen(str));
    if (key%5) {
      if (rt != KEY_NOT_FOUND) {
        Serial.print("Error: ");
        Serial.println(key);
        break;
      }
    }
    else {
      if (rt != KEY_FOUND) {
        Serial.print("Error: ");
        Serial.println(key);
        break;
      }
    }
  }
  Serial.print("Query millis: ");
  Serial.println(millis() - startMillis);

  Serial.println("Querying 1000 random keys from interval [-5000000, 0)...");
  startMillis = millis();
  for (int i = 0; i < 1000; ++i) {
    key = random(-5000000, 0);
    sprintf(str, "%ld", key);
    rt = ucdb.findKey(str, strlen(str));
    if (rt != KEY_NOT_FOUND) {
      Serial.print("Error: ");
      Serial.println(str);
      break;
    }
  }
  Serial.print("Query millis: ");
  Serial.println(millis() - startMillis);

  Serial.println("Querying 1000 random keys with findNextValue() from interval [0, 5000000)...");
  startMillis = millis();
  for (int i = 0; i < 1000; ++i) {
    key = random(5000000);
    sprintf(str, "%ld", key);
    rt = ucdb.findKey(str, strlen(str));
    if (key%5) {
      if (rt != KEY_NOT_FOUND) {
        Serial.print("Error: ");
        Serial.println(key);
        break;
      }
    }
    else {
      if (rt != KEY_FOUND) {
        Serial.print("Error: ");
        Serial.println(key);
        break;
      }
    }
    rt = ucdb.findNextValue();
    if (rt != KEY_NOT_FOUND) {
      Serial.print("Error: ");
      Serial.println(key);
      break;
    }
  }
  Serial.print("Query millis: ");
  Serial.println(millis() - startMillis);

  Serial.println("Querying 1000 random keys with findNextValue() from interval [-5000000, 0)...");
  startMillis = millis();
  for (int i = 0; i < 1000; ++i) {
    key = random(-5000000, 0);
    sprintf(str, "%ld", key);
    rt = ucdb.findKey(str, strlen(str));
    if (rt != KEY_NOT_FOUND) {
      Serial.print("Error: ");
      Serial.println(str);
      break;
    }
    rt = ucdb.findNextValue();
    if (rt != KEY_NOT_FOUND) {
      Serial.print("Error: ");
      Serial.println(key);
      break;
    }
  }
  Serial.print("Query millis: ");
  Serial.println(millis() - startMillis);

  Serial.println("readValue() test...");
  for (int i = 0; i < 100; ++i) {
    sprintf(str, "%ld", (long)5*i);
    rt = ucdb.findKey(str, strlen(str));

    if (rt == KEY_FOUND) {
      Serial.print("Value length in bytes: ");
      Serial.println(ucdb.valueAvailable());
      br = ucdb.readValue(str, 15);
      if (br >= 0) {
        str[br] = '\0';
        Serial.println(str);
      }
    }
    else {
      Serial.print("Error: ");
      Serial.println(5*i);
      break;
    }
  }

  ucdb.close();
  while (Serial.available()) {
    Serial.read();
  }
}
