/*
  uCDB test sketch

  Board: Arduino Uno
  SD card: SDHC 7.31Gb FAT32, sectorsPerCluster - 64
  SD chip select pin: 10
  Arduino IDE Serial Monitors settings: 9600 baud, no line ending.

  Created by Ioulianos Kakoulidis, 2021.
  Released into the public domain.
*/

#include "SdFat.h"

#define TRACE_CDB
#include "uCDB.hpp"

SdFat fat;

void setup() {
  Serial.begin(9600);
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

bool closed_test1() {
  uCDB<SdFat, File> ucdb(fat);
  byte buff[64];

  if (ucdb.findNextValue() != CDB_CLOSED)
    return false;
  if (ucdb.findKey("AAA", 3) != CDB_CLOSED)
    return false;
  if (ucdb.findNextValue() != CDB_CLOSED)
    return false;

  if (ucdb.readValue() != -1)
    return false;
  if (ucdb.readValue(buff, 64) != -1)
    return false;
  if (ucdb.valueAvailable() != 0)
    return false;
  if (ucdb.recordsNumber() != 0)
    return false;
  return true;
}

bool closed_test2() {
  uCDB<SdFat, File> ucdb(fat);
  byte buff[64];

  if (ucdb.open("XXXXXXXXXXXXXXXXXXXXX.$$$" ) != CDB_NOT_FOUND) {
    ucdb.close();
    return false;
  }

  if (ucdb.findNextValue() != CDB_CLOSED)
    return false;
  if (ucdb.findKey("AAA", 3) != CDB_CLOSED)
    return false;
  if (ucdb.findNextValue() != CDB_CLOSED)
    return false;

  if (ucdb.readValue() != -1)
    return false;
  if (ucdb.readValue(buff, 64) != -1)
    return false;
  if (ucdb.valueAvailable() != 0)
    return false;
  if (ucdb.recordsNumber() != 0)
    return false;
  return true;
}

bool closed_test3() {
  uCDB<SdFat, File> ucdb(fat);
  byte buff[64];

  if (ucdb.open("airports.cdb" ) != CDB_OK) {
    ucdb.close();
    return false;
  }
  ucdb.close();

  if (ucdb.findNextValue() != CDB_CLOSED)
    return false;
  if (ucdb.findKey("ZYGH", 3) != CDB_CLOSED)
    return false;
  if (ucdb.findNextValue() != CDB_CLOSED)
    return false;
  if (ucdb.findKey("AAAA", 3) != CDB_CLOSED)
    return false;

  if (ucdb.readValue() != -1)
    return false;
  if (ucdb.readValue(buff, 64) != -1)
    return false;
  if (ucdb.valueAvailable() != 0)
    return false;
  if (ucdb.recordsNumber() != 0)
    return false;
  return true;
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

  Serial.println(closed_test1());
  Serial.println(closed_test2());
  Serial.println(closed_test3());

  while (Serial.available()) {
    Serial.read();
  }
}