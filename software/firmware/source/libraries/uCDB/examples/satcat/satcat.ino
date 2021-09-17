/*
  uCDB example sketch

  Queries satcat Constant DataBase.

  satcat.cdb (`key', `value') data obtained from the satcat.csv (https://celestrak.com/pub/satcat.csv).

  Put file satcat.cdb on SD card. Connect to Ardino board.
  Compile and upload sketch. Open Serial Monitor.

  Board: Arduino Uno
  SD card: SDHC 7.31Gb FAT32, sectorsPerCluster - 64
  SD chip select pin: 10
  Arduino IDE Serial Monitors settings: 115200 baud, no line ending.

  Created by Ioulianos Kakoulidis, 2021.
  Released into the public domain.
*/

#include <SPI.h>
#include <SD.h>

#define TRACE_CDB
#include "uCDB.hpp"

uCDB<SDClass, File> ucdb(SD);

unsigned long startMillis;

void printKey(const void *key, unsigned long keyLen) {
  Serial.write((const byte *)key, keyLen);
}

void printValue() {
  int c;

  while ((c = ucdb.readValue()) != -1) {
    Serial.write((byte)c);
  }
}

void query(const void *key, unsigned long keyLen) {
  cdbResult rt;

  Serial.println();
  startMillis = millis();
  rt = ucdb.findKey(key, keyLen);
  if (rt != KEY_FOUND) {
    Serial.println("Satellite not found: ");
    printKey(key, keyLen);
    Serial.println();
  }
  while (rt == KEY_FOUND) {
    printValue();
    Serial.println();
    rt = ucdb.findNextValue();
  }
  Serial.print("Query millisec: ");
  Serial.println(millis() - startMillis);
  Serial.println();
}

void setup() {
  const char fileName[] = "satcat.cdb";
  const char *sat[] = {"SPUTNIK", "EXPLORER", "DISCOVERER", "THOR", "VENERA"};

  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (SD.begin(10)) {
    Serial.println("SPI OK.");
  }
  else {
    Serial.println("SPI error.");
    while (true) {
      ;
    }
  }

  if (ucdb.open(fileName) == CDB_OK) {
    Serial.print("Total records number: ");
    Serial.println(ucdb.recordsNumber());
    // Find some existing codes.
    for (unsigned int i = 0; i < sizeof (sat) / sizeof (const char *); i++) {
      query(sat[i], strlen(sat[i]));
    }

    // Query not existing codes.
    query("AAAA", 4);
    query("BBBB", 4);
    query("CCCC", 4);
    query("YYYY", 4);
  }
  else {
    Serial.print("Invalid CDB: ");
    Serial.println(fileName);
  }
}

void loop() {
  String code;
  Serial.println("Enter satellite name prefix (^[A-Z,0-9]*) and press `Enter'");
  while (!Serial.available()) {}
  code = Serial.readString();
  query(code.c_str(), code.length());
}
