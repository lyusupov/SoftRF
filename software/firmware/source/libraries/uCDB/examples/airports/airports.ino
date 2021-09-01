/*
  uCDB example sketch
  
  Queries airports Constant DataBase.

  airports.cdb (`key', `value') data obtained from the airports.csv (https://ourairports.com/data/) UTF-8 file.
  Key - column "ident" (ICAO code if available).
  Value - '|' separated columns "type", "name", "latitude_deg", "longitude_deg", "elevation_ft", "continent",
  "iso_country", "iso_region", "municipality", "scheduled_service", "gps_code", "iata_code", "local_code",
  "home_link", "wikipedia_link", "keywords".
  Columns description - https://ourairports.com/help/data-dictionary.html.

  Put file airports.cdb on SD card. Connect to Ardino board.
  Compile and upload sketch. Open Serial Monitor.
  
  Board: Arduino Uno
  SD card: SDHC 7.31Gb FAT32, sectorsPerCluster - 64
  SD chip select pin: 10
  Arduino IDE Serial Monitors settings: 9600 baud, no line ending.

  Created by Ioulianos Kakoulidis, 2021.
  Released into the public domain.     
*/
#include <SPI.h>
#include <SD.h>
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
  Serial.print("Query millisec: ");
  startMillis = millis();
  rt = ucdb.findKey(key, keyLen);
  Serial.println(millis() - startMillis);
  switch (rt) {
    case KEY_FOUND:
      Serial.print("Airport found: ");
      printKey(key, keyLen);
      Serial.println();
      printValue();
      break;
    
    case KEY_NOT_FOUND:
      Serial.print("Airport not found: ");
      printKey(key, keyLen);
      break;
      
    default:
      Serial.println("ERROR");
      break;
  }
  Serial.println();  
}

void setup() {
  const char fileName[] = "airports.cdb";
  const char *air[] = {"SBGL", "00AR", "PG-TFI", "US-0480", "ZYGH"};

  Serial.begin(9600);
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
    for (unsigned int i = 0; i < sizeof (air) / sizeof (const char *); i++) {
      query(air[i], strlen(air[i]));
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
  Serial.println("Enter airport code and press `Enter'");
  while (!Serial.available()) {}
  code = Serial.readString();
  query(code.c_str(), code.length());
}
