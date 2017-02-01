/*
 * TrueRandom - A true random number generator for Arduino.
 * This is variant of original work originally implemented as:
 * https://code.google.com/archive/p/tinkerit/ https://github.com/Cathedrow/TrueRandom
 * Copyright (c) 2010 Peter Knight, Tinker.it! All rights reserved.
 * Now modified for the ESP8266
 */

#include "ESP8266TrueRandom.h"

ICACHE_FLASH_ATTR ESP8266TrueRandomClass::ESP8266TrueRandomClass() {
	useRNG = true;
	lastYield = 0;
}

ICACHE_FLASH_ATTR int ESP8266TrueRandomClass::randomBitRaw(void) {
  // Needed to keep wifi stack running smoothly
  // And to avoid wdt reset
  if (lastYield == 0 || millis() - lastYield >= 50) {
    yield();
    lastYield = millis();
  }
  uint8_t bit = useRNG
	  ? (int)RANDOM_REG32 //using the onboard hardware random number generator (esp8266_peri.h)
	  : analogRead(A0);     //using A0 / TOUT

  return bit & 1;
}

ICACHE_FLASH_ATTR int ESP8266TrueRandomClass::randomBitRaw2(void) {
  // Software whiten bits using Von Neumann algorithm
  //
  // von Neumann, John (1951). "Various techniques used in connection
  // with random digits". National Bureau of Standards Applied Math Series
  // 12:36.
  //
  for(;;) {
    int a = randomBitRaw() | (randomBitRaw()<<1);
    if (a==1) return 0; // 1 to 0 transition: log a zero bit
    if (a==2) return 1; // 0 to 1 transition: log a one bit
    // For other cases, try again.
  }
  return 0;
}

ICACHE_FLASH_ATTR int ESP8266TrueRandomClass::randomBit(void) {
  // Software whiten bits using Von Neumann algorithm
  //
  // von Neumann, John (1951). "Various techniques used in connection
  // with random digits". National Bureau of Standards Applied Math Series
  // 12:36.
  //
  for(;;) {
    int a = randomBitRaw2() | (randomBitRaw2()<<1);
    if (a==1) return 0; // 1 to 0 transition: log a zero bit
    if (a==2) return 1; // 0 to 1 transition: log a one bit
    // For other cases, try again.
  }
  return 0;
}

ICACHE_FLASH_ATTR char ESP8266TrueRandomClass::randomByte(void) {
  char result = 0;
  uint8_t i;
  for (i=8; i--;) result += result + randomBit();
  return result;
}

ICACHE_FLASH_ATTR int ESP8266TrueRandomClass::rand() {
  int result = 0;
  uint8_t i;
  for (i=15; i--;) result += result + randomBit();
  return result;
}

ICACHE_FLASH_ATTR long ESP8266TrueRandomClass::random() {
  long result = 0;
  uint8_t i;
  for (i=31; i--;) result += result + randomBit();
  return result;
}

ICACHE_FLASH_ATTR long ESP8266TrueRandomClass::random(long howBig) {
  long randomValue;
  long topBit;
  long bitPosition;

  if (!howBig) return 0;
  randomValue = 0;
  if (howBig & (howBig-1)) {
    // Range is not a power of 2 - use slow method
    topBit = howBig-1;
    topBit |= topBit>>1;
    topBit |= topBit>>2;
    topBit |= topBit>>4;
    topBit |= topBit>>8;
    topBit |= topBit>>16;
    topBit = (topBit+1) >> 1;

    bitPosition = topBit;
    do {
      // Generate the next bit of the result
      if (randomBit()) randomValue |= bitPosition;

      // Check if bit
      if (randomValue >= howBig) {
        // Number is over the top limit - start again.
        randomValue = 0;
        bitPosition = topBit;
      } else {
        // Repeat for next bit
        bitPosition >>= 1;
      }
    } while (bitPosition);
  } else {
    // Special case, howBig is a power of 2
    bitPosition = howBig >> 1;
    while (bitPosition) {
      if (randomBit()) randomValue |= bitPosition;
      bitPosition >>= 1;
    }
  }
  return randomValue;
}

ICACHE_FLASH_ATTR long ESP8266TrueRandomClass::random(long howSmall, long howBig) {
  if (howSmall >= howBig) return howSmall;
  long diff = howBig - howSmall;
  return ESP8266TrueRandomClass::random(diff) + howSmall;
}

ICACHE_FLASH_ATTR void ESP8266TrueRandomClass::memfill(char* location, int size) {
  for (;size--;) *location++ = randomByte();
}

ICACHE_FLASH_ATTR void ESP8266TrueRandomClass::mac(uint8_t* macLocation) {
  memfill((char*)macLocation,6);
}

ICACHE_FLASH_ATTR void ESP8266TrueRandomClass::uuid(uint8_t* uuidLocation) {
  // Generate a Version 4 UUID according to RFC4122
  memfill((char*)uuidLocation,16);
  // Although the UUID contains 128 bits, only 122 of those are random.
  // The other 6 bits are fixed, to indicate a version number.
  uuidLocation[6] = 0x40 | (0x0F & uuidLocation[6]);
  uuidLocation[8] = 0x80 | (0x3F & uuidLocation[8]);
}

ICACHE_FLASH_ATTR String ESP8266TrueRandomClass::uuidToString(uint8_t* uuidLocation) {
  String string = "";
  int i;
  for (i=0; i<16; i++) {
    if (i==4) string += "-";
    if (i==6) string += "-";
    if (i==8) string += "-";
    if (i==10) string += "-";
    int topDigit = uuidLocation[i] >> 4;
    int bottomDigit = uuidLocation[i] & 0x0f;
    // High hex digit
    string += "0123456789abcdef"[topDigit];
    // Low hex digit
    string += "0123456789abcdef"[bottomDigit];
  }

  return string;
}

ESP8266TrueRandomClass ESP8266TrueRandom;
