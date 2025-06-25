// CRC-CCIT Implementation based on work by Francesco Sacchi

#ifndef CRC_CCIT_H
#define CRC_CCIT_H

#include <stdint.h>

#if defined(__AVR__) || \
    defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2) || \
    (defined(ARDUINO_ARCH_RP2040) && defined(ARDUINO_ARCH_MBED))
#include <avr/pgmspace.h>
#endif

#if defined(ESP8266) || defined(ESP32) || defined(__ASR6501__) || \
    defined(ARDUINO_ARCH_ASR650X) || (defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED))
#include <pgmspace.h>
#endif

#if defined(RASPBERRY_PI) || defined(LUCKFOX_LYRA)
#define pgm_read_word(addr) (*(const unsigned int *)(addr))
#endif /* RASPBERRY_PI */

#define CRC_CCIT_INIT_VAL ((uint16_t)0xFFFF)

extern const uint16_t crc_ccit_table[256];

inline uint16_t update_crc_ccit(uint8_t c, uint16_t prev_crc) {
    return (prev_crc >> 8) ^ pgm_read_word(&crc_ccit_table[(prev_crc ^ c) & 0xff]);
}

#endif
