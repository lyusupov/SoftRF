#ifndef _lmic_arduino_hal_config_h_
#define _lmic_arduino_hal_config_h_

#define CFG_eu868 1
//#define CFG_us915 1

#define CFG_autojoin

// This is the SX1272/SX1273 radio, which is also used on the HopeRF
// RFM92 boards.
//#define BRD_sx1272_radio 1
// This is the SX1276/SX1277/SX1278/SX1279 radio, which is also used on
// the HopeRF RFM95 boards.
#if !defined(__ASR6501__)
#define BRD_sx1276_radio 1
#endif
#define BRD_sx1262_radio 1

// Change the SPI clock speed if you encounter errors
// communicating with the radio.
// The standard range is 125kHz-8MHz, but some boards can go faster.
#ifndef LMIC_SPI_FREQ
#define LMIC_SPI_FREQ 1E6
#endif

// 16 μs per tick
// LMIC requires ticks to be 15.5μs - 100 μs long
#define US_PER_OSTICK_EXPONENT 4
#define US_PER_OSTICK (1 << US_PER_OSTICK_EXPONENT)
#define OSTICKS_PER_SEC (1000000 / US_PER_OSTICK)

//#define CFG_DEBUG
//#define CFG_DEBUG_VERBOSE
//#define DEBUG_TX
//#define DEBUG_RX

// Uncomment this to disable all code related to beacon tracking.
// Requires ping to be disabled too
#define DISABLE_CLASSB

// This allows choosing between multiple included AES implementations.
// Make sure exactly one of these is uncommented.
//
// This selects the original AES implementation included LMIC. This
// implementation is optimized for speed on 32-bit processors using
// fairly big lookup tables, but it takes up big amounts of flash on the
// AVR architecture.
// #define USE_ORIGINAL_AES
//
// This selects the AES implementation written by Ideetroon for their
// own LoRaWAN library. It also uses lookup tables, but smaller
// byte-oriented ones, making it use a lot less flash space (but it is
// also about twice as slow as the original).

#if ! (defined(USE_ORIGINAL_AES) || defined(USE_IDEETRON_AES))
# define USE_IDEETRON_AES
#endif

// Force Original AES on RPI
#ifdef RASPBERRY_PI
#ifdef USE_IDEETRON_AES
#undef USE_IDEETRON_AES
#define USE_ORIGINAL_AES
#endif
#endif

#if defined(USE_ORIGINAL_AES) && defined(USE_IDEETRON_AES)
# error "You may define at most one of USE_ORIGINAL_AES and USE_IDEETRON_AES"
#endif

#if defined(ESP8266)
#ifdef CFG_DEBUG
#undef CFG_DEBUG
#endif
#ifdef CFG_DEBUG_VERBOSE
#undef CFG_DEBUG_VERBOSE
#endif
#endif

#if defined(CFG_eu868)

enum _dr_eu868_t { DR_SF12=0, DR_SF11, DR_SF10, DR_SF9, DR_SF8, DR_SF7, DR_SF7B, DR_FSK, DR_NONE };

#endif /* CFG_eu868 */

#if defined(ESP32) || defined(ESP8266)
#include <pgmspace.h>
#endif

#endif // _lmic_arduino_hal_config_h_
