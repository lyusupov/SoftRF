#if !defined(_RADIOLIB_USER_BUILD_OPTIONS_H)
#define _RADIOLIB_USER_BUILD_OPTIONS_H

// this file can be used to define any user build options
// most commonly, RADIOLIB_EXCLUDE_* macros
// or enabling debug output

//#define RADIOLIB_DEBUG_BASIC        (1)   // basic debugging (e.g. reporting GPIO timeouts or module not being found)
//#define RADIOLIB_DEBUG_PROTOCOL     (1)   // protocol information (e.g. LoRaWAN internal information)
//#define RADIOLIB_DEBUG_SPI          (1)   // verbose transcription of all SPI communication - produces large debug logs!
//#define RADIOLIB_VERBOSE_ASSERT     (1)   // verbose assertions - will print out file and line number on failure

#define RADIOLIB_LOW_LEVEL            (1)   // Low-level hardware access enable

//#define RADIOLIB_EXCLUDE_CC1101           (1)
//#define RADIOLIB_EXCLUDE_NRF24            (1)
//#define RADIOLIB_EXCLUDE_RF69             (1)
//#define RADIOLIB_EXCLUDE_SX1231           (1) // dependent on RADIOLIB_EXCLUDE_RF69
//#define RADIOLIB_EXCLUDE_SI443X           (1)
//#define RADIOLIB_EXCLUDE_RFM2X            (1) // dependent on RADIOLIB_EXCLUDE_SI443X
//#define RADIOLIB_EXCLUDE_SX127X           (1)
//#define RADIOLIB_EXCLUDE_SX126X           (1)
//#define RADIOLIB_EXCLUDE_STM32WLX         (1) // dependent on RADIOLIB_EXCLUDE_SX126X
//#define RADIOLIB_EXCLUDE_SX128X           (1)
//#define RADIOLIB_EXCLUDE_AFSK             (1)
//#define RADIOLIB_EXCLUDE_AX25             (1)
//#define RADIOLIB_EXCLUDE_HELLSCHREIBER    (1)
//#define RADIOLIB_EXCLUDE_MORSE            (1)
//#define RADIOLIB_EXCLUDE_RTTY             (1)
//#define RADIOLIB_EXCLUDE_SSTV             (1)
//#define RADIOLIB_EXCLUDE_DIRECT_RECEIVE   (1)
//#define RADIOLIB_EXCLUDE_BELL             (1)
//#define RADIOLIB_EXCLUDE_APRS             (1)
//#define RADIOLIB_EXCLUDE_LORAWAN          (1)
//#define RADIOLIB_EXCLUDE_LR11X0           (1)
//#define RADIOLIB_EXCLUDE_FSK4             (1)
//#define RADIOLIB_EXCLUDE_PAGER            (1)
#define RADIOLIB_EXCLUDE_AX5X43           (1)

#endif
