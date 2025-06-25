/*
 * Platform_ESP32.h
 * Copyright (C) 2019-2025 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#if defined(ESP32)

#ifndef PLATFORM_ESP32_H
#define PLATFORM_ESP32_H

#if defined(ARDUINO)
#include <Arduino.h>
#endif /* ARDUINO */

#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS  9

#define SerialInput           Serial1

/* Peripherals */
#define INA219_ADDRESS_ALT    (0x43) // 1000011 (A0=SCL, A1=GND)

#define EXCLUDE_ETHERNET

/* TTGO T5 and T5S SPI pins mapping */
#define SOC_GPIO_PIN_MOSI_T5S 23
#define SOC_GPIO_PIN_MISO_T5S 12
#define SOC_GPIO_PIN_SCK_T5S  18
#define SOC_GPIO_PIN_SS_T5S   5

/* TTGO T5S EPD pins mapping */
#define SOC_EPD_PIN_DC_T5S    17
#define SOC_EPD_PIN_RST_T5S   16
#define SOC_EPD_PIN_BUSY_T5S  4

/* TTGO T5S microSD pins mapping */
#define SOC_SD_PIN_MOSI_T5S   15
#define SOC_SD_PIN_MISO_T5S   2
#define SOC_SD_PIN_SCK_T5S    14
#define SOC_SD_PIN_SS_T5S     13

/* TTGO T5S I2S-out pins mapping */
#define SOC_GPIO_PIN_BCLK     26
#define SOC_GPIO_PIN_LRCLK    25
#define SOC_GPIO_PIN_DOUT     19

/* TTGO T5S buttons mapping */
#define SOC_BUTTON_MODE_T5S   37
#define SOC_BUTTON_UP_T5S     38
#define SOC_BUTTON_DOWN_T5S   39

/* TTGO T5S green LED mapping */
#define SOC_GPIO_PIN_LED_T5S  22

#if defined(CONFIG_IDF_TARGET_ESP32)
#define SOC_GPIO_PIN_GNSS_RX  21
#define SOC_GPIO_PIN_GNSS_TX  22

#define SOC_BUTTON_MODE_DEF   0

/* Waveshare ESP32 SPI pins mapping */
#define SOC_GPIO_PIN_MOSI_WS  14
#define SOC_GPIO_PIN_MISO_WS  12
#define SOC_GPIO_PIN_SCK_WS   13
#define SOC_GPIO_PIN_SS_WS    15

/* Waveshare ESP32 EPD pins mapping */
#define SOC_EPD_PIN_DC_WS     27
#define SOC_EPD_PIN_RST_WS    26
#define SOC_EPD_PIN_BUSY_WS   25

#elif defined(CONFIG_IDF_TARGET_ESP32S2)
#define SOC_GPIO_PIN_GNSS_RX  1
#define SOC_GPIO_PIN_GNSS_TX  2

#define SOC_BUTTON_MODE_DEF   0

/* Waveshare ESP32-S2 SPI pins mapping */
#define SOC_GPIO_PIN_MOSI_WS  35
#define SOC_GPIO_PIN_MISO_WS  33
#define SOC_GPIO_PIN_SCK_WS   36
#define SOC_GPIO_PIN_SS_WS    34

/* Waveshare ESP32-S2 EPD pins mapping */
#define SOC_EPD_PIN_DC_WS     37
#define SOC_EPD_PIN_RST_WS    38
#define SOC_EPD_PIN_BUSY_WS   39

#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#define SOC_GPIO_PIN_GNSS_RX  13
#define SOC_GPIO_PIN_GNSS_TX  15

#define SOC_BUTTON_MODE_DEF   0

/* Waveshare Pico & Banana PicoW SPI pins mapping */
#define SOC_GPIO_PIN_MOSI_WS  38
#define SOC_GPIO_PIN_MISO_WS  41
#define SOC_GPIO_PIN_SCK_WS   21
#define SOC_GPIO_PIN_SS_WS    16

/* Waveshare Pico & Banana PicoW EPD pins mapping */
#define SOC_EPD_PIN_DC_WS     18
#define SOC_EPD_PIN_RST_WS    39
#define SOC_EPD_PIN_BUSY_WS   40

/* Waveshare Pico UPS-B & Banana PicoW I2C */
#define SOC_GPIO_PIN_SDA      12
#define SOC_GPIO_PIN_SCL      14

/* Waveshare Pico & Banana PicoW keys mapping */
#define SOC_GPIO_PIN_KEY0     42
#define SOC_GPIO_PIN_KEY1     2  // RTC GPIO
#define SOC_GPIO_PIN_KEY2     47

/* misc. Banana PicoW pins */
#define SOC_GPIO_PIN_LED      46 // active HIGH
#define SOC_GPIO_PIN_NEOPIXEL 48
#define SOC_GPIO_PIN_CAP      34 // RST+RST for UF2 boot

#define LED_STATE_ON          HIGH // State when LED is litted

// USB
#define SOC_GPIO_PIN_USB_DP   20
#define SOC_GPIO_PIN_USB_DN   19

//#define USE_ADAFRUIT_MSC

// Audio
//#define EXCLUDE_AUDIO

#if !defined(EXCLUDE_AUDIO)
//#define USE_EXT_I2S_DAC
#endif /* EXCLUDE_AUDIO */

#define SOC_GPIO_PIN_PDM_OUT  17

/* Waveshare Pico-Audio, PCM5101A I2S DAC */
//#define SOC_GPIO_PIN_DATA     8
//#define SOC_GPIO_PIN_BCK      9
//#define SOC_GPIO_PIN_LRCK     10
//#define SOC_GPIO_PIN_MCK      I2S_PIN_NO_CHANGE

/* Waveshare Pico-Audio Rev2.1, CS4344 I2S DAC */
#define SOC_GPIO_PIN_DATA     7
#define SOC_GPIO_PIN_BCK      10
#define SOC_GPIO_PIN_LRCK     9
#define SOC_GPIO_PIN_MCK      8

#elif defined(CONFIG_IDF_TARGET_ESP32P4)
#define SOC_GPIO_PIN_GNSS_RX  5
#define SOC_GPIO_PIN_GNSS_TX  4

#define SOC_BUTTON_MODE_DEF   35 /* BOOT, active LOW, strapping pin */

#define SOC_GPIO_PIN_MOSI_WS  32
#define SOC_GPIO_PIN_MISO_WS  33
#define SOC_GPIO_PIN_SCK_WS   36 /* strapping pin */
#define SOC_GPIO_PIN_SS_WS    26

#define SOC_EPD_PIN_DC_WS     47
#define SOC_EPD_PIN_RST_WS    48
#define SOC_EPD_PIN_BUSY_WS   27

#define SOC_GPIO_PIN_SDA      7
#define SOC_GPIO_PIN_SCL      8

// USB
#define SOC_GPIO_PIN_USB_DP   25
#define SOC_GPIO_PIN_USB_DN   24

//#define USE_ADAFRUIT_MSC

#undef EXCLUDE_ETHERNET

// Audio
//#define EXCLUDE_AUDIO

#if !defined(EXCLUDE_AUDIO)
#define USE_EXT_I2S_DAC
#endif /* EXCLUDE_AUDIO */

/* I2S ES8311 + MIC */
#define SOC_GPIO_PIN_DATA     11
#define SOC_GPIO_PIN_BCK      12
#define SOC_GPIO_PIN_LRCK     10
#define SOC_GPIO_PIN_MCK      13

// SDIO 1 - SDMMC
#define SOC_GPIO_PIN_SD_CLK   43
#define SOC_GPIO_PIN_SD_CMD   44
#define SOC_GPIO_PIN_SD_D0    39
#define SOC_GPIO_PIN_SD_D1    40
#define SOC_GPIO_PIN_SD_D2    41
#define SOC_GPIO_PIN_SD_D3    42
#define SOC_GPIO_PIN_SD_DET   45
#define SOC_GPIO_PIN_SD_PWR   46 /* NC ? */

// SDIO 2 - WIFI - ESP32-C5
#define SOC_GPIO_PIN_ESPH_CLK 18
#define SOC_GPIO_PIN_ESPH_CMD 19
#define SOC_GPIO_PIN_ESPH_D0  14
#define SOC_GPIO_PIN_ESPH_D1  15
#define SOC_GPIO_PIN_ESPH_D2  16
#define SOC_GPIO_PIN_ESPH_D3  17
#define SOC_GPIO_PIN_ESPH_RST 54 /* C5 EN */
#define SOC_GPIO_PIN_ESPH_WKP 6  /* C5 WAKEUP */

// Ethernet
#define ETH_PHY_TYPE          ETH_PHY_IP101
#define SOC_GPIO_PIN_ETH_MDC  31
#define SOC_GPIO_PIN_ETH_MDIO 52
#define SOC_GPIO_PIN_ETH_PWR  51 /* PHY_RSTN */

#elif defined(CONFIG_IDF_TARGET_ESP32C3) || \
      defined(CONFIG_IDF_TARGET_ESP32C5) || \
      defined(CONFIG_IDF_TARGET_ESP32C6)
#define SOC_GPIO_PIN_GNSS_RX  10  /* D4 */
#define SOC_GPIO_PIN_GNSS_TX  7

#define SOC_BUTTON_MODE_DEF   9  /* D3 */

/* Waveshare ESP32-C3 SPI pins mapping */
#define SOC_GPIO_PIN_MOSI_WS  5  /* D7 */
#define SOC_GPIO_PIN_MISO_WS  4  /* D6 */
#define SOC_GPIO_PIN_SCK_WS   3  /* D5 */
#define SOC_GPIO_PIN_SS_WS    8  /* D8 */

/* Waveshare ESP32-C3 EPD pins mapping */
#define SOC_EPD_PIN_DC_WS     18 /* D2 */
#define SOC_EPD_PIN_RST_WS    19 /* D1 */
#define SOC_EPD_PIN_BUSY_WS   2  /* D0 */

// USB CDC/JTAG
#define SOC_GPIO_PIN_USB_DP   19 /* D1 */
#define SOC_GPIO_PIN_USB_DN   18 /* D2 */

#if defined(CONFIG_IDF_TARGET_ESP32C5) || defined(CONFIG_IDF_TARGET_ESP32C6)
#define USE_NIMBLE
#endif
#define EXCLUDE_AUDIO

#else
#error "This ESP32 family build variant is not supported!"
#endif /* CONFIG_IDF_TARGET_ESP32 */

/* Boya Microelectronics Inc. */
#define BOYA_ID               0x68
#define BOYA_BY25Q32AL        0x4016

/* ST / SGS/Thomson / Numonyx / XMC(later acquired by Micron) */
#define ST_ID                 0x20
#define XMC_XM25QH128C        0x4018
#define XMC_XM25QH32B         0x4016

#define MakeFlashId(v,d)      ((v  << 16) | d)
#define CCCC(c1, c2, c3, c4)  ((c4 << 24) | (c3 << 16) | (c2 << 8) | c1)

#define MAX_FILENAME_LEN      64
#define WAV_FILE_PREFIX       "/Audio/"

#define POWER_SAVING_WIFI_TIMEOUT 600000UL /* 10 minutes */

/* these are data structures to process wav file */
typedef enum headerState_e {
    HEADER_RIFF, HEADER_FMT, HEADER_DATA, DATA
} headerState_t;

typedef struct wavRiff_s {
    uint32_t chunkID;
    uint32_t chunkSize;
    uint32_t format;
} wavRiff_t;

typedef struct wavProperties_s {
    uint32_t chunkID;
    uint32_t chunkSize;
    uint16_t audioFormat;
    uint16_t numChannels;
    uint32_t sampleRate;
    uint32_t byteRate;
    uint16_t blockAlign;
    uint16_t bitsPerSample;
} wavProperties_t;

extern bool loopTaskWDTEnabled;

extern WebServer server;

//#define BUILD_SKYVIEW_HD

#endif /* PLATFORM_ESP32_H */

#endif /* ESP32 */
