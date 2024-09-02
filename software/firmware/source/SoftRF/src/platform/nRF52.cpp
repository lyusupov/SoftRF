/*
 * Platform_nRF52.cpp
 * Copyright (C) 2020-2024 Linar Yusupov
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

#if defined(ARDUINO_ARCH_NRF52) || defined(ARDUINO_ARCH_NRF52840)

#include <SPI.h>
#include <Wire.h>
#if !defined(ARDUINO_ARCH_MBED)
#include <pcf8563.h>
#include <Adafruit_SPIFlash.h>
#include "Adafruit_TinyUSB.h"
#include <Adafruit_SleepyDog.h>
#endif /* ARDUINO_ARCH_MBED */
#include <ArduinoJson.h>
#include "nrf_wdt.h"

#include "../system/SoC.h"
#include "../driver/RF.h"
#include "../driver/EEPROM.h"
#include "../driver/GNSS.h"
#include "../driver/Baro.h"
#include "../driver/LED.h"
#include "../driver/Bluetooth.h"
#include "../driver/EPD.h"
#include "../driver/Battery.h"
#include "../driver/Sound.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/GDL90.h"
#include "../protocol/data/D1090.h"
#include "../protocol/data/JSON.h"
#include "../system/Time.h"

#include "uCDB.hpp"

#if defined(USE_BLE_MIDI)
#include <bluefruit.h>
#endif /* USE_BLE_MIDI */

#if defined(USE_BLE_MIDI) || defined(USE_USB_MIDI)
#include <MIDI.h>
#endif /* USE_BLE_MIDI || USE_USB_MIDI */

#if defined(ENABLE_REMOTE_ID)
#include "../protocol/radio/RemoteID.h"
#endif /* ENABLE_REMOTE_ID */

#if defined(USE_TFT)
#include <TFT_eSPI.h>
#endif /* USE_TFT */

typedef volatile uint32_t REG32;
#define pREG32 (REG32 *)

#define DEVICE_ID_HIGH    (*(pREG32 (0x10000060)))
#define DEVICE_ID_LOW     (*(pREG32 (0x10000064)))

#define EPD_STACK_SZ      (256*3)

// RFM95W pin mapping
lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .txe = LMIC_UNUSED_PIN,
    .rxe = LMIC_UNUSED_PIN,
    .rst = SOC_GPIO_PIN_PCA10059_RST,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
    .busy = SOC_GPIO_PIN_BUSY,
    .tcxo = LMIC_UNUSED_PIN,
};

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIX_NUM, SOC_GPIO_PIN_LED,
                              NEO_GRB + NEO_KHZ800);

#if defined(EXCLUDE_WIFI)
char UDPpacketBuffer[4]; // Dummy definition to satisfy build sequence
#else
#if defined(USING_WIFI101_GENERIC) || defined(WiFiNINA_h)
#include <SoftSPI.h>
SoftSPI WiFiSPI(_PINNUM(1, 7), _PINNUM(1, 6), _PINNUM(0, 8));
#endif /* USING_WIFI101_GENERIC || WiFiNINA_h */
#endif /* EXCLUDE_WIFI */

static uint32_t prev_tx_packets_counter = 0;
static uint32_t prev_rx_packets_counter = 0;

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

static uint32_t bootCount __attribute__ ((section (".noinit")));

static nRF52_board_id nRF52_board = NRF52_LILYGO_TECHO_REV_2; /* default */
static nRF52_display_id nRF52_display = EP_UNKNOWN;

const char *nRF52_Device_Manufacturer = SOFTRF_IDENT;
const char *nRF52_Device_Model = "Badge Edition";
const uint16_t nRF52_Device_Version = SOFTRF_USB_FW_VERSION;
static uint16_t nRF52_USB_VID = 0x239A; /* Adafruit Industries */
static uint16_t nRF52_USB_PID = 0x8029; /* Feather nRF52840 Express */

const char *Hardware_Rev[] = {
  [0] = "2020-8-6",
  [1] = "2020-12-12",
  [2] = "2021-3-26",
  [3] = "Unknown"
};

const prototype_entry_t techo_prototype_boards[] = {
  { 0x684f99bd2d5c7fae, NRF52_LILYGO_TECHO_REV_0, EP_GDEP015OC1,  0 }, /* orange */
  { 0xf353e11726ea8220, NRF52_LILYGO_TECHO_REV_0, EP_GDEH0154D67, 0 }, /* blue   */
  { 0xf4e0f04ded1892da, NRF52_LILYGO_TECHO_REV_1, EP_GDEH0154D67, 0 }, /* green  */
  { 0x65ab5994ea2c9094, NRF52_LILYGO_TECHO_REV_1, EP_GDEH0154D67, 0 }, /* blue   */
  { 0x6460429ea6fb7e39, NRF52_NORDIC_PCA10059,    EP_UNKNOWN,     0 },
};

#if !defined(ARDUINO_ARCH_MBED)
PCF8563_Class *rtc = nullptr;
I2CBus        *i2c = nullptr;
#endif /* ARDUINO_ARCH_MBED */

static bool nRF52_has_rtc      = false;
static bool nRF52_has_spiflash = false;
static bool RTC_sync           = false;
static bool FATFS_is_mounted   = false;
static bool ADB_is_open        = false;
static bool screen_saver       = false;

#if !defined(ARDUINO_ARCH_MBED)
RTC_Date fw_build_date_time = RTC_Date(__DATE__, __TIME__);

static TaskHandle_t EPD_Task_Handle = NULL;
#else
#define ledOn(x)  {} /* TBD */
#define ledOff(x) {} /* TBD */

#define PIN_LED1  0 /* TBD */
#define PIN_LED2  0 /* TBD */
#define PIN_LED3  0 /* TBD */
#define PIN_LED4  0 /* TBD */
#endif /* ARDUINO_ARCH_MBED */

#if !defined(ARDUINO_NRF52840_PCA10056) && !defined(ARDUINO_ARCH_MBED)
#error "This nRF52 build variant is not supported!"
#endif

#if SPI_32MHZ_INTERFACE == 0
#define _SPI_DEV    NRF_SPIM3 // 32 Mhz
#define _SPI1_DEV   NRF_SPIM2
#elif SPI_32MHZ_INTERFACE == 1
#define _SPI_DEV    NRF_SPIM2
#define _SPI1_DEV   NRF_SPIM3 // 32 Mhz
#else
  #error "not supported yet"
#endif

#if defined(USE_EPAPER)

#if SPI_INTERFACES_COUNT == 1
SPIClass SPI1(_SPI1_DEV,
              SOC_GPIO_PIN_EPD_MISO,
              SOC_GPIO_PIN_EPD_SCK,
              SOC_GPIO_PIN_EPD_MOSI);
#endif

GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> epd_d67(GxEPD2_154_D67(
                                                          SOC_GPIO_PIN_EPD_SS,
                                                          SOC_GPIO_PIN_EPD_DC,
                                                          SOC_GPIO_PIN_EPD_RST,
                                                          SOC_GPIO_PIN_EPD_BUSY));
GxEPD2_BW<GxEPD2_154, GxEPD2_154::HEIGHT>         epd_c1 (GxEPD2_154(
                                                          SOC_GPIO_PIN_EPD_SS,
                                                          SOC_GPIO_PIN_EPD_DC,
                                                          SOC_GPIO_PIN_EPD_RST,
                                                          SOC_GPIO_PIN_EPD_BUSY));
GxEPD2_BW<GxEPD2_150_BN, GxEPD2_150_BN::HEIGHT>   epd_bn (GxEPD2_150_BN(
                                                          SOC_GPIO_PIN_EPD_SS,
                                                          SOC_GPIO_PIN_EPD_DC,
                                                          SOC_GPIO_PIN_EPD_RST,
                                                          SOC_GPIO_PIN_EPD_BUSY));
GxEPD2_BW<GxEPD2_371_T03, GxEPD2_371_T03::HEIGHT> epd_t3 (GxEPD2_371_T03(
                                                          SOC_GPIO_PIN_EPD_TULTIMA_SS,
                                                          SOC_GPIO_PIN_EPD_TULTIMA_DC,
                                                          SOC_GPIO_PIN_EPD_TULTIMA_RST,
                                                          SOC_GPIO_PIN_EPD_TULTIMA_BUSY));
GxEPD2_GFX *display;
#endif /* USE_EPAPER */

#if defined(USE_TFT)
static TFT_eSPI *tft = NULL;

static const char SoftRF_text1[]  = "SoftRF";
static const char ID_text[]       = "ID";
static const char PROTOCOL_text[] = "PROTOCOL";
static const char RX_text[]       = "RX";
static const char TX_text[]       = "TX";

static bool TFT_display_frontpage = false;

static void TFT_off()
{
    tft->writecommand(TFT_DISPOFF);
    tft->writecommand(TFT_SLPIN);
}

static void TFT_backlight_adjust(uint32_t pin, uint8_t level)
{
    analogWrite(pin, level);
}
#endif /* USE_TFT */

#if !defined(ARDUINO_ARCH_MBED)
static Adafruit_SPIFlash *SPIFlash = NULL;

#define SFLASH_CMD_READ_CONFIG  0x15

static uint32_t spiflash_id = 0;
static uint8_t mx25_status_config[3] = {0x00, 0x00, 0x00};

/// Flash device list count
enum {
  MX25R1635F_INDEX,
  ZD25WQ16B_INDEX,
  W25Q128JV_INDEX,
  EXTERNAL_FLASH_DEVICE_COUNT
};

/// List of all possible flash devices used by nRF52840 boards
static SPIFlash_Device_t possible_devices[] = {
  // LilyGO T-Echo
  [MX25R1635F_INDEX] = MX25R1635F,
  [ZD25WQ16B_INDEX]  = ZD25WQ16B,
  // LilyGO T-Ultima
  [W25Q128JV_INDEX]  = W25Q128JV_PM,
};

// USB Mass Storage object
Adafruit_USBD_MSC usb_msc;

// file system object from SdFat
FatVolume fatfs;
#endif /* ARDUINO_ARCH_MBED */

#define NRF52_JSON_BUFFER_SIZE  1024

StaticJsonBuffer<NRF52_JSON_BUFFER_SIZE> nRF52_jsonBuffer;

#if defined(USE_WEBUSB_SERIAL) || defined(USE_WEBUSB_SETTINGS)
// USB WebUSB object
Adafruit_USBD_WebUSB usb_web;
#endif

#if defined(USE_WEBUSB_SERIAL)
// Landing Page: scheme (0: http, 1: https), url
WEBUSB_URL_DEF(landingPage, 1 /*https*/, "adafruit.github.io/Adafruit_TinyUSB_Arduino/examples/webusb-serial/index.html");
#endif /* USE_WEBUSB_SERIAL */

#if defined(USE_WEBUSB_SETTINGS)
// Landing Page: scheme (0: http, 1: https), url
WEBUSB_URL_DEF(landingPage, 1 /*https*/, "lyusupov.github.io/SoftRF/settings.html");
#endif /* USE_WEBUSB_SETTINGS */

#if defined(USE_USB_MIDI)
// USB MIDI object
Adafruit_USBD_MIDI usb_midi;

// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI_USB);
#endif /* USE_USB_MIDI */

ui_settings_t ui_settings = {
#if !defined(ARDUINO_ARCH_MBED)
    .units        = UNITS_METRIC,
    .zoom         = ZOOM_MEDIUM,
    .protocol     = PROTOCOL_NMEA,
    .rotate       = ROTATE_0,
    .orientation  = DIRECTION_TRACK_UP,
    .adb          = DB_OGN,
    .idpref       = ID_TYPE,
    .vmode        = VIEW_MODE_STATUS,
    .voice        = VOICE_OFF,
    .aghost       = ANTI_GHOSTING_OFF,
    .filter       = TRAFFIC_FILTER_OFF,
    .team         = 0
#endif /* ARDUINO_ARCH_MBED */
};

ui_settings_t *ui;

#if !defined(EXCLUDE_IMU)
#define IMU_UPDATE_INTERVAL 500 /* ms */

#include <MPU9250.h>
#include <ICM_20948.h>
MPU9250       imu_1;
ICM_20948_I2C imu_2;

static bool nRF52_has_imu = false;
static unsigned long IMU_Time_Marker = 0;

extern float IMU_g;
#endif /* EXCLUDE_IMU */

#if !defined(ARDUINO_ARCH_MBED)
uCDB<FatVolume, File32> ucdb(fatfs);

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
static int32_t nRF52_msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return SPIFlash->readBlocks(lba, (uint8_t*) buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
static int32_t nRF52_msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize)
{
  ledOn(SOC_GPIO_LED_USBMSC);

  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return SPIFlash->writeBlocks(lba, buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
static void nRF52_msc_flush_cb (void)
{
  // sync with flash
  SPIFlash->syncBlocks();

  // clear file system's cache to force refresh
  fatfs.cacheClear();

  ledOff(SOC_GPIO_LED_USBMSC);
}
#endif /* ARDUINO_ARCH_MBED */

#if defined(USE_EXT_I2S_DAC)

#define CCCC(c1, c2, c3, c4)  ((c4 << 24) | (c3 << 16) | (c2 << 8) | c1)

#define MAX_FILENAME_LEN      64
#define WAV_FILE_PREFIX       "/Audio/"
#define WAV_FILE_SUFFIX       ".wav"

#define I2S_DATA_BLOCK_WORDS  8192

/* these are data structures to process wav file */
typedef enum headerState_e {
    HEADER_RIFF,
    HEADER_FMT,
    HEADER_DATA,
    DATA
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

static uint32_t i2s_buffer[I2S_DATA_BLOCK_WORDS];

void I2S_begin(uint8_t pinSDOUT, uint8_t pinSCK, uint8_t pinLRCK, int8_t pinMCK) {
  // Enable transmission
  NRF_I2S->CONFIG.TXEN     = (I2S_CONFIG_TXEN_TXEN_ENABLE << I2S_CONFIG_TXEN_TXEN_Pos);
  // Enable MCK generator
  // NRF_I2S->CONFIG.MCKEN    = (I2S_CONFIG_MCKEN_MCKEN_ENABLE << I2S_CONFIG_MCKEN_MCKEN_Pos);
  // NRF_I2S->CONFIG.MCKFREQ  = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV11 << I2S_CONFIG_MCKFREQ_MCKFREQ_Pos;
  // NRF_I2S->CONFIG.RATIO    = I2S_CONFIG_RATIO_RATIO_64X << I2S_CONFIG_RATIO_RATIO_Pos;
  // Master mode, 16Bit, left aligned
  NRF_I2S->CONFIG.MODE     = I2S_CONFIG_MODE_MODE_MASTER << I2S_CONFIG_MODE_MODE_Pos;
  NRF_I2S->CONFIG.SWIDTH   = I2S_CONFIG_SWIDTH_SWIDTH_16BIT << I2S_CONFIG_SWIDTH_SWIDTH_Pos;
  NRF_I2S->CONFIG.ALIGN    = I2S_CONFIG_ALIGN_ALIGN_LEFT << I2S_CONFIG_ALIGN_ALIGN_Pos;
  // Format = I2S
  NRF_I2S->CONFIG.FORMAT   = I2S_CONFIG_FORMAT_FORMAT_I2S << I2S_CONFIG_FORMAT_FORMAT_Pos;
  // Use left
  NRF_I2S->CONFIG.CHANNELS = I2S_CONFIG_CHANNELS_CHANNELS_LEFT << I2S_CONFIG_CHANNELS_CHANNELS_Pos;

  // Configure pins
  NRF_I2S->PSEL.SCK   = (pinSCK   << I2S_PSEL_SCK_PIN_Pos);
  NRF_I2S->PSEL.LRCK  = (pinLRCK  << I2S_PSEL_LRCK_PIN_Pos);
  NRF_I2S->PSEL.SDOUT = (pinSDOUT << I2S_PSEL_SDOUT_PIN_Pos);
  // NRF_I2S->PSEL.MCK   = (pinMCK   << I2S_PSEL_MCK_PIN_Pos);
}

void I2S_stop()
{
  // Erratta 55 workaround (part 1)
  volatile uint32_t tmp = NRF_I2S->INTEN;
  NRF_I2S->INTEN = 0;

  NRF_I2S->TASKS_STOP = 1;

  // Errata 194 workaround
  *((volatile uint32_t *)0x40025038) = 1;
  *((volatile uint32_t *)0x4002503C) = 1;
  while (NRF_I2S->EVENTS_STOPPED == 0);
  NRF_I2S->EVENTS_STOPPED = 0;
  (void)NRF_I2S->EVENTS_STOPPED;

  // Errata 55 workaround (part 2)
  NRF_I2S->EVENTS_RXPTRUPD = 0;
  NRF_I2S->EVENTS_TXPTRUPD = 0;
  NRF_I2S->EVENTS_STOPPED  = 0;
  NRF_I2S->INTEN = tmp;
}

/*
 *  @brief Mapping Frequency constants to available frequencies
 */
struct I2S_freq_info {
  int id;
  float freq; // in mhz
};

static const I2S_freq_info freq_table[] = {
  { I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV8,   32.0 /   8 },
  { I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV10,  32.0 /  10 },
  { I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV11,  32.0 /  11 },
  { I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV15,  32.0 /  15 },
  { I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV16,  32.0 /  16 },
  { I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV21,  32.0 /  21 },
  { I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV23,  32.0 /  23 },
  { I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV30,  32.0 /  30 },
  { I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV31,  32.0 /  31 },
  { I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV32,  32.0 /  32 },
  { I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV42,  32.0 /  42 },
  { I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV63,  32.0 /  63 },
  { I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV125, 32.0 / 125 }
};

/*
 *  @brief Mapping from Ratio Constants to frequency ratios
 */
struct I2S_ratio_info {
  int id;
  float ratio;
};

static const I2S_ratio_info ratio_table[] = {
  { I2S_CONFIG_RATIO_RATIO_32X,   32.0 },
  { I2S_CONFIG_RATIO_RATIO_48X,   48.0 },
  { I2S_CONFIG_RATIO_RATIO_64X,   64.0 },
  { I2S_CONFIG_RATIO_RATIO_96X,   96.0 },
  { I2S_CONFIG_RATIO_RATIO_128X, 128.0 },
  { I2S_CONFIG_RATIO_RATIO_192X, 192.0 },
  { I2S_CONFIG_RATIO_RATIO_256X, 256.0 },
  { I2S_CONFIG_RATIO_RATIO_384X, 384.0 },
  { I2S_CONFIG_RATIO_RATIO_512X, 512.0 }
};

void I2S_setSampleRate(uint16_t sampleRate)
{
  // find closest frequency for requested sample_rate
  float freq_requested = sampleRate;
  float selected_freq = 0;
  for (auto freq : freq_table) {
    for (auto div : ratio_table) {
        float freq_value = freq.freq * 1000000 / div.ratio;
        if (abs(freq_value-freq_requested) < abs(selected_freq-freq_requested)){
          // MCKFREQ
          NRF_I2S->CONFIG.MCKFREQ = freq.id << I2S_CONFIG_MCKFREQ_MCKFREQ_Pos;
          // Ratio
          NRF_I2S->CONFIG.RATIO   = div.id  << I2S_CONFIG_RATIO_RATIO_Pos;
          selected_freq = freq_value;
          // Serial.printf("frequency requested %f vs %f\r\n", freq_requested, selected_freq);
        }
     }
  }
  // Serial.printf("Frequency req. %f vs eff. %f\r\n", freq_requested, selected_freq);
}

wavProperties_t wavProps;

static bool play_file(char *filename)
{
  bool rval = false;

#if !defined(EXCLUDE_AUDIO)
  headerState_t state = HEADER_RIFF;

  File wavfile = fatfs.open(filename, FILE_READ);

  if (wavfile) {
    int c = 0;
    int n;

    while (wavfile.available()) {
      switch(state){
        case HEADER_RIFF:
        {
          wavRiff_t wavRiff;

          n = wavfile.read((uint8_t *) &wavRiff, sizeof(wavRiff_t));
          if (n == sizeof(wavRiff_t)) {
            if (wavRiff.chunkID == CCCC('R', 'I', 'F', 'F') &&
                wavRiff.format  == CCCC('W', 'A', 'V', 'E')) {
              state = HEADER_FMT;
              // Serial.println("HEADER_RIFF");
            }
          }
        }
        break;
        case HEADER_FMT:
        {
          n = wavfile.read((uint8_t *) &wavProps, sizeof(wavProperties_t));
          if (n == sizeof(wavProperties_t)) {
            state = HEADER_DATA;
          }
        }
        break;
        case HEADER_DATA:
        {
          uint32_t chunkId, chunkSize;
          n = wavfile.read((uint8_t *) &chunkId, sizeof(chunkId));
          if (n == 4) {
            if(chunkId == CCCC('d', 'a', 't', 'a')) {
              // Serial.println("HEADER_DATA");
            }
          }
          n = wavfile.read((uint8_t *) &chunkSize, sizeof(chunkSize));
          if (n == 4) {
            state = DATA;
          }

          I2S_begin(SOC_GPIO_PIN_I2S_TULTIMA_DOUT,
                    SOC_GPIO_PIN_I2S_TULTIMA_BCK,
                    SOC_GPIO_PIN_I2S_TULTIMA_LRCK,
                    SOC_GPIO_PIN_I2S_TULTIMA_MCK);
          I2S_setSampleRate(wavProps.sampleRate);
        }
        break;
        /* after processing wav header, it is time to process music data */
        case DATA:
        while ((n = wavfile.read((uint8_t *) i2s_buffer, sizeof(i2s_buffer))) > 0) {

          NRF_I2S->RXTXD.MAXCNT = n >> 2;
          NRF_I2S->TXD.PTR =  (uint32_t) i2s_buffer;
          NRF_I2S->ENABLE  = 1;
          NRF_I2S->TASKS_START = 1;

          NRF_I2S->EVENTS_RXPTRUPD = 0;
          NRF_I2S->EVENTS_TXPTRUPD = 0;
          NRF_I2S->EVENTS_STOPPED  = 0;

          while (NRF_I2S->EVENTS_TXPTRUPD == 0);
        }

        if (n == 0) {
          NRF_I2S->TASKS_START = 0;
          NRF_I2S->ENABLE = 0;
        }
        break;
      }
    }
    wavfile.close();
    rval = true;
  } else {
    Serial.println(F("error opening WAV file"));
  }
  if (state == DATA) {
    I2S_stop();
  }
#endif /* EXCLUDE_AUDIO */

  return rval;
}
#endif /* USE_EXT_I2S_DAC */

#if !defined(EXCLUDE_PMU)
#define  POWERS_CHIP_SY6970
#define  SDA    SOC_GPIO_PIN_SDA
#define  SCL    SOC_GPIO_PIN_SCL
#include <XPowersLib.h>

PowersSY6970 sy6970;

static bool nRF52_has_pmu = false;
#endif /* EXCLUDE_PMU */

#if defined(ENABLE_RECORDER)
#define SD_CONFIG SdSpiConfig(uSD_SS_pin, SHARED_SPI, SD_SCK_MHZ(8), &SPI)

SdFat uSD;

static bool uSD_is_attached = false;
#endif /* ENABLE_RECORDER */

static void nRF52_setup()
{
  ui = &ui_settings;

#if !defined(ARDUINO_ARCH_MBED)
  uint32_t reset_reason = readResetReason();
#else
  uint32_t reset_reason = 0; /* TBD */
#endif /* ARDUINO_ARCH_MBED */

  if      (reset_reason & POWER_RESETREAS_RESETPIN_Msk)
  {
      reset_info.reason = REASON_EXT_SYS_RST;
  }
  else if (reset_reason & POWER_RESETREAS_DOG_Msk)
  {
      reset_info.reason = REASON_WDT_RST;
  }
  else if (reset_reason & POWER_RESETREAS_SREQ_Msk)
  {
      reset_info.reason = REASON_SOFT_RESTART;
  }
  else if (reset_reason & POWER_RESETREAS_LOCKUP_Msk)
  {
      reset_info.reason = REASON_SOFT_WDT_RST;
  }
  else if (reset_reason & POWER_RESETREAS_OFF_Msk)
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
  }
  else if (reset_reason & POWER_RESETREAS_LPCOMP_Msk)
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
  }
  else if (reset_reason & POWER_RESETREAS_DIF_Msk)
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
  }
  else if (reset_reason & POWER_RESETREAS_NFC_Msk)
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
  }
  else if (reset_reason & POWER_RESETREAS_VBUS_Msk)
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
  }

  /* inactivate initVariant() of PCA10056 */
  pinMode(PIN_LED1, INPUT);
  pinMode(PIN_LED2, INPUT);
  pinMode(PIN_LED3, INPUT);
  pinMode(PIN_LED4, INPUT);

  for (int i=0; i < sizeof(techo_prototype_boards) / sizeof(prototype_entry_t); i++) {
    if (techo_prototype_boards[i].id == ((uint64_t) DEVICE_ID_HIGH << 32 | (uint64_t) DEVICE_ID_LOW)) {
      nRF52_board   = techo_prototype_boards[i].rev;
      nRF52_display = techo_prototype_boards[i].panel;
      break;
    }
  }

#if !defined(EXCLUDE_PMU)
  nRF52_has_pmu = sy6970.init(Wire,
                              SOC_GPIO_PIN_TULTIMA_SDA,
                              SOC_GPIO_PIN_TULTIMA_SCL,
                              SY6970_SLAVE_ADDRESS);
  if (nRF52_has_pmu) {
    nRF52_board        = NRF52_LILYGO_TULTIMA;
    hw_info.model      = SOFTRF_MODEL_NEO;
    hw_info.pmu        = PMU_SY6970;
    nRF52_Device_Model = "Neo Edition";

    // Set the minimum operating voltage. Below this voltage, the PMU will protect
    sy6970.setSysPowerDownVoltage(3300);

    // Set input current limit, default is 500mA
    sy6970.setInputCurrentLimit(3250);

    //Serial.printf("getInputCurrentLimit: %d mA\n",sy6970.getInputCurrentLimit());

    // Disable current limit pin
    sy6970.disableCurrentLimitPin();

    // Set the charging target voltage, Range:3840 ~ 4608mV ,step:16 mV
    sy6970.setChargeTargetVoltage(4208);

    // Set the precharge current , Range: 64mA ~ 1024mA ,step:64mA
    sy6970.setPrechargeCurr(64);

    // The premise is that Limit Pin is disabled, or it will only follow the maximum charging current set by Limi tPin.
    // Set the charging current , Range:0~5056mA ,step:64mA
    sy6970.setChargerConstantCurr(832);

    // Get the set charging current
    sy6970.getChargerConstantCurr();
    //Serial.printf("getChargerConstantCurr: %d mA\n",sy6970.getChargerConstantCurr());


    // To obtain voltage data, the ADC must be enabled first
    sy6970.enableADCMeasure();

    // Turn on charging function
    // If there is no battery connected, do not turn on the charging function
    sy6970.enableCharge();

    // Turn off charging function
    // If USB is used as the only power input, it is best to turn off the charging function,
    // otherwise the VSYS power supply will have a sawtooth wave, affecting the discharge output capability.
    // sy6970.disableCharge();


    // The OTG function needs to enable OTG, and set the OTG control pin to HIGH
    // After OTG is enabled, if an external power supply is plugged in, OTG will be turned off

    // sy6970.enableOTG();
    // sy6970.disableOTG();
    // pinMode(OTG_ENABLE_PIN, OUTPUT);
    // digitalWrite(OTG_ENABLE_PIN, HIGH);
  } else {
    Wire.end();

#if !defined(EXCLUDE_IMU)
    pinMode(SOC_GPIO_PIN_T1000_ACC_EN, INPUT_PULLUP);
    delay(200);

#if !defined(ARDUINO_ARCH_MBED)
    Wire.setPins(SOC_GPIO_PIN_T1000_SDA, SOC_GPIO_PIN_T1000_SCL);
#endif /* ARDUINO_ARCH_MBED */
    Wire.begin();
    Wire.beginTransmission(QMA6100P_ADDRESS);
    nRF52_has_imu = (Wire.endTransmission() == 0);
    if (nRF52_has_imu) {
      nRF52_board        = NRF52_SEEED_T1000E;
      hw_info.model      = SOFTRF_MODEL_CARD;
      hw_info.imu        = ACC_QMA6100P;
      nRF52_Device_Model = "Card Edition";
      nRF52_USB_VID      = 0x2886; /* Seeed Technology */
      nRF52_USB_PID      = 0x0057; /* SenseCAP T1000-E */
    }
    Wire.end();
    pinMode(SOC_GPIO_PIN_T1000_ACC_EN, INPUT);
#endif /* EXCLUDE_IMU */
  }
#endif /* EXCLUDE_PMU */

#if !defined(ARDUINO_ARCH_MBED)
  switch (nRF52_board)
  {
    case NRF52_LILYGO_TULTIMA:
      Wire.setPins(SOC_GPIO_PIN_TULTIMA_SDA, SOC_GPIO_PIN_TULTIMA_SCL);
      break;
    case NRF52_SEEED_T1000E:
      Wire.setPins(SOC_GPIO_PIN_T1000_SDA, SOC_GPIO_PIN_T1000_SCL);
      break;
    case NRF52_LILYGO_TECHO_REV_0:
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_NORDIC_PCA10059:
    case NRF52_HELTEC_T114:
    default:
      Wire.setPins(SOC_GPIO_PIN_SDA, SOC_GPIO_PIN_SCL);
      break;
  }
#endif /* ARDUINO_ARCH_MBED */
  Wire.begin();

  Wire.beginTransmission(PCF8563_SLAVE_ADDRESS);
  nRF52_has_rtc = (Wire.endTransmission() == 0);
  if (!nRF52_has_rtc) {
    delay(200);
    Wire.beginTransmission(PCF8563_SLAVE_ADDRESS);
    nRF52_has_rtc = (Wire.endTransmission() == 0);
    if (!nRF52_has_rtc) {
      delay(200);
      Wire.beginTransmission(PCF8563_SLAVE_ADDRESS);
      nRF52_has_rtc = (Wire.endTransmission() == 0);
    }
  }

#if !defined(EXCLUDE_IMU)
  if (nRF52_has_imu == false) {
    Wire.beginTransmission(MPU9250_ADDRESS);
    nRF52_has_imu = (Wire.endTransmission() == 0);
    if (nRF52_has_imu == false) {
      Wire.beginTransmission(ICM20948_ADDRESS);
      nRF52_has_imu = (Wire.endTransmission() == 0);
    }
  }
#endif /* EXCLUDE_IMU */

  Wire.end();

#if !defined(ARDUINO_ARCH_MBED)
  /* (Q)SPI flash init */
  Adafruit_FlashTransport_QSPI *ft = NULL;

  switch (nRF52_board)
  {
    case NRF52_LILYGO_TECHO_REV_0:
      possible_devices[MX25R1635F_INDEX].max_clock_speed_mhz  = 33;
      possible_devices[MX25R1635F_INDEX].supports_qspi        = false;
      possible_devices[MX25R1635F_INDEX].supports_qspi_writes = false;
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_HELTEC_T114:
      ft = new Adafruit_FlashTransport_QSPI(SOC_GPIO_PIN_SFL_SCK,
                                            SOC_GPIO_PIN_SFL_SS,
                                            SOC_GPIO_PIN_SFL_MOSI,
                                            SOC_GPIO_PIN_SFL_MISO,
                                            SOC_GPIO_PIN_SFL_WP,
                                            SOC_GPIO_PIN_SFL_HOLD);
      break;
    case NRF52_LILYGO_TULTIMA:
      ft = new Adafruit_FlashTransport_QSPI(SOC_GPIO_PIN_SFL_TULTIMA_SCK,
                                            SOC_GPIO_PIN_SFL_TULTIMA_SS,
                                            SOC_GPIO_PIN_SFL_TULTIMA_MOSI,
                                            SOC_GPIO_PIN_SFL_TULTIMA_MISO,
                                            SOC_GPIO_PIN_SFL_TULTIMA_WP,
                                            SOC_GPIO_PIN_SFL_TULTIMA_HOLD);
      break;
    case NRF52_NORDIC_PCA10059:
    case NRF52_SEEED_T1000E:
    default:
      break;
  }

  if (ft != NULL) {
    SPIFlash = new Adafruit_SPIFlash(ft);
    nRF52_has_spiflash = SPIFlash->begin(possible_devices,
                                         EXTERNAL_FLASH_DEVICE_COUNT);
  }
#endif /* ARDUINO_ARCH_MBED */

  hw_info.storage = nRF52_has_spiflash ? STORAGE_FLASH : STORAGE_NONE;

#if defined(USE_TFT)
  if (nRF52_board        == NRF52_LILYGO_TECHO_REV_2 /* default */ &&
      nRF52_has_spiflash == false                                  &&
#if !defined(EXCLUDE_IMU)
      nRF52_has_imu      == false                                  &&
#endif /* EXCLUDE_IMU */
      nRF52_has_rtc      == false) {
    nRF52_board        = NRF52_HELTEC_T114;
    hw_info.model      = SOFTRF_MODEL_COZY;
    nRF52_Device_Model = "Cozy Edition";
  }
#endif /* USE_TFT */

#if !defined(ARDUINO_ARCH_MBED)
  USBDevice.setID(nRF52_USB_VID, nRF52_USB_PID);
  USBDevice.setManufacturerDescriptor(nRF52_Device_Manufacturer);
  USBDevice.setProductDescriptor(nRF52_Device_Model);
  USBDevice.setDeviceVersion(nRF52_Device_Version);
#endif /* ARDUINO_ARCH_MBED */

#if defined(USE_TINYUSB)
  switch (nRF52_board)
  {
    case NRF52_LILYGO_TULTIMA:
      /* TBD */
      break;
    case NRF52_SEEED_T1000E:
      Serial1.setPins(SOC_GPIO_PIN_CONS_T1000_RX, SOC_GPIO_PIN_CONS_T1000_TX);
      break;
    case NRF52_HELTEC_T114:
      Serial1.setPins(SOC_GPIO_PIN_CONS_T114_RX, SOC_GPIO_PIN_CONS_T114_TX);
      break;
    case NRF52_LILYGO_TECHO_REV_0:
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_NORDIC_PCA10059:
    default:
      Serial1.setPins(SOC_GPIO_PIN_CONS_RX, SOC_GPIO_PIN_CONS_TX);
#if defined(EXCLUDE_WIFI)
      Serial1.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
#endif /* EXCLUDE_WIFI */
      break;
  }
#endif /* USE_TINYUSB */

  switch (nRF52_board)
  {
    case NRF52_LILYGO_TULTIMA:
      /* TBD */
      break;

    case NRF52_SEEED_T1000E:
      pinMode(SOC_GPIO_PIN_T1000_3V3_EN, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_T1000_3V3_EN, HIGH);

      pinMode(SOC_GPIO_PIN_T1000_BUZZER_EN, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_T1000_BUZZER_EN, HIGH);
      break;

    case NRF52_HELTEC_T114:
      digitalWrite(SOC_GPIO_PIN_T114_VEXT_EN, HIGH);
      pinMode(SOC_GPIO_PIN_T114_VEXT_EN, OUTPUT);

      digitalWrite(SOC_GPIO_PIN_T114_TFT_EN, LOW);
      pinMode(SOC_GPIO_PIN_T114_TFT_EN, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_T114_TFT_BLGT, LOW);
      pinMode(SOC_GPIO_PIN_T114_TFT_BLGT, OUTPUT);

      digitalWrite(SOC_GPIO_PIN_T114_ADC_EN, HIGH);
      pinMode(SOC_GPIO_PIN_T114_ADC_EN, OUTPUT);
      break;

    case NRF52_LILYGO_TECHO_REV_0:
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_NORDIC_PCA10059:
    default:
      digitalWrite(SOC_GPIO_PIN_IO_PWR,  HIGH);
      pinMode(SOC_GPIO_PIN_IO_PWR,  OUTPUT);  /* VDD_POWR is ON */
      digitalWrite(SOC_GPIO_PIN_3V3_PWR, INPUT);

      delay(200);
      break;
  }

  /* GPIO pins init */
  switch (nRF52_board)
  {
    case NRF52_LILYGO_TECHO_REV_0:
      /* Wake up Air530 GNSS */
      digitalWrite(SOC_GPIO_PIN_GNSS_WKE, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_WKE, OUTPUT);

      pinMode(SOC_GPIO_LED_TECHO_REV_0_GREEN, OUTPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_0_RED,   OUTPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_0_BLUE,  OUTPUT);

      ledOn (SOC_GPIO_LED_TECHO_REV_0_GREEN);
      ledOff(SOC_GPIO_LED_TECHO_REV_0_RED);
      ledOff(SOC_GPIO_LED_TECHO_REV_0_BLUE);

      lmic_pins.rst = SOC_GPIO_PIN_TECHO_REV_0_RST;
#if defined(USE_RADIOLIB)
      lmic_pins.dio[0] = SOC_GPIO_PIN_DIO1;
#endif /* USE_RADIOLIB */

      hw_info.revision = 0;
      break;

    case NRF52_LILYGO_TECHO_REV_1:
      /* Wake up Air530 GNSS */
      digitalWrite(SOC_GPIO_PIN_GNSS_WKE, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_WKE, OUTPUT);

      pinMode(SOC_GPIO_LED_TECHO_REV_1_GREEN, OUTPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_1_RED,   OUTPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_1_BLUE,  OUTPUT);

      ledOn (SOC_GPIO_LED_TECHO_REV_1_GREEN);
      ledOff(SOC_GPIO_LED_TECHO_REV_1_RED);
      ledOff(SOC_GPIO_LED_TECHO_REV_1_BLUE);

      lmic_pins.rst = SOC_GPIO_PIN_TECHO_REV_1_RST;
#if defined(USE_RADIOLIB)
      lmic_pins.dio[0] = SOC_GPIO_PIN_DIO1;
#endif /* USE_RADIOLIB */

      hw_info.revision = 1;
      break;

    case NRF52_LILYGO_TECHO_REV_2:
      /* Wake up Quectel L76K GNSS */
      digitalWrite(SOC_GPIO_PIN_GNSS_RST, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_RST, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_GNSS_WKE, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_WKE, OUTPUT);

      pinMode(SOC_GPIO_LED_TECHO_REV_2_GREEN, OUTPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_2_RED,   OUTPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_2_BLUE,  OUTPUT);

      ledOn (SOC_GPIO_LED_TECHO_REV_2_GREEN);
      ledOff(SOC_GPIO_LED_TECHO_REV_2_RED);
      ledOff(SOC_GPIO_LED_TECHO_REV_2_BLUE);

      lmic_pins.rst = SOC_GPIO_PIN_TECHO_REV_2_RST;
#if defined(USE_RADIOLIB)
      lmic_pins.dio[0] = SOC_GPIO_PIN_DIO1;
#endif /* USE_RADIOLIB */

      hw_info.revision = 2;
      break;

    case NRF52_LILYGO_TULTIMA:
      lmic_pins.nss  = SOC_GPIO_PIN_TULTIMA_SS;
      lmic_pins.rst  = SOC_GPIO_PIN_TULTIMA_RST;
      lmic_pins.busy = SOC_GPIO_PIN_TULTIMA_BUSY;
#if defined(USE_RADIOLIB)
      lmic_pins.dio[0] = SOC_GPIO_PIN_TULTIMA_DIO1;
#endif /* USE_RADIOLIB */

#if defined(ENABLE_RECORDER)
      {
        int uSD_SS_pin = SOC_GPIO_PIN_SD_TULTIMA_SS;

        /* micro-SD SPI is shared with Radio SPI */
        SPI.setPins(SOC_GPIO_PIN_TULTIMA_MISO,
                    SOC_GPIO_PIN_TULTIMA_SCK,
                    SOC_GPIO_PIN_TULTIMA_MOSI);

        pinMode(uSD_SS_pin, OUTPUT);
        digitalWrite(uSD_SS_pin, HIGH);

        uSD_is_attached = uSD.cardBegin(SD_CONFIG);

        if (uSD_is_attached && uSD.card()->cardSize() > 0) {
          hw_info.storage = (hw_info.storage == STORAGE_FLASH) ?
                            STORAGE_FLASH_AND_CARD : STORAGE_CARD;
        }
      }
#endif /* ENABLE_RECORDER */
      break;

    case NRF52_SEEED_T1000E:
      digitalWrite(SOC_GPIO_PIN_GNSS_T1000_EN, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_T1000_EN, OUTPUT);

      pinMode(SOC_GPIO_PIN_GNSS_T1000_VRTC, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_GNSS_T1000_VRTC, HIGH);

      digitalWrite(SOC_GPIO_PIN_GNSS_T1000_RST, LOW);
      pinMode(SOC_GPIO_PIN_GNSS_T1000_RST, OUTPUT);

      pinMode(SOC_GPIO_PIN_GNSS_T1000_SINT, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_GNSS_T1000_SINT, HIGH);

      pinMode(SOC_GPIO_PIN_GNSS_T1000_RINT, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_GNSS_T1000_RINT, LOW);

      pinMode(SOC_GPIO_LED_T1000_GREEN, OUTPUT);
      ledOn (SOC_GPIO_LED_T1000_GREEN);

      lmic_pins.nss  = SOC_GPIO_PIN_T1000_SS;
      lmic_pins.rst  = SOC_GPIO_PIN_T1000_RST;
      lmic_pins.busy = SOC_GPIO_PIN_T1000_BUSY;
#if defined(USE_RADIOLIB)
      lmic_pins.dio[0] = SOC_GPIO_PIN_T1000_DIO9;
#endif /* USE_RADIOLIB */

      break;

    case NRF52_HELTEC_T114:
      digitalWrite(SOC_GPIO_PIN_GNSS_T114_RST, HIGH); /* TBD */
      pinMode(SOC_GPIO_PIN_GNSS_T114_RST, OUTPUT);
      digitalWrite(SOC_GPIO_PIN_GNSS_T114_WKE, HIGH);
      pinMode(SOC_GPIO_PIN_GNSS_T114_WKE, OUTPUT);

      pinMode(SOC_GPIO_LED_T114_GREEN, OUTPUT);
      digitalWrite(SOC_GPIO_LED_T114_GREEN, HIGH);

      lmic_pins.nss  = SOC_GPIO_PIN_T114_SS;
      lmic_pins.rst  = SOC_GPIO_PIN_T114_RST;
      lmic_pins.busy = SOC_GPIO_PIN_T114_BUSY;
#if defined(USE_RADIOLIB)
      lmic_pins.dio[0] = SOC_GPIO_PIN_T114_DIO1;
#endif /* USE_RADIOLIB */

      break;

    case NRF52_NORDIC_PCA10059:
    default:
      pinMode(SOC_GPIO_LED_PCA10059_STATUS, OUTPUT);
      pinMode(SOC_GPIO_LED_PCA10059_GREEN,  OUTPUT);
      pinMode(SOC_GPIO_LED_PCA10059_RED,    OUTPUT);
      pinMode(SOC_GPIO_LED_PCA10059_BLUE,   OUTPUT);

      ledOn (SOC_GPIO_LED_PCA10059_GREEN);
      ledOff(SOC_GPIO_LED_PCA10059_RED);
      ledOff(SOC_GPIO_LED_PCA10059_BLUE);
      ledOff(SOC_GPIO_LED_PCA10059_STATUS);

      hw_info.revision = 3; /* Unknown */
      break;
  }

#if !defined(ARDUINO_ARCH_MBED)
  i2c = new I2CBus(Wire);

  if (nRF52_has_rtc && (i2c != nullptr)) {
    rtc = new PCF8563_Class(*i2c);

    pinMode(SOC_GPIO_PIN_R_INT, INPUT);
    hw_info.rtc = RTC_PCF8563;
  }
#endif /* ARDUINO_ARCH_MBED */

#if !defined(EXCLUDE_IMU)
  if (nRF52_has_imu) {

    Wire.begin();

    if (imu_1.setup(MPU9250_ADDRESS)) {
      imu_1.verbose(false);
      if (imu_1.isSleeping()) {
        imu_1.sleep(false);
      }
      hw_info.imu = IMU_MPU9250;
      hw_info.mag = MAG_AK8963;
      IMU_Time_Marker = millis();
    } else {
      bool ad0 = (ICM20948_ADDRESS == 0x69) ? true : false;

      for (int t=0; t<3; t++) {
        if (imu_2.begin(Wire, ad0) == ICM_20948_Stat_Ok) {
          hw_info.imu = IMU_ICM20948;
          hw_info.mag = MAG_AK09916;
          IMU_Time_Marker = millis();

          break;
        }
        delay(IMU_UPDATE_INTERVAL);
      }
    }
  }
#endif /* EXCLUDE_IMU */

#if !defined(ARDUINO_ARCH_MBED)
  if (nRF52_has_spiflash) {
    spiflash_id = SPIFlash->getJEDECID();

    //mx25_status_config[0] = SPIFlash->readStatus();
    //HWFlashTransport.readCommand(SFLASH_CMD_READ_CONFIG,   mx25_status_config + 1, 2);
    //mx25_status_config[2] |= 0x2;       /* High performance mode */
    //SPIFlash->writeEnable();
    //HWFlashTransport.writeCommand(SFLASH_CMD_WRITE_STATUS, mx25_status_config,     3);
    //SPIFlash->writeDisable();
    //SPIFlash->waitUntilReady();

    //uint32_t const wr_speed = min(80 * 1000000U, (uint32_t)F_CPU);
    //uint32_t rd_speed = wr_speed;
    //HWFlashTransport.setClockSpeed(wr_speed, rd_speed);

    // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
    usb_msc.setID(nRF52_Device_Manufacturer, "External Flash", "1.0");

    // Set callback
    usb_msc.setReadWriteCallback(nRF52_msc_read_cb,
                                 nRF52_msc_write_cb,
                                 nRF52_msc_flush_cb);

    // Set disk size, block size should be 512 regardless of spi flash page size
    usb_msc.setCapacity(SPIFlash->size()/512, 512);

    // MSC is ready for read/write
    usb_msc.setUnitReady(true);

    usb_msc.begin();

    FATFS_is_mounted = fatfs.begin(SPIFlash);
  }
#endif /* ARDUINO_ARCH_MBED */

#if defined(USE_USB_MIDI)
  // Initialize MIDI with no any input channels
  // This will also call usb_midi's begin()
  MIDI_USB.begin(MIDI_CHANNEL_OFF);
#endif /* USE_USB_MIDI */

  Serial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);

#if defined(USE_TINYUSB) && defined(USBCON)
  for (int i=0; i < 20; i++) {if (Serial) break; else delay(100);}
#endif

#if !defined(EXCLUDE_WIFI)
  switch (nRF52_board)
  {
    case NRF52_LILYGO_TULTIMA:
      /* TBD */
      break;
    case NRF52_LILYGO_TECHO_REV_0:
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_NORDIC_PCA10059:
    default:
#if defined(_WIFI_ESP_AT_H_)
      Serial1.begin(115200);
      WiFi.init(&Serial1);
#endif /* _WIFI_ESP_AT_H_ */

#if defined(WiFiNINA_h)
      WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);
#endif /* WiFiNINA_h */
      break;
  }
#endif /* EXCLUDE_WIFI */
}

static void nRF52_post_init()
{
  if (nRF52_board == NRF52_LILYGO_TECHO_REV_0 ||
      nRF52_board == NRF52_LILYGO_TECHO_REV_1 ||
      nRF52_board == NRF52_LILYGO_TECHO_REV_2 ||
      nRF52_board == NRF52_LILYGO_TULTIMA) {

#if 0
    char strbuf[32];
    Serial.println();
    Serial.print  (F("64-bit Device ID: "));
    snprintf(strbuf, sizeof(strbuf),"0x%08x%08x", DEVICE_ID_HIGH, DEVICE_ID_LOW);
    Serial.println(strbuf);
#endif

#if 0
    Serial.println();
    Serial.print  (F("SPI FLASH JEDEC ID: "));
    Serial.print  (spiflash_id, HEX);           Serial.print(" ");
    Serial.print  (F("STATUS/CONFIG: "));
    Serial.print  (mx25_status_config[0], HEX); Serial.print(" ");
    Serial.print  (mx25_status_config[1], HEX); Serial.print(" ");
    Serial.print  (mx25_status_config[2], HEX); Serial.println();
#endif

    Serial.println();
    Serial.print  (F("LilyGO T-"));
    Serial.print  (nRF52_board == NRF52_LILYGO_TULTIMA ? F("Ultima") : F("Echo"));
    Serial.print  (F(" ("));
    Serial.print  (hw_info.revision > 2 ?
                   Hardware_Rev[3] : Hardware_Rev[hw_info.revision]);
    Serial.println(F(") Power-on Self Test"));
    Serial.println();
    Serial.flush();

    Serial.println(F("Built-in components:"));

    Serial.print(F("RADIO   : "));
    Serial.println(hw_info.rf      == RF_IC_SX1262 ||
                   hw_info.rf      == RF_IC_SX1276 ||
                   hw_info.rf      == RF_IC_LR1110 ||
                   hw_info.rf      == RF_IC_LR1121     ? F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("GNSS    : "));
    if (nRF52_board == NRF52_LILYGO_TULTIMA) {
      Serial.println(hw_info.gnss  == GNSS_MODULE_U10  ? F("PASS") : F("FAIL"));
    } else if (nRF52_board == NRF52_LILYGO_TECHO_REV_0 ||
               nRF52_board == NRF52_LILYGO_TECHO_REV_1) {
      Serial.println(hw_info.gnss  == GNSS_MODULE_GOKE ? F("PASS") : F("FAIL"));
    } else {
      Serial.println(hw_info.gnss  == GNSS_MODULE_AT65 ? F("PASS") : F("FAIL"));
    }

    Serial.flush();
    Serial.print(F("DISPLAY : "));
    Serial.println(hw_info.display == DISPLAY_EPD_1_54 ||
                   hw_info.display == DISPLAY_EPD_3_71 ? F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("RTC     : "));
    Serial.println(hw_info.rtc     == RTC_PCF8563      ? F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("FLASH   : "));
    Serial.println(hw_info.storage == STORAGE_FLASH_AND_CARD ||
                   hw_info.storage == STORAGE_FLASH    ? F("PASS") : F("FAIL"));
    Serial.flush();

    if (nRF52_board == NRF52_LILYGO_TECHO_REV_1 ||
        nRF52_board == NRF52_LILYGO_TECHO_REV_2 ||
        nRF52_board == NRF52_LILYGO_TULTIMA) {
      Serial.print(F("BMx280  : "));
      Serial.println(hw_info.baro == BARO_MODULE_BME280AUX ||
                     hw_info.baro == BARO_MODULE_BMP280 ? F("PASS") : F("N/A"));
      Serial.flush();
    }

#if !defined(EXCLUDE_IMU)
    if (nRF52_board != NRF52_LILYGO_TULTIMA) {
      Serial.println();
      Serial.println(F("External components:"));
      Serial.print(F("IMU     : "));
      Serial.println(hw_info.imu   == IMU_MPU9250  ||
                     hw_info.imu   == IMU_ICM20948     ? F("PASS") : F("N/A"));
    } else {
      Serial.print(F("IMU     : "));
      Serial.println(hw_info.imu   == IMU_BHI260AP     ? F("PASS") : F("FAIL"));
    }
    Serial.flush();
#endif /* EXCLUDE_IMU */

    Serial.println();
    Serial.println(F("Power-on Self Test is complete."));
    Serial.println();
    Serial.flush();

    if (nRF52_board == NRF52_LILYGO_TULTIMA) {
#if defined(USE_EXT_I2S_DAC)
      char filename[MAX_FILENAME_LEN];
      strcpy(filename, WAV_FILE_PREFIX);
      strcat(filename, "POST");
      strcat(filename, WAV_FILE_SUFFIX);
      if (FATFS_is_mounted && fatfs.exists(filename)) {
        play_file(filename);
      }
#endif /* USE_EXT_I2S_DAC */
    }
  } else if (nRF52_board == NRF52_NORDIC_PCA10059) {
    Serial.println();
    Serial.println(F("Board: Nordic PCA10059 USB Dongle"));
    Serial.println();
    Serial.flush();
  }

  Serial.println(F("Data output device(s):"));

  Serial.print(F("NMEA   - "));
  switch (settings->nmea_out)
  {
    case NMEA_UART       :  Serial.println(F("UART"));          break;
    case NMEA_USB        :  Serial.println(F("USB CDC"));       break;
    case NMEA_BLUETOOTH  :  Serial.println(F("Bluetooth LE"));  break;
    case NMEA_OFF        :
    default              :  Serial.println(F("NULL"));          break;
  }

  Serial.print(F("GDL90  - "));
  switch (settings->gdl90)
  {
    case GDL90_UART      :  Serial.println(F("UART"));          break;
    case GDL90_USB       :  Serial.println(F("USB CDC"));       break;
    case GDL90_BLUETOOTH :  Serial.println(F("Bluetooth LE"));  break;
    case GDL90_OFF       :
    default              :  Serial.println(F("NULL"));          break;
  }

  Serial.print(F("D1090  - "));
  switch (settings->d1090)
  {
    case D1090_UART      :  Serial.println(F("UART"));          break;
    case D1090_USB       :  Serial.println(F("USB CDC"));       break;
    case D1090_BLUETOOTH :  Serial.println(F("Bluetooth LE"));  break;
    case D1090_OFF       :
    default              :  Serial.println(F("NULL"));          break;
  }

  Serial.println();
  Serial.flush();

#if defined(USE_EPAPER)
  if (nRF52_board == NRF52_LILYGO_TECHO_REV_0 ||
      nRF52_board == NRF52_LILYGO_TECHO_REV_1 ||
      nRF52_board == NRF52_LILYGO_TECHO_REV_2) {
    /* EPD back light on */
    digitalWrite(SOC_GPIO_PIN_EPD_BLGT, HIGH);

    EPD_info1();

    /* EPD back light off */
    digitalWrite(SOC_GPIO_PIN_EPD_BLGT, LOW);

    char key[8];
    char out[64];
    uint8_t tokens[3] = { 0 };
    cdbResult rt;
    int c, i = 0, token_cnt = 0;

    int acfts;
    char *reg, *mam, *cn;
    reg = mam = cn = NULL;

    if (ADB_is_open) {
      acfts = ucdb.recordsNumber();

      snprintf(key, sizeof(key),"%06X", ThisAircraft.addr);

      rt = ucdb.findKey(key, strlen(key));

      switch (rt) {
        case KEY_FOUND:
          while ((c = ucdb.readValue()) != -1 && i < (sizeof(out) - 1)) {
            if (c == '|') {
              if (token_cnt < (sizeof(tokens) - 1)) {
                token_cnt++;
                tokens[token_cnt] = i+1;
              }
              c = 0;
            }
            out[i++] = (char) c;
          }
          out[i] = 0;

          reg = out + tokens[1];
          mam = out + tokens[0];
          cn  = out + tokens[2];

          break;

        case KEY_NOT_FOUND:
        default:
          break;
      }

      reg = (reg != NULL) && strlen(reg) ? reg : (char *) "REG: N/A";
      mam = (mam != NULL) && strlen(mam) ? mam : (char *) "M&M: N/A";
      cn  = (cn  != NULL) && strlen(cn)  ? cn  : (char *) "N/A";

    } else {
      acfts = -1;
    }

    EPD_info2(acfts, reg, mam, cn);
  }
#endif /* USE_EPAPER */
}

static void nRF52_loop()
{
#if !defined(ARDUINO_ARCH_MBED)
  // Reload the watchdog
  if (nrf_wdt_started(NRF_WDT)) {
    Watchdog.reset();
  }

  if (!RTC_sync) {
    if (rtc &&
        gnss.date.isValid()                         &&
        gnss.time.isValid()                         &&
        gnss.date.year() >= fw_build_date_time.year &&
        gnss.date.year() <  fw_build_date_time.year + 15 ) {
      rtc->setDateTime(gnss.date.year(),   gnss.date.month(),
                       gnss.date.day(),    gnss.time.hour(),
                       gnss.time.minute(), gnss.time.second());
      RTC_sync = true;
    }
  }
#endif /* ARDUINO_ARCH_MBED */

#if defined(USE_WEBUSB_SETTINGS) && !defined(USE_WEBUSB_SERIAL)

  if (USBDevice.mounted() && usb_web.connected() && usb_web.available()) {

    JsonObject &root = nRF52_jsonBuffer.parseObject(usb_web);

    if (root.success()) {
      JsonVariant msg_class = root["class"];

      if (msg_class.success()) {
        const char *msg_class_s = msg_class.as<char*>();

        if (!strcmp(msg_class_s,"SOFTRF")) {
          parseSettings  (root);
          parseUISettings(root);

          SoC->WDT_fini();
          if (SoC->Bluetooth_ops) { SoC->Bluetooth_ops->fini(); }
          Serial.println();
          Serial.println(F("Restart is in progress. Please, wait..."));
          Serial.println();
          Serial.flush();
          EEPROM_store();
          RF_Shutdown();
          SoC->reset();
        }
      }
    }
  }

#endif /* USE_WEBUSB_SETTINGS */

#if !defined(EXCLUDE_IMU)
  if (hw_info.imu == IMU_MPU9250 &&
      (millis() - IMU_Time_Marker) > IMU_UPDATE_INTERVAL) {
    if (imu_1.update()) {
      float a_x = imu_1.getAccX();
      float a_y = imu_1.getAccY();
      float a_z = imu_1.getAccZ();

      IMU_g = sqrtf(a_x*a_x + a_y*a_y + a_z*a_z);
    }
    IMU_Time_Marker = millis();
  }

  if (hw_info.imu == IMU_ICM20948 &&
      (millis() - IMU_Time_Marker) > IMU_UPDATE_INTERVAL) {
    if (imu_2.dataReady()) {
      imu_2.getAGMT();

      // milli g's
      float a_x = imu_2.accX();
      float a_y = imu_2.accY();
      float a_z = imu_2.accZ();
#if 0
      Serial.print("{ACCEL: ");
      Serial.print(a_x);
      Serial.print(",");
      Serial.print(a_y);
      Serial.print(",");
      Serial.print(a_z);
      Serial.println("}");
#endif
      IMU_g = sqrtf(a_x*a_x + a_y*a_y + a_z*a_z) / 1000;
    }
    IMU_Time_Marker = millis();
  }
#endif /* EXCLUDE_IMU */
}

static void nRF52_fini(int reason)
{
  uint8_t sd_en;

#if !defined(ARDUINO_ARCH_MBED)
  if (nRF52_has_spiflash) {
    usb_msc.setUnitReady(false);
//  usb_msc.end(); /* N/A */
  }

  if (SPIFlash != NULL) SPIFlash->end();
#endif /* ARDUINO_ARCH_MBED */

#if !defined(EXCLUDE_IMU)
  if (hw_info.imu == IMU_MPU9250) {
    imu_1.sleep(true);
  }

  if (hw_info.imu == IMU_ICM20948) {
    imu_2.sleep(true);
    // imu_2.lowPower(true);
  }
#endif /* EXCLUDE_IMU */

  switch (nRF52_board)
  {
    case NRF52_LILYGO_TECHO_REV_0:
#if 0
      /* Air530 GNSS ultra-low power tracking mode */
      digitalWrite(SOC_GPIO_PIN_GNSS_WKE, LOW);
      pinMode(SOC_GPIO_PIN_GNSS_WKE, OUTPUT);

    // Serial_GNSS_Out.write("$PGKC105,4*33\r\n");
#else
      pinMode(SOC_GPIO_PIN_GNSS_WKE, INPUT);

      // Serial_GNSS_Out.write("$PGKC051,0*37\r\n");
      // Serial_GNSS_Out.write("$PGKC051,1*36\r\n");
#endif
      // Serial_GNSS_Out.flush(); delay(250);

      ledOff(SOC_GPIO_LED_TECHO_REV_0_GREEN);
      ledOff(SOC_GPIO_LED_TECHO_REV_0_RED);
      ledOff(SOC_GPIO_LED_TECHO_REV_0_BLUE);

      pinMode(SOC_GPIO_LED_TECHO_REV_0_GREEN, INPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_0_RED,   INPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_0_BLUE,  INPUT);

      pinMode(SOC_GPIO_PIN_IO_PWR, INPUT);
      pinMode(SOC_GPIO_PIN_SFL_SS, INPUT);
      break;

    case NRF52_LILYGO_TECHO_REV_1:
#if 0
      /* Air530 GNSS ultra-low power tracking mode */
      digitalWrite(SOC_GPIO_PIN_GNSS_WKE, LOW);
      pinMode(SOC_GPIO_PIN_GNSS_WKE, OUTPUT);

    // Serial_GNSS_Out.write("$PGKC105,4*33\r\n");
#else
      pinMode(SOC_GPIO_PIN_GNSS_WKE, INPUT);

      // Serial_GNSS_Out.write("$PGKC051,0*37\r\n");
      // Serial_GNSS_Out.write("$PGKC051,1*36\r\n");
#endif
      // Serial_GNSS_Out.flush(); delay(250);

      ledOff(SOC_GPIO_LED_TECHO_REV_1_GREEN);
      ledOff(SOC_GPIO_LED_TECHO_REV_1_RED);
      ledOff(SOC_GPIO_LED_TECHO_REV_1_BLUE);

      pinMode(SOC_GPIO_LED_TECHO_REV_1_GREEN, INPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_1_RED,   INPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_1_BLUE,  INPUT);

      pinMode(SOC_GPIO_PIN_IO_PWR,    INPUT);
      pinMode(SOC_GPIO_PIN_SFL_WP,    INPUT);
      pinMode(SOC_GPIO_PIN_SFL_HOLD,  INPUT);
      pinMode(SOC_GPIO_PIN_SFL_SS,    INPUT);
      break;

    case NRF52_LILYGO_TECHO_REV_2:
      ledOff(SOC_GPIO_LED_TECHO_REV_2_GREEN);
      ledOff(SOC_GPIO_LED_TECHO_REV_2_RED);
      ledOff(SOC_GPIO_LED_TECHO_REV_2_BLUE);

      pinMode(SOC_GPIO_LED_TECHO_REV_2_GREEN, INPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_2_RED,   INPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_2_BLUE,  INPUT);

      pinMode(SOC_GPIO_PIN_IO_PWR,    INPUT);
      pinMode(SOC_GPIO_PIN_SFL_HOLD,  INPUT);
      pinMode(SOC_GPIO_PIN_SFL_WP,    INPUT);
      pinMode(SOC_GPIO_PIN_SFL_SS,    INPUT);
      pinMode(SOC_GPIO_PIN_GNSS_WKE,  INPUT);
      pinMode(SOC_GPIO_PIN_GNSS_RST,  INPUT);
      /* Cut 3.3V power off on REV_2 board */
      pinMode(SOC_GPIO_PIN_3V3_PWR, INPUT_PULLDOWN);
      break;

    case NRF52_NORDIC_PCA10059:
    default:
//      ledOff(SOC_GPIO_LED_PCA10059_GREEN);
      ledOff(SOC_GPIO_LED_PCA10059_RED);
      ledOff(SOC_GPIO_LED_PCA10059_BLUE);
      ledOff(SOC_GPIO_LED_PCA10059_STATUS);

//      pinMode(SOC_GPIO_LED_PCA10059_GREEN,  INPUT);
      pinMode(SOC_GPIO_LED_PCA10059_RED,    INPUT);
      pinMode(SOC_GPIO_LED_PCA10059_BLUE,   INPUT);
      pinMode(SOC_GPIO_LED_PCA10059_STATUS, INPUT);
      break;
  }

  Serial_GNSS_In.end();

  // pinMode(SOC_GPIO_PIN_GNSS_RX, INPUT);
  // pinMode(SOC_GPIO_PIN_GNSS_TX, INPUT);

  // pinMode(SOC_GPIO_PIN_BATTERY, INPUT);

#if !defined(ARDUINO_ARCH_MBED)
  if (i2c != nullptr) Wire.end();
#endif /* ARDUINO_ARCH_MBED */

  pinMode(SOC_GPIO_PIN_SDA,  INPUT);
  pinMode(SOC_GPIO_PIN_SCL,  INPUT);

  // pinMode(SOC_GPIO_PIN_MOSI, INPUT);
  // pinMode(SOC_GPIO_PIN_MISO, INPUT);
  // pinMode(SOC_GPIO_PIN_SCK,  INPUT);
  pinMode(SOC_GPIO_PIN_SS,   INPUT_PULLUP);
  // pinMode(SOC_GPIO_PIN_BUSY, INPUT);
  pinMode(lmic_pins.rst,  INPUT);

  int mode_button_pin;

  switch (nRF52_board)
  {
    case NRF52_LILYGO_TULTIMA:
      mode_button_pin = SOC_GPIO_PIN_TULTIMA_BUTTON1;
      break;

    case NRF52_SEEED_T1000E:
      mode_button_pin = SOC_GPIO_PIN_T1000_BUTTON;
      break;

    case NRF52_HELTEC_T114:
      mode_button_pin = SOC_GPIO_PIN_T114_BUTTON;
      break;

    case NRF52_LILYGO_TECHO_REV_0:
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_NORDIC_PCA10059:
    default:
      mode_button_pin = SOC_GPIO_PIN_BUTTON;
      break;
  }

  // pinMode(SOC_GPIO_PIN_PAD,    INPUT);
  pinMode(mode_button_pin, nRF52_board == NRF52_LILYGO_TECHO_REV_1 ? INPUT_PULLUP : INPUT);
  while (digitalRead(mode_button_pin) == LOW);
  delay(100);

#if defined(USE_TINYUSB)
  Serial1.end();

  // pinMode(SOC_GPIO_PIN_CONS_RX, INPUT);
  // pinMode(SOC_GPIO_PIN_CONS_TX, INPUT);
#endif

  // setup wake-up pins
  switch (reason)
  {
  case SOFTRF_SHUTDOWN_BUTTON:
  case SOFTRF_SHUTDOWN_LOWBAT:
    NRF_POWER->GPREGRET = DFU_MAGIC_SKIP;
#if !defined(ARDUINO_ARCH_MBED)
    pinMode(mode_button_pin, INPUT_PULLUP_SENSE /* INPUT_SENSE_LOW */);
#endif /* ARDUINO_ARCH_MBED */
    break;
#if defined(USE_SERIAL_DEEP_SLEEP)
  case SOFTRF_SHUTDOWN_NMEA:
#if !defined(ARDUINO_ARCH_MBED)
    pinMode(SOC_GPIO_PIN_CONS_RX, INPUT_PULLUP_SENSE /* INPUT_SENSE_LOW */);
#endif /* ARDUINO_ARCH_MBED */
    break;
#endif
  default:
    break;
  }

  Serial.end();

#if !defined(ARDUINO_ARCH_MBED)
  (void) sd_softdevice_is_enabled(&sd_en);

  // Enter System OFF state
  if ( sd_en ) {
    sd_power_system_off();
  } else {
    NRF_POWER->SYSTEMOFF = 1;
  }
#else
  NRF_POWER->SYSTEMOFF = 1;
#endif /* ARDUINO_ARCH_MBED */
}

static void nRF52_reset()
{
#if !defined(ARDUINO_ARCH_MBED)
  if (nrf_wdt_started(NRF_WDT)) {
    // When WDT is active - CRV, RREN and CONFIG are blocked
    // There is no way to stop/disable watchdog using source code
    // It can only be reset by WDT timeout, Pin reset, Power reset
#if defined(USE_EPAPER)
    if (hw_info.display == DISPLAY_EPD_1_54) {

#if defined(USE_EPD_TASK)
      while (EPD_update_in_progress != EPD_UPDATE_NONE) { delay(100); }
//    while (!SoC->Display_lock()) { delay(10); }
#endif

      EPD_Message("PLEASE,", "WAIT..");
    }
#endif /* USE_EPAPER */
    while (true) { delay(100); }
  } else {
    NVIC_SystemReset();
  }
#else
  /* TBD */
#endif /* ARDUINO_ARCH_MBED */
}

static uint32_t nRF52_getChipId()
{
#if !defined(SOFTRF_ADDRESS)
  uint32_t id = DEVICE_ID_LOW;

  return DevID_Mapper(id);
#else
  return (SOFTRF_ADDRESS & 0xFFFFFFFFU );
#endif
}

static void* nRF52_getResetInfoPtr()
{
  return (void *) &reset_info;
}

static String nRF52_getResetInfo()
{
  switch (reset_info.reason)
  {
    default                     : return F("No reset information available");
  }
}

static String nRF52_getResetReason()
{
  switch (reset_info.reason)
  {
    case REASON_DEFAULT_RST       : return F("DEFAULT");
    case REASON_WDT_RST           : return F("WDT");
    case REASON_EXCEPTION_RST     : return F("EXCEPTION");
    case REASON_SOFT_WDT_RST      : return F("SOFT_WDT");
    case REASON_SOFT_RESTART      : return F("SOFT_RESTART");
    case REASON_DEEP_SLEEP_AWAKE  : return F("DEEP_SLEEP_AWAKE");
    case REASON_EXT_SYS_RST       : return F("EXT_SYS");
    default                       : return F("NO_MEAN");
  }
}

static uint32_t nRF52_getFreeHeap()
{
#if !defined(ARDUINO_ARCH_MBED)
  return dbgHeapTotal() - dbgHeapUsed();
#else
  return 0; /* TBD */
#endif /* ARDUINO_ARCH_MBED */
}

static long nRF52_random(long howsmall, long howBig)
{
  return random(howsmall, howBig);
}

#if defined(USE_USB_MIDI)
byte note_sequence[] = {62,65,69,65,67,67,65,64,69,69,67,67,62,62};
#endif /* USE_USB_MIDI */

static void nRF52_Sound_test(int var)
{
#if defined(USE_USB_MIDI)
  if (USBDevice.mounted() && settings->volume != BUZZER_OFF) {
    unsigned int position = 0;
    unsigned int current  = 0;

    for (; position <= sizeof(note_sequence); position++) {
      // Setup variables for the current and previous
      // positions in the note sequence.
      current = position;
      // If we currently are at position 0, set the
      // previous position to the last note in the sequence.
      unsigned int previous = (current == 0) ? (sizeof(note_sequence)-1) : current - 1;

      // Send Note On for current position at full velocity (127) on channel 1.
      MIDI_USB.sendNoteOn(note_sequence[current], 127, MIDI_CHANNEL_TRAFFIC);

      // Send Note Off for previous note.
      MIDI_USB.sendNoteOff(note_sequence[previous], 0, MIDI_CHANNEL_TRAFFIC);

      delay(286);
    }

    MIDI_USB.sendNoteOff(note_sequence[current], 0, MIDI_CHANNEL_TRAFFIC);
  }
#endif /* USE_USB_MIDI */

#if defined(USE_PWM_SOUND)
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && settings->volume != BUZZER_OFF) {
    tone(SOC_GPIO_PIN_BUZZER, 440,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 640,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 840,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 1040, 500); delay(600);
  }
#endif /* USE_PWM_SOUND */
}

#if defined(USE_BLE_MIDI)
extern BLEMidi blemidi;
extern midi::MidiInterface<BLEMidi> MIDI_BLE;
#endif /* USE_BLE_MIDI */

static void nRF52_Sound_tone(int hz, uint8_t volume)
{
#if defined(USE_PWM_SOUND)
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && volume != BUZZER_OFF) {
    if (hz > 0) {
      tone(SOC_GPIO_PIN_BUZZER, hz, ALARM_TONE_MS);
    } else {
      noTone(SOC_GPIO_PIN_BUZZER);
    }
  }
#endif /* USE_PWM_SOUND */

#if defined(USE_USB_MIDI)
  if (USBDevice.mounted() && volume != BUZZER_OFF) {
    if (hz > 0) {
      MIDI_USB.sendNoteOn (60, 127, MIDI_CHANNEL_TRAFFIC); // 60 == middle C
    } else {
      MIDI_USB.sendNoteOff(60,   0, MIDI_CHANNEL_TRAFFIC);
    }
  }
#endif /* USE_USB_MIDI */

#if defined(USE_BLE_MIDI)
  if (volume != BUZZER_OFF  &&
      Bluefruit.connected() &&
      blemidi.notifyEnabled()) {
    if (hz > 0) {
      MIDI_BLE.sendNoteOn (60, 127, MIDI_CHANNEL_TRAFFIC); // 60 == middle C
    } else {
      MIDI_BLE.sendNoteOff(60,   0, MIDI_CHANNEL_TRAFFIC);
    }
  }
#endif /* USE_BLE_MIDI */
}

static void nRF52_WiFi_set_param(int ndx, int value)
{
  /* NONE */
}

static void nRF52_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  /* NONE */
}

static bool nRF52_EEPROM_begin(size_t size)
{
  if (size > EEPROM.length()) {
    return false;
  }

  EEPROM.begin();

  return true;
}

static void nRF52_EEPROM_extension(int cmd)
{
  uint8_t *raw = (uint8_t *) ui;

  switch (cmd)
  {
    case EEPROM_EXT_STORE:
      for (int i=0; i<sizeof(ui_settings_t); i++) {
        EEPROM.write(sizeof(eeprom_t) + i, raw[i]);
      }
      return;
    case EEPROM_EXT_DEFAULTS:
      ui->adapter      = 0;
      ui->connection   = 0;
      ui->units        = UNITS_METRIC;
      ui->zoom         = ZOOM_MEDIUM;
      ui->protocol     = PROTOCOL_NMEA;
      ui->baudrate     = 0;
      strcpy(ui->server, "");
      strcpy(ui->key,    "");
      ui->rotate       = ROTATE_0;
      ui->orientation  = DIRECTION_TRACK_UP;
      ui->adb          = DB_OGN;
      ui->idpref       = ID_TYPE;
      ui->vmode        = VIEW_MODE_STATUS;
      ui->voice        = VOICE_OFF;
      ui->aghost       = ANTI_GHOSTING_OFF;
      ui->filter       = TRAFFIC_FILTER_OFF;
      ui->power_save   = 0;
      ui->team         = 0;
      break;
    case EEPROM_EXT_LOAD:
    default:
      for (int i=0; i<sizeof(ui_settings_t); i++) {
        raw[i] = EEPROM.read(sizeof(eeprom_t) + i);
      }

#if !defined(ARDUINO_ARCH_MBED)
      if ( nRF52_has_spiflash && FATFS_is_mounted ) {
        File32 file = fatfs.open(SETTINGS_JSON_PATH, FILE_READ);

        if (file) {
          // StaticJsonBuffer<NRF52_JSON_BUFFER_SIZE> nRF52_jsonBuffer;

          JsonObject &root = nRF52_jsonBuffer.parseObject(file);

          if (root.success()) {
            JsonVariant msg_class = root["class"];

            if (msg_class.success()) {
              const char *msg_class_s = msg_class.as<char*>();

              if (!strcmp(msg_class_s,"SOFTRF")) {
                parseSettings  (root);
                parseUISettings(root);

                JsonVariant adb = root["adb"];
                if (adb.success()) {
                  const char * adb_s = adb.as<char*>();
                  if (!strcmp(adb_s,"NONE")) {
                    ui_settings.adb = DB_NONE;
                  } else if (!strcmp(adb_s,"FLN")) {
                    ui_settings.adb = DB_FLN;
                  } else if (!strcmp(adb_s,"OGN")) {
                    ui_settings.adb = DB_OGN;
                  }
                }

#if defined(ENABLE_PROL)
                JsonVariant fromcall = root["fromcall"];
                if (fromcall.success()) {
                  const char * fromcall_s = fromcall.as<char*>();
                  if (strlen(fromcall_s) < sizeof(APRS_FromCall)) {
                    strncpy(APRS_FromCall, fromcall_s, sizeof(APRS_FromCall));
                  }
                }
                JsonVariant tocall = root["tocall"];
                if (tocall.success()) {
                  const char * tocall_s = tocall.as<char*>();
                  if (strlen(tocall_s) < sizeof(APRS_ToCall)) {
                    strncpy(APRS_ToCall, tocall_s, sizeof(APRS_ToCall));
                  }
                }
                JsonVariant path = root["path"];
                if (path.success()) {
                  const char * path_s = path.as<char*>();
                  if (strlen(path_s) < sizeof(APRS_Path)) {
                    strncpy(APRS_Path, path_s, sizeof(APRS_Path));
                  }
                }
#endif /* ENABLE_PROL */
#if defined(ENABLE_REMOTE_ID)
                JsonVariant opid = root["uas_opid"];
                if (opid.success()) {
                  const char * opid_s = opid.as<char*>();
                  if (strlen(opid_s) < sizeof(RID_Operator_ID)) {
                    strncpy(RID_Operator_ID, opid_s, sizeof(RID_Operator_ID));
                  }
                }
                JsonVariant drid = root["uas_drid"];
                if (drid.success()) {
                  const char * drid_s = drid.as<char*>();
                  if (strlen(drid_s) < sizeof(RID_Drone_ID)) {
                    strncpy(RID_Drone_ID, drid_s, sizeof(RID_Drone_ID));
                  }
                }
#endif /* ENABLE_REMOTE_ID */
              }
            }
          }
          file.close();
        }
      }
#endif /* ARDUINO_ARCH_MBED */

#if defined(USE_WEBUSB_SETTINGS) && !defined(USE_WEBUSB_SERIAL)

      usb_web.setLandingPage(&landingPage);
      usb_web.begin();

#endif /* USE_WEBUSB_SETTINGS */

      if (settings->mode != SOFTRF_MODE_NORMAL
#if !defined(EXCLUDE_TEST_MODE)
          &&
          settings->mode != SOFTRF_MODE_TXRX_TEST
#endif /* EXCLUDE_TEST_MODE */
          ) {
        settings->mode = SOFTRF_MODE_NORMAL;
      }

      if (settings->nmea_out == NMEA_UDP  ||
          settings->nmea_out == NMEA_TCP ) {
        settings->nmea_out = NMEA_BLUETOOTH;
      }
      if (settings->gdl90 == GDL90_UDP) {
        settings->gdl90 = GDL90_BLUETOOTH;
      }
      if (settings->d1090 == D1090_UDP) {
        settings->d1090 = D1090_BLUETOOTH;
      }

      /* AUTO and UK RF bands are deprecated since Release v1.3 */
      if (settings->band == RF_BAND_AUTO || settings->band == RF_BAND_UK) {
        settings->band = RF_BAND_EU;
      }

      break;
  }
}

static void nRF52_SPI_begin()
{
#if !defined(ARDUINO_ARCH_MBED)
  switch (nRF52_board)
  {
    case NRF52_LILYGO_TULTIMA:
      SPI.setPins(SOC_GPIO_PIN_TULTIMA_MISO,
                  SOC_GPIO_PIN_TULTIMA_SCK,
                  SOC_GPIO_PIN_TULTIMA_MOSI);
      break;
    case NRF52_SEEED_T1000E:
      SPI.setPins(SOC_GPIO_PIN_T1000_MISO,
                  SOC_GPIO_PIN_T1000_SCK,
                  SOC_GPIO_PIN_T1000_MOSI);
      break;
    case NRF52_NORDIC_PCA10059:
      SPI.setPins(SOC_GPIO_PIN_PCA10059_MISO,
                  SOC_GPIO_PIN_PCA10059_SCK,
                  SOC_GPIO_PIN_PCA10059_MOSI);
      break;
    case NRF52_LILYGO_TECHO_REV_0:
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_HELTEC_T114:
    default:
      SPI.setPins(SOC_GPIO_PIN_TECHO_REV_0_MISO,
                  SOC_GPIO_PIN_TECHO_REV_0_SCK,
                  SOC_GPIO_PIN_TECHO_REV_0_MOSI);
      break;
  }
#endif /* ARDUINO_ARCH_MBED */

  SPI.begin();
}

static void nRF52_swSer_begin(unsigned long baud)
{
#if !defined(ARDUINO_ARCH_MBED)
  switch (nRF52_board)
  {
    case NRF52_LILYGO_TULTIMA:
      Serial_GNSS_In.setPins(SOC_GPIO_PIN_GNSS_TULTIMA_RX,
                             SOC_GPIO_PIN_GNSS_TULTIMA_TX);
      break;
    case NRF52_SEEED_T1000E:
      Serial_GNSS_In.setPins(SOC_GPIO_PIN_GNSS_T1000_RX,
                             SOC_GPIO_PIN_GNSS_T1000_TX);
      baud = 115200; /* Airoha AG3335 default value */
      break;
    case NRF52_HELTEC_T114:
      Serial_GNSS_In.setPins(SOC_GPIO_PIN_GNSS_T114_RX,
                             SOC_GPIO_PIN_GNSS_T114_TX);
      break;
    case NRF52_LILYGO_TECHO_REV_0:
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_NORDIC_PCA10059:
    default:
      Serial_GNSS_In.setPins(SOC_GPIO_PIN_GNSS_RX, SOC_GPIO_PIN_GNSS_TX);
      break;
  }
#endif /* ARDUINO_ARCH_MBED */
  Serial_GNSS_In.begin(baud);
}

static void nRF52_swSer_enableRx(boolean arg)
{
  /* NONE */
}

#if !defined(ARDUINO_ARCH_MBED)
SemaphoreHandle_t Display_Semaphore;
#endif /* ARDUINO_ARCH_MBED */
unsigned long TaskInfoTime;

#if defined(USE_EPAPER)

#include <SoftSPI.h>
SoftSPI swSPI(SOC_GPIO_PIN_EPD_MOSI,
              SOC_GPIO_PIN_EPD_MOSI, /* half duplex */
              SOC_GPIO_PIN_EPD_SCK);

static nRF52_display_id nRF52_EPD_ident()
{
  nRF52_display_id rval = EP_GDEH0154D67; /* default */

  digitalWrite(SOC_GPIO_PIN_EPD_SS, HIGH);
  pinMode(SOC_GPIO_PIN_EPD_SS, OUTPUT);
  digitalWrite(SOC_GPIO_PIN_EPD_DC, HIGH);
  pinMode(SOC_GPIO_PIN_EPD_DC, OUTPUT);

  digitalWrite(SOC_GPIO_PIN_EPD_RST, LOW);
  pinMode(SOC_GPIO_PIN_EPD_RST, OUTPUT);
  delay(20);
  pinMode(SOC_GPIO_PIN_EPD_RST, INPUT_PULLUP);
  delay(200);
  pinMode(SOC_GPIO_PIN_EPD_BUSY, INPUT);

  swSPI.begin();

  uint8_t buf_2D[11];
  uint8_t buf_2E[10];

  taskENTER_CRITICAL();

  digitalWrite(SOC_GPIO_PIN_EPD_DC, LOW);
  digitalWrite(SOC_GPIO_PIN_EPD_SS, LOW);

  swSPI.transfer_out(0x2D);

  pinMode(SOC_GPIO_PIN_EPD_MOSI, INPUT);
  digitalWrite(SOC_GPIO_PIN_EPD_DC, HIGH);

  for (int i=0; i<sizeof(buf_2D); i++) {
    buf_2D[i] = swSPI.transfer_in();
  }

  digitalWrite(SOC_GPIO_PIN_EPD_SCK, LOW);
  digitalWrite(SOC_GPIO_PIN_EPD_DC,  LOW);
  digitalWrite(SOC_GPIO_PIN_EPD_SS,  HIGH);

  taskEXIT_CRITICAL();

  delay(1);

  taskENTER_CRITICAL();

  digitalWrite(SOC_GPIO_PIN_EPD_DC, LOW);
  digitalWrite(SOC_GPIO_PIN_EPD_SS, LOW);

  swSPI.transfer_out(0x2E);

  pinMode(SOC_GPIO_PIN_EPD_MOSI, INPUT);
  digitalWrite(SOC_GPIO_PIN_EPD_DC, HIGH);

  for (int i=0; i<sizeof(buf_2E); i++) {
    buf_2E[i] = swSPI.transfer_in();
  }

  digitalWrite(SOC_GPIO_PIN_EPD_SCK, LOW);
  digitalWrite(SOC_GPIO_PIN_EPD_DC,  LOW);
  digitalWrite(SOC_GPIO_PIN_EPD_SS,  HIGH);

  taskEXIT_CRITICAL();

  swSPI.end();

#if 0
  Serial.print("2D: ");
  for (int i=0; i<sizeof(buf_2D); i++) {
    Serial.print(buf_2D[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

  Serial.print("2E: ");
  for (int i=0; i<sizeof(buf_2E); i++) {
    Serial.print(buf_2E[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

/*
 *  0x2D:
 *  FF FF FF FF FF FF FF FF FF FF FF - C1
 *  00 00 00 00 00 FF 40 00 00 00 01 - D67 SYX 1942
 *  00 00 00 FF 00 00 40 01 00 00 00 - D67 SYX 2118
 *  00 00 00 FF 00 00 40 01 00 00 00 - D67 SYX 2129
 *  00 00 00 00 00 00 00 00 00 00 00 - DEPG0150BN
 *
 *  0x2E:
 *  00 00 00 00 00 00 00 00 00 00    - C1
 *  00 00 00 00 00 00 00 00 00 00    - D67 SYX 1942
 *  00 05 00 9A 00 55 35 37 14 0C    - D67 SYX 2118
 *  00 00 00 00 00 00 00 00 00 00    - D67 SYX 2129
 *  00 00 00 00 00 00 00 00 00 00    - DEPG0150BN
 */
#endif

  bool is_ff = true;
  for (int i=0; i<sizeof(buf_2D); i++) {
    if (buf_2D[i] != 0xFF) {is_ff = false; break;}
  }

  bool is_00 = true;
  for (int i=0; i<sizeof(buf_2D); i++) {
    if (buf_2D[i] != 0x00) {is_00 = false; break;}
  }

  if (is_00) {
    rval = EP_DEPG0150BN;
  }

  return rval;
}

#endif /* USE_EPAPER */

static byte nRF52_Display_setup()
{
  byte rval = DISPLAY_NONE;

  if (nRF52_board == NRF52_NORDIC_PCA10059 ||
      nRF52_board == NRF52_SEEED_T1000E) {
      /* Nothing to do */
  } else if (nRF52_board == NRF52_HELTEC_T114) {
#if defined(USE_TFT)
#if SPI_INTERFACES_COUNT >= 2
    SPI1.setPins(SOC_GPIO_PIN_T114_TFT_MISO,
                 SOC_GPIO_PIN_T114_TFT_SCK,
                 SOC_GPIO_PIN_T114_TFT_MOSI);
#endif

    nRF52_display = TFT_LH114TIF03;

    tft = new TFT_eSPI(LV_HOR_RES, LV_VER_RES);
    tft->init();
#if LV_HOR_RES != 135 && LV_HOR_RES != 80
    uint8_t r = 0;
#else
    uint8_t r = 1; /* 90 degrees */
#endif /* LV_HOR_RES */
    tft->setRotation(r);
    tft->fillScreen(TFT_NAVY);

    int bl_pin = SOC_GPIO_PIN_T114_TFT_BLGT;

    tft->setTextFont(4);
    tft->setTextSize(2);
    tft->setTextColor(TFT_WHITE, TFT_NAVY);

    uint16_t tbw = tft->textWidth(SoftRF_text1);
    uint16_t tbh = tft->fontHeight();
    tft->setCursor((tft->width() - tbw)/2, (tft->height() - tbh)/2);
    tft->println(SoftRF_text1);

    for (int level = 0; level < 255; level += 25) {
      TFT_backlight_adjust(bl_pin, level);
      delay(100);
    }

#if LV_HOR_RES == 135
    rval = DISPLAY_TFT_TTGO_135;
#elif LV_HOR_RES == 80
    rval = DISPLAY_TFT_HELTEC_80;
#else
    rval = DISPLAY_TFT_TTGO_240;
#endif /* LV_HOR_RES */
#endif /* USE_TFT */
  } else {
#if defined(USE_EPAPER)

#if SPI_INTERFACES_COUNT >= 2
    switch (nRF52_board)
    {
      case NRF52_LILYGO_TULTIMA:
        SPI1.setPins(SOC_GPIO_PIN_EPD_TULTIMA_MISO,
                     SOC_GPIO_PIN_EPD_TULTIMA_SCK,
                     SOC_GPIO_PIN_EPD_TULTIMA_MOSI);
        nRF52_display = EP_GDEY037T03;
        break;
      case NRF52_LILYGO_TECHO_REV_0:
      case NRF52_LILYGO_TECHO_REV_1:
      case NRF52_LILYGO_TECHO_REV_2:
      default:
        SPI1.setPins(SOC_GPIO_PIN_EPD_MISO,
                     SOC_GPIO_PIN_EPD_SCK,
                     SOC_GPIO_PIN_EPD_MOSI);
        break;
    }
#endif

    if (nRF52_display == EP_UNKNOWN) {
      nRF52_display = nRF52_EPD_ident();
    }

    switch (nRF52_display)
    {
    case EP_GDEP015OC1:
      display = &epd_c1;
      break;
    case EP_DEPG0150BN:
      display = &epd_bn;
      break;
    case EP_GDEY037T03:
      display = &epd_t3;
      break;
    case EP_GDEH0154D67:
    default:
      display = &epd_d67;
      break;
    }

    display->epd2.selectSPI(SPI1, SPISettings(4000000, MSBFIRST, SPI_MODE0));

    if (EPD_setup(true)) {

#if defined(USE_EPD_TASK)
      Display_Semaphore = xSemaphoreCreateBinary();

      if( Display_Semaphore != NULL ) {
        xSemaphoreGive( Display_Semaphore );
      }

      xTaskCreate(EPD_Task, "EPD", EPD_STACK_SZ, NULL,
                  /* TASK_PRIO_HIGH */ TASK_PRIO_LOW , &EPD_Task_Handle);

      TaskInfoTime = millis();
#endif
      rval = DISPLAY_EPD_1_54;
    }

    /* EPD back light off */
    pinMode(SOC_GPIO_PIN_EPD_BLGT, OUTPUT);
    digitalWrite(SOC_GPIO_PIN_EPD_BLGT, LOW);

#endif /* USE_EPAPER */
  }

  return rval;
}

static void nRF52_Display_loop()
{
  char buf[16];
  uint32_t disp_value;

  uint16_t tbw;
  uint16_t tbh;

  switch (hw_info.display)
  {
#if defined(USE_EPAPER)
  case DISPLAY_EPD_1_54:
    EPD_loop();

#if 0
    if (millis() - TaskInfoTime > 5000) {
      char pcWriteBuffer[512];
      vTaskList(pcWriteBuffer );
      Serial.println(pcWriteBuffer); Serial.flush();
      TaskInfoTime = millis();
    }
#endif
    break;
#endif /* USE_EPAPER */

#if defined(USE_TFT)
#if LV_HOR_RES == 135
  case DISPLAY_TFT_TTGO_135:
    if (tft) {
      if (!TFT_display_frontpage) {
        tft->fillScreen(TFT_NAVY);

        tft->setTextFont(2);
        tft->setTextSize(2);
        tft->setTextColor(TFT_WHITE, TFT_NAVY);

        tbw = tft->textWidth(ID_text);
        tbh = tft->fontHeight();

        tft->setCursor(tft->textWidth(" "), tft->height()/4 - tbh - 1);
        tft->print(ID_text);

        tbw = tft->textWidth(PROTOCOL_text);

        tft->setCursor(tft->width() - tbw - tft->textWidth(" "),
                       tft->height()/4 - tbh - 1);
        tft->print(PROTOCOL_text);

        tbw = tft->textWidth(RX_text);
        tbh = tft->fontHeight();

        tft->setCursor(tft->textWidth("   "), 3*tft->height()/4 - tbh - 1);
        tft->print(RX_text);

        tbw = tft->textWidth(TX_text);

        tft->setCursor(tft->width()/2 + tft->textWidth("   "),
                       3*tft->height()/4 - tbh - 1);
        tft->print(TX_text);

        tft->setTextFont(2);
        tft->setTextSize(3);

        snprintf (buf, sizeof(buf), "%06X", ThisAircraft.addr);

        tbw = tft->textWidth(buf);
        tbh = tft->fontHeight();

        tft->setCursor(tft->textWidth(" "), tft->height()/4 - 7);
        tft->print(buf);

        tbw = tft->textWidth("O");

        tft->setCursor(tft->width() - tbw - tft->textWidth(" "),
                       tft->height()/4 - 7);
        tft->print(Protocol_ID[ThisAircraft.protocol][0]);

        itoa(rx_packets_counter % 1000, buf, 10);
        tft->setCursor(tft->textWidth(" "), 3*tft->height()/4 - 7);
        tft->print(buf);

        itoa(tx_packets_counter % 1000, buf, 10);
        tft->setCursor(tft->width()/2 + tft->textWidth(" "), 3*tft->height()/4 - 7);
        tft->print(buf);

        TFT_display_frontpage = true;

      } else { /* TFT_display_frontpage */

        if (rx_packets_counter > prev_rx_packets_counter) {
          disp_value = rx_packets_counter % 1000;
          itoa(disp_value, buf, 10);

          if (disp_value < 10) {
            strcat_P(buf,PSTR("  "));
          } else {
            if (disp_value < 100) {
              strcat_P(buf,PSTR(" "));
            };
          }

          tft->setTextFont(2);
          tft->setTextSize(3);

          tft->setCursor(tft->textWidth(" "), 3*tft->height()/4 - 7);
          tft->print(buf);

          prev_rx_packets_counter = rx_packets_counter;
        }
        if (tx_packets_counter > prev_tx_packets_counter) {
          disp_value = tx_packets_counter % 1000;
          itoa(disp_value, buf, 10);

          if (disp_value < 10) {
            strcat_P(buf,PSTR("  "));
          } else {
            if (disp_value < 100) {
              strcat_P(buf,PSTR(" "));
            };
          }

          tft->setTextFont(2);
          tft->setTextSize(3);

          tft->setCursor(tft->width()/2 + tft->textWidth(" "), 3*tft->height()/4 - 7);
          tft->print(buf);

          prev_tx_packets_counter = tx_packets_counter;
        }
      }
    }

    break;

#endif /* LV_HOR_RES == 135 */
#endif /* USE_TFT */

  case DISPLAY_NONE:
  default:
    break;
  }
}

static void nRF52_Display_fini(int reason)
{
  switch (hw_info.display)
  {
#if defined(USE_EPAPER)
  case DISPLAY_EPD_1_54:

    EPD_fini(reason, screen_saver);

#if defined(USE_EPD_TASK)
    if( EPD_Task_Handle != NULL )
    {
      vTaskDelete( EPD_Task_Handle );
    }

    if( Display_Semaphore != NULL )
    {
      vSemaphoreDelete( Display_Semaphore );
    }
#endif

    SPI1.end();

    // pinMode(SOC_GPIO_PIN_EPD_MISO, INPUT);
    // pinMode(SOC_GPIO_PIN_EPD_MOSI, INPUT);
    // pinMode(SOC_GPIO_PIN_EPD_SCK,  INPUT);
    pinMode(SOC_GPIO_PIN_EPD_SS,   INPUT);
    pinMode(SOC_GPIO_PIN_EPD_DC,   INPUT);
    pinMode(SOC_GPIO_PIN_EPD_RST,  INPUT);
    // pinMode(SOC_GPIO_PIN_EPD_BUSY, INPUT);
    pinMode(SOC_GPIO_PIN_EPD_BLGT, INPUT);

    break;
#endif /* USE_EPAPER */

#if defined(USE_TFT)
  case DISPLAY_TFT_TTGO_135:
    if (tft) {
        int level;
        const char *msg = (reason == SOFTRF_SHUTDOWN_LOWBAT) ?
                   "LOW BAT" : "  OFF  ";

        int bl_pin = SOC_GPIO_PIN_T114_TFT_BLGT;

        for (level = 250; level >= 0; level -= 25) {
          TFT_backlight_adjust(bl_pin, level);
          delay(100);
        }

        tft->fillScreen(TFT_NAVY);
        tft->setTextFont(4);
#if LV_VER_RES == 160
        if (reason == SOFTRF_SHUTDOWN_LOWBAT) {
          tft->setTextSize(1);
        } else
#endif
        {
          tft->setTextSize(2);
        }
        tft->setTextColor(TFT_WHITE, TFT_NAVY);

        uint16_t tbw = tft->textWidth(msg);
        uint16_t tbh = tft->fontHeight();

        tft->setCursor((tft->width() - tbw)/2, (tft->height() - tbh)/2);
        tft->print(msg);

        for (level = 0; level <= 250; level += 25) {
          TFT_backlight_adjust(bl_pin, level);
          delay(100);
        }

        delay(2000);

        for (level = 250; level >= 0; level -= 25) {
          TFT_backlight_adjust(bl_pin, level);
          delay(100);
        }

        TFT_backlight_adjust(bl_pin, 0);
        /* TODO: turn PWM off */
        pinMode(bl_pin, INPUT_PULLDOWN);

        tft->fillScreen(TFT_NAVY);
        TFT_off();
    }

    SPI1.end();

    // pinMode(SOC_GPIO_PIN_T114_TFT_MISO, INPUT);
    // pinMode(SOC_GPIO_PIN_T114_TFT_MOSI, INPUT);
    // pinMode(SOC_GPIO_PIN_T114_TFT_SCK,  INPUT);
    pinMode(SOC_GPIO_PIN_T114_TFT_SS,   INPUT);
    pinMode(SOC_GPIO_PIN_T114_TFT_DC,   INPUT);
    pinMode(SOC_GPIO_PIN_T114_TFT_RST,  INPUT);
    pinMode(SOC_GPIO_PIN_T114_TFT_BLGT, INPUT);

    break;
#endif /* USE_TFT */

  case DISPLAY_NONE:
  default:
    break;
  }
}

#if !defined(ARDUINO_ARCH_MBED)
static bool nRF52_Display_lock()
{
  bool rval = false;

  if ( Display_Semaphore != NULL ) {
    rval = (xSemaphoreTake( Display_Semaphore, ( TickType_t ) 0 ) == pdTRUE);
  }
//Serial.print("Display_lock: "); Serial.println(rval); Serial.flush();
  return rval;
}

static bool nRF52_Display_unlock()
{
  bool rval = false;

  if ( Display_Semaphore != NULL ) {
    rval = (xSemaphoreGive( Display_Semaphore ) == pdTRUE);
  }
//Serial.print("Display_unlock: "); Serial.println(rval); Serial.flush();
  return rval;
}
#endif /* ARDUINO_ARCH_MBED */

static void nRF52_Battery_setup()
{

}

static float nRF52_Battery_param(uint8_t param)
{
  uint32_t bat_adc_pin;
  float rval, voltage, mult;

  switch (param)
  {
  case BATTERY_PARAM_THRESHOLD:
    rval = hw_info.model == SOFTRF_MODEL_BADGE ? BATTERY_THRESHOLD_LIPO   :
           hw_info.model == SOFTRF_MODEL_NEO   ? BATTERY_THRESHOLD_LIPO   :
           hw_info.model == SOFTRF_MODEL_CARD  ? BATTERY_THRESHOLD_LIPO   :
           hw_info.model == SOFTRF_MODEL_COZY  ? BATTERY_THRESHOLD_LIPO   :
                                                 BATTERY_THRESHOLD_NIMHX2;
    break;

  case BATTERY_PARAM_CUTOFF:
    rval = hw_info.model == SOFTRF_MODEL_BADGE ? BATTERY_CUTOFF_LIPO   :
           hw_info.model == SOFTRF_MODEL_NEO   ? BATTERY_CUTOFF_LIPO   :
           hw_info.model == SOFTRF_MODEL_CARD  ? BATTERY_CUTOFF_LIPO   :
           hw_info.model == SOFTRF_MODEL_COZY  ? BATTERY_CUTOFF_LIPO   :
                                                 BATTERY_CUTOFF_NIMHX2;
    break;

  case BATTERY_PARAM_CHARGE:
    voltage = Battery_voltage();
    if (voltage < Battery_cutoff())
      return 0;

    if (voltage > 4.2)
      return 100;

    if (voltage < 3.6) {
      voltage -= 3.3;
      return (voltage * 100) / 3;
    }

    voltage -= 3.6;
    rval = 10 + (voltage * 150 );
    break;

  case BATTERY_PARAM_VOLTAGE:
  default:
    voltage = 0.0;

    switch (hw_info.pmu)
    {
#if !defined(EXCLUDE_PMU)
    case PMU_SY6970:
      if (sy6970.isBatteryConnect() /* TODO */) {
        voltage = sy6970.getBattVoltage();
      }
      break;
#endif /* EXCLUDE_PMU */
    case PMU_NONE:
    default:
      // Set the analog reference to 3.0V (default = 3.6V)
#if !defined(ARDUINO_ARCH_MBED)
      analogReference(AR_INTERNAL_3_0);
#endif /* ARDUINO_ARCH_MBED */

      // Set the resolution to 12-bit (0..4095)
      analogReadResolution(12); // Can be 8, 10, 12 or 14

      // Let the ADC settle
      delay(1);

      switch (nRF52_board)
      {
        case NRF52_SEEED_T1000E:
          bat_adc_pin = SOC_GPIO_PIN_T1000_BATTERY;
          mult        = SOC_ADC_T1000_VOLTAGE_DIV;
          break;
        case NRF52_HELTEC_T114:
          bat_adc_pin = SOC_GPIO_PIN_T114_BATTERY;
          mult        = SOC_ADC_T114_VOLTAGE_DIV;
          break;
        case NRF52_LILYGO_TECHO_REV_0:
        case NRF52_LILYGO_TECHO_REV_1:
        case NRF52_LILYGO_TECHO_REV_2:
        case NRF52_NORDIC_PCA10059:
        default:
          bat_adc_pin = SOC_GPIO_PIN_BATTERY;
          mult        = SOC_ADC_VOLTAGE_DIV;
          break;
      }

      // Get the raw 12-bit, 0..3000mV ADC value
      voltage = analogRead(bat_adc_pin);

      // Set the ADC back to the default settings
#if !defined(ARDUINO_ARCH_MBED)
      analogReference(AR_DEFAULT);
#endif /* ARDUINO_ARCH_MBED */
      analogReadResolution(10);

      // Convert the raw value to compensated mv, taking the resistor-
      // divider into account (providing the actual LIPO voltage)
      // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
      voltage *= (mult * VBAT_MV_PER_LSB);

      break;
    }

    rval = voltage * 0.001;
    break;
  }

  return rval;
}

void nRF52_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long nRF52_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool nRF52_Baro_setup() {
  return true;
}

static void nRF52_UATSerial_begin(unsigned long baud)
{

}

static void nRF52_UATModule_restart()
{

}

static void nRF52_WDT_setup()
{
#if !defined(ARDUINO_ARCH_MBED)
  Watchdog.enable(12000);
#endif /* ARDUINO_ARCH_MBED */
}

static void nRF52_WDT_fini()
{
#if !defined(ARDUINO_ARCH_MBED)
  // cannot disable nRF's WDT
  if (nrf_wdt_started(NRF_WDT)) {
    Watchdog.reset();
  }
#endif /* ARDUINO_ARCH_MBED */
}

#include <AceButton.h>
using namespace ace_button;

AceButton button_1(SOC_GPIO_PIN_BUTTON);
AceButton button_2(SOC_GPIO_PIN_PAD);

// The event handler for the button.
void handleEvent(AceButton* button, uint8_t eventType,
    uint8_t buttonState) {

  switch (eventType) {
    case AceButton::kEventClicked:
    case AceButton::kEventReleased:
#if defined(USE_EPAPER)
      if (button == &button_1) {
#if 0
        if (eventType == AceButton::kEventClicked) {
          Serial.println(F("kEventClicked."));
        } else if (eventType == AceButton::kEventReleased) {
          Serial.println(F("kEventReleased."));
        }
#endif
        EPD_Mode();
      } else if (button == &button_2) {
        EPD_Up();
      }
#endif
      break;
    case AceButton::kEventDoubleClicked:
#if defined(USE_EPAPER)
      if (button == &button_1) {
//        Serial.println(F("kEventDoubleClicked."));
        digitalWrite(SOC_GPIO_PIN_EPD_BLGT,
                     digitalRead(SOC_GPIO_PIN_EPD_BLGT) == LOW);
      }
#endif
      break;
    case AceButton::kEventLongPressed:
      if (button == &button_1) {

#if defined(USE_EPAPER)
        int up_button_pin = -1;

        switch (nRF52_board)
        {
          case NRF52_LILYGO_TULTIMA:
            up_button_pin = SOC_GPIO_PIN_TULTIMA_BUTTON2;
            break;

          case NRF52_LILYGO_TECHO_REV_0:
          case NRF52_LILYGO_TECHO_REV_1:
          case NRF52_LILYGO_TECHO_REV_2:
          case NRF52_NORDIC_PCA10059:
            up_button_pin = SOC_GPIO_PIN_PAD;
          default:
            break;
        }

        if (up_button_pin >= 0 && digitalRead(up_button_pin) == LOW) {
          screen_saver = true;
        }
#endif

        shutdown(SOFTRF_SHUTDOWN_BUTTON);
        Serial.println(F("This will never be printed."));
      }
      break;
  }
}

/* Callbacks for push button interrupt */
void onModeButtonEvent() {
  button_1.check();
}

/* Callbacks for touch button interrupt */
void onUpButtonEvent() {
  button_2.check();
}

static void nRF52_Button_setup()
{
  int mode_button_pin;
  int up_button_pin = -1;

  switch (nRF52_board)
  {
    case NRF52_LILYGO_TULTIMA:
      mode_button_pin = SOC_GPIO_PIN_TULTIMA_BUTTON1;
      up_button_pin   = SOC_GPIO_PIN_TULTIMA_BUTTON2;
      break;

    case NRF52_SEEED_T1000E:
      mode_button_pin = SOC_GPIO_PIN_T1000_BUTTON;
      break;

    case NRF52_HELTEC_T114:
      mode_button_pin = SOC_GPIO_PIN_T114_BUTTON;
      break;

    case NRF52_LILYGO_TECHO_REV_0:
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_NORDIC_PCA10059:
      up_button_pin   = SOC_GPIO_PIN_PAD;
    default:
      mode_button_pin = SOC_GPIO_PIN_BUTTON;
      break;
  }

  // Button(s) uses external pull up resistor.
  pinMode(mode_button_pin, nRF52_board == NRF52_LILYGO_TECHO_REV_1 ? INPUT_PULLUP : INPUT);
  if (up_button_pin >= 0) { pinMode(up_button_pin, INPUT); }

  button_1.init(mode_button_pin);
  if (up_button_pin >= 0) { button_2.init(up_button_pin); }

  // Configure the ButtonConfig with the event handler, and enable all higher
  // level events.
  ButtonConfig* ModeButtonConfig = button_1.getButtonConfig();
  ModeButtonConfig->setEventHandler(handleEvent);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
  ModeButtonConfig->setFeature(
                    ButtonConfig::kFeatureSuppressClickBeforeDoubleClick);
//  ModeButtonConfig->setDebounceDelay(15);
  ModeButtonConfig->setClickDelay(600);
  ModeButtonConfig->setDoubleClickDelay(1500);
  ModeButtonConfig->setLongPressDelay(2000);

  ButtonConfig* UpButtonConfig = button_2.getButtonConfig();
  UpButtonConfig->setEventHandler(handleEvent);
  UpButtonConfig->setFeature(ButtonConfig::kFeatureClick);
//  UpButtonConfig->setDebounceDelay(15);
  UpButtonConfig->setClickDelay(600);
  UpButtonConfig->setDoubleClickDelay(1500);
  UpButtonConfig->setLongPressDelay(2000);

//  attachInterrupt(digitalPinToInterrupt(mode_button_pin), onModeButtonEvent, CHANGE );
  if (up_button_pin >= 0) {
    attachInterrupt(digitalPinToInterrupt(up_button_pin),   onUpButtonEvent,   CHANGE );
  }
}

static void nRF52_Button_loop()
{
  button_1.check();

  switch (nRF52_board)
  {
    case NRF52_LILYGO_TECHO_REV_0:
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_NORDIC_PCA10059:
    case NRF52_LILYGO_TULTIMA:
      button_2.check();
      break;
    default:
      break;
  }
}

static void nRF52_Button_fini()
{
//  detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_BUTTON));

  switch (nRF52_board)
  {
    case NRF52_LILYGO_TECHO_REV_0:
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
    case NRF52_NORDIC_PCA10059:
      detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_PAD));
      break;
    case NRF52_LILYGO_TULTIMA:
      detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_TULTIMA_BUTTON2));
      break;
    default:
      break;
  }
}

#if defined(USE_WEBUSB_SERIAL) && !defined(USE_WEBUSB_SETTINGS)
void line_state_callback(bool connected)
{
  if ( connected ) usb_web.println("WebUSB Serial example");
}
#endif /* USE_WEBUSB_SERIAL */

static void nRF52_USB_setup()
{
#if !defined(ARDUINO_ARCH_MBED)
  if (USBSerial && USBSerial != Serial) {
    USBSerial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
  }
#endif /* ARDUINO_ARCH_MBED */

#if defined(USE_WEBUSB_SERIAL) && !defined(USE_WEBUSB_SETTINGS)

  usb_web.setLandingPage(&landingPage);
  usb_web.setLineStateCallback(line_state_callback);
  //usb_web.setStringDescriptor("TinyUSB WebUSB");
  usb_web.begin();

#endif /* USE_WEBUSB_SERIAL */
}

static void nRF52_USB_loop()
{

}

static void nRF52_USB_fini()
{
#if !defined(ARDUINO_ARCH_MBED)
  if (USBSerial && USBSerial != Serial) {
    USBSerial.end();
  }
#endif /* ARDUINO_ARCH_MBED */

#if defined(USE_WEBUSB_SERIAL) && !defined(USE_WEBUSB_SETTINGS)

  /* TBD */

#endif /* USE_WEBUSB_SERIAL */
}

static int nRF52_USB_available()
{
  int rval = 0;

  if (USBSerial) {
    rval = USBSerial.available();
  }

#if defined(USE_WEBUSB_SERIAL) && !defined(USE_WEBUSB_SETTINGS)

  if ((rval == 0) && USBDevice.mounted() && usb_web.connected()) {
    rval = usb_web.available();
  }

#endif /* USE_WEBUSB_SERIAL */

  return rval;
}

static int nRF52_USB_read()
{
  int rval = -1;

  if (USBSerial) {
    rval = USBSerial.read();
  }

#if defined(USE_WEBUSB_SERIAL) && !defined(USE_WEBUSB_SETTINGS)

  if ((rval == -1) && USBDevice.mounted() && usb_web.connected()) {
    rval = usb_web.read();
  }

#endif /* USE_WEBUSB_SERIAL */

  return rval;
}

static size_t nRF52_USB_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  if (USBSerial && (size < USBSerial.availableForWrite())) {
    rval = USBSerial.write(buffer, size);
  }

#if defined(USE_WEBUSB_SERIAL) && !defined(USE_WEBUSB_SETTINGS)

  size_t rval_webusb = size;

  if (USBDevice.mounted() && usb_web.connected()) {
    rval_webusb = usb_web.write(buffer, size);
  }

//  rval = min(rval, rval_webusb);

#endif /* USE_WEBUSB_SERIAL */

  return rval;
}

IODev_ops_t nRF52_USBSerial_ops = {
  "nRF52 USBSerial",
  nRF52_USB_setup,
  nRF52_USB_loop,
  nRF52_USB_fini,
  nRF52_USB_available,
  nRF52_USB_read,
  nRF52_USB_write
};

static bool nRF52_ADB_setup()
{
#if !defined(ARDUINO_ARCH_MBED)
  if (FATFS_is_mounted) {
    const char *fileName;

    if (ui->adb == DB_OGN) {
      fileName = "/Aircrafts/ogn.cdb";
      if (ucdb.open(fileName) != CDB_OK) {
        Serial.print("Invalid OGN CDB: ");
        Serial.println(fileName);
      } else {
        ADB_is_open = true;
      }
    }
    if (ui->adb == DB_FLN) {
      fileName = "/Aircrafts/fln.cdb";
      if (ucdb.open(fileName) != CDB_OK) {
        Serial.print("Invalid FLN CDB: ");
        Serial.println(fileName);
      } else {
        ADB_is_open = true;
      }
    }
  }
#endif /* ARDUINO_ARCH_MBED */

  return ADB_is_open;
}

static bool nRF52_ADB_fini()
{
#if !defined(ARDUINO_ARCH_MBED)
  if (ADB_is_open) {
    ucdb.close();
    ADB_is_open = false;
  }
#endif /* ARDUINO_ARCH_MBED */

  return !ADB_is_open;
}

/*
 * One aircraft CDB (20000+ records) query takes:
 * 1)     FOUND : 5-7 milliseconds
 * 2) NOT FOUND :   3 milliseconds
 */
static bool nRF52_ADB_query(uint8_t type, uint32_t id, char *buf, size_t size)
{
  char key[8];
  char out[64];
  uint8_t tokens[3] = { 0 };
  cdbResult rt;
  int c, i = 0, token_cnt = 0;
  bool rval = false;

  if (!ADB_is_open) {
    return rval;
  }

#if !defined(ARDUINO_ARCH_MBED)
  snprintf(key, sizeof(key),"%06X", id);

  rt = ucdb.findKey(key, strlen(key));

  switch (rt) {
    case KEY_FOUND:
      while ((c = ucdb.readValue()) != -1 && i < (sizeof(out) - 1)) {
        if (c == '|') {
          if (token_cnt < (sizeof(tokens) - 1)) {
            token_cnt++;
            tokens[token_cnt] = i+1;
          }
          c = 0;
        }
        out[i++] = (char) c;
      }
      out[i] = 0;

      switch (ui->idpref)
      {
      case ID_TAIL:
        snprintf(buf, size, "CN: %s",
          strlen(out + tokens[2]) ? out + tokens[2] : "N/A");
        break;
      case ID_MAM:
        snprintf(buf, size, "%s",
          strlen(out + tokens[0]) ? out + tokens[0] : "Unknown");
        break;
      case ID_REG:
      default:
        snprintf(buf, size, "%s",
          strlen(out + tokens[1]) ? out + tokens[1] : "REG: N/A");
        break;
      }

      rval = true;
      break;

    case KEY_NOT_FOUND:
    default:
      break;
  }
#endif /* ARDUINO_ARCH_MBED */

  return rval;
}

DB_ops_t nRF52_ADB_ops = {
  nRF52_ADB_setup,
  nRF52_ADB_fini,
  nRF52_ADB_query
};

const SoC_ops_t nRF52_ops = {
  SOC_NRF52,
  "nRF52",
  nRF52_setup,
  nRF52_post_init,
  nRF52_loop,
  nRF52_fini,
  nRF52_reset,
  nRF52_getChipId,
  nRF52_getResetInfoPtr,
  nRF52_getResetInfo,
  nRF52_getResetReason,
  nRF52_getFreeHeap,
  nRF52_random,
  nRF52_Sound_test,
  nRF52_Sound_tone,
  NULL,
  nRF52_WiFi_set_param,
  nRF52_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  nRF52_EEPROM_begin,
  nRF52_EEPROM_extension,
  nRF52_SPI_begin,
  nRF52_swSer_begin,
  nRF52_swSer_enableRx,
#if !defined(EXCLUDE_BLUETOOTH)
#if defined(USE_ARDUINOBLE)
  &ArdBLE_Bluetooth_ops,
#else
  &nRF52_Bluetooth_ops,
#endif /* USE_ARDUINOBLE */
#else
  NULL,
#endif /* EXCLUDE_BLUETOOTH */
  &nRF52_USBSerial_ops,
  NULL,
  nRF52_Display_setup,
  nRF52_Display_loop,
  nRF52_Display_fini,
#if 0
  nRF52_Display_lock,
  nRF52_Display_unlock,
#endif
  nRF52_Battery_setup,
  nRF52_Battery_param,
  nRF52_GNSS_PPS_Interrupt_handler,
  nRF52_get_PPS_TimeMarker,
  nRF52_Baro_setup,
  nRF52_UATSerial_begin,
  nRF52_UATModule_restart,
  nRF52_WDT_setup,
  nRF52_WDT_fini,
  nRF52_Button_setup,
  nRF52_Button_loop,
  nRF52_Button_fini,
  &nRF52_ADB_ops
};

#endif /* ARDUINO_ARCH_NRF52 */
