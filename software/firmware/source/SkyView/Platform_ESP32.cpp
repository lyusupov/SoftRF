/*
 * Platform_ESP32.cpp
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

#include "sdkconfig.h"

#include <SPI.h>
#include <Wire.h>
#include <esp_err.h>
#include <esp_wifi.h>
#if !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32P4)
#include <esp_bt.h>
#endif /* CONFIG_IDF_TARGET_ESP32S2 */
#if !defined(CONFIG_IDF_TARGET_ESP32C5)  && \
    !defined(CONFIG_IDF_TARGET_ESP32C6)  && \
    !defined(CONFIG_IDF_TARGET_ESP32C61) && \
    !defined(CONFIG_IDF_TARGET_ESP32H2)  && \
    !defined(CONFIG_IDF_TARGET_ESP32P4)
#include <soc/rtc_cntl_reg.h>
#endif /* CONFIG_IDF_TARGET_ESP32C5 || C6 || H2 || P4 */
#include <rom/spi_flash.h>
#include <soc/adc_channel.h>
#include <driver/i2s.h>
#include <flashchips.h>

#include "SoCHelper.h"
#include "EPDHelper.h"
#if defined(USE_TFT)
#include "TFTHelper.h"
#endif /* USE_TFT */
#include "EEPROMHelper.h"
#include "WiFiHelper.h"
#include "BluetoothHelper.h"

#include "SkyView.h"

#include <battery.h>

#if defined(CONFIG_IDF_TARGET_ESP32)   || \
    defined(CONFIG_IDF_TARGET_ESP32S2) || \
    defined(CONFIG_IDF_TARGET_ESP32C3) || \
    defined(CONFIG_IDF_TARGET_ESP32C5) || \
    defined(CONFIG_IDF_TARGET_ESP32C6) || \
    defined(CONFIG_IDF_TARGET_ESP32P4)
#include <sqlite3.h>
#include <SD.h>
#endif /* CONFIG_IDF_TARGET_ESP32XX */

#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR>=5
#include <esp_mac.h>
#include <esp_flash.h>
#endif /* ESP_IDF_VERSION_MAJOR */

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  28        /* Time ESP32 will go to sleep (in seconds) */

#if defined(EXCLUDE_WIFI)
char UDPpacketBuffer[UDP_PACKET_BUFSIZE]; // Dummy definition to satisfy build sequence
#else
WebServer server ( 80 );
#endif /* EXCLUDE_WIFI */

#if !defined(EXCLUDE_ETHERNET)
#include <ETH.h>
#include "EthernetHelper.h"
#endif /* EXCLUDE_ETHERNET */

/*
 * TTGO-T5S. Pin definition

#define BUSY_PIN        4
#define CS_PIN          5
#define RST_PIN         16
#define DC_PIN          17
#define SCK_PIN         18
#define MOSI_PIN        23

P1-1                    21
P1-2                    22 (LED)

I2S MAX98357A           26
                        25
                        19

I2S MIC                 27
                        32
                        33

B0                      RST
B1                      38
B2                      37
B3                      39

SD                      2
                        13
                        14
                        15

P2                      0
                        12
                        13
                        RXD
                        TXD
                        34
                        35 (BAT)
 */
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> epd_ttgo_t5s_W3(GxEPD2_270(
                                          /*CS=*/   SOC_GPIO_PIN_SS_T5S,
                                          /*DC=*/   SOC_EPD_PIN_DC_T5S,
                                          /*RST=*/  SOC_EPD_PIN_RST_T5S,
                                          /*BUSY=*/ SOC_EPD_PIN_BUSY_T5S
                                          ));
GxEPD2_BW<GxEPD2_270_T91, GxEPD2_270_T91::HEIGHT> epd_ttgo_t5s_T91(GxEPD2_270_T91(
                                                  /*CS=*/   SOC_GPIO_PIN_SS_T5S,
                                                  /*DC=*/   SOC_EPD_PIN_DC_T5S,
                                                  /*RST=*/  SOC_EPD_PIN_RST_T5S,
                                                  /*BUSY=*/ SOC_EPD_PIN_BUSY_T5S
                                                  ));
/*
 * Waveshare E-Paper ESP32 Driver Board

#define SCK_PIN         13
#define MOSI_PIN        14
#define CS_PIN          15
#define BUSY_PIN        25
#define RST_PIN         26
#define DC_PIN          27

B1                      0
LED                     2

RX0, TX0                3,1

P                       0,2,4,5,12,13,14,15,16,17,18,19,21,22,23,25,26,27,32,33,34,35
 */
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> epd_waveshare_W3(GxEPD2_270(
                                          /*CS=*/   SOC_GPIO_PIN_SS_WS,
                                          /*DC=*/   SOC_EPD_PIN_DC_WS,
                                          /*RST=*/  SOC_EPD_PIN_RST_WS,
                                          /*BUSY=*/ SOC_EPD_PIN_BUSY_WS
                                          ));

GxEPD2_BW<GxEPD2_270_T91, GxEPD2_270_T91::HEIGHT> epd_waveshare_T91(GxEPD2_270_T91(
                                                  /*CS=*/   SOC_GPIO_PIN_SS_WS,
                                                  /*DC=*/   SOC_EPD_PIN_DC_WS,
                                                  /*RST=*/  SOC_EPD_PIN_RST_WS,
                                                  /*BUSY=*/ SOC_EPD_PIN_BUSY_WS
                                                  ));

static union {
  uint8_t efuse_mac[6];
  uint64_t chipmacid;
};

#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32P4)
#if SOC_SDMMC_IO_POWER_EXTERNAL
#include "esp_ldo_regulator.h"
#endif /* SOC_SDMMC_IO_POWER_EXTERNAL */

static sqlite3 *fln_db  = NULL;
static sqlite3 *ogn_db  = NULL;
static sqlite3 *icao_db = NULL;

static uint8_t sdcard_files_to_open = 0;

SPIClass uSD_SPI(HSPI);
#endif /* CONFIG_IDF_TARGET_ESP32 */

#if defined(CONFIG_IDF_TARGET_ESP32)   || \
    defined(CONFIG_IDF_TARGET_ESP32S2) || \
    defined(CONFIG_IDF_TARGET_ESP32C3) || \
    defined(CONFIG_IDF_TARGET_ESP32C5) || \
    defined(CONFIG_IDF_TARGET_ESP32C6) || \
    defined(CONFIG_IDF_TARGET_ESP32P4)

/* variables hold file, state of process wav file and wav file properties */
wavProperties_t wavProps;

//i2s configuration
int i2s_num = 0; // i2s port number
i2s_config_t i2s_config = {
#if 1
     .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
#else
     .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_PDM),
#endif
     .sample_rate          = 22050,
     .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
     .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
     .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
     .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1, // high interrupt priority
     .dma_buf_count        = 8,
     .dma_buf_len          = 128   //Interrupt level 1
    };

#if 1
#if defined(CONFIG_IDF_TARGET_ESP32P4)
i2s_pin_config_t pin_config = {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
    .mck_io_num   = SOC_GPIO_PIN_MCK,
#endif
    .bck_io_num   = SOC_GPIO_PIN_BCK,
    .ws_io_num    = SOC_GPIO_PIN_LRCK,
    .data_out_num = SOC_GPIO_PIN_DATA,
    .data_in_num  = -1  // Not used
};
#else
i2s_pin_config_t pin_config = {
    .bck_io_num   = SOC_GPIO_PIN_BCLK,
    .ws_io_num    = SOC_GPIO_PIN_LRCLK,
    .data_out_num = SOC_GPIO_PIN_DOUT,
    .data_in_num  = -1  // Not used
};
#endif /* CONFIG_IDF_TARGET_ESP32P4 */
#else
i2s_pin_config_t pin_config = {
    .bck_io_num   = I2S_PIN_NO_CHANGE,
    .ws_io_num    = I2S_PIN_NO_CHANGE,
    .data_out_num = SOC_GPIO_PIN_PDM_OUT,
    .data_in_num  = I2S_PIN_NO_CHANGE
};
#endif
#endif /* CONFIG_IDF_TARGET_ESP32XX */


RTC_DATA_ATTR int bootCount          = 0;
static size_t ESP32_Min_AppPart_Size = 0;

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32P4)
//#define SPI_DRIVER_SELECT 3
#include <Adafruit_SPIFlash.h>
#include <Adafruit_INA219.h>
#include <uCDB.hpp>
#include <driver/rtc_io.h>
#include "BatteryHelper.h"

#if !defined(EXCLUDE_AUDIO)
#include "AudioFileSourceSdFat.h"
#include "AudioGeneratorWAV.h"
#include "AudioOutputI2S.h"
#endif /* EXCLUDE_AUDIO */

Adafruit_FlashTransport_ESP32 HWFlashTransport;
Adafruit_SPIFlash QSPIFlash(&HWFlashTransport);

static Adafruit_SPIFlash *SPIFlash = &QSPIFlash;

/// Flash device list count
enum {
  EXTERNAL_FLASH_DEVICE_COUNT
};

/// List of all possible flash devices used by ESP32 boards
static SPIFlash_Device_t possible_devices[] = { };

static bool ESP32_has_CPM       = false;
static bool ESP32_has_spiflash  = false;
static uint32_t spiflash_id     = 0;
static bool FATFS_is_mounted    = false;
static bool ADB_is_open         = false;

#if CONFIG_TINYUSB_MSC_ENABLED
  #if defined(USE_ADAFRUIT_MSC)
    #include "Adafruit_TinyUSB.h"

    // USB Mass Storage object
    Adafruit_USBD_MSC usb_msc;
  #else
    #include "USBMSC.h"

    // USB Mass Storage object
    USBMSC usb_msc;
  #endif /* USE_ADAFRUIT_MSC */
#endif /* CONFIG_TINYUSB_MSC_ENABLED */

// file system object from SdFat
FatVolume fatfs;

uCDB<FatVolume, File32> ucdb(fatfs);

Adafruit_INA219 ina219(INA219_ADDRESS_ALT);

#define isTimeToToggle() (millis() - status_LED_TimeMarker > 300)
static unsigned long status_LED_TimeMarker = 0;

#if !defined(EXCLUDE_AUDIO)
AudioGeneratorWAV    *Audio_Gen;
AudioFileSourceSdFat *Audio_Source;
AudioOutputI2S       *Audio_Sink;
#endif /* EXCLUDE_AUDIO */

#if CONFIG_TINYUSB_MSC_ENABLED
#if defined(USE_ADAFRUIT_MSC)
// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
static int32_t ESP32_msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return SPIFlash->readBlocks(lba, (uint8_t*) buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
static int32_t ESP32_msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return SPIFlash->writeBlocks(lba, buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
static void ESP32_msc_flush_cb (void)
{
  // sync with flash
  SPIFlash->syncBlocks();

  // clear file system's cache to force refresh
  fatfs.cacheClear();
}

#else

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
static int32_t ESP32_msc_read_cb (uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return SPIFlash->readBlocks(lba, offset, (uint8_t*) buffer, bufsize/512) ?
         bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
static int32_t ESP32_msc_write_cb (uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  int32_t rval = SPIFlash->writeBlocks(lba, offset, buffer, bufsize/512) ?
                 bufsize : -1;

#if 1
  // sync with flash
  SPIFlash->syncBlocks();

  // clear file system's cache to force refresh
  fatfs.cacheClear();
#endif

  return rval;
}
#endif /* USE_ADAFRUIT_MSC */
#endif /* CONFIG_TINYUSB_MSC_ENABLED */

const char *ESP32SX_Device_Manufacturer = SOFTRF_IDENT;
const char *ESP32SX_Device_Model = SKYVIEW_IDENT " Pico"; /* 303a:8133 */
const uint16_t ESP32SX_Device_Version = SKYVIEW_USB_FW_VERSION;

#endif /* CONFIG_IDF_TARGET_ESP32S3 */

#if defined(CONFIG_IDF_TARGET_ESP32P4)
#include "esp_check.h"
#include "es8311.h"

#define EXAMPLE_SAMPLE_RATE     11025
#define EXAMPLE_VOICE_VOLUME    75 // 0 - 100
#define EXAMPLE_MIC_GAIN        (es8311_mic_gain_t)(3) // 0 - 7

const char *TAG_ES83 = "esp32p4_i2s_es8311";

esp_err_t es8311_codec_init(const unsigned int i2c_num) {
    es8311_handle_t es_handle = es8311_create(i2c_num, ES8311_ADDRRES_0);
    ESP_RETURN_ON_FALSE(es_handle, ESP_FAIL, TAG_ES83, "es8311 create failed");
    const es8311_clock_config_t es_clk = {
        .mclk_inverted = false,
        .sclk_inverted = false,
        .mclk_from_mclk_pin = true,
        .mclk_frequency = EXAMPLE_SAMPLE_RATE * 256,
        .sample_frequency = EXAMPLE_SAMPLE_RATE
    };

    ESP_ERROR_CHECK(es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16));
    ESP_RETURN_ON_ERROR(es8311_sample_frequency_config(es_handle, es_clk.mclk_frequency, es_clk.sample_frequency), TAG_ES83, "set es8311 sample frequency failed");
    ESP_RETURN_ON_ERROR(es8311_microphone_config(es_handle, false), TAG_ES83, "set es8311 microphone failed");

    ESP_RETURN_ON_ERROR(es8311_voice_volume_set(es_handle, EXAMPLE_VOICE_VOLUME, NULL), TAG_ES83, "set es8311 volume failed");
    ESP_RETURN_ON_ERROR(es8311_microphone_gain_set(es_handle, EXAMPLE_MIC_GAIN), TAG_ES83, "set es8311 microphone gain failed");
    return ESP_OK;
}
#endif /* CONFIG_IDF_TARGET_ESP32P4 */

#if CONFIG_TINYUSB_ENABLED && \
    (defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3))
#include <USB.h>
#endif /* CONFIG_TINYUSB_ENABLED */

static uint32_t ESP32_getFlashId()
{
  return g_rom_flashchip.device_id;
}

#if defined(CORE_DEBUG_LEVEL) && CORE_DEBUG_LEVEL>0 && !defined(TAG_MAC)
#define TAG_MAC "MAC"
#endif

static void ESP32_setup()
{
  esp_err_t ret = ESP_OK;
  uint8_t null_mac[6] = {0};

  ++bootCount;

  ret = esp_efuse_mac_get_custom(efuse_mac);
  if (ret != ESP_OK) {
      ESP_LOGE(TAG_MAC, "Get base MAC address from BLK3 of EFUSE error (%s)", esp_err_to_name(ret));
    /* If get custom base MAC address error, the application developer can decide what to do:
     * abort or use the default base MAC address which is stored in BLK0 of EFUSE by doing
     * nothing.
     */

    ESP_LOGI(TAG_MAC, "Use base MAC address which is stored in BLK0 of EFUSE");
    chipmacid = ESP.getEfuseMac();
  } else {
    if (memcmp(efuse_mac, null_mac, 6) == 0) {
      ESP_LOGI(TAG_MAC, "Use base MAC address which is stored in BLK0 of EFUSE");
      chipmacid = ESP.getEfuseMac();
    }
  }

#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR>=5
  size_t flash_size;
  esp_flash_get_size(NULL, (uint32_t *) &flash_size);
#else
  size_t flash_size = spi_flash_get_chip_size();
#endif /* ESP_IDF_VERSION_MAJOR */

  size_t min_app_size = flash_size;

  esp_partition_iterator_t it;
  const esp_partition_t *part;

  it = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
  if (it) {
    do {
      part = esp_partition_get(it);
      if (part->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY) {
        continue;
      }
      if (part->size < min_app_size) {
        min_app_size = part->size;
      }
    } while (it = esp_partition_next(it));

    if (it) esp_partition_iterator_release(it);
  }

  if (min_app_size && (min_app_size != flash_size)) {
    ESP32_Min_AppPart_Size = min_app_size;
  }

  uint32_t flash_id = ESP32_getFlashId();

  /*
   *    Board         |   Module         |  Flash memory IC
   *  ----------------+------------------+--------------------
   *  DoIt ESP32      | WROOM            | GIGADEVICE_GD25Q32
   *  TTGO T3  V2.0   | PICO-D4 IC       | GIGADEVICE_GD25Q32
   *  TTGO T3  V2.1.6 | PICO-D4 IC       | GIGADEVICE_GD25Q32
   *  TTGO T22 V06    |                  | WINBOND_NEX_W25Q32_V
   *  TTGO T22 V08    |                  | WINBOND_NEX_W25Q32_V
   *  TTGO T22 V11    |                  | BOYA_BY25Q32AL
   *  TTGO T22 V12    |                  | WINBOND_NEX_W25Q32_V
   *  TTGO T8  V1.8   | WROVER           | GIGADEVICE_GD25LQ32
   *  TTGO T8 S2 V1.1 |                  | WINBOND_NEX_W25Q32_V
   *  TTGO T5S V1.9   |                  | WINBOND_NEX_W25Q32_V
   *  TTGO T5S V2.8   |                  | BOYA_BY25Q32AL
   *  TTGO T5  4.7    | WROVER-E         | XMC_XM25QH128C
   *  TTGO T-Watch    |                  | WINBOND_NEX_W25Q128_V
   *  Ai-T NodeMCU-S3 | ESP-S3-12K       | GIGADEVICE_GD25Q64C
   *  TTGO T-Dongle   |                  | BOYA_BY25Q32AL
   *  TTGO S3 Core    |                  | GIGADEVICE_GD25Q64C
   *  TTGO T-01C3     |                  | BOYA_BY25Q32AL
   *                  | ESP-C3-12F       | XMC_XM25QH32B
   *  LilyGO T-TWR    | WROOM-1-N16R8    | GIGADEVICE_GD25Q128
   *  Heltec Tracker  |                  | GIGADEVICE_GD25Q64
   *                  | WT0132C6-S5      | ZBIT_ZB25VQ32B
   *  LilyGO T3-C6    | ESP32-C6-MINI    | XMC_XM25QH32B
   *  LilyGO T3-S3-EP | ESP32-S3-MINI    | XMC_XM25QH32B
   *  LilyGO T3-S3-OL | ESP32-S3FH4R2    |
   *  Elecrow TN-M2   | ESP32-S3-N4R8    | ZBIT_ZB25VQ32B
   *  Elecrow TN-M5   | ESP32-S3-N4R8    |
   *  Ebyte EoRa-HUB  | ESP32-S3FH4R2    |
   *  WT99P4C5-S1 CPU | WT0132P4-A1      | ZBIT_ZB25VQ128ASIG
   *  WT99P4C5-S1 NCU | ESP32-C5-WROOM-1 | XMC_XM25QH64B
   */

  if (psramFound()) {
    switch(flash_id)
    {
    case MakeFlashId(GIGADEVICE_ID, GIGADEVICE_GD25LQ32):
      /* ESP32-WROVER module */
      hw_info.revision = HW_REV_T8_1_8;
      break;
    case MakeFlashId(ST_ID, XMC_XM25QH128C):
      /* custom ESP32-WROVER-E module with 16 MB flash */
      hw_info.revision = HW_REV_T5_1;
      break;
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    case MakeFlashId(WINBOND_NEX_ID, WINBOND_NEX_W25Q64_V): /* BPI */
    case MakeFlashId(WINBOND_NEX_ID, WINBOND_NEX_W25Q64_W): /* TBD */
      hw_info.revision = HW_REV_BPI;
      break;
    case MakeFlashId(GIGADEVICE_ID, GIGADEVICE_GD25Q64):
      hw_info.revision = HW_REV_DEVKIT;
      break;
#endif /* CONFIG_IDF_TARGET_ESP32S3 */
#if defined(CONFIG_IDF_TARGET_ESP32P4)
    case MakeFlashId(ZBIT_ID, ZBIT_ZB25VQ128A): /* WT0132P4-A1 ESP32-P4NRW32 */
      hw_info.revision = HW_REV_DEVKIT;
      break;
#endif /* CONFIG_IDF_TARGET_ESP32P4 */
    default:
      hw_info.revision = HW_REV_UNKNOWN;
      break;
    }
  } else {
    switch(flash_id)
    {
    case MakeFlashId(GIGADEVICE_ID, GIGADEVICE_GD25Q32):
      hw_info.revision = HW_REV_DEVKIT;
      break;
    case MakeFlashId(WINBOND_NEX_ID, WINBOND_NEX_W25Q32_V):
      hw_info.revision = HW_REV_T5S_1_9;
      break;
    case MakeFlashId(BOYA_ID, BOYA_BY25Q32AL):
      hw_info.revision = HW_REV_T5S_2_8;
      break;
    default:
      hw_info.revision = HW_REV_UNKNOWN;
      break;
    }
  }

#if defined(CONFIG_IDF_TARGET_ESP32S3)
  Wire.setPins(SOC_GPIO_PIN_SDA, SOC_GPIO_PIN_SCL);
  Wire.begin();
  Wire.beginTransmission(INA219_ADDRESS_ALT);
  ESP32_has_CPM = (Wire.endTransmission() == 0);
  Wire.end();

  if (ESP32_has_CPM) {
    ina219.begin(&Wire);
    ina219.setCalibration_16V_400mA();
  }

  ESP32_has_spiflash = SPIFlash->begin(possible_devices,
                                       EXTERNAL_FLASH_DEVICE_COUNT);
  if (ESP32_has_spiflash) {
    spiflash_id = SPIFlash->getJEDECID();

    uint32_t capacity = spiflash_id & 0xFF;
    if (capacity >= 0x17) { /* equal or greater than 1UL << 23 (8 MiB) */
#if CONFIG_TINYUSB_MSC_ENABLED
  #if defined(USE_ADAFRUIT_MSC)
      // Set disk vendor id, product id and revision
      // with string up to 8, 16, 4 characters respectively
      usb_msc.setID(ESP32SX_Device_Manufacturer, "Internal Flash", "1.0");

      // Set callback
      usb_msc.setReadWriteCallback(ESP32_msc_read_cb,
                                   ESP32_msc_write_cb,
                                   ESP32_msc_flush_cb);

      // Set disk size, block size should be 512 regardless of spi flash page size
      usb_msc.setCapacity(SPIFlash->size()/512, 512);

      // MSC is ready for read/write
      usb_msc.setUnitReady(true);

      usb_msc.begin();

  #else

      // Set disk vendor id, product id and revision
      // with string up to 8, 16, 4 characters respectively
      usb_msc.vendorID(ESP32SX_Device_Manufacturer);
      usb_msc.productID("Internal Flash");
      usb_msc.productRevision("1.0");

      // Set callback
      usb_msc.onRead(ESP32_msc_read_cb);
      usb_msc.onWrite(ESP32_msc_write_cb);

      // MSC is ready for read/write
      usb_msc.mediaPresent(true);

      // Set disk size, block size should be 512 regardless of spi flash page size
      usb_msc.begin(SPIFlash->size()/512, 512);
  #endif /* USE_ADAFRUIT_MSC */
#endif /* CONFIG_TINYUSB_MSC_ENABLED */

      FATFS_is_mounted = fatfs.begin(SPIFlash);
      hw_info.storage  = FATFS_is_mounted ? STORAGE_FLASH : STORAGE_NONE;
    }
  }

#if !defined(EXCLUDE_AUDIO)
  Audio_Gen    = new AudioGeneratorWAV();
  Audio_Source = new AudioFileSourceSdFat(fatfs);

#if defined(USE_EXT_I2S_DAC)
  Audio_Sink   = new AudioOutputI2S(0, AudioOutputI2S::EXTERNAL_I2S);
  Audio_Sink->SetPinout(SOC_GPIO_PIN_BCK,
                        SOC_GPIO_PIN_LRCK,
                        SOC_GPIO_PIN_DATA,
                        SOC_GPIO_PIN_MCK);
#else
  Audio_Sink   = new AudioOutputI2S(0, AudioOutputI2S::INTERNAL_PDM);
  Audio_Sink->SetPinout(I2S_PIN_NO_CHANGE,
                        I2S_PIN_NO_CHANGE,
                        SOC_GPIO_PIN_PDM_OUT,
                        I2S_PIN_NO_CHANGE);
#endif /* USE_EXT_I2S_DAC */

  Audio_Sink->SetOutputModeMono(true);
  Audio_Sink->SetChannels(1);
#if SOC_GPIO_PIN_MCK != I2S_PIN_NO_CHANGE
  Audio_Sink->SetMclk(true);
#else
  Audio_Sink->SetMclk(false);
#endif /* SOC_GPIO_PIN_MCK */
#endif /* EXCLUDE_AUDIO */

  pinMode(SOC_GPIO_PIN_LED, OUTPUT);
  /* Indicate positive power supply */
  digitalWrite(SOC_GPIO_PIN_LED, LED_STATE_ON);

#endif /* CONFIG_IDF_TARGET_ESP32S3 */

#if ARDUINO_USB_CDC_ON_BOOT && \
    (defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3))
#if CONFIG_TINYUSB_ENABLED
  if (USB.manufacturerName(ESP32SX_Device_Manufacturer)) {
    char usb_serial_number[16];
    uint16_t pid = 0x812B ; /* Banana Pi BPI-PicoW-S3 - Arduino */

    snprintf(usb_serial_number, sizeof(usb_serial_number),
             "%02X%02X%02X%02X%02X%02X",
             efuse_mac[0], efuse_mac[1], efuse_mac[2],
             efuse_mac[3], efuse_mac[4], efuse_mac[5]);

    USB.VID(USB_VID); // USB_ESPRESSIF_VID = 0x303A
    USB.PID(pid);
    USB.productName(ESP32SX_Device_Model);
    USB.firmwareVersion(ESP32SX_Device_Version);
    USB.serialNumber(usb_serial_number);
    USB.begin();
  }
#endif /* CONFIG_TINYUSB_ENABLED */

  Serial.begin(SERIAL_OUT_BR);

  for (int i=0; i < 20; i++) {if (Serial) break; else delay(100);}

#if 0 /* TBD */
  if (Serial.rebootEnabled()) {
    Serial.enableReboot(false);
  }
#endif /* TBD */

#else
#if ARDUINO_USB_CDC_ON_BOOT && \
    (defined(CONFIG_IDF_TARGET_ESP32C5) || \
     defined(CONFIG_IDF_TARGET_ESP32C6) || \
     defined(CONFIG_IDF_TARGET_ESP32P4))
  Serial.begin(SERIAL_OUT_BR);
#else
  Serial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
#endif /* CONFIG_IDF_TARGET_ESP32C6 */
#endif /* ARDUINO_USB_CDC_ON_BOOT && (CONFIG_IDF_TARGET_ESP32S2 || S3) */

#if defined(CONFIG_IDF_TARGET_ESP32P4)
  Wire.begin(SOC_GPIO_PIN_SDA, SOC_GPIO_PIN_SCL);

  Wire.beginTransmission(GT911_ADDRESS);
  if (Wire.endTransmission() == 0) hw_info.revision = HW_REV_DEVKIT;
  Wire.beginTransmission(GT911_ADDRESS_ALT);
  if (Wire.endTransmission() == 0) hw_info.revision = HW_REV_DEVKIT;
  Wire.beginTransmission(HI8561_ADDRESS);
  if (Wire.endTransmission() == 0) hw_info.revision = HW_REV_TDISPLAY_P4_TFT;
  // Wire.beginTransmission(GT9895_ADDRESS);
  // if (Wire.endTransmission() == 0) hw_info.revision = HW_REV_TDISPLAY_P4_AMOLED;

  switch (hw_info.revision)
  {
  case HW_REV_TDISPLAY_P4_TFT:
  case HW_REV_TDISPLAY_P4_AMOLED:

    // I2C #2 (ES8311, AW86224, SGM38121, ICM20948, Camera)
    Wire1.begin(SOC_GPIO_PIN_TDP4_SDA, SOC_GPIO_PIN_TDP4_SCL);

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
    pin_config.mck_io_num   = SOC_GPIO_PIN_TDP4_MCK;
#endif
    pin_config.bck_io_num   = SOC_GPIO_PIN_TDP4_BCK;
    pin_config.ws_io_num    = SOC_GPIO_PIN_TDP4_LR;
    pin_config.data_out_num = SOC_GPIO_PIN_TDP4_DO;

    es8311_codec_init(1);
    break;

  case HW_REV_DEVKIT:
  default:
    pinMode(SOC_GPIO_PIN_PAMP_EN,      OUTPUT);
    digitalWrite(SOC_GPIO_PIN_PAMP_EN, HIGH);

#if SOC_SDMMC_IO_POWER_EXTERNAL
    {
      esp_ldo_channel_handle_t ldo_sdio = NULL;
      esp_ldo_channel_config_t ldo_sdio_config = {
          .chan_id = BOARD_SDMMC_POWER_CHANNEL,
          .voltage_mv = 3300,
      };
      esp_ldo_acquire_channel(&ldo_sdio_config, &ldo_sdio);
    }
#endif /* SOC_SDMMC_IO_POWER_EXTERNAL */

    es8311_codec_init(0);
    break;
  }

#if !defined(EXCLUDE_ETHERNET)
  Ethernet_setup();

  ETH.begin(ETH_PHY_TYPE,
            ETH_PHY_ADDR,
            SOC_GPIO_PIN_ETH_MDC,
            SOC_GPIO_PIN_ETH_MDIO,
            SOC_GPIO_PIN_ETH_PWR,
            ETH_CLK_MODE);
#endif /* EXCLUDE_ETHERNET */

  /* SD-SPI init */
  uSD_SPI.begin(SOC_GPIO_PIN_SD_CLK,
                SOC_GPIO_PIN_SD_D0,
                SOC_GPIO_PIN_SD_CMD,
                SOC_GPIO_PIN_SD_D3);

#endif /* CONFIG_IDF_TARGET_ESP32P4 */
}

static void ESP32_post_init()
{
  uint32_t SerialBaud;

  switch (settings->baudrate)
  {
  case B4800:
    SerialBaud = 4800;
    break;
  case B9600:
    SerialBaud = 9600;
    break;
  case B19200:
    SerialBaud = 19200;
    break;
  case B57600:
    SerialBaud = 57600;
    break;
  case B115200:
    SerialBaud = 115200;
    break;
  case B2000000:
    SerialBaud = 2000000;
    break;
  case B38400:
  default:
    SerialBaud = 38400;
    break;
  }

  Serial.println();

  Serial.print(F("Board        : "));

  switch (hw_info.revision)
  {
  case HW_REV_T5S_1_9    : Serial.println(F("LilyGO T5S"));   break;
  case HW_REV_T5S_2_8    : Serial.println(F("LilyGO T5S"));   break;
  case HW_REV_BPI        : Serial.println(F("Banana PicoW")); break;
  case HW_REV_DEVKIT     : Serial.print(SoC->name);
                           Serial.println(F(" DevKit"));      break;
  default                : Serial.println(F("OTHER"));        break;
  }

  Serial.print(F("Display      : "));

  if (hw_info.display == DISPLAY_TFT_7_0) {
    Serial.println(F("7 inch TFT"));
  } else if (hw_info.display != DISPLAY_EPD_2_7 || display == NULL) {
    Serial.println(F("NONE"));
  } else {
    switch (display->epd2.panel)
    {
    case GxEPD2::GDEY027T91: Serial.println(F("GDEY027T91")); break;
    case GxEPD2::GDEW027W3 : Serial.println(F("GDEW027W3"));  break;
    default                : Serial.println(F("OTHER"));      break;
    }
  }

  Serial.println();

  Serial.print(F("Input source : "));
  switch (settings->connection)
  {
    case CON_SERIAL_MAIN   : Serial.print  (F("UART MAIN "));
                             Serial.println(SerialBaud);    break;
    case CON_SERIAL_AUX    : Serial.print  (F("UART AUX "));
                             Serial.println(SerialBaud);    break;
    case CON_USB           : Serial.println(F("USB"));      break;
    case CON_WIFI_UDP      : Serial.println(F("WIFI UDP")); break;
    case CON_WIFI_TCP      : Serial.println(F("WIFI TCP")); break;
    case CON_BLUETOOTH_SPP : Serial.println(F("BT SPP"));   break;
    case CON_BLUETOOTH_LE  : Serial.println(F("BT LE"));    break;
    case CON_NONE          :
    default                : Serial.println(F("NONE"));     break;
  }

  Serial.print(F("Protocol     : "));
  switch (settings->protocol)
  {
    case PROTOCOL_NMEA     : Serial.println(F("NMEA"));     break;
    case PROTOCOL_GDL90    : Serial.println(F("GDL90"));    break;
    case PROTOCOL_D1090    : Serial.println(F("D1090"));    break;
    case PROTOCOL_NONE     :
    default                : Serial.println(F("NONE"));     break;
  }

  Serial.println();
  Serial.flush();
}

static void ESP32_loop()
{
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  if (Battery_voltage() > Battery_threshold()) {
    /* Indicate positive power supply */
    if (digitalRead(SOC_GPIO_PIN_LED) != LED_STATE_ON) {
      digitalWrite(SOC_GPIO_PIN_LED, LED_STATE_ON);
    }
  } else {
    if (isTimeToToggle()) {
      digitalWrite(SOC_GPIO_PIN_LED, !digitalRead(SOC_GPIO_PIN_LED) ?
                                     HIGH : LOW);  // toggle state
      status_LED_TimeMarker = millis();
    }
  }
#endif /* CONFIG_IDF_TARGET_ESP32S3 */

#if defined(CONFIG_IDF_TARGET_ESP32P4)
#if !defined(EXCLUDE_ETHERNET)
  Ethernet_loop();
#endif /* EXCLUDE_ETHERNET */
#endif /* CONFIG_IDF_TARGET_ESP32P4 */
}

static void ESP32_fini()
{
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  if (ESP32_has_spiflash) {
#if CONFIG_TINYUSB_MSC_ENABLED
  #if defined(USE_ADAFRUIT_MSC)
    usb_msc.setUnitReady(false);
//  usb_msc.end(); /* N/A */
  #else
    usb_msc.mediaPresent(false);
    usb_msc.end();
  #endif /* USE_ADAFRUIT_MSC */
#endif /* CONFIG_TINYUSB_MSC_ENABLED */
  }

  if (SPIFlash != NULL) SPIFlash->end();

  if (ESP32_has_CPM) {
    ina219.powerSave(true);
    Wire.end();
  }

  pinMode(SOC_GPIO_PIN_LED, INPUT);
#endif /* CONFIG_IDF_TARGET_ESP32S3 */

#if defined(CONFIG_IDF_TARGET_ESP32P4)
#if !defined(EXCLUDE_ETHERNET)
  ETH.end();

  Ethernet_fini();
#endif /* EXCLUDE_ETHERNET */
#endif /* CONFIG_IDF_TARGET_ESP32P4 */

  int wake_gpio_num = SOC_BUTTON_MODE_DEF;

#if defined(CONFIG_IDF_TARGET_ESP32)
  if (settings && (settings->adapter == ADAPTER_TTGO_T5S)) {
    uSD_SPI.end();
    wake_gpio_num = SOC_BUTTON_MODE_T5S;
  }
#endif /* CONFIG_IDF_TARGET_ESP32 */

#if defined(CONFIG_IDF_TARGET_ESP32S3)
  if (settings &&
      (settings->adapter == ADAPTER_WAVESHARE_PICO_2_7 ||
       settings->adapter == ADAPTER_WAVESHARE_PICO_2_7_V2)) {
    wake_gpio_num = SOC_GPIO_PIN_KEY1; // RTC GPIO

    if (rtc_gpio_is_valid_gpio((gpio_num_t) wake_gpio_num)) {
      esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
      rtc_gpio_pulldown_dis((gpio_num_t) wake_gpio_num);
      rtc_gpio_pullup_en((gpio_num_t) wake_gpio_num);
    }
  }
#endif /* CONFIG_IDF_TARGET_ESP32S3 */

  esp_wifi_stop();

#if defined(CONFIG_IDF_TARGET_ESP32)
  esp_bt_controller_disable();
#endif /* CONFIG_IDF_TARGET_ESP32 */

//  SPI.end(); // done in EPD_fini

  /*
   * manually apply this fix onto Arduino Core for ESP32:
   * https://github.com/espressif/arduino-esp32/pull/4272
   * to put SD card into idle state
   *
   *  SkyView EZ sleep current (from 3.7V battery source):
   *  ---------------------------------------------------
   *  SD card in  -            0.2 mA
   *  SD card out -            0.1 mA
   */
#if !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32C5) && \
    !defined(CONFIG_IDF_TARGET_ESP32C6)
  esp_sleep_enable_ext1_wakeup(1ULL << wake_gpio_num, ESP_EXT1_WAKEUP_ALL_LOW);
#endif /* CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C6 */

//  Serial.println("Going to sleep now");
//  Serial.flush();

  esp_deep_sleep_start();
}

static void ESP32_reset()
{
  ESP.restart();
}

static uint32_t ESP32_getChipId()
{
  return (uint32_t) efuse_mac[5]        | (efuse_mac[4] << 8) | \
                   (efuse_mac[3] << 16) | (efuse_mac[2] << 24);
}

static uint32_t ESP32_getFreeHeap()
{
  return ESP.getFreeHeap();
}

static bool ESP32_EEPROM_begin(size_t size)
{
  return EEPROM.begin(size);
}

static void ESP32_EEPROM_extension(int cmd)
{
  /* TBD */
}

static const int8_t ESP32_dB_to_power_level[21] = {
  8,  /* 2    dB, #0 */
  8,  /* 2    dB, #1 */
  8,  /* 2    dB, #2 */
  8,  /* 2    dB, #3 */
  8,  /* 2    dB, #4 */
  20, /* 5    dB, #5 */
  20, /* 5    dB, #6 */
  28, /* 7    dB, #7 */
  28, /* 7    dB, #8 */
  34, /* 8.5  dB, #9 */
  34, /* 8.5  dB, #10 */
  44, /* 11   dB, #11 */
  44, /* 11   dB, #12 */
  52, /* 13   dB, #13 */
  52, /* 13   dB, #14 */
  60, /* 15   dB, #15 */
  60, /* 15   dB, #16 */
  68, /* 17   dB, #17 */
  74, /* 18.5 dB, #18 */
  76, /* 19   dB, #19 */
  78  /* 19.5 dB, #20 */
};

static void ESP32_WiFi_setOutputPower(int dB)
{
  if (dB > 20) {
    dB = 20;
  }

  if (dB < 0) {
    dB = 0;
  }

  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(ESP32_dB_to_power_level[dB]));
}

static bool ESP32_WiFi_hostname(String aHostname)
{
  return WiFi.setHostname(aHostname.c_str());
}

static void ESP32_swSer_begin(unsigned long baud)
{
  SerialInput.begin(baud, SERIAL_8N1, SOC_GPIO_PIN_GNSS_RX, SOC_GPIO_PIN_GNSS_TX);
  SerialInput.setRxBufferSize(baud / 10); /* 1 second */
}

static void ESP32_swSer_enableRx(boolean arg)
{

}

static uint32_t ESP32_maxSketchSpace()
{
  return ESP32_Min_AppPart_Size ?
         ESP32_Min_AppPart_Size : 0x1E0000; /* min_spiffs.csv */
}

static void ESP32_WiFiUDP_stopAll()
{
/* not implemented yet */
}

static void ESP32_Battery_setup()
{
#if !defined(ESP_IDF_VERSION_MAJOR) || ESP_IDF_VERSION_MAJOR < 5
#if defined(CONFIG_IDF_TARGET_ESP32)
  calibrate_voltage(settings->adapter == ADAPTER_TTGO_T5S ?
                    (adc1_channel_t) ADC1_GPIO35_CHANNEL  :
                    (adc1_channel_t) ADC1_GPIO36_CHANNEL);
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
  calibrate_voltage((adc1_channel_t) ADC1_GPIO9_CHANNEL); /* TBD */
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  calibrate_voltage((adc1_channel_t) ADC1_GPIO3_CHANNEL);
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  calibrate_voltage((adc1_channel_t) ADC1_GPIO1_CHANNEL); /* TBD */
#else
#error "This ESP32 family build variant is not supported!"
#endif /* CONFIG_IDF_TARGET_ESP32 */
#else
#if defined(CONFIG_IDF_TARGET_ESP32C5) || defined(CONFIG_IDF_TARGET_ESP32P4)
  calibrate_voltage(SOC_GPIO_PIN_BATTERY);
#elif defined(CONFIG_IDF_TARGET_ESP32C6)  || \
      defined(CONFIG_IDF_TARGET_ESP32C61) || \
      defined(CONFIG_IDF_TARGET_ESP32H2)
  /* TBD */
#else
#error "This ESP32 family build variant is not supported!"
#endif /* CONFIG_IDF_TARGET_ESP32C5 */
#endif /* ESP_IDF_VERSION_MAJOR */
}

static float ESP32_Battery_voltage()
{
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  if (ESP32_has_CPM) {
    float shuntvoltage = ina219.getShuntVoltage_mV();
    float busvoltage   = ina219.getBusVoltage_V();

    return (busvoltage + (shuntvoltage / 1000));
  }
#endif /* CONFIG_IDF_TARGET_ESP32S3 */

  float voltage = ((float) read_voltage()) * 0.001 ;

  /* T5 has voltage divider 100k/100k on board */
  return (settings->adapter == ADAPTER_TTGO_T5S    ||
          settings->adapter == ADAPTER_TTGO_T5_4_7 ?
          2 * voltage : voltage);
}

#include <SoftSPI.h>
SoftSPI swSPI(SOC_GPIO_PIN_MOSI_WS,
              SOC_GPIO_PIN_MOSI_WS, /* half duplex */
              SOC_GPIO_PIN_SCK_WS);

#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR>=5
static spinlock_t EPD_ident_mutex;
#else
static portMUX_TYPE EPD_ident_mutex;
#endif /* ESP_IDF_VERSION_MAJOR */

static ep_model_id ESP32_EPD_ident()
{
  ep_model_id rval = EP_GDEW027W3; /* default */

#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR>=5
  spinlock_initialize(&EPD_ident_mutex);
#else
  vPortCPUInitializeMutex(&EPD_ident_mutex);
#endif /* ESP_IDF_VERSION_MAJOR */

  digitalWrite(SOC_GPIO_PIN_SS_WS, HIGH);
  pinMode(SOC_GPIO_PIN_SS_WS, OUTPUT);
  digitalWrite(SOC_EPD_PIN_DC_WS, HIGH);
  pinMode(SOC_EPD_PIN_DC_WS, OUTPUT);

  digitalWrite(SOC_EPD_PIN_RST_WS, LOW);
  pinMode(SOC_EPD_PIN_RST_WS, OUTPUT);
  delay(20);
  pinMode(SOC_EPD_PIN_RST_WS, INPUT_PULLUP);
  delay(200);
  pinMode(SOC_EPD_PIN_BUSY_WS, INPUT);

  swSPI.begin();

  uint8_t buf_2D[11];
  uint8_t buf_2E[10];

  taskENTER_CRITICAL(&EPD_ident_mutex);

  digitalWrite(SOC_EPD_PIN_DC_WS,  LOW);
  digitalWrite(SOC_GPIO_PIN_SS_WS, LOW);

  swSPI.transfer_out(0x2D);

  pinMode(SOC_GPIO_PIN_MOSI_WS, INPUT);
  digitalWrite(SOC_EPD_PIN_DC_WS, HIGH);

  for (int i=0; i<sizeof(buf_2D); i++) {
    buf_2D[i] = swSPI.transfer_in();
  }

  digitalWrite(SOC_GPIO_PIN_SCK_WS, LOW);
  digitalWrite(SOC_EPD_PIN_DC_WS,  LOW);
  digitalWrite(SOC_GPIO_PIN_SS_WS,  HIGH);

  taskEXIT_CRITICAL(&EPD_ident_mutex);

  delay(1);

  taskENTER_CRITICAL(&EPD_ident_mutex);

  digitalWrite(SOC_EPD_PIN_DC_WS,  LOW);
  digitalWrite(SOC_GPIO_PIN_SS_WS, LOW);

  swSPI.transfer_out(0x2E);

  pinMode(SOC_GPIO_PIN_MOSI_WS, INPUT);
  digitalWrite(SOC_EPD_PIN_DC_WS, HIGH);

  for (int i=0; i<sizeof(buf_2E); i++) {
    buf_2E[i] = swSPI.transfer_in();
  }

  digitalWrite(SOC_GPIO_PIN_SCK_WS, LOW);
  digitalWrite(SOC_EPD_PIN_DC_WS,  LOW);
  digitalWrite(SOC_GPIO_PIN_SS_WS,  HIGH);

  taskEXIT_CRITICAL(&EPD_ident_mutex);

  swSPI.end();

#if 0
  Serial.println();

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
 *  FF FF FF FF FF FF FF FF FF FF FF - W3
 *  00 00 00 FF 00 00 40 01 00 00 00
 *  00 00 40 20 10 00 40 01 00 00 00 - T91
 *
 *  0x2E:
 *  FF FF FF FF FF FF FF FF FF FF    - W3
 *  FF FF FF FF FF FF FF FF FF FF
 *  FF FF FF FF FF FF FF FF FF FF    - T91
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

  if (is_ff || is_00) {
    rval = EP_GDEW027W3;
  } else {
    rval = EP_GDEY027T91;
  }

  return rval;
}

#define EPD_STACK_SZ      (256*6)
static TaskHandle_t EPD_Task_Handle = NULL;

static ep_model_id ESP32_display = EP_UNKNOWN;

#if defined(USE_TFT)

#include <esp_display_panel.hpp>

using namespace esp_panel::drivers;
using namespace esp_panel::board;

extern Board *panel;

#undef _TO_STR
#undef TO_STR
#define _TO_STR(name) #name
#define TO_STR(name) _TO_STR(name)

const BoardConfig Board_Config_WTP4C5MP07S = {
    .name = "WTP4C5MP07S",

    .lcd = BoardConfig::LCD_Config{

        .bus_config = BusDSI::Config{
            .host = BusDSI::HostPartialConfig{
                .num_data_lanes = 2,
                .lane_bit_rate_mbps = 1000,
            },
            .refresh_panel = BusDSI::RefreshPanelPartialConfig{
                .dpi_clock_freq_mhz = 52,
                .bits_per_pixel = ESP_PANEL_LCD_COLOR_BITS_RGB565,
                .h_size = 1024,
                .v_size = 600,
                .hsync_pulse_width = 10,
                .hsync_back_porch = 160,
                .hsync_front_porch = 160,
                .vsync_pulse_width = 1,
                .vsync_back_porch = 23,
                .vsync_front_porch = 12,
            },
            .phy_ldo = BusDSI::PHY_LDO_PartialConfig{
                .chan_id = 3
            },
        },
        .device_name = TO_STR(EK79007),
        .device_config = {
            .device = LCD::DevicePartialConfig{
                .reset_gpio_num = SOC_GPIO_PIN_LCD_RST,
                .rgb_ele_order = 0,
                .bits_per_pixel = ESP_PANEL_LCD_COLOR_BITS_RGB565,
                .flags_reset_active_high = 1,
            },
            .vendor = LCD::VendorPartialConfig{
                .hor_res = 1024,
                .ver_res = 600,
            },
        },
        .pre_process = {
            .invert_color = 0,
        },
    },

    .touch = BoardConfig::TouchConfig{
        .bus_config = BusI2C::Config{
            .host_id = 0,
            .host = BusI2C::HostPartialConfig{
                .sda_io_num = SOC_GPIO_PIN_SDA,
                .scl_io_num = SOC_GPIO_PIN_SCL,
                .sda_pullup_en = 0,
                .scl_pullup_en = 0,
                .clk_speed = 400 * 1000,
            },
            .control_panel = BusI2C::ControlPanelFullConfig
                ESP_PANEL_TOUCH_I2C_CONTROL_PANEL_CONFIG_WITH_ADDR(GT911, 0x5D),
        },
        .device_name = TO_STR(GT911),
        .device_config = {
            .device = Touch::DevicePartialConfig{
                .x_max = 1024,
                .y_max = 600,
                .rst_gpio_num = -1, // SOC_GPIO_PIN_TP_RST
                .int_gpio_num = SOC_GPIO_PIN_TP_INT,
                .levels_reset = 1,
                .levels_interrupt = 0,
            },
        },
        .pre_process = {
            .swap_xy = 0,
            .mirror_x = 1,
            .mirror_y = 1,
        },
    },

    .backlight = BoardConfig::BacklightConfig{
        .config = BacklightSwitchGPIO::Config{
            .io_num = SOC_GPIO_PIN_LCD_BLED,
            .on_level = 1,
        },
        .pre_process = {
            .idle_off = 0,
        },
    },

    .stage_callbacks = {
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
    },
};

const BoardConfig Board_Config_LilyGO_TDP4_TFT = {
    .name = "T-DISPLAY-P4-TFT",

    .lcd = BoardConfig::LCD_Config{

        .bus_config = BusDSI::Config{
            .host = BusDSI::HostPartialConfig{
                .num_data_lanes = 2,
                .lane_bit_rate_mbps = 1000,
            },
            .refresh_panel = BusDSI::RefreshPanelPartialConfig{
                .dpi_clock_freq_mhz = 52,
                .bits_per_pixel = ESP_PANEL_LCD_COLOR_BITS_RGB565, /* TBD */
                .h_size = 1168,
                .v_size = 540,
                .hsync_pulse_width = 10,
                .hsync_back_porch = 160,
                .hsync_front_porch = 160,
                .vsync_pulse_width = 1,
                .vsync_back_porch = 23,
                .vsync_front_porch = 12,
            },
            .phy_ldo = BusDSI::PHY_LDO_PartialConfig{
                .chan_id = 3
            },
        },
        .device_name = TO_STR(HI8561),
        .device_config = {
            .device = LCD::DevicePartialConfig{
                .reset_gpio_num = -1, /* XL 2 */
                .rgb_ele_order = 0,
                .bits_per_pixel = ESP_PANEL_LCD_COLOR_BITS_RGB565, /* TBD */
                .flags_reset_active_high = 1, /* TBD */
            },
            .vendor = LCD::VendorPartialConfig{
                .hor_res = 1168,
                .ver_res = 540,
            },
        },
        .pre_process = {
            .invert_color = 0,
        },
    },

    .touch = BoardConfig::TouchConfig{
        .bus_config = BusI2C::Config{
            .host_id = 0,
            .host = BusI2C::HostPartialConfig{
                .sda_io_num = SOC_GPIO_PIN_SDA,
                .scl_io_num = SOC_GPIO_PIN_SCL,
                .sda_pullup_en = 0,
                .scl_pullup_en = 0,
                .clk_speed = 400 * 1000,
            },
            .control_panel = BusI2C::ControlPanelFullConfig
                ESP_PANEL_TOUCH_I2C_CONTROL_PANEL_CONFIG(GT911), /* HI8561 */
        },
        .device_name = TO_STR(GT911),/* HI8561 */
        .device_config = {
            .device = Touch::DevicePartialConfig{
                .x_max = 1168,
                .y_max = 540,
                .rst_gpio_num = -1, /* XL 3 */
                .int_gpio_num = -1, /* XL 4 */
                .levels_reset = 0,
                .levels_interrupt = 0,
            },
        },
        .pre_process = {
            .swap_xy = 0,
            .mirror_x = 1,
            .mirror_y = 1,
        },
    },

    .backlight = BoardConfig::BacklightConfig{
        .config = BacklightSwitchGPIO::Config{
            .io_num = SOC_GPIO_PIN_TDP4_BL,
            .on_level = 1,
        },
        .pre_process = {
            .idle_off = 0,
        },
    },

    .stage_callbacks = {
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
    },
};

const BoardConfig Board_Config_LilyGO_TDP4_AMOLED = {
    .name = "T-DISPLAY-P4-AMOLED",

    .lcd = BoardConfig::LCD_Config{

        .bus_config = BusDSI::Config{
            .host = BusDSI::HostPartialConfig{
                .num_data_lanes = 2,
                .lane_bit_rate_mbps = 1000,
            },
            .refresh_panel = BusDSI::RefreshPanelPartialConfig{
                .dpi_clock_freq_mhz = 52,
                .bits_per_pixel = ESP_PANEL_LCD_COLOR_BITS_RGB565, /* TBD */
                .h_size = 1232,
                .v_size = 568,
                .hsync_pulse_width = 10,
                .hsync_back_porch = 160,
                .hsync_front_porch = 160,
                .vsync_pulse_width = 1,
                .vsync_back_porch = 23,
                .vsync_front_porch = 12,
            },
            .phy_ldo = BusDSI::PHY_LDO_PartialConfig{
                .chan_id = 3
            },
        },
        .device_name = TO_STR(RM69A10),
        .device_config = {
            .device = LCD::DevicePartialConfig{
                .reset_gpio_num = -1, /* XL 2 */
                .rgb_ele_order = 0,
                .bits_per_pixel = ESP_PANEL_LCD_COLOR_BITS_RGB565, /* TBD */
                .flags_reset_active_high = 1, /* TBD */
            },
            .vendor = LCD::VendorPartialConfig{
                .hor_res = 1232,
                .ver_res = 568,
            },
        },
        .pre_process = {
            .invert_color = 0,
        },
    },

    .touch = BoardConfig::TouchConfig{
        .bus_config = BusI2C::Config{
            .host_id = 0,
            .host = BusI2C::HostPartialConfig{
                .sda_io_num = SOC_GPIO_PIN_SDA,
                .scl_io_num = SOC_GPIO_PIN_SCL,
                .sda_pullup_en = 0,
                .scl_pullup_en = 0,
                .clk_speed = 400 * 1000,
            },
            .control_panel = BusI2C::ControlPanelFullConfig
                ESP_PANEL_TOUCH_I2C_CONTROL_PANEL_CONFIG(GT911), /* GT9895 */
        },
        .device_name = TO_STR(GT911), /* GT9895 */
        .device_config = {
            .device = Touch::DevicePartialConfig{
                .x_max = 1232,
                .y_max = 568,
                .rst_gpio_num = -1, /* XL 3 */
                .int_gpio_num = -1, /* XL 4 */
                .levels_reset = 0,
                .levels_interrupt = 0,
            },
        },
        .pre_process = {
            .swap_xy = 0,
            .mirror_x = 1,
            .mirror_y = 1,
        },
    },

    .backlight = BoardConfig::BacklightConfig{
        .config = BacklightSwitchGPIO::Config{
            .io_num = -1, /* AMOLED */
            .on_level = 1,
        },
        .pre_process = {
            .idle_off = 0,
        },
    },

    .stage_callbacks = {
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
    },
};
#endif /* USE_TFT */

static byte ESP32_Display_setup(bool splash_screen)
{
  switch(settings->adapter)
  {
  case ADAPTER_WAVESHARE_ESP32:
  case ADAPTER_WAVESHARE_PICO_2_7:
#if defined(CONFIG_IDF_TARGET_ESP32P4)
  case ADAPTER_WAVESHARE_PI_HAT_2_7:
#endif /* CONFIG_IDF_TARGET_ESP32P4 */
    if (ESP32_display == EP_UNKNOWN) {
      ESP32_display = ESP32_EPD_ident();
    }

    switch (ESP32_display)
    {
    case EP_GDEY027T91:
      display = &epd_waveshare_T91;
      break;
    case EP_GDEW027W3:
    default:
      display = &epd_waveshare_W3;
      break;
    }

    display->epd2.selectSPI(SPI, SPISettings(4000000, MSBFIRST, SPI_MODE0));
    SPI.begin(SOC_GPIO_PIN_SCK_WS,
              SOC_GPIO_PIN_MISO_WS,
              SOC_GPIO_PIN_MOSI_WS,
              SOC_GPIO_PIN_SS_WS);
    break;
  case ADAPTER_WAVESHARE_PICO_2_7_V2:
#if defined(CONFIG_IDF_TARGET_ESP32P4)
  case ADAPTER_WAVESHARE_PI_HAT_2_7_V2:
#endif /* CONFIG_IDF_TARGET_ESP32P4 */
    display = &epd_waveshare_T91;
    display->epd2.selectSPI(SPI, SPISettings(4000000, MSBFIRST, SPI_MODE0));
    SPI.begin(SOC_GPIO_PIN_SCK_WS,
              SOC_GPIO_PIN_MISO_WS,
              SOC_GPIO_PIN_MOSI_WS,
              SOC_GPIO_PIN_SS_WS);
    break;
#if defined(BUILD_SKYVIEW_HD)
  case ADAPTER_TTGO_T5_4_7:
    display = NULL;
    break;
#endif /* BUILD_SKYVIEW_HD */
  case ADAPTER_TTGO_T5S:
  default:
    display = &epd_ttgo_t5s_W3;
    display->epd2.selectSPI(SPI, SPISettings(4000000, MSBFIRST, SPI_MODE0));

    SPI.begin(SOC_GPIO_PIN_SCK_T5S,
              SOC_GPIO_PIN_MISO_T5S,
              SOC_GPIO_PIN_MOSI_T5S,
              SOC_GPIO_PIN_SS_T5S);

#if defined(CONFIG_IDF_TARGET_ESP32)
    /* SD-SPI init */
    uSD_SPI.begin(SOC_SD_PIN_SCK_T5S,
                  SOC_SD_PIN_MISO_T5S,
                  SOC_SD_PIN_MOSI_T5S,
                  SOC_SD_PIN_SS_T5S);
#endif /* CONFIG_IDF_TARGET_ESP32 */
    break;
  }

  xTaskCreateUniversal(EPD_Task, "EPD update", EPD_STACK_SZ, NULL, 1,
                       &EPD_Task_Handle, CONFIG_ARDUINO_RUNNING_CORE);

  byte rval = EPD_setup(splash_screen);

#if defined(USE_TFT)
  if (rval == DISPLAY_NONE) {

    if( EPD_Task_Handle != NULL )
    {
      vTaskDelete( EPD_Task_Handle );
    }

    switch (hw_info.revision)
    {
    case HW_REV_TDISPLAY_P4_TFT:
      panel = new Board(Board_Config_LilyGO_TDP4_TFT);
      break;
    case HW_REV_TDISPLAY_P4_AMOLED:
      panel = new Board(Board_Config_LilyGO_TDP4_AMOLED);
      break;
    case HW_REV_DEVKIT:
    default:
      panel = new Board(Board_Config_WTP4C5MP07S);
      break;
    }

    panel->init();

#if LVGL_PORT_AVOID_TEARING_MODE
    auto lcd = panel->getLCD();
    lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
#endif

    static_cast<esp_panel::drivers::BusI2C *>(panel->getTouch()->getBus())->configI2C_HostSkipInit();

    assert(panel->begin());

    rval = TFT_setup();
  }
#endif /* USE_TFT */

  return rval;
}

static void ESP32_Display_loop()
{
  switch (hw_info.display)
  {
#if defined(USE_TFT)
  case DISPLAY_TFT_7_0:
    TFT_loop();
    break;
#endif /* USE_TFT */
  case DISPLAY_EPD_2_7:
    EPD_loop();
    break;
  case DISPLAY_NONE:
  default:
    break;
  }
}

static void ESP32_Display_fini(const char *msg, bool screen_saver)
{
  switch (hw_info.display)
  {
#if defined(USE_TFT)
  case DISPLAY_TFT_7_0:
    TFT_fini();
    break;
#endif /* USE_TFT */
  case DISPLAY_EPD_2_7:
    if( EPD_Task_Handle != NULL )
    {
      vTaskDelete( EPD_Task_Handle );
    }

    EPD_fini(msg, screen_saver);
    break;
  case DISPLAY_NONE:
  default:
    break;
  }
}

static bool ESP32_Display_is_ready()
{
  switch (hw_info.display)
  {
#if defined(USE_TFT)
  case DISPLAY_TFT_7_0:
    /* TBD */
    break;
#endif /* USE_TFT */
  case DISPLAY_EPD_2_7:
//    return true;
    return (EPD_task_command == EPD_UPDATE_NONE);
    break;
  case DISPLAY_NONE:
  default:
    break;
  }

  return true;
}

static void ESP32_Display_update(int val)
{
  switch (hw_info.display)
  {
#if defined(USE_TFT)
  case DISPLAY_TFT_7_0:
    /* TBD */
    break;
#endif /* USE_TFT */
  case DISPLAY_EPD_2_7:
//    EPD_Update_Sync(val);
    EPD_task_command = val;
    break;
  case DISPLAY_NONE:
  default:
    break;
  }
}

static size_t ESP32_WiFi_Receive_UDP(uint8_t *buf, size_t max_size)
{
  return WiFi_Receive_UDP(buf, max_size);
}

static int ESP32_WiFi_clients_count()
{
  WiFiMode_t mode = WiFi.getMode();

  switch (mode)
  {
  case WIFI_AP:
    wifi_sta_list_t stations;
    ESP_ERROR_CHECK(esp_wifi_ap_get_sta_list(&stations));

#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR>=5
    /* TBD */

    return stations.num;
#else
    tcpip_adapter_sta_list_t infoList;
    ESP_ERROR_CHECK(tcpip_adapter_get_sta_list(&stations, &infoList));

    return infoList.num;
#endif /* ESP_IDF_VERSION_MAJOR */
  case WIFI_STA:
  default:
    return -1; /* error */
  }
}

static bool ESP32_DB_init()
{
  bool rval = false;

  switch (settings->adapter)
  {
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32P4)
  case ADAPTER_TTGO_T5S:
#if defined(CONFIG_IDF_TARGET_ESP32P4)
  case ADAPTER_WAVESHARE_PI_HAT_2_7:
  case ADAPTER_WAVESHARE_PI_HAT_2_7_V2:
#endif /* CONFIG_IDF_TARGET_ESP32P4 */
#if !defined(BUILD_SKYVIEW_HD)
    {
      sdcard_files_to_open += (settings->adb   == DB_FLN    ? 1 : 0);
      sdcard_files_to_open += (settings->adb   == DB_OGN    ? 1 : 0);
      sdcard_files_to_open += (settings->adb   == DB_ICAO   ? 1 : 0);
      sdcard_files_to_open += (settings->voice != VOICE_OFF ? 1 : 0);

#if defined(CONFIG_IDF_TARGET_ESP32)
      int uSD_SS_pin = SOC_SD_PIN_SS_T5S;
#endif /* CONFIG_IDF_TARGET_ESP32 */
#if defined(CONFIG_IDF_TARGET_ESP32P4)
      int uSD_SS_pin = SOC_GPIO_PIN_SD_D3;
#endif /* CONFIG_IDF_TARGET_ESP32P4 */

      if (!SD.begin(uSD_SS_pin, uSD_SPI, 4000000, "/sd", sdcard_files_to_open)) {
        Serial.println(F("ERROR: Failed to mount microSD card."));
        return rval;
      }

      if (settings->adb == DB_NONE) {
        return rval;
      }

      sqlite3_initialize();

      if (settings->adb == DB_FLN) {
        sqlite3_open("/sd/Aircrafts/fln.db", &fln_db);

        if (fln_db == NULL)
        {
          Serial.println(F("Failed to open FlarmNet DB\n"));
        }  else {
          rval = true;
        }
      }

      if (settings->adb == DB_OGN) {
        sqlite3_open("/sd/Aircrafts/ogn.db", &ogn_db);

        if (ogn_db == NULL)
        {
          Serial.println(F("Failed to open OGN DB\n"));
        }  else {
          rval = true;
        }
      }

      if (settings->adb == DB_ICAO) {
        sqlite3_open("/sd/Aircrafts/icao.db", &icao_db);

        if (icao_db == NULL)
        {
          Serial.println(F("Failed to open ICAO DB\n"));
        }  else {
          rval = true;
        }
      }
    }
#endif /* BUILD_SKYVIEW_HD */
    break;
#endif /* CONFIG_IDF_TARGET_ESP32 */
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  case ADAPTER_WAVESHARE_PICO_2_7:
  case ADAPTER_WAVESHARE_PICO_2_7_V2:
#if !defined(BUILD_SKYVIEW_HD)
    if (FATFS_is_mounted) {
      const char *fileName;

      if (settings->adb == DB_OGN) {
        fileName = "/Aircrafts/ogn.cdb";

        if (ucdb.open(fileName) != CDB_OK) {
          Serial.print("Invalid CDB: ");
          Serial.println(fileName);
        } else {
          ADB_is_open = true;
        }
      }
      if (settings->adb == DB_FLN) {
        fileName = "/Aircrafts/fln.cdb";

        if (ucdb.open(fileName) != CDB_OK) {
          Serial.print("Invalid CDB: ");
          Serial.println(fileName);
        } else {
          ADB_is_open = true;
        }
      }
    }

    rval = ADB_is_open;
#endif /* BUILD_SKYVIEW_HD */
    break;
#endif /* CONFIG_IDF_TARGET_ESP32S3 */
  default:
    break;
  }

  return rval;
}

static bool ESP32_DB_query(uint8_t type, uint32_t id, char *buf, size_t size)
{
  bool rval = false;

  switch (settings->adapter)
  {
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32P4)
  case ADAPTER_TTGO_T5S:
#if defined(CONFIG_IDF_TARGET_ESP32P4)
  case ADAPTER_WAVESHARE_PI_HAT_2_7:
  case ADAPTER_WAVESHARE_PI_HAT_2_7_V2:
#endif /* CONFIG_IDF_TARGET_ESP32P4 */
#if !defined(BUILD_SKYVIEW_HD)
    {
      sqlite3_stmt *stmt;
      char *query = NULL;
      int error;
      const char *reg_key, *db_key;
      sqlite3 *db;

      switch (type)
      {
      case DB_OGN:
        switch (settings->idpref)
        {
        case ID_TAIL:
          reg_key = "accn";
          break;
        case ID_MAM:
          reg_key = "acmodel";
          break;
        case ID_REG:
        default:
          reg_key = "acreg";
          break;
        }
        db_key  = "devices";
        db      = ogn_db;
        break;
      case DB_ICAO:
        switch (settings->idpref)
        {
        case ID_TAIL:
          reg_key = "owner";
          break;
        case ID_MAM:
          reg_key = "type";
          break;
        case ID_REG:
        default:
          reg_key = "registration";
          break;
        }
        db_key  = "aircrafts";
        db      = icao_db;
        break;
      case DB_FLN:
      default:
        switch (settings->idpref)
        {
        case ID_TAIL:
          reg_key = "tail";
          break;
        case ID_MAM:
          reg_key = "type";
          break;
        case ID_REG:
        default:
          reg_key = "registration";
          break;
        }
        db_key  = "aircrafts";
        db      = fln_db;
        break;
      }

      if (db == NULL) {
        return false;
      }

      error = asprintf(&query, "select %s from %s where id = %d",reg_key, db_key, id);

      if (error == -1) {
        return false;
      }

      sqlite3_prepare_v2(db, query, strlen(query), &stmt, NULL);

      while (sqlite3_step(stmt) != SQLITE_DONE) {
        if (sqlite3_column_type(stmt, 0) == SQLITE3_TEXT) {

          size_t len = strlen((char *) sqlite3_column_text(stmt, 0));

          if (len > 0) {
            len = len > size ? size : len;
            strncpy(buf, (char *) sqlite3_column_text(stmt, 0), len);
            if (len < size) {
              buf[len] = 0;
            } else if (len == size) {
              buf[len-1] = 0;
            }
            rval = true;
          }
        }
      }

      sqlite3_finalize(stmt);

      free(query);
    }
#endif /* BUILD_SKYVIEW_HD */
    break;
#endif /* CONFIG_IDF_TARGET_ESP32 */
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  case ADAPTER_WAVESHARE_PICO_2_7:
  case ADAPTER_WAVESHARE_PICO_2_7_V2:
#if !defined(BUILD_SKYVIEW_HD)
    {
      char key[8];
      char out[64];
      uint8_t tokens[3] = { 0 };
      cdbResult rt;
      int c, i = 0, token_cnt = 0;
      int tok_num = 1;

      if (!ADB_is_open) {
        return rval;
      }

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

          switch (settings->idpref)
          {
          case ID_TAIL:
            tok_num = 2;
            break;
          case ID_MAM:
            tok_num = 0;
            break;
          case ID_REG:
          default:
            tok_num = 1;
            break;
          }

          if (strlen(out + tokens[tok_num]) > 0) {
            snprintf(buf, size, "%s", out + tokens[tok_num]);
            rval = true;
          }
          break;

        case KEY_NOT_FOUND:
        default:
          break;
      }
    }
#endif /* BUILD_SKYVIEW_HD */
    break;
#endif /* CONFIG_IDF_TARGET_ESP32S3 */
  default:
    break;
  }

  return rval;
}

static void ESP32_DB_fini()
{
  switch (settings->adapter)
  {
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32P4)
  case ADAPTER_TTGO_T5S:
#if defined(CONFIG_IDF_TARGET_ESP32P4)
  case ADAPTER_WAVESHARE_PI_HAT_2_7:
  case ADAPTER_WAVESHARE_PI_HAT_2_7_V2:
#endif /* CONFIG_IDF_TARGET_ESP32P4 */
#if !defined(BUILD_SKYVIEW_HD)
    if (settings->adb != DB_NONE) {
      if (fln_db != NULL) {
        sqlite3_close(fln_db);
      }

      if (ogn_db != NULL) {
        sqlite3_close(ogn_db);
      }

      if (icao_db != NULL) {
        sqlite3_close(icao_db);
      }

      sqlite3_shutdown();
    }

    SD.end();
#endif /* BUILD_SKYVIEW_HD */
    break;
#endif /* CONFIG_IDF_TARGET_ESP32 */
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  case ADAPTER_WAVESHARE_PICO_2_7:
  case ADAPTER_WAVESHARE_PICO_2_7_V2:
#if !defined(BUILD_SKYVIEW_HD)
    if (ADB_is_open) {
      ucdb.close();
      ADB_is_open = false;
    }
#endif /* BUILD_SKYVIEW_HD */
    break;
#endif /* CONFIG_IDF_TARGET_ESP32S3 */
  default:
    break;
  }
}

#if defined(CONFIG_IDF_TARGET_ESP32)   || \
    defined(CONFIG_IDF_TARGET_ESP32S2) || \
    defined(CONFIG_IDF_TARGET_ESP32C3) || \
    defined(CONFIG_IDF_TARGET_ESP32C5) || \
    defined(CONFIG_IDF_TARGET_ESP32C6) || \
    defined(CONFIG_IDF_TARGET_ESP32P4)
/* write sample data to I2S */
int i2s_write_sample_nb(uint32_t sample)
{
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR>=4
  size_t i2s_bytes_written;
  i2s_write((i2s_port_t)i2s_num, (const char*)&sample, sizeof(uint32_t),
             &i2s_bytes_written, 100);
  return i2s_bytes_written;
#else
  return i2s_write_bytes((i2s_port_t)i2s_num, (const char *)&sample,
                          sizeof(uint32_t), 100);
#endif
}

/* read 4 bytes of data from wav file */
int read4bytes(File file, uint32_t *chunkId)
{
  int n = file.read((uint8_t *)chunkId, sizeof(uint32_t));
  return n;
}

/* these are functions to process wav file */
int readRiff(File file, wavRiff_t *wavRiff)
{
  int n = file.read((uint8_t *)wavRiff, sizeof(wavRiff_t));
  return n;
}
int readProps(File file, wavProperties_t *wavProps)
{
  int n = file.read((uint8_t *)wavProps, sizeof(wavProperties_t));
  return n;
}

static bool play_file(char *filename)
{
  bool rval = false;

#if !defined(EXCLUDE_AUDIO)
  headerState_t state = HEADER_RIFF;

#if 1
  File wavfile = SD.open(filename);
#else
  File wavfile = fatfs.open(filename, FILE_READ);
#endif

  if (wavfile) {
    int c = 0;
    int n;
    while (wavfile.available()) {
      switch(state){
        case HEADER_RIFF:
        wavRiff_t wavRiff;
        n = readRiff(wavfile, &wavRiff);
        if(n == sizeof(wavRiff_t)){
          if(wavRiff.chunkID == CCCC('R', 'I', 'F', 'F') && wavRiff.format == CCCC('W', 'A', 'V', 'E')){
            state = HEADER_FMT;
//            Serial.println("HEADER_RIFF");
          }
        }
        break;
        case HEADER_FMT:
        n = readProps(wavfile, &wavProps);
        if(n == sizeof(wavProperties_t)){
          state = HEADER_DATA;
#if 0
            Serial.print("chunkID = "); Serial.println(wavProps.chunkID);
            Serial.print("chunkSize = "); Serial.println(wavProps.chunkSize);
            Serial.print("audioFormat = "); Serial.println(wavProps.audioFormat);
            Serial.print("numChannels = "); Serial.println(wavProps.numChannels);
            Serial.print("sampleRate = "); Serial.println(wavProps.sampleRate);
            Serial.print("byteRate = "); Serial.println(wavProps.byteRate);
            Serial.print("blockAlign = "); Serial.println(wavProps.blockAlign);
            Serial.print("bitsPerSample = "); Serial.println(wavProps.bitsPerSample);
#endif
        }
        break;
        case HEADER_DATA:
        uint32_t chunkId, chunkSize;
        n = read4bytes(wavfile, &chunkId);
        if(n == 4){
          if(chunkId == CCCC('d', 'a', 't', 'a')){
//            Serial.println("HEADER_DATA");
          }
        }
        n = read4bytes(wavfile, &chunkSize);
        if(n == 4){
//          Serial.println("prepare data");
          state = DATA;
        }
        //initialize i2s with configurations above
        i2s_driver_install((i2s_port_t)i2s_num, &i2s_config, 0, NULL);
        i2s_set_pin((i2s_port_t)i2s_num, &pin_config);
        //set sample rates of i2s to sample rate of wav file
        i2s_set_sample_rates((i2s_port_t)i2s_num, wavProps.sampleRate);
        break;
        /* after processing wav file, it is time to process music data */
        case DATA:
        uint32_t data;
        n = read4bytes(wavfile, &data);
        i2s_write_sample_nb(data);
        break;
      }
    }
    wavfile.close();
    rval = true;
  } else {
    Serial.println(F("error opening WAV file"));
  }
  if (state == DATA) {
    i2s_driver_uninstall((i2s_port_t)i2s_num); //stop & destroy i2s driver
  }
#endif /* EXCLUDE_AUDIO */

  return rval;
}
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
static bool play_file(char *filename)
{
  bool rval = false;

#if !defined(EXCLUDE_AUDIO)
  if (Audio_Source->open(filename)) {
    unsigned long Audio_Timemarker = millis();
    Audio_Gen->begin(Audio_Source, Audio_Sink);

    while (Audio_Gen->loop()) {
      if (millis() - Audio_Timemarker > 10000) {
        Audio_Gen->stop();
        Serial.println("ERROR: Audio timeout. Playback aborted.");
        Serial.flush();
      }
    }

    rval = true;
  }
#endif /* EXCLUDE_AUDIO */

  return rval;
}
#else
#error "This ESP32 family build variant is not supported!"
#endif /* CONFIG_IDF_TARGET_ESP32XX */


static void ESP32_TTS(char *message)
{
  char filename[MAX_FILENAME_LEN];

  if (strcmp(message, "POST")) {
    if ( settings->voice   != VOICE_OFF                       &&
        (settings->adapter == ADAPTER_TTGO_T5S                ||
#if defined(CONFIG_IDF_TARGET_ESP32P4)
         settings->adapter == ADAPTER_WAVESHARE_PI_HAT_2_7    ||
         settings->adapter == ADAPTER_WAVESHARE_PI_HAT_2_7_V2 ||
#endif /* CONFIG_IDF_TARGET_ESP32P4 */
         settings->adapter == ADAPTER_WAVESHARE_PICO_2_7      ||
         settings->adapter == ADAPTER_WAVESHARE_PICO_2_7_V2)) {

#if defined(CONFIG_IDF_TARGET_ESP32)   || \
    defined(CONFIG_IDF_TARGET_ESP32S2) || \
    defined(CONFIG_IDF_TARGET_ESP32C3) || \
    defined(CONFIG_IDF_TARGET_ESP32C5) || \
    defined(CONFIG_IDF_TARGET_ESP32C6) || \
    defined(CONFIG_IDF_TARGET_ESP32P4)
      if (SD.cardType() == CARD_NONE)
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
      if (!FATFS_is_mounted)
#else
#error "This ESP32 family build variant is not supported!"
#endif /* CONFIG_IDF_TARGET_ESP32XX */
        return;

      if (hw_info.display == DISPLAY_EPD_2_7) {
        while (!SoC->Display_is_ready()) {yield();}
        EPD_Message("VOICE", "ALERT");
        SoC->Display_update(EPD_UPDATE_FAST);
        while (!SoC->Display_is_ready()) {yield();}
      }

      bool wdt_status = loopTaskWDTEnabled;

      if (wdt_status) {
        disableLoopWDT();
      }

      char *word = strtok (message, " ");

      while (word != NULL)
      {
          strcpy(filename, WAV_FILE_PREFIX);
          strcat(filename,  settings->voice == VOICE_1 ? VOICE1_SUBDIR :
                           (settings->voice == VOICE_2 ? VOICE2_SUBDIR :
                           (settings->voice == VOICE_3 ? VOICE3_SUBDIR :
                            "" )));
          strcat(filename, word);
          strcat(filename, WAV_FILE_SUFFIX);
          play_file(filename);
          word = strtok (NULL, " ");

          yield();

          /* Poll input source(s) */
          Input_loop();
      }

      if (wdt_status) {
        enableLoopWDT();
      }
    }
  } else {
    if ( settings->voice   != VOICE_OFF                       &&
        (settings->adapter == ADAPTER_TTGO_T5S                ||
#if defined(CONFIG_IDF_TARGET_ESP32P4)
         settings->adapter == ADAPTER_WAVESHARE_PI_HAT_2_7    ||
         settings->adapter == ADAPTER_WAVESHARE_PI_HAT_2_7_V2 ||
#endif /* CONFIG_IDF_TARGET_ESP32P4 */
         settings->adapter == ADAPTER_WAVESHARE_PICO_2_7      ||
         settings->adapter == ADAPTER_WAVESHARE_PICO_2_7_V2)) {

      strcpy(filename, WAV_FILE_PREFIX);
      strcat(filename, "POST");
      strcat(filename, WAV_FILE_SUFFIX);

#if defined(CONFIG_IDF_TARGET_ESP32)   || \
    defined(CONFIG_IDF_TARGET_ESP32S2) || \
    defined(CONFIG_IDF_TARGET_ESP32C3) || \
    defined(CONFIG_IDF_TARGET_ESP32C5) || \
    defined(CONFIG_IDF_TARGET_ESP32C6) || \
    defined(CONFIG_IDF_TARGET_ESP32P4)
      if (SD.cardType() == CARD_NONE ||
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
      if (!FATFS_is_mounted          ||
#else
#error "This ESP32 family build variant is not supported!"
#endif /* CONFIG_IDF_TARGET_ESP32XX */
          !play_file(filename)) {
        /* keep boot-time SkyView logo on the screen for 7 seconds */
        delay(7000);
      }
    } else {
      if (hw_info.display == DISPLAY_EPD_2_7) {
        /* keep boot-time SkyView logo on the screen for 7 seconds */
        delay(7000);
      }
    }
  }
}

#include <AceButton.h>
using namespace ace_button;

AceButton button_mode(SOC_BUTTON_MODE_T5S);
AceButton button_up  (SOC_BUTTON_UP_T5S);
AceButton button_down(SOC_BUTTON_DOWN_T5S);

// The event handler for the button.
void handleEvent(AceButton* button, uint8_t eventType,
    uint8_t buttonState) {

#if 0
  // Print out a message for all events.
  if        (button == &button_mode) {
    Serial.print(F("MODE "));
  } else if (button == &button_up) {
    Serial.print(F("UP   "));
  } else if (button == &button_down) {
    Serial.print(F("DOWN "));
  }

  Serial.print(F("handleEvent(): eventType: "));
  Serial.print(eventType);
  Serial.print(F("; buttonState: "));
  Serial.println(buttonState);
#endif

  switch (eventType) {
    case AceButton::kEventPressed:
      break;
    case AceButton::kEventReleased:
      if (button == &button_mode) {
        EPD_Mode();
      } else if (button == &button_up) {
        EPD_Up();
      } else if (button == &button_down) {
        EPD_Down();
      }
      break;
    case AceButton::kEventLongPressed:
      if (button == &button_mode) {

        if (settings->adapter == ADAPTER_TTGO_T5S &&
            digitalRead(SOC_BUTTON_DOWN_T5S) == LOW) {
          screen_saver = true;
        }

#if defined(CONFIG_IDF_TARGET_ESP32S3)
        if ((settings->adapter == ADAPTER_WAVESHARE_PICO_2_7     ||
             settings->adapter == ADAPTER_WAVESHARE_PICO_2_7_V2) &&
            digitalRead(SOC_GPIO_PIN_KEY2) == LOW) {
          screen_saver = true;
        }
#endif /* CONFIG_IDF_TARGET_ESP32S3 */

        shutdown("NORMAL OFF");
        Serial.println(F("This will never be printed."));
      }
      break;
  }
}

/* Callbacks for push button interrupt */
void onModeButtonEvent() {
  button_mode.check();
}

void onUpButtonEvent() {
  button_up.check();
}

void onDownButtonEvent() {
  button_down.check();
}

static void ESP32_Button_setup()
{
  int mode_button_pin = SOC_BUTTON_MODE_DEF;

#if defined(CONFIG_IDF_TARGET_ESP32)
  if (settings->adapter == ADAPTER_TTGO_T5S) {
    mode_button_pin = SOC_BUTTON_MODE_T5S;
  }
#endif /* CONFIG_IDF_TARGET_ESP32 */

#if defined(CONFIG_IDF_TARGET_ESP32S3)
  if (settings->adapter == ADAPTER_WAVESHARE_PICO_2_7 ||
      settings->adapter == ADAPTER_WAVESHARE_PICO_2_7_V2) {
    mode_button_pin = SOC_GPIO_PIN_KEY1;
  }
#endif /* CONFIG_IDF_TARGET_ESP32S3 */

#if defined(CONFIG_IDF_TARGET_ESP32P4)
  if (settings->adapter == ADAPTER_WAVESHARE_PI_HAT_2_7 ||
      settings->adapter == ADAPTER_WAVESHARE_PI_HAT_2_7_V2) {
    mode_button_pin = SOC_GPIO_BUTTON_MODE;
  }
#endif /* CONFIG_IDF_TARGET_ESP32P4 */

  pinMode(mode_button_pin, settings->adapter == ADAPTER_WAVESHARE_PICO_2_7      ||
#if defined(CONFIG_IDF_TARGET_ESP32P4)
                           settings->adapter == ADAPTER_WAVESHARE_PI_HAT_2_7    ||
                           settings->adapter == ADAPTER_WAVESHARE_PI_HAT_2_7_V2 ||
#endif /* CONFIG_IDF_TARGET_ESP32P4 */
                           settings->adapter == ADAPTER_WAVESHARE_PICO_2_7_V2   ?
                           INPUT_PULLUP : INPUT);

  button_mode.init(mode_button_pin);

  // Configure the ButtonConfig with the event handler, and enable all higher
  // level events.
  ButtonConfig* ModeButtonConfig = button_mode.getButtonConfig();
  ModeButtonConfig->setEventHandler(handleEvent);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureClick);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  ModeButtonConfig->setDebounceDelay(15);
  ModeButtonConfig->setClickDelay(100);
  ModeButtonConfig->setDoubleClickDelay(1000);
  ModeButtonConfig->setLongPressDelay(2000);

//  attachInterrupt(digitalPinToInterrupt(mode_button_pin), onModeButtonEvent, CHANGE );

#if defined(CONFIG_IDF_TARGET_ESP32)
  if (settings->adapter == ADAPTER_TTGO_T5S) {

    // Button(s) uses external pull up resistor.
    pinMode(SOC_BUTTON_UP_T5S,   INPUT);
    pinMode(SOC_BUTTON_DOWN_T5S, INPUT);

    if (settings->rotate == ROTATE_180) {
      button_up.init(SOC_BUTTON_DOWN_T5S);
      button_down.init(SOC_BUTTON_UP_T5S);
    } else {
      button_up.init(SOC_BUTTON_UP_T5S);
      button_down.init(SOC_BUTTON_DOWN_T5S);
    }

    ButtonConfig* UpButtonConfig = button_up.getButtonConfig();
    UpButtonConfig->setEventHandler(handleEvent);
    UpButtonConfig->setFeature(ButtonConfig::kFeatureClick);
    UpButtonConfig->setDebounceDelay(15);
    UpButtonConfig->setClickDelay(100);
    UpButtonConfig->setDoubleClickDelay(1000);
    UpButtonConfig->setLongPressDelay(2000);

    ButtonConfig* DownButtonConfig = button_down.getButtonConfig();
    DownButtonConfig->setEventHandler(handleEvent);
    DownButtonConfig->setFeature(ButtonConfig::kFeatureClick);
    DownButtonConfig->setDebounceDelay(15);
    DownButtonConfig->setClickDelay(100);
    DownButtonConfig->setDoubleClickDelay(1000);
    DownButtonConfig->setLongPressDelay(2000);

    if (settings->rotate == ROTATE_180) {
//      attachInterrupt(digitalPinToInterrupt(SOC_BUTTON_UP_T5S),   onDownButtonEvent, CHANGE);
//      attachInterrupt(digitalPinToInterrupt(SOC_BUTTON_DOWN_T5S), onUpButtonEvent,   CHANGE);
    } else {
//      attachInterrupt(digitalPinToInterrupt(SOC_BUTTON_UP_T5S),   onUpButtonEvent,   CHANGE);
//      attachInterrupt(digitalPinToInterrupt(SOC_BUTTON_DOWN_T5S), onDownButtonEvent, CHANGE);
    }
  }
#endif /* CONFIG_IDF_TARGET_ESP32 */

#if defined(CONFIG_IDF_TARGET_ESP32S3)
  if (settings->adapter == ADAPTER_WAVESHARE_PICO_2_7 ||
      settings->adapter == ADAPTER_WAVESHARE_PICO_2_7_V2) {
    pinMode(SOC_GPIO_PIN_KEY0, INPUT_PULLUP);
    pinMode(SOC_GPIO_PIN_KEY2, INPUT_PULLUP);

    if (settings->rotate == ROTATE_180) {
      button_up.init(SOC_GPIO_PIN_KEY0, HIGH);
      button_down.init(SOC_GPIO_PIN_KEY2, HIGH);
    } else {
      button_up.init(SOC_GPIO_PIN_KEY2, HIGH);
      button_down.init(SOC_GPIO_PIN_KEY0, HIGH);
    }

    ButtonConfig* UpButtonConfig = button_up.getButtonConfig();
    UpButtonConfig->setEventHandler(handleEvent);
    UpButtonConfig->setFeature(ButtonConfig::kFeatureClick);
    UpButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
    UpButtonConfig->setClickDelay(100);
    UpButtonConfig->setLongPressDelay(2000);

    ButtonConfig* DownButtonConfig = button_down.getButtonConfig();
    DownButtonConfig->setEventHandler(handleEvent);
    DownButtonConfig->setFeature(ButtonConfig::kFeatureClick);
    DownButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
    DownButtonConfig->setClickDelay(100);
    DownButtonConfig->setLongPressDelay(2000);
  }
#endif /* CONFIG_IDF_TARGET_ESP32S3 */

#if defined(CONFIG_IDF_TARGET_ESP32P4)
  if (settings->adapter == ADAPTER_WAVESHARE_PI_HAT_2_7 ||
      settings->adapter == ADAPTER_WAVESHARE_PI_HAT_2_7_V2) {
    pinMode(SOC_GPIO_BUTTON_UP,   INPUT_PULLUP);
    pinMode(SOC_GPIO_BUTTON_DOWN, INPUT_PULLUP);

    button_up.init(SOC_GPIO_BUTTON_UP,     HIGH);
    button_down.init(SOC_GPIO_BUTTON_DOWN, HIGH);

    ButtonConfig* UpButtonConfig = button_up.getButtonConfig();
    UpButtonConfig->setEventHandler(handleEvent);
    UpButtonConfig->setFeature(ButtonConfig::kFeatureClick);
    UpButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
    UpButtonConfig->setClickDelay(100);
    UpButtonConfig->setLongPressDelay(2000);

    ButtonConfig* DownButtonConfig = button_down.getButtonConfig();
    DownButtonConfig->setEventHandler(handleEvent);
    DownButtonConfig->setFeature(ButtonConfig::kFeatureClick);
    DownButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
    DownButtonConfig->setClickDelay(100);
    DownButtonConfig->setLongPressDelay(2000);
  }
#endif /* CONFIG_IDF_TARGET_ESP32P4 */
}

static void ESP32_Button_loop()
{
  button_mode.check();

  if (settings->adapter == ADAPTER_TTGO_T5S                ||
#if defined(CONFIG_IDF_TARGET_ESP32P4)
      settings->adapter == ADAPTER_WAVESHARE_PI_HAT_2_7    ||
      settings->adapter == ADAPTER_WAVESHARE_PI_HAT_2_7_V2 ||
#endif /* CONFIG_IDF_TARGET_ESP32P4 */
      settings->adapter == ADAPTER_WAVESHARE_PICO_2_7      ||
      settings->adapter == ADAPTER_WAVESHARE_PICO_2_7_V2) {
    button_up.check();
    button_down.check();
  }
}

static void ESP32_Button_fini()
{
  int mode_button_pin = SOC_BUTTON_MODE_DEF;

#if defined(CONFIG_IDF_TARGET_ESP32)
  if (settings->adapter == ADAPTER_TTGO_T5S) {
    mode_button_pin = SOC_BUTTON_MODE_T5S;
//    detachInterrupt(digitalPinToInterrupt(SOC_BUTTON_UP_T5S));
//    detachInterrupt(digitalPinToInterrupt(SOC_BUTTON_DOWN_T5S));
  }
#endif /* CONFIG_IDF_TARGET_ESP32 */

#if defined(CONFIG_IDF_TARGET_ESP32S3)
  if (settings->adapter == ADAPTER_WAVESHARE_PICO_2_7 ||
      settings->adapter == ADAPTER_WAVESHARE_PICO_2_7_V2) {
    mode_button_pin = SOC_GPIO_PIN_KEY1;
  }
#endif /* CONFIG_IDF_TARGET_ESP32S3 */

#if defined(CONFIG_IDF_TARGET_ESP32P4)
  if (settings->adapter == ADAPTER_WAVESHARE_PI_HAT_2_7 ||
      settings->adapter == ADAPTER_WAVESHARE_PI_HAT_2_7_V2) {
    mode_button_pin = SOC_GPIO_BUTTON_MODE;
  }
#endif /* CONFIG_IDF_TARGET_ESP32P4 */

//  detachInterrupt(digitalPinToInterrupt(mode_button_pin));
  while (digitalRead(mode_button_pin) == LOW);
}

static void ESP32_WDT_setup()
{
  enableLoopWDT();
}

static void ESP32_WDT_fini()
{
  disableLoopWDT();
}

#if defined(CONFIG_IDF_TARGET_ESP32P4) && defined(USE_USB_HOST)

#include "libusb.h"
#include "rtl-sdr.h"

#include <mode-s.h>
#include <sdr/common.h>

#include "freertos/ringbuf.h"
#include "esp_heap_caps.h"

#define MODE_S_PREAMBLE_US      8
#define MODE_S_LONG_MSG_BITS    112
#define MODE_S_SHORT_MSG_BITS   56
#define MODE_S_ASYNC_BUF_NUMBER 3
#define MODE_S_DATA_LEN         65536
#define MODE_S_FULL_LEN         (MODE_S_PREAMBLE_US + MODE_S_LONG_MSG_BITS)

#define SDR_RINGBUFFER_SIZE     (1024 * 1024)
#define RTLSDR_TASK_PRIORITY    10 // 4

rtlsdr_dev_t *rtlsdr_dev;
TaskHandle_t rtlsdr_task_handle = NULL;

mag_t *mag;

int do_exit = 0;

mode_s_t state;

char STR_TMP_BUFFER[128];

StaticRingbuffer_t *buffer_struct = nullptr;
uint8_t *buffer_storage = nullptr;
RingbufHandle_t s_ringbuf_sdr;
bool rb_reader_is_ready = false;

extern int rtlsdr_is_connected;
extern mag_t maglut[129*129];

void on_msg(mode_s_t *self, struct mode_s_msg *mm) {

    MODES_NOTUSED(self);

#if 0
    if (self->check_crc == 0 || mm->crcok) {
      sprintf(STR_TMP_BUFFER, "%02d %03d %02x%02x%02x\r\n",
                              mm->msgtype, mm->msgbits, mm->aa1, mm->aa2, mm->aa3);

      Serial.write(STR_TMP_BUFFER, strlen(STR_TMP_BUFFER));
      Serial.flush();
    }
#endif

    if (self->check_crc == 0 || mm->crcok) {

//    printf("%02d %03d %02x%02x%02x\r\n", mm->msgtype, mm->msgbits, mm->aa1, mm->aa2, mm->aa3);

        int acfts_in_sight = 0;
        struct mode_s_aircraft *a = state.aircrafts;

        while (a) {
          acfts_in_sight++;
          a = a->next;
        }

        if (acfts_in_sight < MAX_TRACKING_OBJECTS) {
          interactiveReceiveData(self, mm);
        }
    }
}

void process_iq8u_buffer(uint8_t *buf, size_t size) {

    for (int j=0; j<size; j+=2) {
      int8_t i = buf[j    ] - 127;
      int8_t q = buf[j + 1] - 127;

      if (i < 0) i = -i;
      if (q < 0) q = -q;

      mag[(MODE_S_FULL_LEN-1)*2 + (j>>1)] = maglut[i*129+q];
    }

    mode_s_detect(&state, mag, (size + (MODE_S_FULL_LEN-1)*4) / 2, on_msg);
    memcpy(mag, mag + size/2, (MODE_S_FULL_LEN-1)*2);
}

static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
  if (do_exit)
    return;

  if (rtlsdr_is_connected == 0) {
    do_exit = 1;
    rtlsdr_cancel_async(rtlsdr_dev);
  }

  if (rb_reader_is_ready) {
    if (xRingbufferSend(s_ringbuf_sdr, buf, len, pdMS_TO_TICKS(10)) != pdTRUE) {
        Serial.println("Failed to send message into ring buffer!");
    }
  }
}

void rtlsdr_task()
{
  while (true)
  {
    while (rtlsdr_is_connected == 0) {
      delay(10);
    }

    int i;
    char vendor[256], product[256], serial[256];

    int device_count = rtlsdr_get_device_count();

    fprintf(stderr, "Found %d device(s):\n", device_count);
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		fprintf(stderr, "  %d:  %s, %s, SN: %s\n", i, vendor, product, serial);
	}

    int r = rtlsdr_open(&rtlsdr_dev, 0);
    if (r < 0) {
        fprintf(stderr, "Failed to open rtlsdr device #%d.\n", 0);
    }

    r = rtlsdr_set_sample_rate(rtlsdr_dev, 2000000);
    if (r < 0) {
        fprintf(stderr, "WARNING: Failed to set sample rate.\n");
    } else {
        fprintf(stderr, "Sampling at %u S/s.\n", 2000000);
    }

    r = rtlsdr_set_center_freq(rtlsdr_dev, 1090000000);
    if (r < 0) {
        fprintf(stderr, "WARNING: Failed to set center freq.\n");
    } else {
        fprintf(stderr, "Tuned to %u Hz.\n", 1090000000);
    }
#if 1
    r = rtlsdr_set_tuner_gain_mode(rtlsdr_dev, 0);
    if (r != 0)
    {
        fprintf(stderr, "WARNING: Failed to set tuner gain.\r\n");
    }
    else
    {
        fprintf(stderr, "Tuner gain set to automatic.\r\n");
    }
#else
    r = rtlsdr_set_tuner_gain_mode(rtlsdr_dev, 1);
    if (r < 0) {
        fprintf(stderr, "WARNING: Failed to enable manual gain.\r\n");
    }

    int gain = nearest_gain(rtlsdr_dev, 316);
    r = rtlsdr_set_tuner_gain(rtlsdr_dev, gain);
    if (r != 0) {
        fprintf(stderr, "WARNING: Failed to set tuner gain.\r\n");
    } else {
        fprintf(stderr, "Tuner gain set to %0.2f dB.\r\n", gain/10.0);
    }
#endif
    r = rtlsdr_reset_buffer(rtlsdr_dev);
    if (r < 0) {
        fprintf(stderr, "WARNING: Failed to reset buffers.\n");}

    r = rtlsdr_read_async(rtlsdr_dev, rtlsdr_callback, 0,
                          MODE_S_ASYNC_BUF_NUMBER, MODE_S_DATA_LEN);

    rtlsdr_close(rtlsdr_dev);

    do_exit = 0;
  }
}

static void ESP32PX_USB_setup()
{
  mode_s_init(&state);

  mag = (mag_t *) ps_malloc((MODE_S_DATA_LEN + (MODE_S_FULL_LEN-1)*4) / (sizeof(mag_t) * 2));

  buffer_struct = (StaticRingbuffer_t *)heap_caps_malloc(
                                                    sizeof(StaticRingbuffer_t),
                                                    MALLOC_CAP_SPIRAM);
  buffer_storage = (uint8_t *)heap_caps_malloc(SDR_RINGBUFFER_SIZE,
                                               MALLOC_CAP_SPIRAM);
  s_ringbuf_sdr = xRingbufferCreateStatic(SDR_RINGBUFFER_SIZE,
                                          RINGBUF_TYPE_BYTEBUF,
                                          buffer_storage,
                                          buffer_struct);
  if (s_ringbuf_sdr == NULL) {
     fprintf(stderr, "xRingbufferCreateStatic failure\n");
     while (true);
  }

  usbhost_begin();

  if (libusb_init() == 1) {
	BaseType_t rtlsdr_task_status = xTaskCreatePinnedToCore(
		(TaskFunction_t) rtlsdr_task,
		"rtlsdr_reader",
		4096,
		NULL,
		RTLSDR_TASK_PRIORITY,
		&rtlsdr_task_handle,
		CONFIG_ARDUINO_RUNNING_CORE
	);

	if (rtlsdr_task_status != pdPASS) {
		ESP_LOGE("rtlsdr","Error creating rtlsdr reader task!");
		while (1) {
		  vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
	}
  }
}

#include <TinyGPS++.h>

#include "GDL90Helper.h"
#include "NMEAHelper.h"
#include "TrafficHelper.h"

#define MODES_TASK_INTERVAL 998

unsigned long ModeS_Time_Marker = 0;

bool es1090_decode(void *pkt, traffic_t *this_aircraft, traffic_t *fop) {

  struct mode_s_aircraft *a = (struct mode_s_aircraft *) pkt;

  fop->ID        = a->addr;
  fop->IDType = ADDR_TYPE_ICAO;

  fop->latitude  = a->lat;
  fop->longitude = a->lon;
  if (a->unit == MODE_S_UNIT_FEET) {
    fop->altitude = a->altitude / _GPS_FEET_PER_METER;
  } else {
    fop->altitude = a->altitude;
  }

  fop->AlarmLevel = ALARM_LEVEL_NONE;

  fop->Track = a->track;
//  fop->ClimbRate = a->vert_rate;                              /* TBD */
  fop->TurnRate = 0;                                            /* TBD */
  fop->GroundSpeed = a->speed;
  fop->AcftType = GDL90_TO_AT(a->aircraft_type);

  /* sizeof(mdb.callsign) = 9 ; sizeof(fop->callsign) = 8 */
  memcpy(fop->callsign, a->flight, sizeof(fop->callsign));

  fop->timestamp = now();

  return true;
}

static void ESP32PX_USB_loop()
{
  if (!rb_reader_is_ready) { rb_reader_is_ready = true; }

  if (rtlsdr_is_connected == 1) {
    size_t receivedMessageSize;
    uint8_t *receivedMessage;
    receivedMessage = (uint8_t *) xRingbufferReceiveUpTo(s_ringbuf_sdr,
                                                         &receivedMessageSize,
                                                         pdMS_TO_TICKS(1000),
                                                         MODE_S_DATA_LEN);
    if (receivedMessage != NULL) {
        process_iq8u_buffer(receivedMessage, receivedMessageSize);
        vRingbufferReturnItem(s_ringbuf_sdr, (void*) receivedMessage);
    } else {
        Serial.println("Failed to receive message from ring buffer!");
    }
  }

  if (millis() - ModeS_Time_Marker > MODES_TASK_INTERVAL) {
    struct mode_s_aircraft *a;

    for (a = state.aircrafts; a; a = a->next) {
#if 0
      Serial.print("even_cprtime = ");  Serial.print(a->even_cprtime);
      Serial.print(" ");
      Serial.print("odd_cprtime = ");   Serial.println(a->odd_cprtime);
#endif
      if (a->even_cprtime && a->odd_cprtime &&
          abs((long) (a->even_cprtime - a->odd_cprtime)) <= MODE_S_INTERACTIVE_TTL * 1000 ) {
#if 0
        fo = EmptyFO;
        if (es1090_decode(a, &ThisAircraft, &fo)) {
          Traffic_Update(&fo);
          Traffic_Add();
        }
#else
        Serial.print("addr = "); Serial.print(a->addr, HEX); Serial.print(" ");
        Serial.print("lat = ");  Serial.print(a->lat);       Serial.print(" ");
        Serial.print("lon = ");  Serial.println(a->lon);
#endif
      }
    }

    interactiveRemoveStaleAircrafts(&state);

    ModeS_Time_Marker = millis();
  }
}

static void ESP32PX_USB_fini()
{
  if (libusb_init() == 1) {
      do_exit = 1;
      vTaskDelete( rtlsdr_task_handle );
  }

  vRingbufferDelete(s_ringbuf_sdr);
  free(buffer_storage);
  free(buffer_struct);
  free(mag);
}

static int ESP32PX_USB_available()
{
  int rval = 0;

  return rval;
}

static int ESP32PX_USB_read()
{
  int rval = -1;

  return rval;
}

static size_t ESP32PX_USB_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  return rval;
}

IODev_ops_t ESP32PX_USB_ops = {
  "ESP32PX USB",
  ESP32PX_USB_setup,
  ESP32PX_USB_loop,
  ESP32PX_USB_fini,
  ESP32PX_USB_available,
  ESP32PX_USB_read,
  ESP32PX_USB_write
};
#endif /* CONFIG_IDF_TARGET_ESP32P4 */

const SoC_ops_t ESP32_ops = {
#if defined(CONFIG_IDF_TARGET_ESP32)
  SOC_ESP32,
  "ESP32",
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
  SOC_ESP32S2,
  "ESP32-S2",
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  SOC_ESP32S3,
  "ESP32-S3",
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  SOC_ESP32C3,
  "ESP32-C3",
#elif defined(CONFIG_IDF_TARGET_ESP32C5)
  SOC_ESP32C5,
  "ESP32-C5",
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
  SOC_ESP32C6,
  "ESP32-C6",
#elif defined(CONFIG_IDF_TARGET_ESP32P4)
  SOC_ESP32P4,
  "ESP32-P4",
#else
#error "This ESP32 family build variant is not supported!"
#endif /* CONFIG_IDF_TARGET_ESP32-S2-S3-C3-C5-C6-P4 */
  ESP32_setup,
  ESP32_post_init,
  ESP32_loop,
  ESP32_fini,
  ESP32_reset,
  ESP32_getChipId,
  ESP32_getFreeHeap,
  ESP32_EEPROM_begin,
  ESP32_EEPROM_extension,
  ESP32_WiFi_setOutputPower,
  ESP32_WiFi_hostname,
  ESP32_swSer_begin,
  ESP32_swSer_enableRx,
  ESP32_maxSketchSpace,
  ESP32_WiFiUDP_stopAll,
  ESP32_Battery_setup,
  ESP32_Battery_voltage,
  ESP32_Display_setup,
  ESP32_Display_loop,
  ESP32_Display_fini,
  ESP32_Display_is_ready,
  ESP32_Display_update,
  ESP32_WiFi_Receive_UDP,
  ESP32_WiFi_clients_count,
  ESP32_DB_init,
  ESP32_DB_query,
  ESP32_DB_fini,
  ESP32_TTS,
  ESP32_Button_setup,
  ESP32_Button_loop,
  ESP32_Button_fini,
  ESP32_WDT_setup,
  ESP32_WDT_fini,
#if !defined(CONFIG_IDF_TARGET_ESP32S2)
  &ESP32_Bluetooth_ops,
#else
  NULL,
#endif /* CONFIG_IDF_TARGET_ESP32S2 */
#if defined(CONFIG_IDF_TARGET_ESP32P4) && defined(USE_USB_HOST)
  &ESP32PX_USB_ops,
#else
  NULL,
#endif /* CONFIG_IDF_TARGET_ESP32P4 */
};

#endif /* ESP32 */
