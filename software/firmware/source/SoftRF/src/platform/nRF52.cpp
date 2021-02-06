/*
 * Platform_nRF52.cpp
 * Copyright (C) 2020-2021 Linar Yusupov
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

#if defined(ARDUINO_ARCH_NRF52)

#include <SPI.h>
#include <Wire.h>
#include <pcf8563.h>
#include <SoftSPI.h>
#include <Adafruit_SPIFlash.h>
#include "Adafruit_TinyUSB.h"
#include <Adafruit_SleepyDog.h>
#include "nrf_wdt.h"

#include "../system/SoC.h"
#include "../driver/RF.h"
#include "../driver/EEPROM.h"
#include "../driver/GNSS.h"
#include "../driver/Baro.h"
#include "../driver/LED.h"
#include "../driver/Bluetooth.h"
#include "../driver/EPD.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/GDL90.h"
#include "../protocol/data/D1090.h"

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

char UDPpacketBuffer[4]; // Dummy definition to satisfy build sequence

static uint32_t prev_tx_packets_counter = 0;
static uint32_t prev_rx_packets_counter = 0;
extern uint32_t tx_packets_counter, rx_packets_counter;

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

static uint32_t bootCount = 0;

static nRF52_board_id nRF52_board = NRF52_NORDIC_PCA10059; /* default */

PCF8563_Class *rtc = nullptr;
I2CBus        *i2c = nullptr;

static bool nRF52_has_rtc      = false;
static bool nRF52_has_spiflash = false;
static bool RTC_sync           = false;

static TaskHandle_t EPD_Task_Handle = NULL;

#if !defined(ARDUINO_NRF52840_PCA10056)
#error "This nRF52 build variant is not supported!"
#endif

#define _SPI_DEV    NRF_SPIM2
#define _SPI1_DEV   NRF_SPIM3 // 32 Mhz

SPIClass SPI0(_SPI_DEV,
              SOC_GPIO_PIN_TECHO_REV_0_MISO,
              SOC_GPIO_PIN_TECHO_REV_0_SCK,
              SOC_GPIO_PIN_TECHO_REV_0_MOSI);
/*
SPIClass SPI0(_SPI_DEV,
              SOC_GPIO_PIN_PCA10059_MISO,
              SOC_GPIO_PIN_PCA10059_SCK,
              SOC_GPIO_PIN_PCA10059_MOSI);
*/
SPIClass SPI1(_SPI1_DEV,
              SOC_GPIO_PIN_EPD_MISO,
              SOC_GPIO_PIN_EPD_SCK,
              SOC_GPIO_PIN_EPD_MOSI);

#if defined(USE_EPAPER)
GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> epd_ttgo_techo(GxEPD2_154_D67(
                                                            SOC_GPIO_PIN_EPD_SS,
                                                            SOC_GPIO_PIN_EPD_DC,
                                                            SOC_GPIO_PIN_EPD_RST,
                                                            SOC_GPIO_PIN_EPD_BUSY));

GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> *display;
#endif /* USE_EPAPER */

SoftSPI SPI2(SOC_GPIO_PIN_SFL_MOSI, SOC_GPIO_PIN_SFL_MISO, SOC_GPIO_PIN_SFL_SCK);

Adafruit_FlashTransport_SPI SWFlashTransport (SOC_GPIO_PIN_SFL_SS, &SPI2);
Adafruit_FlashTransport_QSPI HWFlashTransport(SOC_GPIO_PIN_SFL_SCK,
                                              SOC_GPIO_PIN_SFL_SS,
                                              SOC_GPIO_PIN_SFL_MOSI,
                                              SOC_GPIO_PIN_SFL_MISO,
                                              SOC_GPIO_PIN_SFL_WP,
                                              SOC_GPIO_PIN_SFL_HOLD);

Adafruit_SPIFlash SoftSPIFlash(&SWFlashTransport);
Adafruit_SPIFlash QSPIFlash   (&HWFlashTransport);

static Adafruit_SPIFlash *SPIFlash = NULL;

#define MX25R1635F                                                             \
  {                                                                            \
    .total_size = (1 << 21), /* 2 MiB */                                       \
        .start_up_time_us = 5000, .manufacturer_id = 0xc2,                     \
    .memory_type = 0x28, .capacity = 0x15, .max_clock_speed_mhz = 8,           \
    .quad_enable_bit_mask = 0x40, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = true, .is_fram = false,                              \
  }

/// List of all possible flash devices used by nRF52840 boards
static const SPIFlash_Device_t possible_devices[] = {
    // LilyGO T-Echo
    MX25R1635F,

    // Nordic PCA10056
    MX25R6435F,
};

/// Flash device list count
enum {
  EXTERNAL_FLASH_DEVICE_COUNT =
      sizeof(possible_devices) / sizeof(possible_devices[0])
};

// USB Mass Storage object
Adafruit_USBD_MSC usb_msc;

// file system object from SdFat
FatFileSystem fatfs;

ui_settings_t ui_settings = {
    .units        = UNITS_METRIC,
    .zoom         = ZOOM_MEDIUM,
    .protocol     = PROTOCOL_NMEA,
    .orientation  = DIRECTION_TRACK_UP,
    .adb          = DB_NONE,
    .idpref       = ID_REG,
    .vmode        = VIEW_MODE_STATUS,
    .voice        = VOICE_OFF,
    .aghost       = ANTI_GHOSTING_OFF,
    .filter       = TRAFFIC_FILTER_OFF,
    .team         = 0
};

ui_settings_t *ui;

static void nRF52_setup()
{
  ui = &ui_settings;

//  uint32_t u32Reset_reason = NRF_POWER->RESETREAS;
//  reset_info.reason = u32Reset_reason;

  /* inactivate initVariant() of PCA10056 */
  pinMode(PIN_LED1, INPUT);
  pinMode(PIN_LED2, INPUT);
  pinMode(PIN_LED3, INPUT);
  pinMode(PIN_LED4, INPUT);

#if defined(USE_TINYUSB)
  Serial1.setPins(SOC_GPIO_PIN_CONS_RX, SOC_GPIO_PIN_CONS_TX);
  Serial1.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
#endif

  digitalWrite(SOC_GPIO_PIN_IO_PWR,  HIGH);
  digitalWrite(SOC_GPIO_PIN_3V3_PWR, LOW); /* PWR_EN is OFF */
  pinMode(SOC_GPIO_PIN_3V3_PWR, OUTPUT);
  delay(100);

  if (SoC->Battery_voltage() > 2.0) {  /* VBUS is ON */
    nRF52_board = NRF52_LILYGO_TECHO_REV_0;
    pinMode(SOC_GPIO_PIN_IO_PWR,  OUTPUT);  /* VDD_POWR is ON */
    pinMode(SOC_GPIO_PIN_3V3_PWR, INPUT);
  } else {
    pinMode(SOC_GPIO_PIN_IO_PWR, OUTPUT);  /* VDD_POWR is ON */
    delay(100);

    if (SoC->Battery_voltage() > 2.0) {
      nRF52_board = NRF52_LILYGO_TECHO_REV_0;
      pinMode(SOC_GPIO_PIN_3V3_PWR, INPUT);
    } else {
      digitalWrite(SOC_GPIO_PIN_3V3_PWR, HIGH); /* PWR_EN is ON */
      delay(100);

      if (SoC->Battery_voltage() > 2.0) {
        nRF52_board = NRF52_LILYGO_TECHO_REV_2;
      } else {
        pinMode(SOC_GPIO_PIN_3V3_PWR, INPUT);
        pinMode(SOC_GPIO_PIN_IO_PWR,  INPUT);
      }
    }
  }

  Wire.setPins(SOC_GPIO_PIN_SDA, SOC_GPIO_PIN_SCL);

  Wire.begin();
  Wire.beginTransmission(PCF8563_SLAVE_ADDRESS);
  nRF52_has_rtc = (Wire.endTransmission() == 0);
  if (nRF52_has_rtc) {
    hw_info.model = SOFTRF_MODEL_BADGE;
  }

  if (nRF52_board == NRF52_LILYGO_TECHO_REV_0) {
    Wire.beginTransmission(BME280_ADDRESS);
    nRF52_board = Wire.endTransmission() == 0 ?
                  NRF52_LILYGO_TECHO_REV_1 : nRF52_board;
  }
  Wire.end();

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
      hw_info.revision = 2;
      break;

    case NRF52_NORDIC_PCA10059:
    default:
      pinMode(SOC_GPIO_LED_PCA10059_STATUS, OUTPUT);
//      pinMode(SOC_GPIO_LED_PCA10059_GREEN,  OUTPUT);
      pinMode(SOC_GPIO_LED_PCA10059_RED,    OUTPUT);
      pinMode(SOC_GPIO_LED_PCA10059_BLUE,   OUTPUT);

//      ledOff(SOC_GPIO_LED_PCA10059_GREEN);
      ledOff(SOC_GPIO_LED_PCA10059_RED);
      ledOff(SOC_GPIO_LED_PCA10059_BLUE);
      ledOn (SOC_GPIO_LED_PCA10059_STATUS);
      break;
  }

  i2c = new I2CBus(Wire);

  if (nRF52_has_rtc && (i2c != nullptr)) {
    rtc = new PCF8563_Class(*i2c);

    pinMode(SOC_GPIO_PIN_R_INT, INPUT);
  }
}

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
//  fatfs.cacheClear();

  ledOff(SOC_GPIO_LED_USBMSC);
}

static void nRF52_post_init()
{
  /* (Q)SPI flash init */
  switch (nRF52_board)
  {
    case NRF52_LILYGO_TECHO_REV_0:
      SPIFlash = &SoftSPIFlash;
      break;
    case NRF52_LILYGO_TECHO_REV_1:
    case NRF52_LILYGO_TECHO_REV_2:
      SPIFlash = &QSPIFlash;
      break;
    case NRF52_NORDIC_PCA10059:
    default:
      break;
  }

  if (SPIFlash != NULL) {
    nRF52_has_spiflash = SPIFlash->begin(possible_devices,
                                         EXTERNAL_FLASH_DEVICE_COUNT);
  }

  /* SoftSPI on REV_0 is slow which causes the system crash in main loop */
  if (nRF52_has_spiflash && (nRF52_board != NRF52_LILYGO_TECHO_REV_0)) {
    uint32_t spi_flash_id = SPIFlash->getJEDECID();

    // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
    usb_msc.setID("SoftRF", "External Flash", "1.0");

    // Set callback
    usb_msc.setReadWriteCallback(nRF52_msc_read_cb,
                                 nRF52_msc_write_cb,
                                 nRF52_msc_flush_cb);

    // Set disk size, block size should be 512 regardless of spi flash page size
    usb_msc.setCapacity(SPIFlash->size()/512, 512);

    // MSC is ready for read/write
    usb_msc.setUnitReady(true);

    usb_msc.begin();

    // Init file system on the flash
//    fatfs.begin(&flash);


//    fatfs.end();
//    usb_msc.end();
  }

//    if (SPIFlash != NULL) SPIFlash->end();

  if (nRF52_board == NRF52_LILYGO_TECHO_REV_0 ||
      nRF52_board == NRF52_LILYGO_TECHO_REV_1 ||
      nRF52_board == NRF52_LILYGO_TECHO_REV_2) {
    Serial.println();
    Serial.print  (F("LilyGO T-Echo (rev."));
    Serial.print  (hw_info.revision);
    Serial.println(F(") Power-on Self Test"));
    Serial.println();
    Serial.flush();

    Serial.println(F("Built-in components:"));

    Serial.print(F("RADIO   : "));
    Serial.println(hw_info.rf      == RF_IC_SX1262 ||
                   hw_info.rf      == RF_IC_SX1276     ? F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("GNSS    : "));
    Serial.println(hw_info.gnss == ((hw_info.revision == 0 || hw_info.revision == 1) ?
                                   GNSS_MODULE_GOKE : GNSS_MODULE_AT65)
                                   ? F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("DISPLAY : "));
    Serial.println(hw_info.display == DISPLAY_EPD_1_54 ? F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("RTC     : "));
    Serial.println(nRF52_has_rtc                       ? F("PASS") : F("FAIL"));
    Serial.flush();
    Serial.print(F("FLASH   : "));
    Serial.println(nRF52_has_spiflash                  ? F("PASS") : F("FAIL"));
    Serial.flush();

    if (nRF52_board == NRF52_LILYGO_TECHO_REV_1 ||
        nRF52_board == NRF52_LILYGO_TECHO_REV_2) {
      Serial.print(F("BMx280  : "));
      Serial.println(hw_info.baro == BARO_MODULE_BMP280 ? F("PASS") : F("FAIL"));
      Serial.flush();
    }

    Serial.println();
    Serial.println(F("Power-on Self Test is completed."));
    Serial.println();
    Serial.flush();

#if defined(USE_EPAPER)
    /* EPD back light on */
    digitalWrite(SOC_GPIO_PIN_EPD_BLGT, HIGH);

    EPD_info1(nRF52_has_rtc, nRF52_has_spiflash);

    /* EPD back light off */
    digitalWrite(SOC_GPIO_PIN_EPD_BLGT, LOW);
#endif /* USE_EPAPER */
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
}

static void nRF52_loop()
{
  // Reload the watchdog
  if (nrf_wdt_started(NRF_WDT)) {
    Watchdog.reset();
  }

  if (!RTC_sync) {
    if (rtc &&
        gnss.date.isValid()     &&
        gnss.time.isValid()     &&
        gnss.date.year() > 2018 &&
        gnss.date.year() < 2030 ) {
      rtc->setDateTime(gnss.date.year(),   gnss.date.month(),
                       gnss.date.day(),    gnss.time.hour(),
                       gnss.time.minute(), gnss.time.second());
      RTC_sync = true;
    }
  }
}

static void nRF52_fini(int reason)
{
  uint8_t sd_en;

  switch (nRF52_board)
  {
    case NRF52_LILYGO_TECHO_REV_0:
#if 0
      /* Air530 GNSS ultra-low power tracking mode */
      digitalWrite(SOC_GPIO_PIN_GNSS_WKE, LOW);
      pinMode(SOC_GPIO_PIN_GNSS_WKE, OUTPUT);

    //swSer.write("$PGKC105,4*33\r\n");
#else
      pinMode(SOC_GPIO_PIN_GNSS_WKE, INPUT);

      //swSer.write("$PGKC051,0*37\r\n");
      // swSer.write("$PGKC051,1*36\r\n");
#endif
      //swSer.flush(); delay(250);

      ledOff(SOC_GPIO_LED_TECHO_REV_0_GREEN);
      ledOff(SOC_GPIO_LED_TECHO_REV_0_RED);
      ledOff(SOC_GPIO_LED_TECHO_REV_0_BLUE);

      pinMode(SOC_GPIO_LED_TECHO_REV_0_GREEN, INPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_0_RED,   INPUT);
      pinMode(SOC_GPIO_LED_TECHO_REV_0_BLUE,  INPUT);

      pinMode(SOC_GPIO_PIN_IO_PWR, INPUT);
      SPI2.end(); /* SoftSPI */
      pinMode(SOC_GPIO_PIN_SFL_SS, INPUT);
      break;

    case NRF52_LILYGO_TECHO_REV_1:
#if 0
      /* Air530 GNSS ultra-low power tracking mode */
      digitalWrite(SOC_GPIO_PIN_GNSS_WKE, LOW);
      pinMode(SOC_GPIO_PIN_GNSS_WKE, OUTPUT);

    //swSer.write("$PGKC105,4*33\r\n");
#else
      pinMode(SOC_GPIO_PIN_GNSS_WKE, INPUT);

      //swSer.write("$PGKC051,0*37\r\n");
      // swSer.write("$PGKC051,1*36\r\n");
#endif
      //swSer.flush(); delay(250);

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
      digitalWrite(SOC_GPIO_PIN_3V3_PWR, LOW);
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

  swSer.end();

  // pinMode(SOC_GPIO_PIN_SWSER_RX, INPUT);
  // pinMode(SOC_GPIO_PIN_SWSER_TX, INPUT);

  // pinMode(SOC_GPIO_PIN_BATTERY, INPUT);

  if (i2c != nullptr) Wire.end();

  // pinMode(SOC_GPIO_PIN_SDA,  INPUT);
  // pinMode(SOC_GPIO_PIN_SCL,  INPUT);

  // pinMode(SOC_GPIO_PIN_MOSI, INPUT);
  // pinMode(SOC_GPIO_PIN_MISO, INPUT);
  // pinMode(SOC_GPIO_PIN_SCK,  INPUT);
  pinMode(SOC_GPIO_PIN_SS,   INPUT);
  // pinMode(SOC_GPIO_PIN_BUSY, INPUT);
  pinMode(lmic_pins.rst,  INPUT);

  // pinMode(SOC_GPIO_PIN_PAD,    INPUT);
  pinMode(SOC_GPIO_PIN_BUTTON, INPUT);
  while (digitalRead(SOC_GPIO_PIN_BUTTON) == LOW);
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
    pinMode(SOC_GPIO_PIN_BUTTON, INPUT_PULLUP_SENSE /* INPUT_SENSE_LOW */);
    break;
#if defined(USE_SERIAL_DEEP_SLEEP)
  case SOFTRF_SHUTDOWN_NMEA:
    pinMode(SOC_GPIO_PIN_CONS_RX, INPUT_PULLUP_SENSE /* INPUT_SENSE_LOW */);
    break;
#endif
  default:
    break;
  }

  /* Cut 3.3V power off on modded REV_1 board */
  if (nRF52_board == NRF52_LILYGO_TECHO_REV_1 && hw_info.rf == RF_IC_SX1262) {
    pinMode(SOC_GPIO_PIN_TECHO_REV_1_3V3_PWR, INPUT_PULLDOWN);
  }

  Serial.end();

  (void) sd_softdevice_is_enabled(&sd_en);

  // Enter System OFF state
  if ( sd_en ) {
    sd_power_system_off();
  } else {
    NRF_POWER->SYSTEMOFF = 1;
  }
}

static void nRF52_reset()
{
  NVIC_SystemReset();
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
  return dbgHeapTotal() - dbgHeapUsed();
}

static long nRF52_random(long howsmall, long howBig)
{
  return random(howsmall, howBig);
}

static void nRF52_Sound_test(int var)
{
  /* NONE */
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

static void nRF52_SPI_begin()
{
  SPI0.begin();
}

static void nRF52_swSer_begin(unsigned long baud)
{
  swSer.setPins(SOC_GPIO_PIN_SWSER_RX, SOC_GPIO_PIN_SWSER_TX);
  swSer.begin(baud);
}

static void nRF52_swSer_enableRx(boolean arg)
{
  /* NONE */
}

static byte nRF52_Display_setup()
{
  byte rval = DISPLAY_NONE;

#if defined(USE_EPAPER)
  display = &epd_ttgo_techo;

  if (EPD_setup(true)) {

    xTaskCreate(EPD_Task, "EPD update", EPD_STACK_SZ, NULL, TASK_PRIO_LOW, &EPD_Task_Handle);

    rval = DISPLAY_EPD_1_54;
  }

  /* EPD back light off */
  pinMode(SOC_GPIO_PIN_EPD_BLGT, OUTPUT);
  digitalWrite(SOC_GPIO_PIN_EPD_BLGT, LOW);

#endif /* USE_EPAPER */

  return rval;
}

static void nRF52_Display_loop()
{
#if defined(USE_EPAPER)
  EPD_loop();
#endif /* USE_EPAPER */
}

static void nRF52_Display_fini(int reason)
{
#if defined(USE_EPAPER)

  EPD_Clear_Screen();
  EPD_fini(reason);

  if( EPD_Task_Handle != NULL )
  {
    vTaskDelete( EPD_Task_Handle );
  }

  SPI1.end();

  // pinMode(SOC_GPIO_PIN_EPD_MISO, INPUT);
  // pinMode(SOC_GPIO_PIN_EPD_MOSI, INPUT);
  // pinMode(SOC_GPIO_PIN_EPD_SCK,  INPUT);
  pinMode(SOC_GPIO_PIN_EPD_SS,   INPUT);
  pinMode(SOC_GPIO_PIN_EPD_DC,   INPUT);
  pinMode(SOC_GPIO_PIN_EPD_RST,  INPUT);
  // pinMode(SOC_GPIO_PIN_EPD_BUSY, INPUT);
  pinMode(SOC_GPIO_PIN_EPD_BLGT, INPUT);

#endif /* USE_EPAPER */
}

static void nRF52_Battery_setup()
{

}

static float nRF52_Battery_voltage()
{
  float raw;

  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(1);

  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(SOC_GPIO_PIN_BATTERY);

  // Set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);

  // Convert the raw value to compensated mv, taking the resistor-
  // divider into account (providing the actual LIPO voltage)
  // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
  return raw * REAL_VBAT_MV_PER_LSB * 0.001;
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
  Watchdog.enable(8000);
}

static void nRF52_WDT_fini()
{
  // cannot disable nRF's WDT
  if (nrf_wdt_started(NRF_WDT)) {
    Watchdog.reset();
  }
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
  int mode_button_pin = SOC_GPIO_PIN_BUTTON;
  int up_button_pin   = SOC_GPIO_PIN_PAD;

  // Button(s) uses external pull up resistor.
  pinMode(mode_button_pin, INPUT);
  pinMode(up_button_pin, INPUT);

  button_1.init(mode_button_pin);
  button_2.init(up_button_pin);

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
  attachInterrupt(digitalPinToInterrupt(up_button_pin),   onUpButtonEvent,   CHANGE );
}

static void nRF52_Button_loop()
{
  button_1.check();
  button_2.check();
}

static void nRF52_Button_fini()
{
//  detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_BUTTON));
  detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_PAD));
}

static void nRF52_USB_setup()
{
  if (USBSerial && USBSerial != Serial) {
    USBSerial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
  }
}

static void nRF52_USB_loop()
{

}

static void nRF52_USB_fini()
{
  if (USBSerial && USBSerial != Serial) {
    USBSerial.end();
  }
}

static int nRF52_USB_available()
{
  int rval = 0;

  if (USBSerial) {
    rval = USBSerial.available();
  }

  return rval;
}

static int nRF52_USB_read()
{
  int rval = -1;

  if (USBSerial) {
    rval = USBSerial.read();
  }

  return rval;
}

static size_t nRF52_USB_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  if (USBSerial && USBSerial.availableForWrite()) {
    rval = USBSerial.write(buffer, size);
  }

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
  NULL,
  nRF52_WiFi_set_param,
  nRF52_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  nRF52_EEPROM_begin,
  nRF52_SPI_begin,
  nRF52_swSer_begin,
  nRF52_swSer_enableRx,
  &nRF52_Bluetooth_ops,
  &nRF52_USBSerial_ops,
  NULL,
  nRF52_Display_setup,
  nRF52_Display_loop,
  nRF52_Display_fini,
  nRF52_Battery_setup,
  nRF52_Battery_voltage,
  nRF52_GNSS_PPS_Interrupt_handler,
  nRF52_get_PPS_TimeMarker,
  nRF52_Baro_setup,
  nRF52_UATSerial_begin,
  nRF52_UATModule_restart,
  nRF52_WDT_setup,
  nRF52_WDT_fini,
  nRF52_Button_setup,
  nRF52_Button_loop,
  nRF52_Button_fini
};

#endif /* ARDUINO_ARCH_NRF52 */
