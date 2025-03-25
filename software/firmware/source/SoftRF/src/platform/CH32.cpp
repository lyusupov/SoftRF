/*
 * Platform_CH32.cpp
 * Copyright (C) 2024-2025 Linar Yusupov
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

#if defined(ARDUINO_ARCH_CH32)

#include <SPI.h>
#include <Wire.h>

#if defined(USE_TINYUSB)
#include <Adafruit_TinyUSB.h>
#endif

#include "../system/SoC.h"
#include "../driver/RF.h"
#include "../driver/EEPROM.h"
#include "../driver/LED.h"
#include "../driver/OLED.h"
#include "../driver/Baro.h"
#include "../driver/Sound.h"
#include "../driver/Battery.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/GDL90.h"
#include "../protocol/data/D1090.h"

#include <Adafruit_SPIFlash.h>
#include "uCDB.hpp"

// SX127x pin mapping
lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .txe = LMIC_UNUSED_PIN,
    .rxe = LMIC_UNUSED_PIN,
    .rst = SOC_GPIO_PIN_RST,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
    .busy = SOC_GPIO_PIN_BUSY,
    .tcxo = LMIC_UNUSED_PIN,
};

#if !defined(EXCLUDE_LED_RING)
// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIX_NUM, SOC_GPIO_PIN_LED,
                              NEO_GRB + NEO_KHZ800);
#endif /* EXCLUDE_LED_RING */

#if defined(EXCLUDE_WIFI)
char UDPpacketBuffer[4]; // Dummy definition to satisfy build sequence
#endif /* EXCLUDE_WIFI */

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

static uint32_t bootCount __attribute__ ((section (".noinit")));
static bool wdt_is_active = false;

static CH32_board_id CH32_board = CH32_WCH_V307V_R1; /* default */

static bool CH32_has_eeprom    = false;
static bool CH32_has_spiflash  = false;
static bool FATFS_is_mounted   = false;
static bool ADB_is_open        = false;

#ifdef UART_MODULE_ENABLED
const PinMap PinMap_UART_TX[] = {
  {PA_9, USART1, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, 0, AFIO_NONE)},
  {PD_5, USART2, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, 0, AFIO_Remap_USART2_ENABLE)},
  {PD_8, USART3, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, 0, AFIO_Partial1Remap_USART3_ENABLE)},
  {NC,   NP,     0}
};

const PinMap PinMap_UART_RX[] = {
  {PA_10, USART1, CH_PIN_DATA(CH_MODE_INPUT, CH_CNF_INPUT_PUPD, PULLUP, AFIO_NONE)},
  {PD_6,  USART2, CH_PIN_DATA(CH_MODE_INPUT, CH_CNF_INPUT_PUPD, PULLUP, AFIO_Remap_USART2_ENABLE)},
  {PD_9,  USART3, CH_PIN_DATA(CH_MODE_INPUT, CH_CNF_INPUT_PUPD, PULLUP, AFIO_Partial1Remap_USART3_ENABLE)},
  {NC,    NP,     0}
};

const PinMap PinMap_UART_RTS[] = {
  {NC, USART1, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, 0, AFIO_NONE)},
  {NC, USART2, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, 0, AFIO_NONE)},
  {NC, USART3, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, 0, AFIO_NONE)},
  {NC,    NP,     0}
};

const PinMap PinMap_UART_CTS[] = {
  {NC, USART1, CH_PIN_DATA(CH_MODE_INPUT, CH_CNF_INPUT_PUPD, PULLUP, AFIO_NONE)},
  {NC, USART2, CH_PIN_DATA(CH_MODE_INPUT, CH_CNF_INPUT_PUPD, PULLUP, AFIO_NONE)},
  {NC, USART3, CH_PIN_DATA(CH_MODE_INPUT, CH_CNF_INPUT_PUPD, PULLUP, AFIO_NONE)},
  {NC,    NP,     0}
};
#endif

HardwareSerial Serial2(USART2);
HardwareSerial Serial3(USART3);

#if defined(USE_SOFTSPI)
SoftSPI RadioSPI(SOC_GPIO_PIN_MOSI, SOC_GPIO_PIN_MISO, SOC_GPIO_PIN_SCK);
SoftSPI FlashSPI(SOC_GPIO_YD_FL_MOSI, SOC_GPIO_YD_FL_MISO, SOC_GPIO_YD_FL_CLK);
#else
SPIClass RadioSPI;
SPIClass FlashSPI;
#endif /* USE_SOFTSPI */

static Adafruit_SPIFlash *SPIFlash = NULL;

static uint32_t spiflash_id = 0;

/// Flash device list count
enum {
  W25Q32JV_INDEX,
  EXTERNAL_FLASH_DEVICE_COUNT
};

static SPIFlash_Device_t possible_devices[] = {
  [W25Q32JV_INDEX] = W25Q32JV_IQ,
};

// file system object from SdFat
FatVolume fatfs;

uCDB<FatVolume, File32> ucdb(fatfs);

#if defined(EXCLUDE_EEPROM)
eeprom_t eeprom_block;
settings_t *settings = &eeprom_block.field.settings;
#endif /* EXCLUDE_EEPROM */

#if defined(USE_TINYUSB)
const char *CH32_Device_Manufacturer = SOFTRF_IDENT;
const char *CH32_Device_Model = "Academy Edition";
const uint16_t CH32_Device_Version = SOFTRF_USB_FW_VERSION;

// USB Mass Storage object
Adafruit_USBD_MSC usb_msc;

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
static int32_t CH32_msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return SPIFlash->readBlocks(lba, (uint8_t*) buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
static int32_t CH32_msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize)
{
//  ledOn(SOC_GPIO_LED_USBMSC);

  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return SPIFlash->writeBlocks(lba, buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
static void CH32_msc_flush_cb (void)
{
  // sync with flash
  SPIFlash->syncBlocks();

  // clear file system's cache to force refresh
  fatfs.cacheClear();

//  ledOff(SOC_GPIO_LED_USBMSC);
}
#endif /* USE_TINYUSB */

#if defined(ENABLE_RECORDER)
#include <SdFat_Adafruit_Fork.h>

SoftSpiDriver<SOC_GPIO_YD_SD_D0, SOC_GPIO_YD_SD_CMD, SOC_GPIO_YD_SD_CLK> uSD_SPI;

// Speed argument is ignored for software SPI.
#if ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(uSD_SS_pin, DEDICATED_SPI, SD_SCK_MHZ(8), &uSD_SPI)
#else  // ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(uSD_SS_pin, SHARED_SPI, SD_SCK_MHZ(8), &uSD_SPI)
#endif  // ENABLE_DEDICATED_SPI

SdFat uSD;

static bool uSD_is_attached = false;
#endif /* ENABLE_RECORDER */

void CH32_attachInterrupt_func(uint32_t pin, void (*userFunc)(void), int mode)
{
  attachInterrupt(pin, GPIO_Mode_IN_FLOATING, userFunc, EXTI_Mode_Interrupt,
                  mode == RISING  ? EXTI_Trigger_Rising  :
                  mode == FALLING ? EXTI_Trigger_Falling :
                  EXTI_Trigger_Rising_Falling /* CHANGE */ );
}

static void CH32_setup()
{
#if SOC_GPIO_RADIO_LED_TX != SOC_UNUSED_PIN
  pinMode(SOC_GPIO_RADIO_LED_TX, OUTPUT);
  digitalWrite(SOC_GPIO_RADIO_LED_TX, ! LED_STATE_ON);
#endif /* SOC_GPIO_RADIO_LED_TX */
#if SOC_GPIO_RADIO_LED_RX != SOC_UNUSED_PIN
  pinMode(SOC_GPIO_RADIO_LED_RX, OUTPUT);
  digitalWrite(SOC_GPIO_RADIO_LED_RX, ! LED_STATE_ON);
#endif /* SOC_GPIO_RADIO_LED_RX */

  SerialOutput.setRx(SOC_GPIO_PIN_CONS_RX);
  SerialOutput.setTx(SOC_GPIO_PIN_CONS_TX);
  SerialOutput.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);

  Serial_GNSS_In.setRx(SOC_GPIO_PIN_GNSS_RX);
  Serial_GNSS_In.setTx(SOC_GPIO_PIN_GNSS_TX);

#if !defined(USE_SOFTSPI)
  RadioSPI.setMISO(SOC_GPIO_PIN_MISO);
  RadioSPI.setMOSI(SOC_GPIO_PIN_MOSI);
  RadioSPI.setSCLK(SOC_GPIO_PIN_SCK);
#endif /* USE_SOFTSPI */

  Wire.setSCL(SOC_GPIO_PIN_SCL);
  Wire.setSDA(SOC_GPIO_PIN_SDA);

#if defined(USE_TINYUSB)
  USBDevice.setManufacturerDescriptor(CH32_Device_Manufacturer);
  USBDevice.setProductDescriptor(CH32_Device_Model);
  USBDevice.setDeviceVersion(CH32_Device_Version);
#endif /* USE_TINYUSB */

  Wire.begin();
  Wire.beginTransmission(FT24C64_ADDRESS);
  CH32_has_eeprom = (Wire.endTransmission() == 0);
  Wire.end();

  if (CH32_has_eeprom) {
    CH32_board = CH32_YD_V307VCT6;
  }

  /* (Q)SPI flash init */
  Adafruit_FlashTransport_SPI *ft = NULL;

  switch (CH32_board)
  {
    case CH32_YD_V307VCT6:
      pin_function(SOC_GPIO_YD_LED_BLUE,
                   CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_PP, 0, 0));
      digitalWriteFast(SOC_GPIO_YD_LED_BLUE, HIGH);

      possible_devices[W25Q32JV_INDEX].supports_qspi        = false;
      possible_devices[W25Q32JV_INDEX].supports_qspi_writes = false;

#if !defined(USE_SOFTSPI)
      FlashSPI.setMISO(SOC_GPIO_YD_FL_MISO);
      FlashSPI.setMOSI(SOC_GPIO_YD_FL_MOSI);
      FlashSPI.setSCLK(SOC_GPIO_YD_FL_CLK);
#endif /* USE_SOFTSPI */

      ft = new Adafruit_FlashTransport_SPI(SOC_GPIO_YD_FL_SS, &FlashSPI);

#if defined(ENABLE_RECORDER)
      {
        int uSD_SS_pin = SOC_GPIO_YD_SD_D3;

#if !defined(USE_SOFTSPI) && SPI_DRIVER_SELECT != 2
        uSD_SPI.setMISO(SOC_GPIO_YD_SD_D0);
        uSD_SPI.setMOSI(SOC_GPIO_YD_SD_CMD);
        uSD_SPI.setSCLK(SOC_GPIO_YD_SD_CLK);
#endif /* USE_SOFTSPI */

        pinMode(uSD_SS_pin, OUTPUT);
        digitalWrite(uSD_SS_pin, HIGH);

        uSD_is_attached = uSD.cardBegin(SD_CONFIG);
      }
#endif /* ENABLE_RECORDER */
      break;

    case CH32_WCH_V307V_R1:
    default:
      break;
  }

  if (ft != NULL) {
    SPIFlash = new Adafruit_SPIFlash(ft);
    CH32_has_spiflash = SPIFlash->begin(possible_devices,
                                        EXTERNAL_FLASH_DEVICE_COUNT);
  }

  hw_info.storage = CH32_has_spiflash ? STORAGE_FLASH : STORAGE_NONE;

#if defined(ENABLE_RECORDER)
  if (uSD_is_attached && uSD.card()->cardSize() > 0) {
    hw_info.storage = (hw_info.storage == STORAGE_FLASH) ?
                      STORAGE_FLASH_AND_CARD : STORAGE_CARD;
  }
#endif /* ENABLE_RECORDER */

  if (CH32_has_spiflash) {
    spiflash_id = SPIFlash->getJEDECID();

#if defined(USE_TINYUSB)
    // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
    usb_msc.setID(CH32_Device_Manufacturer, "External Flash", "1.0");

    // Set callback
    usb_msc.setReadWriteCallback(CH32_msc_read_cb,
                                 CH32_msc_write_cb,
                                 CH32_msc_flush_cb);

    // Set disk size, block size should be 512 regardless of spi flash page size
    usb_msc.setCapacity(SPIFlash->size()/512, 512);

    // MSC is ready for read/write
    usb_msc.setUnitReady(true);

    usb_msc.begin();
#endif /* USE_TINYUSB */

    FATFS_is_mounted = fatfs.begin(SPIFlash);
  }

#if defined(USE_RADIOLIB)
  lmic_pins.dio[0] = SOC_GPIO_PIN_DIO1;
#endif /* USE_RADIOLIB */

  Serial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);

#if defined(USE_TINYUSB) && defined(USBCON)
  for (int i=0; i < 40; i++) {if (Serial) break; else delay(100);}
#endif
}

static void CH32_post_init()
{
  {
    Serial.println();
    Serial.println(F("SoftRF Academy Edition Power-on Self Test"));
    Serial.println();
    Serial.flush();

    Serial.print(F("Board: "));
    Serial.println(CH32_board == CH32_YD_V307VCT6 ?
                   "YD CH32V307VCT6" : "WCH CH32V307V-R1");
    Serial.println();
    Serial.flush();

    Serial.print  (F("SPI FLASH JEDEC ID: "));
    Serial.print  (spiflash_id, HEX);
    Serial.println();
    Serial.println();
    Serial.flush();

    Serial.println(F("Built-in components:"));

    Serial.print(F("RADIO   : "));
    Serial.println(hw_info.rf      != RF_IC_NONE        ? F("PASS") : F("FAIL"));
    Serial.print(F("GNSS    : "));
    Serial.println(hw_info.gnss    != GNSS_MODULE_NONE  ? F("PASS") : F("FAIL"));

    Serial.print(F("FLASH   : "));
    Serial.println(hw_info.storage == STORAGE_FLASH_AND_CARD ||
                   hw_info.storage == STORAGE_FLASH     ? F("PASS") : F("N/A"));
    Serial.flush();

    Serial.println();
    Serial.println(F("External components:"));
    Serial.print(F("BARO    : "));
    Serial.println(hw_info.baro    != BARO_MODULE_NONE  ? F("PASS") : F("N/A"));
    Serial.print(F("DISPLAY : "));
    Serial.println(hw_info.display != DISPLAY_NONE      ? F("PASS") : F("N/A"));

    Serial.println();
    Serial.println(F("Power-on Self Test is complete."));
    Serial.println();
    Serial.flush();
  }

#if defined(ENABLE_RECORDER)
  if (CH32_board == CH32_YD_V307VCT6)
  {
    if (!uSD_is_attached) {
      Serial.println(F("WARNING: unable to attach micro-SD card."));
    } else {
      // The number of 512 byte sectors in the card
      // or zero if an error occurs.
      size_t cardSize = uSD.card()->cardSize();

      if (cardSize == 0) {
        Serial.println(F("WARNING: invalid micro-SD card size."));
      } else {
        uint8_t cardType = uSD.card()->type();

        Serial.print(F("SD Card Type: "));
        if(cardType == SD_CARD_TYPE_SD1){
            Serial.println(F("V1"));
        } else if(cardType == SD_CARD_TYPE_SD2){
            Serial.println(F("V2"));
        } else if(cardType == SD_CARD_TYPE_SDHC){
            Serial.println(F("SDHC"));
        } else {
            Serial.println(F("UNKNOWN"));
        }

        Serial.print("SD Card Size: ");
        Serial.print(cardSize / (2 * 1024));
        Serial.println(" MB");
      }
    }
    Serial.println();
  }
#endif /* ENABLE_RECORDER */

  Serial.println(F("Data output device(s):"));

  Serial.print(F("NMEA   - "));
  switch (settings->nmea_out)
  {
    case NMEA_UART       :  Serial.println(F("UART"));    break;
    case NMEA_USB        :  Serial.println(F("USB CDC")); break;
    case NMEA_OFF        :
    default              :  Serial.println(F("NULL"));    break;
  }

  Serial.print(F("GDL90  - "));
  switch (settings->gdl90)
  {
    case GDL90_UART      :  Serial.println(F("UART"));    break;
    case GDL90_USB       :  Serial.println(F("USB CDC")); break;
    case GDL90_OFF       :
    default              :  Serial.println(F("NULL"));    break;
  }

  Serial.print(F("D1090  - "));
  switch (settings->d1090)
  {
    case D1090_UART      :  Serial.println(F("UART"));    break;
    case D1090_USB       :  Serial.println(F("USB CDC")); break;
    case D1090_OFF       :
    default              :  Serial.println(F("NULL"));    break;
  }

  Serial.println();
  Serial.flush();

#if defined(USE_OLED)
  OLED_info1();
#endif /* USE_OLED */
}

static uint32_t prev_tx_packets_counter = 0;
static uint32_t prev_rx_packets_counter = 0;
static unsigned long tx_led_time_marker = 0;
static unsigned long rx_led_time_marker = 0;

#define	LED_BLINK_TIME 100

static void CH32_loop()
{
#if SOC_GPIO_RADIO_LED_TX != SOC_UNUSED_PIN
  if (digitalRead(SOC_GPIO_RADIO_LED_TX) != LED_STATE_ON) {
    if (tx_packets_counter != prev_tx_packets_counter) {
      digitalWrite(SOC_GPIO_RADIO_LED_TX, LED_STATE_ON);
      prev_tx_packets_counter = tx_packets_counter;
      tx_led_time_marker = millis();
    }
  } else {
    if (millis() - tx_led_time_marker > LED_BLINK_TIME) {
      digitalWrite(SOC_GPIO_RADIO_LED_TX, ! LED_STATE_ON);
      prev_tx_packets_counter = tx_packets_counter;
    }
  }
#endif /* SOC_GPIO_RADIO_LED_TX */

#if SOC_GPIO_RADIO_LED_RX != SOC_UNUSED_PIN
  if (digitalRead(SOC_GPIO_RADIO_LED_RX) != LED_STATE_ON) {
    if (rx_packets_counter != prev_rx_packets_counter) {
      digitalWrite(SOC_GPIO_RADIO_LED_RX, LED_STATE_ON);
      prev_rx_packets_counter = rx_packets_counter;
      rx_led_time_marker = millis();
    }
  } else {
    if (millis() - rx_led_time_marker > LED_BLINK_TIME) {
      digitalWrite(SOC_GPIO_RADIO_LED_RX, ! LED_STATE_ON);
      prev_rx_packets_counter = rx_packets_counter;
    }
  }
#endif /* SOC_GPIO_RADIO_LED_RX */
}

static void CH32_fini(int reason)
{
  if (CH32_has_spiflash) {
#if defined(USE_TINYUSB)
    usb_msc.setUnitReady(false);
//  usb_msc.end(); /* N/A */
#endif /* USE_TINYUSB */
  }

  if (SPIFlash != NULL) SPIFlash->end();

  NVIC_SystemReset(); /* TODO */
}

static void CH32_reset()
{
  NVIC_SystemReset();
}

static uint32_t CH32_getChipId()
{
#if !defined(SOFTRF_ADDRESS)
  volatile uint32_t *ch32_uuid = ((volatile uint32_t *) 0x1FFFF7E8UL);

  /* Same method as STM32 OGN tracker does */
  uint32_t id = ch32_uuid[0] ^ ch32_uuid[1] ^ ch32_uuid[2];

  return DevID_Mapper(id);
#else
  return (SOFTRF_ADDRESS & 0xFFFFFFFFU );
#endif
}

static void* CH32_getResetInfoPtr()
{
  return (void *) &reset_info;
}

static String CH32_getResetInfo()
{
  switch (reset_info.reason)
  {
    default                       : return F("No reset information available");
  }
}

static String CH32_getResetReason()
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

extern "C" void * _sbrk   (int);

static uint32_t CH32_getFreeHeap()
{
  char top;
  return &top - reinterpret_cast<char*>(_sbrk(0));
}

static long CH32_random(long howsmall, long howBig)
{
  if(howsmall >= howBig) {
      return howsmall;
  }
  long diff = howBig - howsmall;

  return random(diff) + howsmall;
}

static void CH32_Sound_test(int var)
{
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && settings->volume != BUZZER_OFF) {
    tone(SOC_GPIO_PIN_BUZZER, 440,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 640,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 840,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 1040, 500); delay(600);
    noTone(SOC_GPIO_PIN_BUZZER);
    pinMode(SOC_GPIO_PIN_BUZZER, INPUT);
  }
}

static void CH32_Sound_tone(int hz, uint8_t volume)
{
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && volume != BUZZER_OFF) {
    if (hz > 0) {
      tone(SOC_GPIO_PIN_BUZZER, hz, ALARM_TONE_MS);
    } else {
      noTone(SOC_GPIO_PIN_BUZZER);
      pinMode(SOC_GPIO_PIN_BUZZER, INPUT);
    }
  }
}

static void CH32_WiFi_set_param(int ndx, int value)
{
  /* NONE */
}

static void CH32_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  /* NONE */
}

static bool CH32_EEPROM_begin(size_t size)
{
  return true;
}

static void CH32_EEPROM_extension(int cmd)
{
  if (cmd == EEPROM_EXT_LOAD) {
    if (settings->mode != SOFTRF_MODE_NORMAL
#if !defined(EXCLUDE_TEST_MODE)
        &&
        settings->mode != SOFTRF_MODE_TXRX_TEST
#endif /* EXCLUDE_TEST_MODE */
        ) {
      settings->mode = SOFTRF_MODE_NORMAL;
    }

    if (settings->nmea_out == NMEA_BLUETOOTH ||
        settings->nmea_out == NMEA_UDP       ||
        settings->nmea_out == NMEA_TCP ) {
      settings->nmea_out = NMEA_UART;
    }
    if (settings->gdl90 == GDL90_BLUETOOTH  ||
        settings->gdl90 == GDL90_UDP) {
      settings->gdl90 = GDL90_UART;
    }
    if (settings->d1090 == D1090_BLUETOOTH  ||
        settings->d1090 == D1090_UDP) {
      settings->d1090 = D1090_UART;
    }

    /* AUTO and UK RF bands are deprecated since Release v1.3 */
    if (settings->band == RF_BAND_AUTO || settings->band == RF_BAND_UK) {
      settings->band = RF_BAND_EU;
    }
  }
}

static void CH32_SPI_begin()
{
  RadioSPI.begin();
}

static void CH32_swSer_begin(unsigned long baud)
{
  Serial_GNSS_In.begin(baud);
}

static void CH32_swSer_enableRx(boolean arg)
{
  /* NONE */
}

static byte CH32_Display_setup()
{
  byte rval = DISPLAY_NONE;

#if defined(USE_OLED)
  rval = OLED_setup();
#endif /* USE_OLED */

  return rval;
}

static void CH32_Display_loop()
{
#if defined(USE_OLED)
  OLED_loop();
#endif /* USE_OLED */
}

static void CH32_Display_fini(int reason)
{
#if defined(USE_OLED)
  OLED_fini(reason);
#endif /* USE_OLED */
}

static void CH32_Battery_setup()
{

}

static float CH32_Battery_param(uint8_t param)
{
  float rval, voltage;

  switch (param)
  {
  case BATTERY_PARAM_THRESHOLD:
    rval = hw_info.model == SOFTRF_MODEL_ACADEMY ? BATTERY_THRESHOLD_LIPO   :
                                                   BATTERY_THRESHOLD_NIMHX2;
    break;

  case BATTERY_PARAM_CUTOFF:
    rval = hw_info.model == SOFTRF_MODEL_ACADEMY ? BATTERY_CUTOFF_LIPO      :
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

    {
      uint16_t mV = 0;
#if SOC_GPIO_PIN_BATTERY != SOC_UNUSED_PIN
      mV = analogRead(SOC_GPIO_PIN_BATTERY);
#endif
      rval = mV * SOC_ADC_VOLTAGE_DIV / 1000.0;
    }
    break;
  }

  return rval;
}

void CH32_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long CH32_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool CH32_Baro_setup() {
  return true;
}

static void CH32_UATSerial_begin(unsigned long baud)
{

}

static void CH32_UATModule_restart()
{

}

static void CH32_WDT_setup()
{
  /* TBD */
}

static void CH32_WDT_fini()
{
  /* TBD */
}

#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
#include <AceButton.h>
using namespace ace_button;

AceButton button_1(SOC_GPIO_PIN_BUTTON);

// The event handler for the button.
void handleEvent(AceButton* button, uint8_t eventType,
    uint8_t buttonState) {

  switch (eventType) {
    case AceButton::kEventClicked:
    case AceButton::kEventReleased:
#if defined(USE_OLED)
      if (button == &button_1) {
        OLED_Next_Page();
      }
#endif
      break;
    case AceButton::kEventDoubleClicked:
      break;
    case AceButton::kEventLongPressed:
      if (button == &button_1) {
        shutdown(SOFTRF_SHUTDOWN_BUTTON);
//      Serial.println(F("This will never be printed."));
      }
      break;
  }
}

/* Callbacks for push button interrupt */
void onPageButtonEvent() {
  button_1.check();
}
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */

static void CH32_Button_setup()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  if (hw_info.model == SOFTRF_MODEL_ACADEMY) {
    int button_pin = SOC_GPIO_PIN_BUTTON;

    // Button(s) uses external pull up resistor.
    pinMode(button_pin, CH32_board == CH32_YD_V307VCT6 ? INPUT_PULLUP : INPUT);

    button_1.init(button_pin);

    // Configure the ButtonConfig with the event handler, and enable all higher
    // level events.
    ButtonConfig* PageButtonConfig = button_1.getButtonConfig();
    PageButtonConfig->setEventHandler(handleEvent);
    PageButtonConfig->setFeature(ButtonConfig::kFeatureClick);
//    PageButtonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
    PageButtonConfig->setFeature(ButtonConfig::kFeatureLongPress);
    PageButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
//    PageButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
//    PageButtonConfig->setFeature(
//                      ButtonConfig::kFeatureSuppressClickBeforeDoubleClick);
//  PageButtonConfig->setDebounceDelay(15);
    PageButtonConfig->setClickDelay(600);
//    PageButtonConfig->setDoubleClickDelay(1500);
    PageButtonConfig->setLongPressDelay(2000);

//  attachInterrupt(digitalPinToInterrupt(button_pin), onPageButtonEvent, CHANGE );
  }
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */
}

static void CH32_Button_loop()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  if (hw_info.model == SOFTRF_MODEL_ACADEMY) {
    button_1.check();
  }
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */
}

static void CH32_Button_fini()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  if (hw_info.model == SOFTRF_MODEL_ACADEMY) {
//  detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_BUTTON));
    while (digitalRead(SOC_GPIO_PIN_BUTTON) == LOW);
    pinMode(SOC_GPIO_PIN_BUTTON, INPUT);
  }
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */
}

#if defined(USE_TINYUSB)
static void CH32_USB_setup() {
#if !defined(USBCON)
  USBSerial.begin(SERIAL_OUT_BR);
#endif /* USBCON */
}

static void CH32_USB_loop()  {

}

static void CH32_USB_fini()  {
#if !defined(USBCON)
  USBSerial.end();
#endif /* USBCON */
}

static int CH32_USB_available()
{
  int rval = 0;

  if (USBSerial) {
    rval = USBSerial.available();
  }

  return rval;
}

static int CH32_USB_read()
{
  int rval = -1;

  if (USBSerial) {
    rval = USBSerial.read();
  }

  return rval;
}

static size_t CH32_USB_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  if (USBSerial && (size < USBSerial.availableForWrite())) {
    rval = USBSerial.write(buffer, size);
  }

  return rval;
}

IODev_ops_t CH32_USBSerial_ops = {
  "CH32 USBSerial",
  CH32_USB_setup,
  CH32_USB_loop,
  CH32_USB_fini,
  CH32_USB_available,
  CH32_USB_read,
  CH32_USB_write
};
#endif /* USE_TINYUSB */

const SoC_ops_t CH32_ops = {
  SOC_CH32,
  "CH32",
  CH32_setup,
  CH32_post_init,
  CH32_loop,
  CH32_fini,
  CH32_reset,
  CH32_getChipId,
  CH32_getResetInfoPtr,
  CH32_getResetInfo,
  CH32_getResetReason,
  CH32_getFreeHeap,
  CH32_random,
  CH32_Sound_test,
  CH32_Sound_tone,
  NULL,
  CH32_WiFi_set_param,
  CH32_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  CH32_EEPROM_begin,
  CH32_EEPROM_extension,
  CH32_SPI_begin,
  CH32_swSer_begin,
  CH32_swSer_enableRx,
  NULL,
#if defined(USE_TINYUSB)
  &CH32_USBSerial_ops,
#else
  NULL,
#endif /* USE_TINYUSB */
  NULL,
  CH32_Display_setup,
  CH32_Display_loop,
  CH32_Display_fini,
  CH32_Battery_setup,
  CH32_Battery_param,
  CH32_GNSS_PPS_Interrupt_handler,
  CH32_get_PPS_TimeMarker,
  CH32_Baro_setup,
  CH32_UATSerial_begin,
  CH32_UATModule_restart,
  CH32_WDT_setup,
  CH32_WDT_fini,
  CH32_Button_setup,
  CH32_Button_loop,
  CH32_Button_fini,
  NULL
};

#endif /* ARDUINO_ARCH_CH32 */
