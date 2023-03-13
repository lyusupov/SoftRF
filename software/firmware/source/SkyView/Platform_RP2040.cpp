/*
 * Platform_RP2040.cpp
 * Copyright (C) 2023 Linar Yusupov
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
#if defined(ARDUINO_ARCH_RP2040)

#include "SoCHelper.h"
#include "EPDHelper.h"
#include "EEPROMHelper.h"
#if !defined(EXCLUDE_WIFI)
#include "WiFiHelper.h"
#endif /* EXCLUDE_WIFI */

#include "SkyView.h"

#include <hardware/watchdog.h>

#if !defined(ARDUINO_ARCH_MBED)
#include "pico/unique_id.h"

#define PICO_ON_DEVICE 1
extern "C" {
#include "pico/binary_info.h"
}
#else
extern "C"
{
#include "hardware/flash.h"
}
#endif /* ARDUINO_ARCH_MBED */

#include <pico_sleep.h>

#if defined(USE_TINYUSB)
#if defined(USE_USB_HOST)
#include "pio_usb.h"
#endif /* USE_USB_HOST */
#include "Adafruit_TinyUSB.h"
#endif /* USE_TINYUSB */

#define SOFTRF_DESC "Multifunctional, compatible DIY general aviation proximity awareness system"
#define SOFTRF_URL  "https://github.com/lyusupov/SoftRF"

#if !defined(ARDUINO_ARCH_MBED)
bi_decl(bi_program_name(SKYVIEW_IDENT));
bi_decl(bi_program_description(SOFTRF_DESC));
bi_decl(bi_program_version_string(SKYVIEW_FIRMWARE_VERSION));
bi_decl(bi_program_build_date_string(__DATE__));
bi_decl(bi_program_url(SOFTRF_URL));
extern char __flash_binary_end;
bi_decl(bi_binary_end((intptr_t)&__flash_binary_end));
bi_decl(bi_1pin_with_name(SOC_EPD_PIN_MOSI_WS, "EPD MOSI"));
bi_decl(bi_1pin_with_name(SOC_EPD_PIN_MISO_WS, "EPD MISO"));
bi_decl(bi_1pin_with_name(SOC_EPD_PIN_SCK_WS,  "EPD SCK"));
bi_decl(bi_1pin_with_name(SOC_EPD_PIN_SS_WS,   "EPD SS"));
bi_decl(bi_1pin_with_name(SOC_EPD_PIN_RST_WS,  "EPD RST"));
bi_decl(bi_1pin_with_name(SOC_EPD_PIN_BUSY_WS, "EPD BUSY"));
bi_decl(bi_1pin_with_name(SOC_EPD_PIN_DC_WS,   "EPD DC"));
#endif /* ARDUINO_ARCH_MBED */

#if defined(EXCLUDE_WIFI)
char UDPpacketBuffer[4]; // Dummy definition to satisfy build sequence
#else
#define ENABLE_ARDUINO_FEATURES 0
WebServer server ( 80 );
#endif /* EXCLUDE_WIFI */

/* Waveshare E-Paper Driver Board */
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> epd_waveshare_W3(GxEPD2_270(
                                          /*CS=*/   SOC_EPD_PIN_SS_WS,
                                          /*DC=*/   SOC_EPD_PIN_DC_WS,
                                          /*RST=*/  SOC_EPD_PIN_RST_WS,
                                          /*BUSY=*/ SOC_EPD_PIN_BUSY_WS
                                          ));
GxEPD2_BW<GxEPD2_270_T91, GxEPD2_270_T91::HEIGHT> epd_waveshare_T91(GxEPD2_270_T91(
                                                  /*CS=*/   SOC_EPD_PIN_SS_WS,
                                                  /*DC=*/   SOC_EPD_PIN_DC_WS,
                                                  /*RST=*/  SOC_EPD_PIN_RST_WS,
                                                  /*BUSY=*/ SOC_EPD_PIN_BUSY_WS
                                                  ));
static uint32_t bootCount __attribute__ ((section (".noinit")));
static bool wdt_is_active = false;

static RP2040_board_id RP2040_board    = RP2040_RPIPICO; /* default */
const char *RP2040_Device_Manufacturer = SOFTRF_IDENT;
const char *RP2040_Device_Model        = SKYVIEW_IDENT " Light";
const uint16_t RP2040_Device_Version   = SKYVIEW_USB_FW_VERSION;

#define UniqueIDsize 2

static union {
#if !defined(ARDUINO_ARCH_MBED)
  pico_unique_board_id_t RP2040_unique_flash_id;
#endif /* ARDUINO_ARCH_MBED */
  uint32_t RP2040_chip_id[UniqueIDsize];
};

#include <Adafruit_SPIFlash.h>

#if !defined(ARDUINO_ARCH_MBED)
Adafruit_FlashTransport_RP2040 HWFlashTransport;
Adafruit_SPIFlash QSPIFlash(&HWFlashTransport);

static Adafruit_SPIFlash *SPIFlash = &QSPIFlash;

/// Flash device list count
enum {
  W25Q16JV_IQ_INDEX,
  EXTERNAL_FLASH_DEVICE_COUNT
};

/// List of all possible flash devices used by RP2040 boards
static SPIFlash_Device_t possible_devices[] = {
  [W25Q16JV_IQ_INDEX] = W25Q16JV_IQ,
};
#endif /* ARDUINO_ARCH_MBED */

static bool RP2040_has_spiflash = false;
static uint32_t spiflash_id     = 0;
static bool FATFS_is_mounted    = false;

#if defined(USE_TINYUSB)
// USB Mass Storage object
Adafruit_USBD_MSC usb_msc;
#endif /* USE_TINYUSB */

// file system object from SdFat
FatFileSystem fatfs;

#if !defined(ARDUINO_ARCH_MBED)
// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
static int32_t RP2040_msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return SPIFlash->readBlocks(lba, (uint8_t*) buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
static int32_t RP2040_msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return SPIFlash->writeBlocks(lba, buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
static void RP2040_msc_flush_cb (void)
{
  // sync with flash
  SPIFlash->syncBlocks();

  // clear file system's cache to force refresh
  fatfs.cacheClear();
}
#endif /* ARDUINO_ARCH_MBED */

static void RP2040_setup()
{
#if !defined(ARDUINO_ARCH_MBED)
  pico_get_unique_board_id(&RP2040_unique_flash_id);
#else
  flash_get_unique_id((uint8_t *)&RP2040_chip_id);
#endif /* ARDUINO_ARCH_MBED */

#if defined(USE_TINYUSB)
  USBDevice.setManufacturerDescriptor(RP2040_Device_Manufacturer);
  USBDevice.setProductDescriptor(RP2040_Device_Model);
  USBDevice.setDeviceVersion(RP2040_Device_Version);
#endif /* USE_TINYUSB */

#if !defined(ARDUINO_ARCH_MBED)
  SerialOutput.setRX(SOC_GPIO_PIN_CONS_RX);
  SerialOutput.setTX(SOC_GPIO_PIN_CONS_TX);
  SerialOutput.setFIFOSize(255);
  SerialOutput.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);

  SerialInput.setRX(SOC_GPIO_PIN_GNSS_RX);
  SerialInput.setTX(SOC_GPIO_PIN_GNSS_TX);
  SerialInput.setFIFOSize(255);
#endif /* ARDUINO_ARCH_MBED */

#if defined(ARDUINO_RASPBERRY_PI_PICO)
  RP2040_board = (SoC->getChipId() == 0xcf516424) ?
                  RP2040_WEACT : RP2040_RPIPICO;
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W)
  RP2040_board = RP2040_RPIPICO_W;
#endif /* ARDUINO_RASPBERRY_PI_PICO */

#if !defined(ARDUINO_ARCH_MBED)
  RP2040_has_spiflash = SPIFlash->begin(possible_devices,
                                        EXTERNAL_FLASH_DEVICE_COUNT);
  if (RP2040_has_spiflash) {
    spiflash_id = SPIFlash->getJEDECID();

    uint32_t capacity = spiflash_id & 0xFF;
    if (capacity >= 0x15) { /* equal or greater than 1UL << 21 (2 MiB) */

#if defined(USE_TINYUSB)
      // Set disk vendor id, product id and revision
      // with string up to 8, 16, 4 characters respectively
      usb_msc.setID(RP2040_Device_Manufacturer, "Internal Flash", "1.0");

      // Set callback
      usb_msc.setReadWriteCallback(RP2040_msc_read_cb,
                                   RP2040_msc_write_cb,
                                   RP2040_msc_flush_cb);

      // Set disk size, block size should be 512 regardless of spi flash page size
      usb_msc.setCapacity(SPIFlash->size()/512, 512);

      // MSC is ready for read/write
      usb_msc.setUnitReady(true);

      usb_msc.begin();
#endif /* USE_TINYUSB */

      FATFS_is_mounted = fatfs.begin(SPIFlash);
    }
  }
#endif /* ARDUINO_ARCH_MBED */

  USBSerial.begin(SERIAL_OUT_BR);

  for (int i=0; i < 20; i++) {if (USBSerial) break; else delay(100);}
}

static void RP2040_fini()
{
  if (RP2040_has_spiflash) {
#if defined(USE_TINYUSB)
    usb_msc.setUnitReady(false);
//  usb_msc.end(); /* N/A */
#endif /* USE_TINYUSB */
  }

#if !defined(ARDUINO_ARCH_MBED)
  if (SPIFlash != NULL) SPIFlash->end();
#endif /* ARDUINO_ARCH_MBED */

#if defined(USE_TINYUSB)
  // Disable USB
  USBDevice.detach();
#endif /* USE_TINYUSB */

  sleep_run_from_xosc();

#if SOC_GPIO_PIN_KEY0 != SOC_UNUSED_PIN
  #if SOC_GPIO_PIN_KEY0 == SOC_GPIO_PIN_BUTTON
    sleep_goto_dormant_until_pin(SOC_GPIO_PIN_KEY0, 0, HIGH);
  #else
    sleep_goto_dormant_until_pin(SOC_GPIO_PIN_KEY0, 0, LOW);
  #endif
#else
  datetime_t alarm = {0};
  sleep_goto_sleep_until(&alarm, NULL);
#endif /* SOC_GPIO_PIN_KEY0 != SOC_UNUSED_PIN */
}

static void RP2040_reset()
{
  NVIC_SystemReset();
}

static uint32_t RP2040_getChipId()
{
  return __builtin_bswap32(RP2040_chip_id[UniqueIDsize - 1]);
}

extern "C" void * _sbrk   (int);

static uint32_t RP2040_getFreeHeap()
{
  char top;
  return &top - reinterpret_cast<char*>(_sbrk(0));
}

static bool RP2040_EEPROM_begin(size_t size)
{
  EEPROM.begin(size);
  return true;
}

static void RP2040_WiFi_setOutputPower(int dB)
{
#if !defined(EXCLUDE_WIFI)
  WiFi.defaultLowPowerMode();
#endif /* EXCLUDE_WIFI */
}

static bool RP2040_WiFi_hostname(String aHostname)
{
#if !defined(EXCLUDE_WIFI)
  WiFi.hostname(aHostname.c_str());
#endif /* EXCLUDE_WIFI */
  return true;
}

static void RP2040_swSer_begin(unsigned long baud)
{
  SerialInput.begin(baud);
}

static void RP2040_swSer_enableRx(boolean arg)
{
  /* NONE */
}

static uint32_t RP2040_maxSketchSpace()
{
  return 1048576; /* TBD */
}

static void RP2040_WiFiUDP_stopAll()
{
#if !defined(EXCLUDE_WIFI)
  WiFiUDP::stopAll();
#endif /* EXCLUDE_WIFI */
}

static void RP2040_Battery_setup()
{

}

static float RP2040_Battery_voltage()
{
  return analogRead (SOC_GPIO_PIN_BATTERY) / SOC_A0_VOLTAGE_DIVIDER ;
}

#include <SoftSPI.h>
SoftSPI swSPI(SOC_EPD_PIN_MOSI_WS,
              SOC_EPD_PIN_MOSI_WS, /* half duplex */
              SOC_EPD_PIN_SCK_WS);

static ep_model_id RP2040_EPD_ident()
{
  ep_model_id rval = EP_GDEW027W3; /* default */

  digitalWrite(SOC_EPD_PIN_SS_WS, HIGH);
  pinMode(SOC_EPD_PIN_SS_WS, OUTPUT);
  digitalWrite(SOC_EPD_PIN_DC_WS, HIGH);
  pinMode(SOC_EPD_PIN_DC_WS, OUTPUT);

  digitalWrite(SOC_EPD_PIN_RST_WS, LOW);
  pinMode(SOC_EPD_PIN_RST_WS, OUTPUT);
  delay(20);
  pinMode(SOC_EPD_PIN_RST_WS, INPUT_PULLUP);
  delay(200);
  pinMode(SOC_EPD_PIN_BUSY_WS, INPUT);

  swSPI.begin();

  digitalWrite(SOC_EPD_PIN_DC_WS,  LOW);
  digitalWrite(SOC_EPD_PIN_SS_WS, LOW);

  swSPI.transfer_out(0x71);

  pinMode(SOC_EPD_PIN_MOSI_WS, INPUT);
  digitalWrite(SOC_EPD_PIN_DC_WS, HIGH);

  uint8_t status = swSPI.transfer_in();

  digitalWrite(SOC_EPD_PIN_SCK_WS, LOW);
  digitalWrite(SOC_EPD_PIN_DC_WS,  LOW);
  digitalWrite(SOC_EPD_PIN_SS_WS,  HIGH);

  swSPI.end();

#if 0
  Serial.print("REG 71H: ");
  Serial.println(status, HEX);
  Serial.flush();
#endif

//  if (status != 2) {
//    rval = EP_GDEY027T91; /* TBD */
//  }

  return rval;
}

static ep_model_id RP2040_display = EP_UNKNOWN;

static void RP2040_EPD_setup()
{
  switch(settings->adapter)
  {
  case ADAPTER_WAVESHARE_PICO:
  default:
    if (RP2040_display == EP_UNKNOWN) {
      RP2040_display = RP2040_EPD_ident();
    }

    switch (RP2040_display)
    {
    case EP_GDEY027T91:
      display = &epd_waveshare_T91;
      break;
    case EP_GDEW027W3:
    default:
      display = &epd_waveshare_W3;
      break;
    }

    SPI1.setRX(SOC_EPD_PIN_MISO_WS);
    SPI1.setTX(SOC_EPD_PIN_MOSI_WS);
    SPI1.setSCK(SOC_EPD_PIN_SCK_WS);
    SPI1.setCS(SOC_EPD_PIN_SS_WS);
    break;
  }
}

static void RP2040_EPD_fini()
{

}

static bool RP2040_EPD_is_ready()
{
  return true;
}

static void RP2040_EPD_update(int val)
{
  EPD_Update_Sync(val);
}

static size_t RP2040_WiFi_Receive_UDP(uint8_t *buf, size_t max_size)
{
#if !defined(EXCLUDE_WIFI)
  return WiFi_Receive_UDP(buf, max_size);
#else
  return 0;
#endif /* EXCLUDE_WIFI */
}

static int RP2040_WiFi_clients_count()
{
#if !defined(EXCLUDE_WIFI)
  struct station_info *stat_info;
  int clients = 0;
  WiFiMode_t mode = WiFi.getMode();

  switch (mode)
  {
#if 0 /* TBD */
  case WIFI_AP:
    stat_info = wifi_softap_get_station_info();

    while (stat_info != NULL) {
      clients++;

      stat_info = STAILQ_NEXT(stat_info, next);
    }
    wifi_softap_free_station_info();

    return clients;
#endif
  case WIFI_STA:
  default:
    return -1; /* error */
  }
#else
  return -1;
#endif /* EXCLUDE_WIFI */
}

static bool RP2040_DB_init()
{
  return false;
}

static bool RP2040_DB_query(uint8_t type, uint32_t id, char *buf, size_t size)
{
  return false;
}

static void RP2040_DB_fini()
{

}

static void RP2040_TTS(char *message)
{
  if (!strcmp(message, "POST")) {
    if (hw_info.display == DISPLAY_EPD_2_7) {
      /* keep boot-time SkyView logo on the screen for 7 seconds */
      delay(7000);
    }
  }
}

#include <AceButton.h>
using namespace ace_button;

AceButton button_mode(SOC_GPIO_PIN_KEY0);
AceButton button_up  (SOC_GPIO_PIN_KEY1);
AceButton button_down(SOC_GPIO_PIN_KEY2);

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
    case AceButton::kEventClicked:
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

static void RP2040_Button_setup()
{
  int mode_button_pin = SOC_GPIO_PIN_KEY0;

  pinMode(mode_button_pin,
          mode_button_pin == SOC_GPIO_PIN_BUTTON ? INPUT : INPUT_PULLUP);

  button_mode.init(mode_button_pin,
                   mode_button_pin == SOC_GPIO_PIN_BUTTON ? LOW : HIGH);

  // Configure the ButtonConfig with the event handler, and enable all higher
  // level events.
  ButtonConfig* ModeButtonConfig = button_mode.getButtonConfig();
  ModeButtonConfig->setEventHandler(handleEvent);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureClick);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
//  ModeButtonConfig->setDebounceDelay(15);
  ModeButtonConfig->setClickDelay(100);
//  ModeButtonConfig->setDoubleClickDelay(1000);
  ModeButtonConfig->setLongPressDelay(2000);

//  attachInterrupt(digitalPinToInterrupt(mode_button_pin), onModeButtonEvent, CHANGE );

  pinMode(SOC_GPIO_PIN_KEY1, INPUT_PULLUP);
  pinMode(SOC_GPIO_PIN_KEY2, INPUT_PULLUP);

  ButtonConfig* UpButtonConfig = button_up.getButtonConfig();
  UpButtonConfig->setEventHandler(handleEvent);
  UpButtonConfig->setFeature(ButtonConfig::kFeatureClick);
  UpButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
//  UpButtonConfig->setDebounceDelay(15);
  UpButtonConfig->setClickDelay(100);
//  UpButtonConfig->setDoubleClickDelay(1000);
  UpButtonConfig->setLongPressDelay(2000);

  ButtonConfig* DownButtonConfig = button_down.getButtonConfig();
  DownButtonConfig->setEventHandler(handleEvent);
  DownButtonConfig->setFeature(ButtonConfig::kFeatureClick);
  DownButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
//  DownButtonConfig->setDebounceDelay(15);
  DownButtonConfig->setClickDelay(100);
//  DownButtonConfig->setDoubleClickDelay(1000);
  DownButtonConfig->setLongPressDelay(2000);

//  attachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_KEY1), onUpButtonEvent,   CHANGE );
//  attachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_KEY2), onDownButtonEvent, CHANGE );
}

#if defined(USE_BOOTSEL_BUTTON)
#define BOOTSEL_CLICK_DELAY                200
#define BOOTSEL_LONGPRESS_DELAY            2000

static unsigned long bootsel_time_marker = 0;
static bool prev_bootsel_state           = false;
static bool is_bootsel_click             = false;
#endif /* USE_BOOTSEL_BUTTON */

static void RP2040_Button_loop()
{
  if (wdt_is_active) {
#if !defined(ARDUINO_ARCH_MBED)
    watchdog_update();
#endif /* ARDUINO_ARCH_MBED */
  }

  button_mode.check();
  button_up.check();
  button_down.check();

#if defined(USE_BOOTSEL_BUTTON)
  if (BOOTSEL) {
    if (!prev_bootsel_state) {
      bootsel_time_marker = millis();
      prev_bootsel_state = true;
    } else {
      if (bootsel_time_marker && !is_bootsel_click &&
          millis() - bootsel_time_marker > BOOTSEL_CLICK_DELAY) {
        is_bootsel_click = true;
      }
      if (bootsel_time_marker &&
          millis() - bootsel_time_marker > BOOTSEL_LONGPRESS_DELAY) {
        shutdown("NORMAL OFF");
//      Serial.println(F("This will never be printed."));
      }
    }
  } else {
    if (prev_bootsel_state) {
      if (is_bootsel_click) {
        EPD_Mode();
        is_bootsel_click = false;
      }
      bootsel_time_marker = 0;
      prev_bootsel_state = false;
    }
  }
#endif /* USE_BOOTSEL_BUTTON */
}

static void RP2040_Button_fini()
{
//  detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_KEY2));
//  detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_KEY1));
//  detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_KEY0));

  int pin = SOC_GPIO_PIN_KEY0;
  while (digitalRead(pin) == (pin == SOC_GPIO_PIN_BUTTON ? HIGH : LOW));

#if defined(USE_BOOTSEL_BUTTON)
  while (BOOTSEL);
#endif /* USE_BOOTSEL_BUTTON */
}

static void RP2040_WDT_setup()
{
#if !defined(ARDUINO_ARCH_MBED)
  watchdog_enable(8000, 1);
#endif /* ARDUINO_ARCH_MBED */
  wdt_is_active = true;
}

static void RP2040_WDT_fini()
{
  if (wdt_is_active) {
#if !defined(ARDUINO_ARCH_MBED)
    hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
#endif /* ARDUINO_ARCH_MBED */
    wdt_is_active = false;
  }
}

const SoC_ops_t RP2040_ops = {
  SOC_RP2040,
  "RP2040",
  RP2040_setup,
  RP2040_fini,
  RP2040_reset,
  RP2040_getChipId,
  RP2040_getFreeHeap,
  RP2040_EEPROM_begin,
  RP2040_WiFi_setOutputPower,
  RP2040_WiFi_hostname,
  RP2040_swSer_begin,
  RP2040_swSer_enableRx,
  RP2040_maxSketchSpace,
  RP2040_WiFiUDP_stopAll,
  RP2040_Battery_setup,
  RP2040_Battery_voltage,
  RP2040_EPD_setup,
  RP2040_EPD_fini,
  RP2040_EPD_is_ready,
  RP2040_EPD_update,
  RP2040_WiFi_Receive_UDP,
  RP2040_WiFi_clients_count,
  RP2040_DB_init,
  RP2040_DB_query,
  RP2040_DB_fini,
  RP2040_TTS,
  RP2040_Button_setup,
  RP2040_Button_loop,
  RP2040_Button_fini,
  RP2040_WDT_setup,
  RP2040_WDT_fini,
  NULL
};

#endif /* ARDUINO_ARCH_RP2040 */
