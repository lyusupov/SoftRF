/*
 * Platform_RP2040.cpp
 * Copyright (C) 2023-2024 Linar Yusupov
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
#include "BatteryHelper.h"

#include "SkyView.h"

#include <hardware/watchdog.h>
#include <Adafruit_SleepyDog.h>

#if !defined(ARDUINO_ARCH_MBED)
#include <pico/unique_id.h>

#define PICO_ON_DEVICE 1
extern "C" {
#include <pico/binary_info.h>
}
#else
extern "C"
{
#include <hardware/flash.h>
}
#endif /* ARDUINO_ARCH_MBED */

#if !defined(ARDUINO_RASPBERRY_PI_PICO_2)
#include <pico_sleep.h>
#include <pico_rosc.h>
#endif /* ARDUINO_RASPBERRY_PI_PICO_2 */

#if defined(USE_TINYUSB)
#if defined(USE_USB_HOST)
#include <pio_usb.h>
#endif /* USE_USB_HOST */
#include <Adafruit_TinyUSB.h>
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
WebServer server ( 80 );

#define isTimeToAP() (millis() - AP_clients_TimeMarker > 5000)
static unsigned long AP_clients_TimeMarker = 0;
static int AP_clients_cache                = 0;
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
static bool wdt_is_active              = false;

static RP2040_board_id RP2040_board    = RP2040_RPIPICO; /* default */
const char *RP2040_Device_Manufacturer = SOFTRF_IDENT;
const char *RP2040_Device_Model        = SKYVIEW_IDENT " Pico";
const uint16_t RP2040_Device_Version   = SKYVIEW_USB_FW_VERSION;

static volatile bool core1_booting     = true;
static volatile bool core0_booting     = true;

#define UniqueIDsize                   2

static union {
#if !defined(ARDUINO_ARCH_MBED)
  pico_unique_board_id_t RP2040_unique_flash_id;
#endif /* ARDUINO_ARCH_MBED */
  uint32_t RP2040_chip_id[UniqueIDsize];
};

#include <Adafruit_SPIFlash.h>
#include <Adafruit_INA219.h>
#include <uCDB.hpp>
#include "JSONHelper.h"

#if !defined(EXCLUDE_AUDIO)
#include <AudioFileSourceSdFat.h>
#include <AudioGeneratorWAV.h>
#if defined(USE_EXT_I2S_DAC)
#include <AudioOutputI2S.h>
#else
#include <AudioOutputPWM.h>
#endif /* USE_EXT_I2S_DAC */
#endif /* EXCLUDE_AUDIO */

#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include <pico/cyw43_arch.h>
#include <boards/pico_w.h>
#endif /* ARDUINO_RASPBERRY_PI_PICO_W */

Adafruit_INA219 ina219(INA219_ADDRESS_ALT);

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
static bool RP2040_has_CPM      = false;
static uint32_t spiflash_id     = 0;
static bool FATFS_is_mounted    = false;
static bool ADB_is_open         = false;

#if defined(USE_TINYUSB)
// USB Mass Storage object
Adafruit_USBD_MSC usb_msc;
#endif /* USE_TINYUSB */

// file system object from SdFat
FatVolume fatfs;

#define RP2040_JSON_BUFFER_SIZE  1024

uCDB<FatVolume, File32> ucdb(fatfs);
StaticJsonBuffer<RP2040_JSON_BUFFER_SIZE> RP2040_jsonBuffer;

#define isTimeToToggle() (millis() - status_LED_TimeMarker > 300)
static unsigned long status_LED_TimeMarker = 0;
static uint8_t LED_state_cache = !(LED_STATE_ON);

#if !defined(EXCLUDE_AUDIO)
AudioGeneratorWAV    *Audio_Gen;
AudioFileSourceSdFat *Audio_Source;
#if defined(USE_EXT_I2S_DAC)
AudioOutputI2S       *Audio_Sink;
#else
AudioOutputPWM       *Audio_Sink;
#endif /* USE_EXT_I2S_DAC */
#endif /* EXCLUDE_AUDIO */

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

#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
  RP2040_board = rp2040.isPicoW() ? RP2040_RPIPICO_W : RP2040_board;
#endif /* ARDUINO_RASPBERRY_PI_PICO_W */

  RP2040_board = (SoC->getChipId() == 0xcf516424) ? RP2040_WEACT : RP2040_board;

  hw_info.revision = RP2040_board == RP2040_RPIPICO_W ? HW_REV_PICO_W : HW_REV_PICO;

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
      hw_info.storage  = FATFS_is_mounted ? STORAGE_FLASH : STORAGE_NONE;
    }
  }
#endif /* ARDUINO_ARCH_MBED */

  Wire1.setSCL(SOC_GPIO_PIN_SCL);
  Wire1.setSDA(SOC_GPIO_PIN_SDA);
  Wire1.begin();
  Wire1.beginTransmission(INA219_ADDRESS_ALT);
  RP2040_has_CPM = (Wire1.endTransmission() == 0);
  Wire1.end();

#if !defined(EXCLUDE_AUDIO)
  Audio_Gen    = new AudioGeneratorWAV();
  Audio_Source = new AudioFileSourceSdFat(fatfs);

#if defined(USE_EXT_I2S_DAC)
  Audio_Sink   = new AudioOutputI2S(11025);
  Audio_Sink->SetPinout(SOC_GPIO_PIN_BCK,  SOC_GPIO_PIN_LRCK,
                        SOC_GPIO_PIN_DATA, SOC_GPIO_PIN_MCK);
#if SOC_GPIO_PIN_MCK != SOC_UNUSED_PIN
  Audio_Sink->SetMclk(true);
#endif /* SOC_GPIO_PIN_MCK */
#else
  Audio_Sink   = new AudioOutputPWM(11025, SOC_GPIO_PIN_PWM_OUT);
  Audio_Sink->SetBuffers(8, 512);
#endif /* USE_EXT_I2S_DAC */

  Audio_Sink->SetOutputModeMono(true);
  Audio_Sink->SetChannels(1);
#endif /* EXCLUDE_AUDIO */

  if (RP2040_has_CPM) {
    ina219.begin(&Wire1);
    ina219.setCalibration_16V_400mA();
  }

  pin_size_t pin_LED = RP2040_board == RP2040_RPIPICO_W ?
                       SOC_GPIO_PIN_LED_W : SOC_GPIO_PIN_LED;
  pinMode(pin_LED, OUTPUT);
  /* Indicate positive power supply */
  LED_state_cache = LED_STATE_ON;
  digitalWrite(pin_LED, LED_state_cache);

  switch (RP2040_board)
  {
  case RP2040_RPIPICO:
    pinMode(SOC_GPIO_PIN_VBUS, INPUT);
    break;
  case RP2040_RPIPICO_W:
    pinMode(SOC_GPIO_PIN_VBUS_W, INPUT);
    break;
  case RP2040_WEACT:
  default:
    break;
  }

  USBSerial.begin(SERIAL_OUT_BR);

  for (int i=0; i < 20; i++) {if (USBSerial) break; else delay(100);}

#if defined(USE_USB_HOST)
  while (core1_booting) {}
#endif /* USE_USB_HOST */
}

static void RP2040_post_init()
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

  switch (RP2040_board)
  {
  case RP2040_RPIPICO_W    : Serial.println(F("Pico W"));   break;
  case RP2040_WEACT        : Serial.println(F("WeAct"));    break;
  case RP2040_RPIPICO      :
  default                  : Serial.println(F("Pico"));     break;
  }

  Serial.print(F("Display      : "));

  if (hw_info.display != DISPLAY_EPD_2_7 || display == NULL) {
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

  core0_booting = false;
}

static void RP2040_loop()
{
  if (wdt_is_active) {
#if !defined(ARDUINO_ARCH_MBED)
    Watchdog.reset();
#endif /* ARDUINO_ARCH_MBED */
  }

  if (isTimeToToggle()) {
    bool vbus_is_on = false;
    pin_size_t pin_LED = RP2040_board == RP2040_RPIPICO_W ?
                         SOC_GPIO_PIN_LED_W : SOC_GPIO_PIN_LED;

    switch (RP2040_board)
    {
    case RP2040_RPIPICO:
      vbus_is_on = digitalRead(SOC_GPIO_PIN_VBUS);
      break;
    case RP2040_RPIPICO_W:
      if (RP2040_board != RP2040_RPIPICO_W || settings->connection != CON_USB) {
        vbus_is_on = digitalRead(SOC_GPIO_PIN_VBUS_W);
      }
      break;
    case RP2040_WEACT:
    default:
      break;
    }

    if (vbus_is_on || Battery_voltage() > Battery_threshold()) {
      /* Indicate positive power supply */
      if (LED_state_cache != LED_STATE_ON) {
        LED_state_cache = LED_STATE_ON;
        digitalWrite(pin_LED, LED_state_cache);
      }
    } else {
      LED_state_cache = !(LED_state_cache); // toggle state
      digitalWrite(pin_LED, LED_state_cache);
    }

    status_LED_TimeMarker = millis();
  }
}

static void RP2040_fini()
{
  if (RP2040_has_CPM) {
    ina219.powerSave(true);
    Wire1.end();
  }

#if !defined(EXCLUDE_AUDIO)
  pinMode(SOC_GPIO_PIN_PWM_OUT, INPUT);
#endif /* EXCLUDE_AUDIO */

  pin_size_t pin_LED = RP2040_board == RP2040_RPIPICO_W ?
                       SOC_GPIO_PIN_LED_W : SOC_GPIO_PIN_LED;
  pinMode(pin_LED, INPUT);

#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
  if (RP2040_board == RP2040_RPIPICO_W) {
    if (cyw43_is_initialized(&cyw43_state)) cyw43_arch_deinit();
    pinMode(CYW43_PIN_WL_REG_ON, INPUT_PULLDOWN);
  }
#endif /* ARDUINO_RASPBERRY_PI_PICO_W */

  if (RP2040_has_spiflash) {
#if defined(USE_TINYUSB)
    usb_msc.setUnitReady(false);
//  usb_msc.end(); /* N/A */
#endif /* USE_TINYUSB */
  }

#if !defined(ARDUINO_ARCH_MBED)
  if (SPIFlash != NULL) SPIFlash->end();
#endif /* ARDUINO_ARCH_MBED */

  SerialInput.end();
  SerialOutput.end();
  USBSerial.end();

#if defined(USE_TINYUSB)
  // Disable USB
  USBDevice.detach();
#endif /* USE_TINYUSB */

#if !defined(ARDUINO_RASPBERRY_PI_PICO_2)
  sleep_run_from_xosc();

#if SOC_GPIO_PIN_KEY1 != SOC_UNUSED_PIN
  uint mode_button_pin = SOC_GPIO_PIN_KEY1;

  #if SOC_GPIO_PIN_KEY1 == SOC_GPIO_PIN_BUTTON
    sleep_goto_dormant_until_pin(mode_button_pin, 0, HIGH);
  #else
    sleep_goto_dormant_until_pin(mode_button_pin, 0, LOW);
  #endif
#else
  datetime_t alarm = {0};
  sleep_goto_sleep_until(&alarm, NULL); /* TBD */
#endif /* SOC_GPIO_PIN_KEY1 != SOC_UNUSED_PIN */

  // back from dormant state
  rosc_enable();
#endif /* ARDUINO_RASPBERRY_PI_PICO_2 */

#if PICO_SDK_VERSION_MAJOR < 2
  clocks_init();
#endif /* PICO_SDK_VERSION_MAJOR */

  rp2040.restart();
}

static void RP2040_reset()
{
  rp2040.restart();
}

static uint32_t RP2040_getChipId()
{
  return __builtin_bswap32(RP2040_chip_id[UniqueIDsize - 1]);
}

static uint32_t RP2040_getFreeHeap()
{
  return rp2040.getFreeHeap();
}

static bool RP2040_EEPROM_begin(size_t size)
{
  EEPROM.begin(size);
  return true;
}

static void RP2040_EEPROM_extension(int cmd)
{
  if (cmd == EEPROM_EXT_LOAD) {

    if ( RP2040_has_spiflash && FATFS_is_mounted ) {
      File32 file = fatfs.open("/settings.json", FILE_READ);

      if (file) {
        // StaticJsonBuffer<RP2040_JSON_BUFFER_SIZE> RP2040_jsonBuffer;

        JsonObject &root = RP2040_jsonBuffer.parseObject(file);

        if (root.success()) {
          JsonVariant msg_class = root["class"];

          if (msg_class.success()) {
            const char *msg_class_s = msg_class.as<char*>();

            if (!strcmp(msg_class_s,"SKYVIEW")) {
              parseSettings(root);
            }
          }
        }
        file.close();
      }
    }

    if (RP2040_board != RP2040_RPIPICO_W &&
        (settings->connection == CON_BLUETOOTH_SPP ||
         settings->connection == CON_BLUETOOTH_LE)) {
      settings->connection = CON_USB; /* matches EEPROMHelper.cpp default value */
    }
  }
}

static void RP2040_WiFi_setOutputPower(int dB)
{
#if !defined(EXCLUDE_WIFI)
  WiFi.defaultLowPowerMode();
#endif /* EXCLUDE_WIFI */
}

static bool RP2040_WiFi_hostname(String aHostname)
{
  bool rval = false;
#if !defined(EXCLUDE_WIFI)
  if (RP2040_board == RP2040_RPIPICO_W) {
    WiFi.hostname(aHostname.c_str());
    rval = true;
  }
#endif /* EXCLUDE_WIFI */
  return rval;
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
#if SOC_GPIO_PIN_BATTERY != SOC_UNUSED_PIN
  analogReadResolution(12);
  analogRead(SOC_GPIO_PIN_BATTERY);
#endif
}

#include <hardware/gpio.h>
#include <hardware/adc.h>

static float RP2040_Battery_voltage()
{
  uint16_t mV = 0;

  if (RP2040_has_CPM) {
    float shuntvoltage = ina219.getShuntVoltage_mV();
    float busvoltage   = ina219.getBusVoltage_V();

    return (busvoltage + (shuntvoltage / 1000));
  }

#if SOC_GPIO_PIN_BATTERY != SOC_UNUSED_PIN
#if PICO_SDK_VERSION_MAJOR < 2
  enum gpio_function pin25_func;
  enum gpio_function pin29_func;
#else
  gpio_function_t pin25_func;
  gpio_function_t pin29_func;
#endif /* PICO_SDK_VERSION_MAJOR */
  uint pin25_dir;
  uint pin29_dir;

  if (RP2040_board == RP2040_RPIPICO_W) {
    pin29_dir  = gpio_get_dir(SOC_GPIO_PIN_BATTERY);
    pin29_func = gpio_get_function(SOC_GPIO_PIN_BATTERY);
    adc_gpio_init(SOC_GPIO_PIN_BATTERY);

    pin25_dir  = gpio_get_dir(SOC_GPIO_PIN_CYW43_EN);
    pin25_func = gpio_get_function(SOC_GPIO_PIN_CYW43_EN);
    pinMode(SOC_GPIO_PIN_CYW43_EN, OUTPUT);
    digitalWrite(SOC_GPIO_PIN_CYW43_EN, HIGH);
  }

  mV = (analogRead(SOC_GPIO_PIN_BATTERY) * 3300UL) >> 12;

  if (RP2040_board == RP2040_RPIPICO_W) {
    gpio_set_function(SOC_GPIO_PIN_CYW43_EN, pin25_func);
    gpio_set_dir(SOC_GPIO_PIN_CYW43_EN, pin25_dir);
    gpio_set_function(SOC_GPIO_PIN_BATTERY,  pin29_func);
    gpio_set_dir(SOC_GPIO_PIN_BATTERY,  pin29_dir);
  }
#endif

  float mult = (RP2040_board == RP2040_WEACT) ? 1 : SOC_ADC_VOLTAGE_DIV;
  return (mV * mult / 1000.0);
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

  uint8_t buf_2D[11];
  uint8_t buf_2E[10];

  digitalWrite(SOC_EPD_PIN_DC_WS,  LOW);
  digitalWrite(SOC_EPD_PIN_SS_WS, LOW);

  swSPI.transfer_out(0x2D);

  pinMode(SOC_EPD_PIN_MOSI_WS, INPUT);
  digitalWrite(SOC_EPD_PIN_DC_WS, HIGH);

  for (int i=0; i<sizeof(buf_2D); i++) {
    buf_2D[i] = swSPI.transfer_in();
  }

  digitalWrite(SOC_EPD_PIN_SCK_WS, LOW);
  digitalWrite(SOC_EPD_PIN_DC_WS,  LOW);
  digitalWrite(SOC_EPD_PIN_SS_WS,  HIGH);

  delay(1);

  digitalWrite(SOC_EPD_PIN_DC_WS,  LOW);
  digitalWrite(SOC_EPD_PIN_SS_WS, LOW);

  swSPI.transfer_out(0x2E);

  pinMode(SOC_EPD_PIN_MOSI_WS, INPUT);
  digitalWrite(SOC_EPD_PIN_DC_WS, HIGH);

  for (int i=0; i<sizeof(buf_2E); i++) {
    buf_2E[i] = swSPI.transfer_in();
  }

  digitalWrite(SOC_EPD_PIN_SCK_WS, LOW);
  digitalWrite(SOC_EPD_PIN_DC_WS,  LOW);
  digitalWrite(SOC_EPD_PIN_SS_WS,  HIGH);

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
 *  FF FF FF FF FF FF FF FF FF FF FF - W3
 *  00 00 00 FF 00 00 40 01 00 00 00
 *  00 00 00 00 00 00 00 00 00 00 00 - T91
 *
 *  0x2E:
 *  FF FF FF FF FF FF FF FF FF FF    - W3
 *  FF FF FF FF FF FF FF FF FF FF
 *  00 00 00 00 00 00 00 00 00 00    - T91
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

  if (is_ff) {
    rval = EP_GDEW027W3;
  } else {
    rval = EP_GDEY027T91;
  }

  return rval;
}

static ep_model_id RP2040_display = EP_UNKNOWN;

static void RP2040_EPD_setup()
{
  switch(settings->adapter)
  {
  case ADAPTER_WAVESHARE_PICO_2_7_V2:
    display = &epd_waveshare_T91;

    SPI1.setRX(SOC_EPD_PIN_MISO_WS);
    SPI1.setTX(SOC_EPD_PIN_MOSI_WS);
    SPI1.setSCK(SOC_EPD_PIN_SCK_WS);
    SPI1.setCS(SOC_EPD_PIN_SS_WS);

    display->epd2.selectSPI(SPI1, SPISettings(4000000, MSBFIRST, SPI_MODE0));
    break;
  case ADAPTER_WAVESHARE_PICO_2_7:
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

    display->epd2.selectSPI(SPI1, SPISettings(4000000, MSBFIRST, SPI_MODE0));
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

static void RP2040_EPD_Busy_Callback(const void* p)
{
  if (wdt_is_active) {
#if !defined(ARDUINO_ARCH_MBED)
    Watchdog.reset();
#endif /* ARDUINO_ARCH_MBED */
  }

  if (SoC->Bluetooth_ops) {
    SoC->Bluetooth_ops->loop();
  }

  if (SoC->USB_ops) {
    SoC->USB_ops->loop();
  }

  Input_loop();

  // Traffic_ClearExpired();

  // WiFi_loop();

  // Handle Web
  // Web_loop();

  // SoC->Button_loop();
  // button_up.check();
  // button_down.check();

  yield();
}

static void RP2040_EPD_update(int val)
{
  display->epd2.setBusyCallback(RP2040_EPD_Busy_Callback);
  EPD_Update_Sync(val);
  display->epd2.setBusyCallback(NULL);
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
  WiFiMode_t mode = WiFi.getMode();

  switch (mode)
  {
  case WIFI_AP:
    if (isTimeToAP()) {
      AP_clients_cache = WiFi.softAPgetStationNum();
      AP_clients_TimeMarker = millis();
    }
    return AP_clients_cache;
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

  return ADB_is_open;
}

static bool RP2040_DB_query(uint8_t type, uint32_t id, char *buf, size_t size)
{
  bool rval = false;

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

  return rval;
}

static void RP2040_DB_fini()
{
  if (ADB_is_open) {
    ucdb.close();
    ADB_is_open = false;
  }
}

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

static void RP2040_TTS(char *message)
{
  char filename[MAX_FILENAME_LEN];

  if (strcmp(message, "POST")) {
    if ( settings->voice   != VOICE_OFF                  &&
        (settings->adapter == ADAPTER_WAVESHARE_PICO_2_7 ||
         settings->adapter == ADAPTER_WAVESHARE_PICO_2_7_V2)) {

      if (!FATFS_is_mounted)
        return;

      if (wdt_is_active) {
        Watchdog.reset();
      }

      if (hw_info.display == DISPLAY_EPD_2_7) {
        while (!SoC->EPD_is_ready()) {yield();}
        EPD_Message("VOICE", "ALERT");
        SoC->EPD_update(EPD_UPDATE_FAST);
        while (!SoC->EPD_is_ready()) {yield();}
      }

      char *word = strtok (message, " ");

      while (word != NULL)
      {
          if (wdt_is_active) {
            Watchdog.reset();
          }

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
    }
  } else {
    if ( settings->voice   != VOICE_OFF                  &&
        (settings->adapter == ADAPTER_WAVESHARE_PICO_2_7 ||
         settings->adapter == ADAPTER_WAVESHARE_PICO_2_7_V2)) {

      strcpy(filename, WAV_FILE_PREFIX);
      strcat(filename, "POST");
      strcat(filename, WAV_FILE_SUFFIX);

      if (!FATFS_is_mounted || !play_file(filename)) {
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

#if SOC_GPIO_PIN_KEY1 != SOC_UNUSED_PIN
AceButton button_mode(SOC_GPIO_PIN_KEY1);
#endif /* SOC_GPIO_PIN_KEY1 != SOC_UNUSED_PIN */
AceButton button_up  (SOC_GPIO_PIN_KEY2);
AceButton button_down(SOC_GPIO_PIN_KEY0);

// The event handler for the button.
void handleEvent(AceButton* button, uint8_t eventType,
    uint8_t buttonState) {

#if 0
  // Print out a message for all events.
#if SOC_GPIO_PIN_KEY1 != SOC_UNUSED_PIN
  if (button == &button_mode) {
    Serial.print(F("MODE "));
  } else
#endif /* SOC_GPIO_PIN_KEY1 != SOC_UNUSED_PIN */
  if (button == &button_up) {
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
#if SOC_GPIO_PIN_KEY1 != SOC_UNUSED_PIN
      if (button == &button_mode) {
        EPD_Mode();
      } else
#endif /* SOC_GPIO_PIN_KEY1 != SOC_UNUSED_PIN */
      if (button == &button_up) {
        EPD_Up();
      } else if (button == &button_down) {
        EPD_Down();
      }
      break;
    case AceButton::kEventLongPressed:
#if SOC_GPIO_PIN_KEY1 != SOC_UNUSED_PIN
      if (button == &button_mode) {

        if (digitalRead(SOC_GPIO_PIN_KEY2) == LOW) {
          screen_saver = true;
        }

        shutdown("NORMAL OFF");
//        Serial.println(F("This will never be printed."));
      }
#endif /* SOC_GPIO_PIN_KEY1 != SOC_UNUSED_PIN */
      break;
  }
}

/* Callbacks for push button interrupt */
void onModeButtonEvent() {
#if SOC_GPIO_PIN_KEY1 != SOC_UNUSED_PIN
  button_mode.check();
#endif /* SOC_GPIO_PIN_KEY1 != SOC_UNUSED_PIN */
}

void onUpButtonEvent() {
  button_up.check();
}

void onDownButtonEvent() {
  button_down.check();
}

static void RP2040_Button_setup()
{
#if SOC_GPIO_PIN_KEY1 != SOC_UNUSED_PIN
  int mode_button_pin = SOC_GPIO_PIN_KEY1;

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
#endif /* SOC_GPIO_PIN_KEY1 != SOC_UNUSED_PIN */

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

//  attachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_KEY2), onUpButtonEvent,   CHANGE );
//  attachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_KEY0), onDownButtonEvent, CHANGE );
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
#if SOC_GPIO_PIN_KEY1 != SOC_UNUSED_PIN
  button_mode.check();
#endif /* SOC_GPIO_PIN_KEY1 != SOC_UNUSED_PIN */
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
//  detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_KEY0));
//  detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_KEY1));
//  detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_KEY2));

#if SOC_GPIO_PIN_KEY1 != SOC_UNUSED_PIN
  int pin = SOC_GPIO_PIN_KEY1;
  while (digitalRead(pin) == (pin == SOC_GPIO_PIN_BUTTON ? HIGH : LOW));
#endif /* SOC_GPIO_PIN_KEY1 != SOC_UNUSED_PIN */

#if defined(USE_BOOTSEL_BUTTON)
  while (BOOTSEL);
#endif /* USE_BOOTSEL_BUTTON */
}

static void RP2040_WDT_setup()
{
#if !defined(ARDUINO_ARCH_MBED)
  Watchdog.enable(8000);
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

#include <api/RingBuffer.h>

#define USB_TX_FIFO_SIZE (256)
#define USB_RX_FIFO_SIZE (MAX_TRACKING_OBJECTS * 65 + 75 + 75 + 42 + 20)

#if !defined(USBD_CDC_IN_OUT_MAX_SIZE)
#define USBD_CDC_IN_OUT_MAX_SIZE (64)
#endif /* USBD_CDC_IN_OUT_MAX_SIZE */

RingBufferN<USB_TX_FIFO_SIZE> USB_TX_FIFO = RingBufferN<USB_TX_FIFO_SIZE>();
RingBufferN<USB_RX_FIFO_SIZE> USB_RX_FIFO = RingBufferN<USB_RX_FIFO_SIZE>();

#if defined(USE_USB_HOST)
/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#define LANGUAGE_ID 0x0409  // English

// USB Host object
Adafruit_USBH_Host USBHost;

// CDC Host object
Adafruit_USBH_CDC  SerialHost;

// holding device descriptor
tusb_desc_device_t desc_device;

void setup1() {

  // Check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if ( cpu_hz != 120000000UL && cpu_hz != 240000000UL ) {
    while ( !Serial ) delay(10);   // wait for native usb
    Serial.printf("Error: CPU Clock = %u, PIO USB require CPU clock must be multiple of 120 Mhz\r\n", cpu_hz);
    Serial.printf("Change your CPU Clock to either 120 or 240 Mhz in Menu->CPU Speed \r\n", cpu_hz);
    while(1) delay(1);
  }

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp     = SOC_GPIO_PIN_USBH_DP;
  pio_cfg.sm_tx      = 3;
  pio_cfg.sm_rx      = 2;
  pio_cfg.sm_eop     = 3;
  pio_cfg.pio_rx_num = 0;
  pio_cfg.pio_tx_num = 1;
  pio_cfg.tx_ch      = 9;
  USBHost.configure_pio_usb(1, &pio_cfg);

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  USBHost.begin(1);

  SerialHost.begin(SERIAL_OUT_BR);

  core1_booting = false;
  while (core0_booting) { }
}

// core1's loop
void loop1()
{
  USBHost.task();

#if 0
  uint8_t buf[64];
  // SerialHost -> Serial
  if ( SerialHost.connected() && SerialHost.available() ) {
    size_t count = SerialHost.read(buf, sizeof(buf));
//Serial.println(count);
    Serial.write(buf, count);
  }
#endif

  // periodically flush SerialHost if connected
  if ( SerialHost && SerialHost.connected() ) {
    SerialHost.flush();
  }
}

//--------------------------------------------------------------------+
// String Descriptor Helper
//--------------------------------------------------------------------+

static void _convert_utf16le_to_utf8(const uint16_t *utf16, size_t utf16_len, uint8_t *utf8, size_t utf8_len) {
  // TODO: Check for runover.
  (void)utf8_len;
  // Get the UTF-16 length out of the data itself.

  for (size_t i = 0; i < utf16_len; i++) {
    uint16_t chr = utf16[i];
    if (chr < 0x80) {
      *utf8++ = chr & 0xff;
    } else if (chr < 0x800) {
      *utf8++ = (uint8_t)(0xC0 | (chr >> 6 & 0x1F));
      *utf8++ = (uint8_t)(0x80 | (chr >> 0 & 0x3F));
    } else {
      // TODO: Verify surrogate.
      *utf8++ = (uint8_t)(0xE0 | (chr >> 12 & 0x0F));
      *utf8++ = (uint8_t)(0x80 | (chr >> 6 & 0x3F));
      *utf8++ = (uint8_t)(0x80 | (chr >> 0 & 0x3F));
    }
    // TODO: Handle UTF-16 code points that take two entries.
  }
}

// Count how many bytes a utf-16-le encoded string will take in utf-8.
static int _count_utf8_bytes(const uint16_t *buf, size_t len) {
  size_t total_bytes = 0;
  for (size_t i = 0; i < len; i++) {
    uint16_t chr = buf[i];
    if (chr < 0x80) {
      total_bytes += 1;
    } else if (chr < 0x800) {
      total_bytes += 2;
    } else {
      total_bytes += 3;
    }
    // TODO: Handle UTF-16 code points that take two entries.
  }
  return total_bytes;
}

static void print_utf16(uint16_t *temp_buf, size_t buf_len) {
  size_t utf16_len = ((temp_buf[0] & 0xff) - 2) / sizeof(uint16_t);
  size_t utf8_len = _count_utf8_bytes(temp_buf + 1, utf16_len);

  _convert_utf16le_to_utf8(temp_buf + 1, utf16_len, (uint8_t *) temp_buf, sizeof(uint16_t) * buf_len);
  ((uint8_t*) temp_buf)[utf8_len] = '\0';

  Serial.printf((char*)temp_buf);
}

void print_device_descriptor(tuh_xfer_t* xfer)
{
  if ( XFER_RESULT_SUCCESS != xfer->result )
  {
    Serial.printf("Failed to get device descriptor\r\n");
    return;
  }

  uint8_t const daddr = xfer->daddr;

  Serial.printf("Device %u: ID %04x:%04x\r\n", daddr, desc_device.idVendor, desc_device.idProduct);
  Serial.printf("Device Descriptor:\r\n");
  Serial.printf("  bLength             %u\r\n"     , desc_device.bLength);
  Serial.printf("  bDescriptorType     %u\r\n"     , desc_device.bDescriptorType);
  Serial.printf("  bcdUSB              %04x\r\n"   , desc_device.bcdUSB);
  Serial.printf("  bDeviceClass        %u\r\n"     , desc_device.bDeviceClass);
  Serial.printf("  bDeviceSubClass     %u\r\n"     , desc_device.bDeviceSubClass);
  Serial.printf("  bDeviceProtocol     %u\r\n"     , desc_device.bDeviceProtocol);
  Serial.printf("  bMaxPacketSize0     %u\r\n"     , desc_device.bMaxPacketSize0);
  Serial.printf("  idVendor            0x%04x\r\n" , desc_device.idVendor);
  Serial.printf("  idProduct           0x%04x\r\n" , desc_device.idProduct);
  Serial.printf("  bcdDevice           %04x\r\n"   , desc_device.bcdDevice);

  // Get String descriptor using Sync API
  uint16_t temp_buf[128];

  Serial.printf("  iManufacturer       %u     "     , desc_device.iManufacturer);
  if (XFER_RESULT_SUCCESS == tuh_descriptor_get_manufacturer_string_sync(daddr, LANGUAGE_ID, temp_buf, sizeof(temp_buf)) )
  {
    print_utf16(temp_buf, TU_ARRAY_SIZE(temp_buf));
  }
  Serial.printf("\r\n");

  Serial.printf("  iProduct            %u     "     , desc_device.iProduct);
  if (XFER_RESULT_SUCCESS == tuh_descriptor_get_product_string_sync(daddr, LANGUAGE_ID, temp_buf, sizeof(temp_buf)))
  {
    print_utf16(temp_buf, TU_ARRAY_SIZE(temp_buf));
  }
  Serial.printf("\r\n");

  Serial.printf("  iSerialNumber       %u     "     , desc_device.iSerialNumber);
  if (XFER_RESULT_SUCCESS == tuh_descriptor_get_serial_string_sync(daddr, LANGUAGE_ID, temp_buf, sizeof(temp_buf)))
  {
    print_utf16(temp_buf, TU_ARRAY_SIZE(temp_buf));
  }
  Serial.printf("\r\n");

  Serial.printf("  bNumConfigurations  %u\r\n"     , desc_device.bNumConfigurations);
}

//--------------------------------------------------------------------+
// TinyUSB Host callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted (configured)
void tuh_mount_cb (uint8_t daddr)
{
  Serial.printf("Device attached, address = %d\r\n", daddr);

  // Get Device Descriptor
  tuh_descriptor_get_device(daddr, &desc_device, 18, print_device_descriptor, 0);
}

/// Invoked when device is unmounted (bus reset/unplugged)
void tuh_umount_cb(uint8_t daddr)
{
  Serial.printf("Device removed, address = %d\r\n", daddr);
}

extern "C" {

// Invoked when a device with CDC interface is mounted
// idx is index of cdc interface in the internal pool.
void tuh_cdc_mount_cb(uint8_t idx) {

  // bind SerialHost object to this interface index
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

  SerialHost.mount(idx);

  SerialHost.setBaudrate(SerialBaud);
  SerialHost.setDtrRts(true, false);

  Serial.println("SerialHost is connected to a new CDC device");
}

// Invoked when a device with CDC interface is unmounted
void tuh_cdc_umount_cb(uint8_t idx) {
  SerialHost.umount(idx);
  Serial.println("SerialHost is disconnected");
}

} // extern "C"

#endif /* USE_USB_HOST */

static void RP2040_USB_setup()
{
#if defined(USE_USB_HOST)
  /* TBD */
#elif !defined(ARDUINO_ARCH_MBED)
  if (USBSerial && USBSerial != Serial) {
    USBSerial.begin(SERIAL_OUT_BR);
  }
#endif /* ARDUINO_ARCH_MBED */
}

static void RP2040_USB_loop()
{
#if defined(USE_USB_HOST)
  uint8_t buf[USBD_CDC_IN_OUT_MAX_SIZE];
  size_t size;

  while (SerialHost             &&
         SerialHost.connected() &&
         (size = SerialHost.availableForWrite()) > 0) {
    size_t avail = USB_TX_FIFO.available();

    if (avail == 0) {
      break;
    }

    if (size > avail) {
      size = avail;
    }

    if (size > sizeof(buf)) {
      size = sizeof(buf);
    }

    for (size_t i=0; i < size; i++) {
      buf[i] = USB_TX_FIFO.read_char();
    }

    if (SerialHost && SerialHost.connected()) {
      SerialHost.write(buf, size);
    }
  }

  if ( SerialHost             &&
       SerialHost.connected() &&
       (size = USB_RX_FIFO.availableForStore()) > 0 ) {
    if (size > sizeof(buf)) {
      size = sizeof(buf);
    }
    size_t avail = SerialHost.available();
    if (size > avail) {
      size = avail;
    }

    size_t count = SerialHost.read(buf, size);
//    Serial.write(buf, count);
    for (size_t i=0; i < count; i++) {
      if (!USB_RX_FIFO.isFull()) {
        USB_RX_FIFO.store_char(buf[i]);
      }
    }
  }
#elif !defined(USE_TINYUSB)
  uint8_t buf[USBD_CDC_IN_OUT_MAX_SIZE];
  size_t size;

  while (USBSerial && (size = USBSerial.availableForWrite()) > 0) {
    size_t avail = USB_TX_FIFO.available();

    if (avail == 0) {
      break;
    }

    if (size > avail) {
      size = avail;
    }

    if (size > sizeof(buf)) {
      size = sizeof(buf);
    }

    for (size_t i=0; i < size; i++) {
      buf[i] = USB_TX_FIFO.read_char();
    }

    if (USBSerial) {
      USBSerial.write(buf, size);
    }
  }

  while (USBSerial && USBSerial.available() > 0) {
    if (!USB_RX_FIFO.isFull()) {
      USB_RX_FIFO.store_char(USBSerial.read());
    } else {
      break;
    }
  }
#endif /* USE_TINYUSB */
}

static void RP2040_USB_fini()
{
#if defined(USE_USB_HOST)
  /* TBD */
#elif !defined(ARDUINO_ARCH_MBED)
  if (USBSerial && USBSerial != Serial) {
    USBSerial.end();
  }
#endif /* ARDUINO_ARCH_MBED */
}

static int RP2040_USB_available()
{
  int rval = 0;

#if defined(USE_USB_HOST) || !defined(USE_TINYUSB)
  rval = USB_RX_FIFO.available();
#else
  if (USBSerial) {
    rval = USBSerial.available();
  }
#endif /* USE_TINYUSB */

  return rval;
}

static int RP2040_USB_read()
{
  int rval = -1;

#if defined(USE_USB_HOST) || !defined(USE_TINYUSB)
  rval = USB_RX_FIFO.read_char();
#else
  if (USBSerial) {
    rval = USBSerial.read();
  }
#endif /* USE_TINYUSB */

  return rval;
}

static size_t RP2040_USB_write(const uint8_t *buffer, size_t size)
{
#if defined(USE_USB_HOST) || !defined(USE_TINYUSB)
  size_t written;

  for (written=0; written < size; written++) {
    if (!USB_TX_FIFO.isFull()) {
      USB_TX_FIFO.store_char(buffer[written]);
    } else {
      break;
    }
  }
  return written;
#else
  size_t rval = size;

  if (USBSerial && (size < USBSerial.availableForWrite())) {
    rval = USBSerial.write(buffer, size);
  }

  return rval;
#endif /* USE_TINYUSB */
}

IODev_ops_t RP2040_USBSerial_ops = {
  "RP2040 USB ACM",
  RP2040_USB_setup,
  RP2040_USB_loop,
  RP2040_USB_fini,
  RP2040_USB_available,
  RP2040_USB_read,
  RP2040_USB_write
};

const SoC_ops_t RP2040_ops = {
  SOC_RP2040,
  "RP2040",
  RP2040_setup,
  RP2040_post_init,
  RP2040_loop,
  RP2040_fini,
  RP2040_reset,
  RP2040_getChipId,
  RP2040_getFreeHeap,
  RP2040_EEPROM_begin,
  RP2040_EEPROM_extension,
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
#if !defined(EXCLUDE_BLUETOOTH)
  &CYW43_Bluetooth_ops,
#else
  NULL,
#endif /* EXCLUDE_BLUETOOTH */
  &RP2040_USBSerial_ops,
};

#endif /* ARDUINO_ARCH_RP2040 */
