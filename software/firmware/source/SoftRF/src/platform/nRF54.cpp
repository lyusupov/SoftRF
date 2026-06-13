/*
 * Platform_nRF54.cpp
 * Copyright (C) 2026 Linar Yusupov
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

#if defined(ARDUINO_ARCH_NRF54L15CLEAN)

#include <SPI.h>
#include <Wire.h>

#include <ArduinoJson.h>

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

#if defined(USE_OLED)
#include "../driver/OLED.h"
#endif /* USE_OLED */

#if defined(ARDUINO_XIAO_NRF54L15_CLEAN)    || \
    defined(ARDUINO_XIAO_NRF54LM20A_CLEAN)  || \
    defined(ARDUINO_XIAO_NRF54LM20B_CLEAN)  || \
    defined(ARDUINO_HOLYIOT_25007_NRF54L15) || \
    defined(ARDUINO_NRF54L15DK_PCA10156)    || \
    defined(ARDUINO_GENERIC_NRF54L15_MODULE_36PIN)
#include <variant.h>
#include "nrf54l15_hal.h"

using namespace xiao_nrf54l15;
#else
#error "This nRF54 build variant is not supported!"
#endif /* ARDUINO_NRF54L15 */

// RFM95W pin mapping
lmic_pinmap lmic_pins = {
    .nss = LMIC_UNUSED_PIN,
    .txe = LMIC_UNUSED_PIN,
    .rxe = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
    .busy = LMIC_UNUSED_PIN,
    .tcxo = LMIC_UNUSED_PIN,
};

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

static uint32_t bootCount __attribute__ ((section (".noinit")));
static Watchdog g_wdt;
static bool wdt_is_active = false;

static nRF54_board_id nRF54_board = NRF54_LR2021EVK1XCS1; /* default */

const char *nRF5x_Device_Manufacturer = SOFTRF_IDENT;
const char *nRF5x_Device_Model = "Academy Edition";

const char *Hardware_Rev[] = {
  [0] = "Unknown",
  [1] = "Unknown",
  [2] = "Unknown",
  [3] = "Unknown"
};

#if defined(EXCLUDE_EEPROM)
eeprom_t eeprom_block;
settings_t *settings = &eeprom_block.field.settings;
#endif /* EXCLUDE_EEPROM */

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
char UDPpacketBuffer[256]; // Dummy definition to satisfy build sequence
#endif /* EXCLUDE_WIFI */

#if defined(USE_SOFTSPI)
#include <SoftSPI.h>
SoftSPI RadioSPI(SOC_GPIO_PIN_EVK_MOSI,
                 SOC_GPIO_PIN_EVK_MISO,
                 SOC_GPIO_PIN_EVK_SCK);
#endif /* USE_SOFTSPI */

#if defined(USE_RTT)
#include <RTTStream.h>

RTTStream RTTSerial;
#endif /* USE_RTT */

char *dtostrf_workaround(double number, signed char width, unsigned char prec, char *s) {
    bool negative = false;

    if (isnan(number)) {
        strcpy(s, "nan");
        return s;
    }
    if (isinf(number)) {
        strcpy(s, "inf");
        return s;
    }

    char* out = s;

    int fillme = width; // how many cells to fill for the integer part
    if (prec > 0) {
        fillme -= (prec+1);
    }

    // Handle negative numbers
    if (number < 0.0) {
        negative = true;
        fillme--;
        number = -number;
    }

    // Round correctly so that print(1.999, 2) prints as "2.00"
    // I optimized out most of the divisions
    double rounding = 2.0;
    for (uint8_t i = 0; i < prec; ++i)
        rounding *= 10.0;
    rounding = 1.0 / rounding;

    number += rounding;

    // Figure out how big our number really is
    double tenpow = 1.0;
    int digitcount = 1;
    while (number >= 10.0 * tenpow) {
        tenpow *= 10.0;
        digitcount++;
    }

    number /= tenpow;
    fillme -= digitcount;

    // Pad unused cells with spaces
    while (fillme-- > 0) {
        *out++ = ' ';
    }

    // Handle negative sign
    if (negative) *out++ = '-';

    // Print the digits, and if necessary, the decimal point
    digitcount += prec;
    int8_t digit = 0;
    while (digitcount-- > 0) {
        digit = (int8_t)number;
        if (digit > 9) digit = 9; // insurance
        *out++ = (char)('0' | digit);
        if ((digitcount == prec) && (prec > 0)) {
            *out++ = '.';
        }
        number -= digit;
        number *= 10.0;
    }

    // make sure the string is terminated
    *out = 0;
    return s;
}

#ifdef NRF_TRUSTZONE_NONSECURE
static constexpr uintptr_t kPowerBase = 0x4010E000UL;
static constexpr uintptr_t kResetBase = 0x4010E000UL;
static constexpr uintptr_t kRegulatorsBase = 0x40120000UL;
#else
static constexpr uintptr_t kPowerBase = 0x5010E000UL;
static constexpr uintptr_t kResetBase = 0x5010E000UL;
static constexpr uintptr_t kRegulatorsBase = 0x50120000UL;
#endif

static NRF_POWER_Type* const g_power =
    reinterpret_cast<NRF_POWER_Type*>(kPowerBase);
static NRF_RESET_Type* const g_reset =
    reinterpret_cast<NRF_RESET_Type*>(kResetBase);
static NRF_REGULATORS_Type* const g_regulators =
    reinterpret_cast<NRF_REGULATORS_Type*>(kRegulatorsBase);

static inline void cpuIdleWfi() {
  __asm volatile("wfi");
}

static uint32_t readResetReason() {
  return g_reset->RESETREAS;
}

static uint8_t readGpregret0() {
  return static_cast<uint8_t>(g_power->GPREGRET[0] &
                              POWER_GPREGRET_GPREGRET_Msk);
}

#if defined(ARDUINO_XIAO_NRF54LM20A_CLEAN)
#include <Adafruit_FlashTransport_QSPI_NRF54.h>
static Adafruit_FlashTransport_QSPI_NRF54 *FlashTrans = NULL;
#else
#include <Adafruit_FlashTransport_SPI.h>
static Adafruit_FlashTransport_SPI        *FlashTrans = NULL;
#endif /* ARDUINO_XIAO_NRF54LM20A_CLEAN */

#include <Adafruit_SPIFlash.h>
static Adafruit_SPIFlash                  *SPIFlash   = NULL;
static bool nRF54_has_spiflash                        = false;

/// Flash device list count
enum {
  MX25R6435F_INDEX,
  PY25Q64HA_INDEX,

  NRF54_EXTERNAL_FLASH_DEVICE_COUNT
};

#if !defined(MX25R6435F)
// Settings for the Macronix MX25R6435F 8MiB SPI flash.
// Datasheet:
// http://www.macronix.com/Lists/Datasheet/Attachments/7428/MX25R6435F,%20Wide%20Range,%2064Mb,%20v1.4.pdf
// By default its in lower power mode which can only do 8mhz. In high power mode
// it can do 80mhz.
#define MX25R6435F                                                             \
  {                                                                            \
    .total_size = (1UL << 23), /* 8 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xc2,                     \
    .memory_type = 0x28, .capacity = 0x17, .max_clock_speed_mhz = 8,           \
    .quad_enable_bit_mask = 0x40, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = true, .is_fram = false,                              \
  }
#endif /* MX25R6435F */

/// List of all possible flash devices used by nRF54 boards
static SPIFlash_Device_t nrf54_possible_devices[] = {
  [MX25R6435F_INDEX] = MX25R6435F,
  [PY25Q64HA_INDEX]  = P25Q64HA,
};


static void nRF54_setup()
{
  uint32_t reset_reason = readResetReason();

  if      (reset_reason & RESET_RESETREAS_RESETPIN_Msk)
  {
      reset_info.reason = REASON_EXT_SYS_RST;
  }
  else if (reset_reason & (RESET_RESETREAS_DOG0_Msk | RESET_RESETREAS_DOG1_Msk))
  {
      reset_info.reason = REASON_WDT_RST;
  }
  else if (reset_reason & RESET_RESETREAS_SREQ_Msk)
  {
      reset_info.reason = REASON_SOFT_RESTART;
  }
  else if (reset_reason & RESET_RESETREAS_LOCKUP_Msk)
  {
      reset_info.reason = REASON_SOFT_WDT_RST;
  }
  else if (reset_reason & RESET_RESETREAS_OFF_Msk)
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
  }
  else if (reset_reason & RESET_RESETREAS_LPCOMP_Msk)
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
  }
  else if (reset_reason & RESET_RESETREAS_DIF_Msk)
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
  }
  else if (reset_reason & RESET_RESETREAS_NFC_Msk)
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
#if NRF_RESET_HAS_VBUS_RESET
  }
  else if (reset_reason & RESET_RESETREAS_VBUS_Msk)
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
#endif
  }

  Wire.setPins(SOC_GPIO_PIN_EVK_SDA, SOC_GPIO_PIN_EVK_SCL);
  Wire.begin();
  Wire.beginTransmission(SSD1306_OLED_I2C_ADDR);
  if (Wire.endTransmission() == 0) {
    nRF54_board = NRF54_LR2021EVK1XCS1;
  }
  // Wire.end();

  pinMode(SOC_GPIO_PIN_MX25_RST,  OUTPUT);
  pinMode(SOC_GPIO_PIN_MX25_BUSY, INPUT);

  digitalWrite(SOC_GPIO_PIN_MX25_RST, LOW);

  delay(10);

  if (digitalRead(SOC_GPIO_PIN_MX25_BUSY) == HIGH) {
    digitalWrite(SOC_GPIO_PIN_MX25_RST, HIGH);

    delay(50);

    if (digitalRead(SOC_GPIO_PIN_MX25_BUSY) == LOW) {
    nRF54_board = NRF54_MX25LE02;
    }
  }

  pinMode(SOC_GPIO_PIN_MX25_RST, INPUT);

  /* (Q)SPI flash init */
  switch (nRF54_board)
  {
    case NRF54_LR2021EVK1XCS1:
#if defined(ARDUINO_XIAO_NRF54LM20A_CLEAN)
      nrf54_possible_devices[PY25Q64HA_INDEX].max_clock_speed_mhz  = 33;
      nrf54_possible_devices[PY25Q64HA_INDEX].supports_qspi        = false;
      nrf54_possible_devices[PY25Q64HA_INDEX].supports_qspi_writes = false;
#if 1
      FlashTrans = new Adafruit_FlashTransport_QSPI_NRF54(
                     SOC_GPIO_PIN_SFL_EVK_SCK,
                     SOC_GPIO_PIN_SFL_EVK_SS,
                     SOC_GPIO_PIN_SFL_EVK_MOSI,
                     SOC_GPIO_PIN_SFL_EVK_MISO,
                     SOC_GPIO_PIN_SFL_EVK_WP,
                     SOC_GPIO_PIN_SFL_EVK_HOLD);
#else
      SPI_HS.setPins(SOC_GPIO_PIN_SFL_EVK_SCK,
                     SOC_GPIO_PIN_SFL_EVK_MISO,
                     SOC_GPIO_PIN_SFL_EVK_MOSI,
                     SOC_GPIO_PIN_SFL_EVK_SS);
      FlashTrans = new Adafruit_FlashTransport_SPI(SOC_GPIO_PIN_SFL_EVK_SS, SPI_HS);
#endif
#endif /* ARDUINO_XIAO_NRF54LM20A_CLEAN */
      break;

    case NRF54_PCA10156:
#if defined(ARDUINO_NRF54L15DK_PCA10156)
      nrf54_possible_devices[MX25R6435F_INDEX].max_clock_speed_mhz  = 33;
      nrf54_possible_devices[MX25R6435F_INDEX].supports_qspi        = false;
      nrf54_possible_devices[MX25R6435F_INDEX].supports_qspi_writes = false;
#if 0
      FlashTrans = new Adafruit_FlashTransport_QSPI_NRF54(
                     SOC_GPIO_PIN_SFL_DK_SCK,
                     SOC_GPIO_PIN_SFL_DK_SS,
                     SOC_GPIO_PIN_SFL_DK_MOSI,
                     SOC_GPIO_PIN_SFL_DK_MISO,
                     SOC_GPIO_PIN_SFL_DK_WP,
                     SOC_GPIO_PIN_SFL_DK_HOLD);
#else
      SPI_HS.setPins(SOC_GPIO_PIN_EVK_SCK,
                     SOC_GPIO_PIN_EVK_MISO,
                     SOC_GPIO_PIN_EVK_MOSI,
                     SOC_GPIO_PIN_SFL_DK_SS);
      FlashTrans = new Adafruit_FlashTransport_SPI(SOC_GPIO_PIN_SFL_DK_SS, SPI_HS);
#endif
#endif /* ARDUINO_NRF54L15DK_PCA10156 */
      break;
    default:
      break;
  }

  if (FlashTrans != NULL) {
    FlashTrans->begin();
    FlashTrans->runCommand(0xAB); /* RDP/RES */
    FlashTrans->end();

    SPIFlash = new Adafruit_SPIFlash(FlashTrans);
    nRF54_has_spiflash = SPIFlash->begin(nrf54_possible_devices,
                                         NRF54_EXTERNAL_FLASH_DEVICE_COUNT);
  }

  if (nRF54_has_spiflash) {
    nRF54_board = NRF54_PCA10156;
    hw_info.storage = STORAGE_FLASH;
  }

  switch (nRF54_board)
  {
    case NRF54_MX25LE02:
#if !defined(USE_RTT)
      Serial.setPins(SOC_GPIO_PIN_CONS_MX25_RX, SOC_GPIO_PIN_CONS_MX25_TX);
#endif /* USE_RTT */
      Wire.setPins(SOC_GPIO_PIN_MX25_SDA, SOC_GPIO_PIN_MX25_SCL);

      lmic_pins.nss  = SOC_GPIO_PIN_MX25_SS;
      lmic_pins.rst  = SOC_GPIO_PIN_MX25_RST;
      lmic_pins.busy = SOC_GPIO_PIN_MX25_BUSY;
#if defined(USE_RADIOLIB)
      lmic_pins.dio[0] = SOC_GPIO_PIN_MX25_DIO1;
#endif /* USE_RADIOLIB */

      pinMode(SOC_GPIO_PIN_MX25_STATUS,       OUTPUT);
      digitalWrite(SOC_GPIO_PIN_MX25_STATUS,  LED_STATE_ON);

      pinMode(SOC_GPIO_PIN_MX25_BUTTON,       INPUT_PULLUP);

      pinMode(SOC_GPIO_PIN_MX25_ANT_SW2,      OUTPUT);
      digitalWrite(SOC_GPIO_PIN_MX25_ANT_SW2, HIGH);

      break;

#if defined(ARDUINO_NRF54L15DK_PCA10156)
    case NRF54_PCA10156:
#if !defined(USE_RTT)
      Serial.setPins(SOC_GPIO_PIN_CONS_DK_RX, SOC_GPIO_PIN_CONS_DK_TX);
#endif /* USE_RTT */

      lmic_pins.nss  = SOC_GPIO_PIN_EVK_SS;
      lmic_pins.rst  = SOC_GPIO_PIN_EVK_RST;
      lmic_pins.busy = SOC_GPIO_PIN_EVK_BUSY;
#if defined(USE_RADIOLIB)
      lmic_pins.dio[0] = SOC_GPIO_PIN_EVK_DIO8;
#endif /* USE_RADIOLIB */

      pinMode(SOC_GPIO_PIN_DK_LED0,     OUTPUT);
      digitalWrite(SOC_GPIO_PIN_DK_LED0, ! LED_STATE_ON);

      break;
#endif /* ARDUINO_NRF54L15DK_PCA10156 */

    case NRF54_LR2021EVK1XCS1:
    default:
#if !defined(USE_RTT)
      Serial.setPins(SOC_GPIO_PIN_CONS_EVK_RX, SOC_GPIO_PIN_CONS_EVK_TX);
#endif /* USE_RTT */

      lmic_pins.nss  = SOC_GPIO_PIN_EVK_SS;
      lmic_pins.rst  = SOC_GPIO_PIN_EVK_RST;
      lmic_pins.busy = SOC_GPIO_PIN_EVK_BUSY;
#if defined(USE_RADIOLIB)
      lmic_pins.dio[0] = SOC_GPIO_PIN_EVK_DIO8;
#endif /* USE_RADIOLIB */

      pinMode(SOC_GPIO_PIN_EVK_STATUS,     OUTPUT);
      digitalWrite(SOC_GPIO_PIN_EVK_STATUS, LED_STATE_ON);

      pinMode(SOC_GPIO_PIN_EVK_BUTTON_AUX, INPUT_PULLUP);

#if !defined(ARDUINO_XIAO_NRF54LM20A_CLEAN)
      pinMode(SOC_GPIO_PIN_EVK_ANT_PWR,    OUTPUT);
      digitalWrite(SOC_GPIO_PIN_EVK_ANT_PWR, HIGH);
#endif /* ARDUINO_XIAO_NRF54LM20A_CLEAN */

#if defined(ARDUINO_XIAO_NRF54L15_CLEAN) || defined(ARDUINO_XIAO_NRF54LM20A_CLEAN)
      BoardControl::setBatterySenseEnabled(true);

      xiaoNrf54l15SetAntenna(XIAO_NRF54L15_ANTENNA_CERAMIC);
#else
      // pinMode(SOC_GPIO_PIN_EVK_VBAT_EN,    INPUT_PULLDOWN);

      pinMode(SOC_GPIO_PIN_EVK_ANT_SW,     INPUT_PULLDOWN); /* ANT 1 */
#endif /* ARDUINO_XIAO_NRF54L15_CLEAN */

      break;
  }

#if !defined(USE_RTT)
  Serial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
#endif /* USE_RTT */
}

static void nRF54_post_init()
{
  Serial.println();
  Serial.println(F("SoftRF Academy Edition Power-on Self Test"));
  Serial.println();
  Serial.flush();

  Serial.print(F("Board: "));
  Serial.println(nRF54_board == NRF54_LR2021EVK1XCS1 ?
                 "Seeed & Semtech LR2021EVK1XCS1" : "Minewsemi MX25LE02");
  Serial.println();
  Serial.flush();

  Serial.println(F("Built-in components:"));

  Serial.print(F("RADIO   : "));
  Serial.println(hw_info.rf      != RF_IC_NONE        ? F("PASS") : F("FAIL"));

  Serial.println();

  Serial.println(F("External components:"));

  Serial.print(F("GNSS    : "));
  Serial.println(hw_info.gnss    != GNSS_MODULE_NONE  ? F("PASS") : F("FAIL"));
  Serial.print(F("BARO    : "));
  Serial.println(hw_info.baro    != BARO_MODULE_NONE  ? F("PASS") : F("N/A"));
  Serial.print(F("DISPLAY : "));
  Serial.println(hw_info.display != DISPLAY_NONE      ? F("PASS") : F("N/A"));

  Serial.println();
  Serial.println(F("Power-on Self Test is complete."));
  Serial.println();
  Serial.flush();

  Serial.println(F("Data output device(s):"));

  Serial.print(F("NMEA   - "));
  switch (settings->nmea_out)
  {
    case NMEA_UART       :  Serial.println(F("UART"));          break;
    case NMEA_BLUETOOTH  :  Serial.println(F("Bluetooth LE"));  break;
    case NMEA_OFF        :
    default              :  Serial.println(F("NULL"));          break;
  }

  Serial.print(F("GDL90  - "));
  switch (settings->gdl90)
  {
    case GDL90_UART      :  Serial.println(F("UART"));          break;
    case GDL90_BLUETOOTH :  Serial.println(F("Bluetooth LE"));  break;
    case GDL90_OFF       :
    default              :  Serial.println(F("NULL"));          break;
  }

  Serial.print(F("D1090  - "));
  switch (settings->d1090)
  {
    case D1090_UART      :  Serial.println(F("UART"));          break;
    case D1090_BLUETOOTH :  Serial.println(F("Bluetooth LE"));  break;
    case D1090_OFF       :
    default              :  Serial.println(F("NULL"));          break;
  }

  Serial.println();
  Serial.flush();

#if defined(USE_OLED)
  if (hw_info.display == DISPLAY_OLED_1_3 ||
      hw_info.display == DISPLAY_OLED_TTGO) {
    OLED_info1();
  }
#endif /* USE_OLED */
}

static void nRF54_loop()
{
  if (wdt_is_active && g_wdt.isRunning()) {
    g_wdt.feed();
  }

#if defined(NOT_AN_INTERRUPT)
  if (SOC_GPIO_PIN_GNSS_PPS != SOC_UNUSED_PIN) {
    static bool prev_PPS_state = LOW;

    if (digitalPinToInterrupt(SOC_GPIO_PIN_GNSS_PPS) == NOT_AN_INTERRUPT) {
      bool PPS_state = digitalRead(SOC_GPIO_PIN_GNSS_PPS);

      if (PPS_state == HIGH && prev_PPS_state == LOW) {
        PPS_TimeMarker = millis();
      }
      prev_PPS_state = PPS_state;
    }
  }
#endif
}

static PowerManager g_powerManager;
static uint32_t nRF54_getChipId(void);

static constexpr uint8_t kSystemOffMagic = 0xA5U;

static void configureButtonSenseLowWake() {
  if (kPinUserButton.port != 0U) {
    return;
  }

  (void)Gpio::configure(kPinUserButton, GpioDirection::kInput,
                        nRF54_getChipId() == 0x21A44298 ? GpioPull::kPullUp :
                        nRF54_getChipId() == 0xFCE0D9E0 ? GpioPull::kPullUp :
                        GpioPull::kDisabled);

  // SYSTEM OFF wake is done through the GPIO DETECT sense mechanism, not
  // through a normal attachInterrupt() wake path.
  uint32_t cnf = NRF_P0->PIN_CNF[kPinUserButton.pin];
  cnf &= ~GPIO_PIN_CNF_SENSE_Msk;
  cnf |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
  if (nRF54_getChipId() == 0x21A44298 || nRF54_getChipId() == 0xFCE0D9E0) {
    cnf |= GPIO_PIN_CNF_PULL_Pullup;
  }
  NRF_P0->PIN_CNF[kPinUserButton.pin] = cnf;
}

static void requestLowPowerLatencyMode() {
  g_power->TASKS_LOWPWR = POWER_TASKS_LOWPWR_TASKS_LOWPWR_Trigger;
}

static void writeGpregret0(uint8_t value) {
  g_power->GPREGRET[0] = static_cast<uint32_t>(value);
}

static void nRF54_fini(int reason)
{
  switch (nRF54_board)
  {
    case NRF54_MX25LE02:
      digitalWrite(SOC_GPIO_PIN_MX25_STATUS, !LED_STATE_ON);
      pinMode(SOC_GPIO_PIN_MX25_STATUS,      INPUT);

      pinMode(SOC_GPIO_PIN_MX25_ANT_SW2,     INPUT);
      break;

    case NRF54_LR2021EVK1XCS1:
    default:
      digitalWrite(SOC_GPIO_PIN_EVK_STATUS,  !LED_STATE_ON);
      pinMode(SOC_GPIO_PIN_EVK_STATUS,       INPUT);

#if !defined(ARDUINO_XIAO_NRF54LM20A_CLEAN)
      pinMode(SOC_GPIO_PIN_EVK_BUTTON_AUX,   INPUT);
      pinMode(SOC_GPIO_PIN_EVK_ANT_PWR,      INPUT);
#endif /* ARDUINO_XIAO_NRF54LM20A_CLEAN */

#if defined(ARDUINO_XIAO_NRF54L15_CLEAN) || defined(ARDUINO_XIAO_NRF54LM20A_CLEAN)
      BoardControl::setBatterySenseEnabled(false);
#else
      // pinMode(SOC_GPIO_PIN_EVK_VBAT_EN,      INPUT);
      pinMode(SOC_GPIO_PIN_EVK_ANT_SW,       INPUT);
#endif /* ARDUINO_XIAO_NRF54L15_CLEAN */

      break;
  }

  Serial_GNSS_In.end();
  Wire.end();

  pinMode(lmic_pins.nss, INPUT_PULLUP);
  pinMode(lmic_pins.rst, INPUT);

  int mode_button_pin;

  switch (nRF54_board)
  {
    case NRF54_MX25LE02:
      mode_button_pin = SOC_GPIO_PIN_MX25_BUTTON;
      break;

#if defined(ARDUINO_NRF54L15DK_PCA10156)
    case NRF54_PCA10156:
      mode_button_pin = SOC_GPIO_PIN_DK_BUTTON0;
      break;
#endif /* ARDUINO_NRF54L15DK_PCA10156 */

    case NRF54_LR2021EVK1XCS1:
    default:
      mode_button_pin = SOC_GPIO_PIN_EVK_BUTTON;
      break;
  }

  pinMode(mode_button_pin, nRF54_getChipId() == 0x21A44298 ? INPUT_PULLUP :
                           nRF54_getChipId() == 0xFCE0D9E0 ? INPUT_PULLUP :
                           nRF54_board  ==  NRF54_PCA10156 ? INPUT_PULLUP :
                           INPUT);
  while (digitalRead(mode_button_pin) == LOW);
  delay(100);

#if 0
  Serial.end();

  g_regulators->SYSTEMOFF = REGULATORS_SYSTEMOFF_SYSTEMOFF_Enter;

  __asm volatile("dsb 0xF" ::: "memory");

  while (true) {
    cpuIdleWfi();
  }
#else
  configureButtonSenseLowWake();
  requestLowPowerLatencyMode();
  writeGpregret0(kSystemOffMagic);

  Serial.println("Entering SYSTEM OFF. Wake by pressing USER button.");
  Serial.flush();
  delay(2);

#if !defined(USE_RTT)
  Serial.end();
#endif /* USE_RTT */

  g_powerManager.systemOffNoRetention();
#endif
}

static void nRF54_reset()
{
  if (wdt_is_active && g_wdt.isRunning()) {
    switch (hw_info.display)
    {
#if defined(USE_OLED)
    case DISPLAY_OLED_1_3:
    case DISPLAY_OLED_TTGO:
    case DISPLAY_OLED_HELTEC:
      OLED_fini(SOFTRF_SHUTDOWN_NONE);
      break;
#endif /* USE_OLED */

    case DISPLAY_NONE:
    default:
      break;
    }

    while (true) { delay(100); }
  } else {
    SoftReset();
  }
}

static uint32_t nRF54_getChipId()
{
  uint32_t ficrBase = nrf54l15::FICR_BASE;
  NRF_FICR_Type* const ficr = reinterpret_cast<NRF_FICR_Type*>(ficrBase);

  const uint32_t lo = ficr->DEVICEADDR[0];
  const uint32_t hi = ficr->DEVICEADDR[1];

#if !defined(SOFTRF_ADDRESS)
  uint32_t id = lo;

  return DevID_Mapper(id);
#else
  return (SOFTRF_ADDRESS & 0xFFFFFFFFU );
#endif
}

static void* nRF54_getResetInfoPtr()
{
  return (void *) &reset_info;
}

static String nRF54_getResetInfo()
{
  switch (reset_info.reason)
  {
    default                     : return F("No reset information available");
  }
}

static String nRF54_getResetReason()
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

static uint32_t nRF54_getFreeHeap()
{
  return getFreeHeap();
}

static long nRF54_random(long howsmall, long howBig)
{
  return random(howsmall, howBig);
}

static void nRF54_Sound_test(int var)
{
#if defined(USE_PWM_SOUND)
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && settings->volume != BUZZER_OFF) {
    pinMode(SOC_GPIO_PIN_BUZZER, OUTPUT);

    tone(SOC_GPIO_PIN_BUZZER, 440,  500); // delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 640,  500); // delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 840,  500); // delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 1040, 500); // delay(600);

    noTone(SOC_GPIO_PIN_BUZZER);
    pinMode(SOC_GPIO_PIN_BUZZER, INPUT);
  }
#endif /* USE_PWM_SOUND */
}

static void nRF54_Sound_tone(int hz, uint8_t volume)
{
#if defined(USE_PWM_SOUND)
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && volume != BUZZER_OFF) {
    if (hz > 0) {
      pinMode(SOC_GPIO_PIN_BUZZER, OUTPUT);
      tone(SOC_GPIO_PIN_BUZZER, hz, ALARM_TONE_MS);
    } else {
      noTone(SOC_GPIO_PIN_BUZZER);
      pinMode(SOC_GPIO_PIN_BUZZER, INPUT);
    }
  }
#endif /* USE_PWM_SOUND */
}

static void nRF54_WiFi_set_param(int ndx, int value)
{
  /* NONE */
}

static void nRF54_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  /* NONE */
}

static bool nRF54_EEPROM_begin(size_t size)
{
#if !defined(EXCLUDE_EEPROM)
  if (size > EEPROM.length()) {
    return false;
  }

  EEPROM.begin();
#endif /* EXCLUDE_EEPROM */

  return true;
}

static void nRF54_EEPROM_extension(int cmd)
{

}

static void nRF54_SPI_begin()
{
#if defined(USE_SOFTSPI)
  SPI.begin();
#else
  switch (nRF54_board)
  {
    case NRF54_MX25LE02:
      SPI.setPins(SOC_GPIO_PIN_MX25_SCK,
                  SOC_GPIO_PIN_MX25_MISO,
                  SOC_GPIO_PIN_MX25_MOSI,
                  SOC_GPIO_PIN_MX25_SS);
      SPI.begin(SOC_GPIO_PIN_MX25_SS);
      break;

    case NRF54_LR2021EVK1XCS1:
    case NRF54_PCA10156:
    default:
      SPI.setPins(SOC_GPIO_PIN_EVK_SCK,
                  SOC_GPIO_PIN_EVK_MISO,
                  SOC_GPIO_PIN_EVK_MOSI,
                  SOC_GPIO_PIN_EVK_SS);
      SPI.begin(SOC_GPIO_PIN_EVK_SS);
      break;
  }
#endif /* USE_SOFTSPI */
}

static void nRF54_swSer_begin(unsigned long baud)
{
  switch (nRF54_board)
  {
    case NRF54_MX25LE02:
      Serial_GNSS_In.setPins(SOC_GPIO_PIN_GNSS_MX25_RX,
                             SOC_GPIO_PIN_GNSS_MX25_TX);
      break;

    case NRF54_LR2021EVK1XCS1:
    case NRF54_PCA10156:
    default:
      Serial_GNSS_In.setPins(SOC_GPIO_PIN_GNSS_EVK_RX,
                             SOC_GPIO_PIN_GNSS_EVK_TX);
      break;
  }

  Serial_GNSS_In.begin(baud);
}

static void nRF54_swSer_enableRx(boolean arg)
{
  /* NONE */
}

static byte nRF54_Display_setup()
{
  byte rval = DISPLAY_NONE;

  if (nRF54_board == NRF54_LR2021EVK1XCS1 ||
      nRF54_board == NRF54_MX25LE02       ||
      nRF54_board == NRF54_PCA10156) {
#if defined(USE_OLED)
    rval = OLED_setup();
#endif /* USE_OLED */
  }

  return rval;
}

static void nRF54_Display_loop()
{
  switch (hw_info.display)
  {
#if defined(USE_OLED)
  case DISPLAY_OLED_1_3:
  case DISPLAY_OLED_TTGO:
  case DISPLAY_OLED_HELTEC:
    OLED_loop();
    break;
#endif /* USE_OLED */

  case DISPLAY_NONE:
  default:
    break;
  }
}

static void nRF54_Display_fini(int reason)
{
  switch (hw_info.display)
  {
#if defined(USE_OLED)
  case DISPLAY_OLED_1_3:
  case DISPLAY_OLED_TTGO:
  case DISPLAY_OLED_HELTEC:
    OLED_fini(reason);
    break;
#endif /* USE_OLED */

  case DISPLAY_NONE:
  default:
    break;
  }
}

static void nRF54_Battery_setup()
{

}

static float nRF54_Battery_param(uint8_t param)
{
  uint32_t bat_adc_pin;
  float rval, voltage, mult;

  switch (param)
  {
  case BATTERY_PARAM_THRESHOLD:
    rval = hw_info.model == SOFTRF_MODEL_ACADEMY  ? BATTERY_THRESHOLD_LIPO   :
                                                    BATTERY_THRESHOLD_NIMHX2;
    break;

  case BATTERY_PARAM_CUTOFF:
    rval = hw_info.model == SOFTRF_MODEL_ACADEMY  ? BATTERY_CUTOFF_LIPO   :
                                                    BATTERY_CUTOFF_NIMHX2;
    break;

  case BATTERY_PARAM_CHARGE:
#if defined(ARDUINO_XIAO_NRF54L15_CLEAN) || defined(ARDUINO_XIAO_NRF54LM20A_CLEAN)
    {
      uint8_t vbatPercent = 0;
      BoardControl::sampleBatteryPercent(&vbatPercent);
      rval = vbatPercent;
    }
#else
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
#endif /* ARDUINO_XIAO_NRF54L15_CLEAN */
    break;

  case BATTERY_PARAM_VOLTAGE:
  default:
#if defined(ARDUINO_XIAO_NRF54L15_CLEAN) || defined(ARDUINO_XIAO_NRF54LM20A_CLEAN)
    {
      int32_t vbatMilliVolts = 0;
      BoardControl::sampleBatteryMilliVolts(&vbatMilliVolts);
      voltage = vbatMilliVolts;
    }
#else
    voltage = 0.0;
#if 0
    // Set the analog reference to 3.0V (default = 3.6V)
    analogReference(AR_INTERNAL_3_0);
#endif
    // Set the resolution to 12-bit (0..4095)
    analogReadResolution(12); // Can be 8, 10, 12 or 14

    // Let the ADC settle
    delay(1);

    switch (nRF54_board)
    {
      case NRF54_MX25LE02:
      case NRF54_PCA10156:
      case NRF54_LR2021EVK1XCS1:
      default:
        bat_adc_pin = SOC_GPIO_PIN_EVK_BATTERY;
        mult        = SOC_ADC_EVK_VOLTAGE_DIV;
        break;
    }

    // Get the raw 12-bit, 0..3000mV ADC value
    voltage = analogRead(bat_adc_pin);

    // Set the ADC back to the default settings
#if 0
    analogReference(AR_DEFAULT);
#endif
    analogReadResolution(10);

    // Convert the raw value to compensated mv, taking the resistor-
    // divider into account (providing the actual LIPO voltage)
    // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
    voltage *= (mult * VBAT_MV_PER_LSB);
#endif /* ARDUINO_XIAO_NRF54L15_CLEAN */
    rval = voltage * 0.001;
    break;
  }

  return rval;
}

void nRF54_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long nRF54_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool nRF54_Baro_setup() {
  return true;
}

static void nRF54_UATSerial_begin(unsigned long baud)
{

}

static void nRF54_UATModule_restart()
{

}

static void nRF54_WDT_setup()
{
  wdt_is_active = g_wdt.configure(4000U, 0U, false, false, true);

  if (wdt_is_active) {
    g_wdt.start();
    g_wdt.feed();
  }
}

static void nRF54_WDT_fini()
{
  if (wdt_is_active && g_wdt.isRunning()) {
    g_wdt.stop();
  }
}

#include <AceButton.h>
using namespace ace_button;

AceButton button_1(SOC_UNUSED_PIN);

// The event handler for the button.
void handleEvent(AceButton* button, uint8_t eventType, uint8_t buttonState) {

  switch (eventType) {
    case AceButton::kEventClicked:
    case AceButton::kEventReleased:
#if defined(USE_OLED)
      if (button == &button_1 &&
         (hw_info.display == DISPLAY_OLED_1_3 ||
          hw_info.display == DISPLAY_OLED_TTGO)) {
        OLED_Next_Page();
      }
#endif /* USE_OLED */
      break;
    case AceButton::kEventDoubleClicked:

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

static void nRF54_Button_setup()
{
  int mode_button_pin = -1;

  switch (nRF54_board)
  {
    case NRF54_MX25LE02:
      mode_button_pin = SOC_GPIO_PIN_MX25_BUTTON;
      break;

#if defined(ARDUINO_NRF54L15DK_PCA10156)
    case NRF54_PCA10156:
      mode_button_pin = SOC_GPIO_PIN_DK_BUTTON0;
      break;
#endif /* ARDUINO_NRF54L15DK_PCA10156 */

    case NRF54_LR2021EVK1XCS1:
    default:
      mode_button_pin = SOC_GPIO_PIN_EVK_BUTTON;
      break;
  }

  // Button(s) uses external pull up resistor.
  pinMode(mode_button_pin, nRF54_getChipId() == 0x21A44298 ? INPUT_PULLUP :
                           nRF54_getChipId() == 0xFCE0D9E0 ? INPUT_PULLUP :
                           nRF54_board  ==  NRF54_PCA10156 ? INPUT_PULLUP :
                           INPUT);
  button_1.init(mode_button_pin, HIGH);

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

//  attachInterrupt(digitalPinToInterrupt(mode_button_pin), onModeButtonEvent, CHANGE );
}

static void nRF54_Button_loop()
{
  button_1.check();
}

static void nRF54_Button_fini()
{

}

static void nRF54_TTS(char *message)
{

}

static bool nRF54_ADB_setup()
{
  return false;
}

static bool nRF54_ADB_fini()
{
  return false;
}

/*
 * One aircraft CDB (20000+ records) query takes:
 * 1)     FOUND : 5-7 milliseconds
 * 2) NOT FOUND :   3 milliseconds
 */
static bool nRF54_ADB_query(uint8_t type, uint32_t id, char *buf, size_t size)
{
  return false;
}

DB_ops_t nRF54_ADB_ops = {
  nRF54_ADB_setup,
  nRF54_ADB_fini,
  nRF54_ADB_query
};

const SoC_ops_t nRF54_ops = {
  SOC_NRF54,
  "nRF54",
  nRF54_setup,
  nRF54_post_init,
  nRF54_loop,
  nRF54_fini,
  nRF54_reset,
  nRF54_getChipId,
  nRF54_getResetInfoPtr,
  nRF54_getResetInfo,
  nRF54_getResetReason,
  nRF54_getFreeHeap,
  nRF54_random,
  nRF54_Sound_test,
  nRF54_Sound_tone,
  NULL,
  nRF54_WiFi_set_param,
  nRF54_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  nRF54_EEPROM_begin,
  nRF54_EEPROM_extension,
  nRF54_SPI_begin,
  nRF54_swSer_begin,
  nRF54_swSer_enableRx,
#if !defined(EXCLUDE_BLUETOOTH)
  &nRF5x_Bluetooth_ops,
#else
  NULL,
#endif /* EXCLUDE_BLUETOOTH */
  NULL,
  NULL,
  nRF54_Display_setup,
  nRF54_Display_loop,
  nRF54_Display_fini,
  nRF54_Battery_setup,
  nRF54_Battery_param,
  nRF54_GNSS_PPS_Interrupt_handler,
  nRF54_get_PPS_TimeMarker,
  nRF54_Baro_setup,
  nRF54_UATSerial_begin,
  nRF54_UATModule_restart,
  nRF54_WDT_setup,
  nRF54_WDT_fini,
  nRF54_Button_setup,
  nRF54_Button_loop,
  nRF54_Button_fini,
  nRF54_TTS,
  &nRF54_ADB_ops
};

#endif /* ARDUINO_ARCH_NRF54L15CLEAN */
