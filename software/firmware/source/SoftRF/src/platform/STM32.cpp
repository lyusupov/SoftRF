/*
 * Platform_STM32.cpp
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

#if defined(ARDUINO_ARCH_STM32)

#include <SPI.h>
#include <Wire.h>

#if !defined(ARDUINO_WisDuo_RAK3172_Evaluation_Board)
#include <IWatchdog.h>
#endif /* ARDUINO_WisDuo_RAK3172_Evaluation_Board */

#include "../system/SoC.h"
#include "../driver/RF.h"
#include "../driver/LED.h"
#include "../driver/Sound.h"
#include "../driver/EEPROM.h"
#include "../driver/Battery.h"
#include "../driver/OLED.h"
#include "../driver/Baro.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/GDL90.h"
#include "../protocol/data/D1090.h"

#if !defined(ARDUINO_WisDuo_RAK3172_Evaluation_Board)
#include <STM32LowPower.h>
#endif /* ARDUINO_WisDuo_RAK3172_Evaluation_Board */

// RFM95W pin mapping
lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .txe = LMIC_UNUSED_PIN,
    .rxe = LMIC_UNUSED_PIN,
#if !defined(USE_OGN_RF_DRIVER)
    .rst = LMIC_UNUSED_PIN,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
#else
    .rst = SOC_GPIO_PIN_RST,
    .dio = {SOC_GPIO_PIN_DIO0, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
#endif
    .busy = SOC_GPIO_PIN_BUSY,
    .tcxo = LMIC_UNUSED_PIN,
};

#if defined(USBD_USE_CDC)
#if !defined(DISABLE_GENERIC_SERIALUSB) || \
    (defined(STM32_CORE_VERSION) && (STM32_CORE_VERSION > 0x01090000))
HardwareSerial Serial1(SOC_GPIO_PIN_CONS_RX, SOC_GPIO_PIN_CONS_TX);
#endif
#endif /* USBD_USE_CDC */

#if defined(ARDUINO_NUCLEO_L073RZ)

HardwareSerial Serial2(USART2);
HardwareSerial Serial4(SOC_GPIO_PIN_GNSS_RX, SOC_GPIO_PIN_GNSS_TX);

static bool STM32_has_TCXO = false;
static bool STM32_has_IMU  = false;

#if !defined(EXCLUDE_IMU)
#include <ICM_20948.h>

#define IMU_UPDATE_INTERVAL 500 /* ms */

ICM_20948_I2C imu;

static unsigned long IMU_Time_Marker = 0;

extern int32_t IMU_g_x10;
#endif /* EXCLUDE_IMU */
#elif defined(ARDUINO_BLUEPILL_F103CB)

HardwareSerial Serial2(SOC_GPIO_PIN_GNSS_RX, SOC_GPIO_PIN_GNSS_TX);
HardwareSerial Serial3(SOC_GPIO_PIN_RX3,     SOC_GPIO_PIN_TX3);

#elif defined(ARDUINO_GENERIC_WLE5CCUX) || defined(ARDUINO_GENERIC_WL55CCUX)

HardwareSerial Serial2(USART2);

static bool STM32_has_TCXO = false;

#ifdef HAL_SUBGHZ_MODULE_ENABLED
void HAL_SUBGHZ_MspInit(SUBGHZ_HandleTypeDef * hsubghz)
{
    __HAL_RCC_SUBGHZSPI_CLK_ENABLE();
    __HAL_RCC_SUBGHZSPI_FORCE_RESET();
    __HAL_RCC_SUBGHZSPI_RELEASE_RESET();
}
#endif /* HAL_SUBGHZ_MODULE_ENABLED */

#elif defined(ARDUINO_WisDuo_RAK3172_Evaluation_Board)

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

#ifndef SUPPORT_LORA

extern "C" void service_lora_suspend(void) {
}

extern "C" void service_lora_resume(void) {
}

typedef enum _SERVICE_LORA_CLASS
{
  SERVICE_LORA_CLASS_A = 0,
  SERVICE_LORA_CLASS_B = 1,
  SERVICE_LORA_CLASS_C = 2,
} SERVICE_LORA_CLASS;

extern "C" int service_lora_get_real_class_from_stack(void)
{
  return SERVICE_LORA_CLASS_C;
}

#endif /* SUPPORT_LORA */

#else
#error "This hardware platform is not supported!"
#endif

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

static int stm32_board = STM32_BLUE_PILL; /* default */

char UDPpacketBuffer[4]; // Dummy definition to satisfy build sequence

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

static uint32_t bootCount = 0;

#if defined(EXCLUDE_EEPROM)
eeprom_t eeprom_block;
settings_t *settings = &eeprom_block.field.settings;
#endif /* EXCLUDE_EEPROM */

static int STM32_probe_pin(uint32_t pin, uint32_t mode)
{
  int rval;

  pinMode(pin, mode);

  delay(20);
  rval = digitalRead(pin);
  pinMode(pin, INPUT);

  return rval;
}

static void STM32_SerialWakeup() { }
static void STM32_ButtonWakeup() { }

static void STM32_ULP_stop()
{
#if defined(ARDUINO_NUCLEO_L073RZ)
  __disable_irq();

  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();
  HAL_PWR_EnterSTOPMode( PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI );
#else
#if !defined(ARDUINO_WisDuo_RAK3172_Evaluation_Board)
  LowPower_shutdown();
#endif /* ARDUINO_WisDuo_RAK3172_Evaluation_Board */
#endif /* ARDUINO_NUCLEO_L073RZ */
}

static void STM32_setup()
{
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
    {
        reset_info.reason = REASON_WDT_RST; // "LOW_POWER_RESET"
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
    {
        reset_info.reason = REASON_WDT_RST; // "WINDOW_WATCHDOG_RESET"
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
    {
        reset_info.reason = REASON_SOFT_WDT_RST; // "INDEPENDENT_WATCHDOG_RESET"
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
    {
        // This reset is induced by calling the ARM CMSIS `NVIC_SystemReset()` function!
        reset_info.reason = REASON_SOFT_RESTART; // "SOFTWARE_RESET"
    }
#if !defined(ARDUINO_GENERIC_WLE5CCUX) && \
    !defined(ARDUINO_GENERIC_WL55CCUX) && \
    !defined(ARDUINO_WisDuo_RAK3172_Evaluation_Board)
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
    {
        reset_info.reason = REASON_DEFAULT_RST; // "POWER-ON_RESET (POR) / POWER-DOWN_RESET (PDR)"
    }
#endif /* ARDUINO_GENERIC_WLE5CCUX */
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
    {
        reset_info.reason = REASON_EXT_SYS_RST; // "EXTERNAL_RESET_PIN_RESET"
    }

    // Clear all the reset flags or else they will remain set during future resets until system power is fully removed.
    __HAL_RCC_CLEAR_RESET_FLAGS();

#if !defined(ARDUINO_WisDuo_RAK3172_Evaluation_Board)
    LowPower.begin();
#endif /* ARDUINO_WisDuo_RAK3172_Evaluation_Board */

    hw_info.model = SOFTRF_MODEL_RETRO;

#if !defined(ARDUINO_WisDuo_RAK3172_Evaluation_Board)
    Wire.setSCL(SOC_GPIO_PIN_SCL);
    Wire.setSDA(SOC_GPIO_PIN_SDA);
#endif /* ARDUINO_WisDuo_RAK3172_Evaluation_Board */

#if defined(ARDUINO_NUCLEO_L073RZ)
    stm32_board = STM32_TTGO_TWATCH_EB_1_3;

    /* Probe on presence of external pull-up resistors connected to I2C bus */
    if (            STM32_probe_pin(SOC_GPIO_PIN_SCL, INPUT_PULLDOWN) == HIGH  &&
        (delay(50), STM32_probe_pin(SOC_GPIO_PIN_SCL, INPUT_PULLDOWN) == HIGH) &&
                    STM32_probe_pin(SOC_GPIO_PIN_SDA, INPUT_PULLDOWN) == HIGH  &&
        (delay(50), STM32_probe_pin(SOC_GPIO_PIN_SDA, INPUT_PULLDOWN) == HIGH)) {

      hw_info.model = SOFTRF_MODEL_DONGLE;
      stm32_board   = STM32_TTGO_TMOTION_1_1;

//      pinMode(TTGO_TIMPULSE_OLED_PIN_RST, INPUT_PULLUP); /* LP modded early rev. */

      pinMode(TTGO_TIMPULSE_VDD_1V8_EN, INPUT_PULLUP);
      delay(150);
      pinMode(TTGO_TIMPULSE_GPS_PWR_EN, INPUT_PULLUP);
      delay(50);

      Wire.begin();
      Wire.beginTransmission(ICM20948_ADDRESS);
      STM32_has_IMU = (Wire.endTransmission() == 0);
      Wire.end();

      pinMode(SOC_GPIO_PIN_SDA,  INPUT_ANALOG);
      pinMode(SOC_GPIO_PIN_SCL,  INPUT_ANALOG);

      if (STM32_has_IMU) {
        stm32_board   = STM32_TTGO_TIMPULSE_1_0;
        hw_info.model = SOFTRF_MODEL_BRACELET;
        hw_info.imu   = IMU_ICM20948;
        hw_info.mag   = MAG_AK09916;

        if (SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN) {
          pinMode(TTGO_TIMPULSE_GPIO_PAD_PWR, INPUT_PULLUP);
        }

        pinMode(TTGO_TIMPULSE_OLED_PIN_RST, INPUT_PULLDOWN);
      } else {
        pinMode(TTGO_TIMPULSE_VDD_1V8_EN, INPUT_ANALOG);
        pinMode(TTGO_TIMPULSE_GPS_PWR_EN, INPUT_ANALOG);
      }
    }

    // PC_1 is Low for TCXO or High for Crystal
#if !defined(ENFORCE_S78G)
    STM32_has_TCXO = (STM32_probe_pin(SOC_GPIO_PIN_OSC_SEL, INPUT) == 0);
#endif
    if (STM32_has_TCXO) {
      lmic_pins.tcxo = SOC_GPIO_PIN_TCXO_OE;
      hal_pin_tcxo_init();
      hal_pin_tcxo(0); // disable TCXO
    }

    /* De-activate 1.8V<->3.3V level shifters */
    digitalWrite(SOC_GPIO_PIN_GNSS_LS, LOW);
    delay(200);
    pinMode(SOC_GPIO_PIN_GNSS_LS, INPUT_ANALOG);

    digitalWrite(SOC_GPIO_PIN_ANT_RXTX, LOW);
    pinMode(SOC_GPIO_PIN_ANT_RXTX, OUTPUT_OPEN_DRAIN);

    if (!STM32_has_TCXO) {
      // because PC1 = high for LoRa Crystal, need to be careful of leakage current
      pinMode(SOC_GPIO_PIN_OSC_SEL, INPUT_PULLUP);
    }

    pinMode(SOC_GPIO_PIN_SS,  INPUT_PULLUP);

#elif defined(ARDUINO_BLUEPILL_F103CB)
    stm32_board = STM32_BLUE_PILL;

#elif defined(ARDUINO_GENERIC_WLE5CCUX)

    /* TBD */
    stm32_board = (SoC->getChipId() == 0x725c6907) ? STM32_EBYTE_E77 : STM32_OLIMEX_WLE5CC;
    hw_info.model = SOFTRF_MODEL_BALKAN;

#elif defined(ARDUINO_GENERIC_WL55CCUX)

    /* TBD */
    stm32_board = STM32_LILYGO_T3_1_0;
    hw_info.model = SOFTRF_MODEL_LABUBU;

#elif defined(ARDUINO_WisDuo_RAK3172_Evaluation_Board)

    /* TBD */
    stm32_board   = STM32_RAK_3172_EB;
    hw_info.model = SOFTRF_MODEL_BALKAN;

#else
#error "This hardware platform is not supported!"
#endif

#if defined(USBD_USE_CDC) && !defined(DISABLE_GENERIC_SERIALUSB)
    SerialOutput.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
#endif

    uint32_t shudown_reason;
#if !defined(ARDUINO_WisDuo_RAK3172_Evaluation_Board)
    shudown_reason = getBackupRegister(SHUTDOWN_REASON_INDEX);
    setBackupRegister(SHUTDOWN_REASON_INDEX, SOFTRF_SHUTDOWN_NONE);
#endif /* ARDUINO_WisDuo_RAK3172_Evaluation_Board */

    switch (shudown_reason)
    {
    case SOFTRF_SHUTDOWN_NONE:
      break;
#if defined(USE_SERIAL_DEEP_SLEEP)
    case SOFTRF_SHUTDOWN_NMEA:
#if !defined(USBD_USE_CDC) || defined(DISABLE_GENERIC_SERIALUSB)
      SerialOutput.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
#endif
      LowPower.enableWakeupFrom(&SerialOutput, STM32_SerialWakeup);

      LowPower.deepSleep();

      HAL_NVIC_SystemReset();
      break;
#endif /* USE_SERIAL_DEEP_SLEEP */
    case SOFTRF_SHUTDOWN_BUTTON:
    case SOFTRF_SHUTDOWN_LOWBAT:
#if defined(ARDUINO_NUCLEO_L073RZ)
      if (hw_info.model == SOFTRF_MODEL_BRACELET) {
        pinMode(TTGO_TIMPULSE_GPS_PWR_EN, INPUT_PULLDOWN);
        pinMode(TTGO_TIMPULSE_VDD_1V8_EN, INPUT_PULLDOWN);
      }
#endif /* ARDUINO_NUCLEO_L073RZ */
      if (SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN) {
        pinMode(SOC_GPIO_PIN_BUTTON, hw_info.model == SOFTRF_MODEL_DONGLE ||
                                     hw_info.model == SOFTRF_MODEL_LABUBU ?
                                     INPUT_PULLDOWN : INPUT);
#if !defined(ARDUINO_WisDuo_RAK3172_Evaluation_Board)
        LowPower.attachInterruptWakeup(SOC_GPIO_PIN_BUTTON,
                                       STM32_ButtonWakeup, RISING);

        LowPower.deepSleep();
#endif /* ARDUINO_WisDuo_RAK3172_Evaluation_Board */

        /* do not enter into DFU mode when BOOT button has dual function */
        while ((hw_info.model == SOFTRF_MODEL_DONGLE ||
                hw_info.model == SOFTRF_MODEL_LABUBU) &&
                digitalRead(SOC_GPIO_PIN_BUTTON) == HIGH);
        HAL_NVIC_SystemReset();
      } else {
        STM32_ULP_stop();
      }
      break;
    default:
      STM32_ULP_stop();
      break;
    }

#if !defined(ARDUINO_WisDuo_RAK3172_Evaluation_Board)
    bootCount = getBackupRegister(BOOT_COUNT_INDEX);
    bootCount++;
    setBackupRegister(BOOT_COUNT_INDEX, bootCount);
#endif /* ARDUINO_WisDuo_RAK3172_Evaluation_Board */

#if !defined(ARDUINO_WisDuo_RAK3172_Evaluation_Board)
    pinMode(SOC_GPIO_PIN_BATTERY, INPUT_ANALOG);
#endif /* ARDUINO_WisDuo_RAK3172_Evaluation_Board */

#if defined(ARDUINO_NUCLEO_L073RZ)
    lmic_pins.rxe = SOC_GPIO_PIN_ANT_RXTX;

    // Set default value at Rx
    digitalWrite(SOC_GPIO_PIN_ANT_RXTX, HIGH);

    if (stm32_board == STM32_TTGO_TIMPULSE_1_0) {
      delay(1);
      pinMode(TTGO_TIMPULSE_OLED_PIN_RST, INPUT_PULLUP);

#if !defined(EXCLUDE_IMU)
      imu.begin();

      pinMode(TTGO_TIMPULSE_SENSOR_INT, INPUT);

      IMU_Time_Marker = millis();
#endif /* EXCLUDE_IMU */
    }
#elif defined(ARDUINO_GENERIC_WLE5CCUX)
    switch (stm32_board)
    {
    case STM32_EBYTE_E77:
      lmic_pins.rxe = SOC_GPIO_ANT_RX_E77;
      lmic_pins.txe = SOC_GPIO_ANT_TX_E77;
      break;
    case STM32_SEEED_E5:
      lmic_pins.rxe = SOC_GPIO_ANT_RX_E5;
      lmic_pins.txe = SOC_GPIO_ANT_TX_E5;
      STM32_has_TCXO = true;
      lmic_pins.tcxo = SOC_GPIO_TCXO_E5;
      break;
    case STM32_ACSIP_ST50H: /* a.k.a. "RAK3172-SiP" */
      lmic_pins.rxe = SOC_GPIO_ANT_RX_ST50;
      lmic_pins.txe = SOC_GPIO_ANT_TX_ST50;
      STM32_has_TCXO = true;
      lmic_pins.tcxo = SOC_GPIO_TCXO_ST50;
      break;
    case STM32_RAK_3172_EB:
      lmic_pins.rxe = SOC_GPIO_ANT_RX_3172;
      lmic_pins.txe = SOC_GPIO_ANT_TX_3172;
      break;
    case STM32_OLIMEX_WLE5CC:
    default:
      lmic_pins.rxe = SOC_GPIO_ANT_RX_OLI;
      lmic_pins.txe = SOC_GPIO_ANT_TX_OLI;

      hal_set_rf_output(LOW); /* RFO_LP (default) */

      Wire.begin();
      Wire.beginTransmission(IIS2MDC_ADDRESS);
      hw_info.mag = (Wire.endTransmission() == 0) ? MAG_IIS2MDC : MAG_NONE;
      Wire.end();

      break;
    }

    if (STM32_has_TCXO) {
      hal_pin_tcxo(1);
      pinMode(lmic_pins.tcxo, OUTPUT);
    }
#endif /* ARDUINO_NUCLEO_L073RZ || ARDUINO_GENERIC_WLE5CCUX */

#if defined(USE_RADIOLIB)
    lmic_pins.dio[0] = SOC_GPIO_PIN_DIO0;
#endif /* USE_RADIOLIB */

    Serial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);

#if defined(USBD_USE_CDC) && !defined(DISABLE_GENERIC_SERIALUSB)
    /* Let host's USB and console drivers to warm-up */
    delay(2000);
#endif
}

static void STM32_post_init()
{
#if defined(ARDUINO_NUCLEO_L073RZ)
  if (hw_info.model == SOFTRF_MODEL_DONGLE ||
      hw_info.model == SOFTRF_MODEL_BRACELET ) {
    Serial.println();
    Serial.print(F("TTGO "));
    Serial.print(hw_info.model == SOFTRF_MODEL_BRACELET   ? F("T-Impulse") : F("T-Motion"));
    Serial.print(F(" (S7"));
    Serial.print(STM32_has_TCXO ? '6' : '8');
    Serial.println(F("G) Power-on Self Test"));
    Serial.println();
    Serial.flush();

    Serial.println(F("Built-in components:"));

    Serial.print(F("RADIO   : "));
    Serial.println(hw_info.rf      == RF_IC_SX1276        ? F("PASS") : F("FAIL"));
    if (hw_info.rf == RF_IC_SX1276) {
      Serial.print(F("CLK SRC : "));
      Serial.println(STM32_has_TCXO                       ? F("TCXO") : F("XTAL"));
    }
    Serial.print(F("GNSS    : "));
    Serial.println(hw_info.gnss    == GNSS_MODULE_SONY    ? F("PASS") : F("FAIL"));

    if (hw_info.model == SOFTRF_MODEL_BRACELET) {
      Serial.print(F("DISPLAY : "));
      Serial.println(hw_info.display == DISPLAY_OLED_0_49 ? F("PASS") : F("N/A"));
      Serial.print(F("IMU     : "));
      Serial.println(STM32_has_IMU                        ? F("PASS") : F("N/A"));
    }

    if (hw_info.model == SOFTRF_MODEL_DONGLE) {
      Serial.println();
      Serial.println(F("External components:"));
      Serial.print(F("DISPLAY : "));
      Serial.println(hw_info.display == DISPLAY_OLED_TTGO ? F("PASS") : F("N/A"));
      Serial.print(F("BMx280  : "));
      Serial.println(hw_info.baro    == BARO_MODULE_BMP280 ? F("PASS") : F("N/A"));
    }

    Serial.println();
    Serial.println(F("Power-on Self Test is complete."));
    Serial.println();
    Serial.flush();
  }
#elif defined(ARDUINO_GENERIC_WLE5CCUX)
  if (hw_info.model == SOFTRF_MODEL_BALKAN) {
    Serial.println();
    Serial.println(F("SoftRF Balkan Edition Power-on Self Test"));
    Serial.println();
    Serial.flush();

    Serial.println(F("Built-in components:"));

    Serial.print(F("RADIO   : "));
    Serial.println(hw_info.rf      == RF_IC_SX1262       ? F("PASS") : F("FAIL"));
    Serial.print(F("CLK SRC : "));
    Serial.println(STM32_has_TCXO                        ? F("TCXO") : F("XTAL"));
    Serial.print(F("BMx280  : "));
    Serial.println(hw_info.baro    == BARO_MODULE_BMP280 ? F("PASS") : F("FAIL"));

    Serial.println();
    Serial.println(F("External components:"));
    Serial.print(F("GNSS    : "));
    Serial.println(hw_info.gnss    != GNSS_MODULE_NONE   ? F("PASS") : F("N/A"));
    Serial.print(F("DISPLAY : "));
    Serial.println(hw_info.display != DISPLAY_NONE       ? F("PASS") : F("N/A"));

    Serial.println();
    Serial.println(F("Power-on Self Test is complete."));
    Serial.println();
    Serial.flush();
  }
#endif /* ARDUINO_NUCLEO_L073RZ || ARDUINO_GENERIC_WLE5CCUX */

  Serial.println(F("Data output device(s):"));

  Serial.print(F("NMEA   - "));
  switch (settings->nmea_out)
  {
    case NMEA_UART       :  Serial.println(F("UART"));          break;
    case NMEA_USB        :  Serial.println(F("USB CDC"));       break;
    case NMEA_OFF        :
    default              :  Serial.println(F("NULL"));          break;
  }

  Serial.print(F("GDL90  - "));
  switch (settings->gdl90)
  {
    case GDL90_UART      :  Serial.println(F("UART"));          break;
    case GDL90_USB       :  Serial.println(F("USB CDC"));       break;
    case GDL90_OFF       :
    default              :  Serial.println(F("NULL"));          break;
  }

  Serial.print(F("D1090  - "));
  switch (settings->d1090)
  {
    case D1090_UART      :  Serial.println(F("UART"));          break;
    case D1090_USB       :  Serial.println(F("USB CDC"));       break;
    case D1090_OFF       :
    default              :  Serial.println(F("NULL"));          break;
  }

  Serial.println();
  Serial.flush();

#if defined(USE_OLED)
  OLED_info1();
#endif /* USE_OLED */
}

static void STM32_loop()
{
#if !defined(ARDUINO_WisDuo_RAK3172_Evaluation_Board)
  // Reload the watchdog
  if (IWatchdog.isEnabled()) {
    IWatchdog.reload();
  }
#endif /* ARDUINO_WisDuo_RAK3172_Evaluation_Board */

#if !defined(EXCLUDE_IMU)
  if (hw_info.imu == IMU_ICM20948 &&
      (millis() - IMU_Time_Marker) > IMU_UPDATE_INTERVAL) {
    if (imu.dataReady()) {
      imu.getAGMT();

      // milli g's
      float a_x = imu.accX();
      float a_y = imu.accY();
      float a_z = imu.accZ();
#if 0
      Serial.print("{ACCEL: ");
      Serial.print(a_x);
      Serial.print(",");
      Serial.print(a_y);
      Serial.print(",");
      Serial.print(a_z);
      Serial.println("}");
#endif
      IMU_g_x10 = (int) (sqrtf(a_x*a_x + a_y*a_y + a_z*a_z) / 100);
    }
    IMU_Time_Marker = millis();
  }
#endif /* EXCLUDE_IMU */
}

static void STM32_fini(int reason)
{
#if defined(ARDUINO_NUCLEO_L073RZ)

  if (hw_info.model == SOFTRF_MODEL_BRACELET) {
#if !defined(EXCLUDE_IMU)
    if (hw_info.imu == IMU_ICM20948) {
      imu.sleep(true);
      // imu.lowPower(true);
    }
#endif /* EXCLUDE_IMU */
  }
#endif /* ARDUINO_NUCLEO_L073RZ */

  Serial_GNSS_In.end();
  Wire.end();

#if !defined(ARDUINO_WisDuo_RAK3172_Evaluation_Board)
  pinMode(SOC_GPIO_PIN_SDA,  INPUT_ANALOG);
  pinMode(SOC_GPIO_PIN_SCL,  INPUT_ANALOG);

  if (lmic_pins.rst != LMIC_UNUSED_PIN) pinMode(lmic_pins.rst,  INPUT_ANALOG);
#endif /* ARDUINO_WisDuo_RAK3172_Evaluation_Board */

#if defined(USBD_USE_CDC) && !defined(DISABLE_GENERIC_SERIALUSB)
  SerialOutput.end();
#endif

#if !defined(ARDUINO_WisDuo_RAK3172_Evaluation_Board)
  /*
   * Work around an issue that
   * WDT (once enabled) is active all the time
   * until hardware restart
   */
  setBackupRegister(SHUTDOWN_REASON_INDEX, reason);
#endif /* ARDUINO_WisDuo_RAK3172_Evaluation_Board */

  HAL_NVIC_SystemReset();
}

static void STM32_reset()
{
  HAL_NVIC_SystemReset();
}

static uint32_t STM32_getChipId()
{
#if !defined(SOFTRF_ADDRESS)
  /* Same method as STM32 OGN tracker does */
  uint32_t id = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();
  return DevID_Mapper(id);
#else
  return (SOFTRF_ADDRESS & 0xFFFFFFFFU );
#endif
}

static void* STM32_getResetInfoPtr()
{
  return (void *) &reset_info;
}

static String STM32_getResetInfo()
{
  switch (reset_info.reason)
  {
    default                     : return F("No reset information available");
  }
}

static String STM32_getResetReason()
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

#include <malloc.h>
extern "C" char *sbrk(int);
/* Use linker definition */
extern char _estack;
extern char _Min_Stack_Size;

static char *minSP = (char*)(&_estack - &_Min_Stack_Size);

static uint32_t STM32_getFreeHeap()
{
  char *heapend = (char*)sbrk(0);
  char * stack_ptr = (char*)__get_MSP();
  struct mallinfo mi = mallinfo();

  return ((stack_ptr < minSP) ? stack_ptr : minSP) - heapend + mi.fordblks ;
}

static long STM32_random(long howsmall, long howBig)
{
  return random(howsmall, howBig);
}

static void STM32_Sound_test(int var)
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

static void STM32_Sound_tone(int hz, uint8_t volume)
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

static void STM32_WiFi_set_param(int ndx, int value)
{
  /* NONE */
}

static void STM32_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  /* NONE */
}

static bool STM32_EEPROM_begin(size_t size)
{
#if !defined(ARDUINO_WisDuo_RAK3172_Evaluation_Board)
  if (size > E2END) {
    return false;
  }

#if !defined(EXCLUDE_EEPROM)
  EEPROM.begin();
#endif /* EXCLUDE_EEPROM */
#endif /* ARDUINO_WisDuo_RAK3172_Evaluation_Board */

  return true;
}

static void STM32_EEPROM_extension(int cmd)
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
#if defined(USBD_USE_CDC) && !defined(DISABLE_GENERIC_SERIALUSB)
      settings->nmea_out = NMEA_USB;
#else
      settings->nmea_out = NMEA_UART;
#endif
    }
    if (settings->gdl90 == GDL90_BLUETOOTH  ||
        settings->gdl90 == GDL90_UDP) {
#if defined(USBD_USE_CDC) && !defined(DISABLE_GENERIC_SERIALUSB)
      settings->gdl90 = GDL90_USB;
#else
      settings->gdl90 = GDL90_UART;
#endif
    }
    if (settings->d1090 == D1090_BLUETOOTH  ||
        settings->d1090 == D1090_UDP) {
#if defined(USBD_USE_CDC) && !defined(DISABLE_GENERIC_SERIALUSB)
      settings->d1090 = D1090_USB;
#else
      settings->d1090 = D1090_UART;
#endif
    }

#if defined(ARDUINO_NUCLEO_L073RZ)
    if (hw_info.model == SOFTRF_MODEL_BRACELET) {
      if (settings->nmea_out == NMEA_UART)  { settings->nmea_out = NMEA_USB;  }
      if (settings->gdl90    == GDL90_UART) { settings->gdl90    = GDL90_USB; }
      if (settings->d1090    == D1090_UART) { settings->d1090    = D1090_USB; }
    }
#elif defined(ARDUINO_GENERIC_WLE5CCUX)
    if (settings->nmea_out != NMEA_OFF) {
      settings->nmea_out = NMEA_UART;
    }
    if (settings->gdl90 != GDL90_OFF) {
      settings->gdl90 = GDL90_UART;
    }
    if (settings->d1090 != D1090_OFF) {
      settings->d1090 = D1090_UART;
    }
#endif /* ARDUINO_NUCLEO_L073RZ || ARDUINO_GENERIC_WLE5CCUX */

    /* AUTO and UK RF bands are deprecated since Release v1.3 */
    if (settings->band == RF_BAND_AUTO || settings->band == RF_BAND_UK) {
      settings->band = RF_BAND_EU;
    }
  }
}

static void STM32_SPI_begin()
{
#if !defined(ARDUINO_WisDuo_RAK3172_Evaluation_Board)
  SPI.setMISO(SOC_GPIO_PIN_MISO);
  SPI.setMOSI(stm32_board == STM32_EBYTE_E77 ? PB5 /*NC*/ : SOC_GPIO_PIN_MOSI);
  SPI.setSCLK(stm32_board == STM32_SEEED_E5  ? PB3 /*NC*/ : SOC_GPIO_PIN_SCK);
  // Slave Select pin is driven by RF driver
#endif /* ARDUINO_WisDuo_RAK3172_Evaluation_Board */

  SPI.begin();
}

static void STM32_swSer_begin(unsigned long baud)
{
  Serial_GNSS_In.begin(baud);

#if defined(ARDUINO_NUCLEO_L073RZ)
  /* drive GNSS RST pin low */
  pinMode(SOC_GPIO_PIN_GNSS_RST, OUTPUT);
  digitalWrite(SOC_GPIO_PIN_GNSS_RST, LOW);

  /* activate 1.8V<->3.3V level shifters */
  pinMode(SOC_GPIO_PIN_GNSS_LS,  OUTPUT);
  digitalWrite(SOC_GPIO_PIN_GNSS_LS,  HIGH);

  /* keep RST low to ensure proper IC reset */
  delay(200);

  /* release */
  digitalWrite(SOC_GPIO_PIN_GNSS_RST, HIGH);

  /* give Sony GNSS few ms to warm up */
  delay(100);

  /* Leave pin floating */
  pinMode(SOC_GPIO_PIN_GNSS_RST, INPUT);

#endif /* ARDUINO_NUCLEO_L073RZ */
}

static void STM32_swSer_enableRx(boolean arg)
{
  /* NONE */
}

static byte STM32_Display_setup()
{
  byte rval = DISPLAY_NONE;

#if defined(USE_OLED)
#if defined(ARDUINO_GENERIC_WL55CCUX)
  if (stm32_board == STM32_LILYGO_T3_1_0) {
    u8x8 = new U8X8_SSD1315_128X64_NONAME_4W_HW_SPI(SOC_GPIO_PIN_OLED_SS,
                                                    SOC_GPIO_PIN_OLED_DC,
                                                    U8X8_PIN_NONE);
    u8x8->begin();
    u8x8->setFlipMode(OLED_flip);
    u8x8->setFont(u8x8_font_chroma48medium8_r);

    u8x8->draw2x2String( 2, 2, SoftRF_text1);
    u8x8->drawString   ( 3, 6, SOFTRF_FIRMWARE_VERSION);
    u8x8->drawString   (11, 6, ISO3166_CC[settings->band]);

    rval = DISPLAY_OLED_TTGO;
  } else
#endif /* ARDUINO_GENERIC_WL55CCUX */
  {
    rval = OLED_setup();
  }
#endif /* USE_OLED */

  return rval;
}

static void STM32_Display_loop()
{
#if defined(USE_OLED)
  OLED_loop();
#endif /* USE_OLED */
}

static void STM32_Display_fini(int reason)
{
#if defined(USE_OLED)
  OLED_fini(reason);

#if defined(ARDUINO_GENERIC_WL55CCUX)
  if (stm32_board == STM32_LILYGO_T3_1_0) {
    delay(3000); /* Keep shutdown message on OLED for 3 seconds */

    u8x8->noDisplay();

    delete u8x8;
    u8x8 = NULL;
  }
#endif /* ARDUINO_GENERIC_WL55CCUX */
#endif /* USE_OLED */
}

static void STM32_Battery_setup()
{

}

static float STM32_Battery_param(uint8_t param)
{
  float rval, voltage;

  switch (param)
  {
  case BATTERY_PARAM_THRESHOLD:
    rval = hw_info.model == SOFTRF_MODEL_DONGLE   ||
           hw_info.model == SOFTRF_MODEL_BRACELET ||
           hw_info.model == SOFTRF_MODEL_LABUBU ? BATTERY_THRESHOLD_LIPO   :
                                                  BATTERY_THRESHOLD_NIMHX2;
    break;

  case BATTERY_PARAM_CUTOFF:
    rval = hw_info.model == SOFTRF_MODEL_DONGLE   ||
           hw_info.model == SOFTRF_MODEL_BRACELET ||
           hw_info.model == SOFTRF_MODEL_LABUBU ? BATTERY_CUTOFF_LIPO   :
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
#ifdef __LL_ADC_CALC_VREFANALOG_VOLTAGE
    int32_t Vref = (__LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION));
#else
    int32_t Vref = (VREFINT * ADC_RANGE / analogRead(AVREF)); // ADC sample to mV
#endif

    int32_t mV = (__LL_ADC_CALC_DATA_TO_VOLTAGE(Vref,
                                                analogRead(SOC_GPIO_PIN_BATTERY),
                                                LL_ADC_RESOLUTION));

    rval = mV * SOC_ADC_VOLTAGE_DIV / 1000.0;
    break;
  }

  return rval;
}

void STM32_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long STM32_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool STM32_Baro_setup() {
  return true;
}

static void STM32_UATSerial_begin(unsigned long baud)
{
  UATSerial.begin(baud);
}

static void STM32_UATModule_restart()
{
  digitalWrite(SOC_GPIO_PIN_TXE, LOW);
  pinMode(SOC_GPIO_PIN_TXE, OUTPUT);

  delay(100);

  digitalWrite(SOC_GPIO_PIN_TXE, HIGH);

  delay(100);

  pinMode(SOC_GPIO_PIN_TXE, INPUT);
}

static void STM32_WDT_setup()
{
#if !defined(ARDUINO_WisDuo_RAK3172_Evaluation_Board)
  // Init the watchdog timer with 5 seconds timeout
  IWatchdog.begin(5000000);
#endif /* ARDUINO_WisDuo_RAK3172_Evaluation_Board */
}

static void STM32_WDT_fini()
{
  /* once emabled - there is no way to disable WDT on STM32 */

#if !defined(ARDUINO_WisDuo_RAK3172_Evaluation_Board)
  if (IWatchdog.isEnabled()) {
    IWatchdog.set(IWDG_TIMEOUT_MAX);
  }
#endif /* ARDUINO_WisDuo_RAK3172_Evaluation_Board */
}

#include <AceButton.h>
using namespace ace_button;

AceButton button_1(SOC_GPIO_PIN_BUTTON, LOW);

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
      }
      break;
  }
}

/* Callbacks for push button interrupt */
void onPageButtonEvent() {
  button_1.check();
}

static void STM32_Button_setup()
{
  if (SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN   &&
      (hw_info.model == SOFTRF_MODEL_DONGLE   ||
       hw_info.model == SOFTRF_MODEL_BRACELET ||
       hw_info.model == SOFTRF_MODEL_BALKAN   ||
       hw_info.model == SOFTRF_MODEL_LABUBU)) {
    int button_pin = SOC_GPIO_PIN_BUTTON;

    // BOOT0 button(s) uses external pull DOWN resistor.
    pinMode(button_pin,
            hw_info.model == SOFTRF_MODEL_DONGLE ||
            hw_info.model == SOFTRF_MODEL_LABUBU ? INPUT_PULLDOWN :
            hw_info.model == SOFTRF_MODEL_BALKAN ? INPUT_PULLUP : INPUT);

    button_1.init(button_pin, hw_info.model == SOFTRF_MODEL_BALKAN ? HIGH : LOW);

    // Configure the ButtonConfig with the event handler, and enable all higher
    // level events.
    ButtonConfig* PageButtonConfig = button_1.getButtonConfig();
    PageButtonConfig->setEventHandler(handleEvent);
    PageButtonConfig->setFeature(ButtonConfig::kFeatureClick);
    PageButtonConfig->setFeature(ButtonConfig::kFeatureLongPress);
    PageButtonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
//  PageButtonConfig->setDebounceDelay(15);
    PageButtonConfig->setClickDelay(600);
    PageButtonConfig->setLongPressDelay(2000);
  }
}

static void STM32_Button_loop()
{
  if (SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN   &&
      (hw_info.model == SOFTRF_MODEL_DONGLE   ||
       hw_info.model == SOFTRF_MODEL_BRACELET ||
       hw_info.model == SOFTRF_MODEL_BALKAN   ||
       hw_info.model == SOFTRF_MODEL_LABUBU)) {
    button_1.check();
  }
}

static void STM32_Button_fini()
{
  if (SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN   &&
      (hw_info.model == SOFTRF_MODEL_DONGLE   ||
       hw_info.model == SOFTRF_MODEL_BRACELET ||
       hw_info.model == SOFTRF_MODEL_BALKAN   ||
       hw_info.model == SOFTRF_MODEL_LABUBU)) {
    pinMode(SOC_GPIO_PIN_BUTTON,
            hw_info.model == SOFTRF_MODEL_DONGLE ||
            hw_info.model == SOFTRF_MODEL_LABUBU ? INPUT_PULLDOWN :
            hw_info.model == SOFTRF_MODEL_BALKAN ? INPUT_PULLUP : INPUT);
    bool button_is_active = (hw_info.model == SOFTRF_MODEL_BALKAN ? LOW : HIGH);
    while (digitalRead(SOC_GPIO_PIN_BUTTON) == button_is_active);

#if !defined(ARDUINO_WisDuo_RAK3172_Evaluation_Board)
    pinMode(SOC_GPIO_PIN_BUTTON, INPUT_ANALOG);
#endif /* ARDUINO_WisDuo_RAK3172_Evaluation_Board */
  }
}

#if defined(USBD_USE_CDC)

#include <USBSerial.h>

static void STM32_USB_setup()
{
#if defined(DISABLE_GENERIC_SERIALUSB)
  SerialUSB.begin();
#endif
}

static void STM32_USB_loop()
{

}

static void STM32_USB_fini()
{
#if defined(DISABLE_GENERIC_SERIALUSB)
  SerialUSB.end();
#endif
}

static int STM32_USB_available()
{
  return SerialUSB.available();
}

static int STM32_USB_read()
{
  return SerialUSB.read();
}

static size_t STM32_USB_write(const uint8_t *buffer, size_t size)
{
  return SerialUSB.write(buffer, size);
}

IODev_ops_t STM32_USBSerial_ops = {
  "STM32 USBSerial",
  STM32_USB_setup,
  STM32_USB_loop,
  STM32_USB_fini,
  STM32_USB_available,
  STM32_USB_read,
  STM32_USB_write
};

#endif /* USBD_USE_CDC */

const SoC_ops_t STM32_ops = {
  SOC_STM32,
  "STM32",
  STM32_setup,
  STM32_post_init,
  STM32_loop,
  STM32_fini,
  STM32_reset,
  STM32_getChipId,
  STM32_getResetInfoPtr,
  STM32_getResetInfo,
  STM32_getResetReason,
  STM32_getFreeHeap,
  STM32_random,
  STM32_Sound_test,
  STM32_Sound_tone,
  NULL,
  STM32_WiFi_set_param,
  STM32_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  STM32_EEPROM_begin,
  STM32_EEPROM_extension,
  STM32_SPI_begin,
  STM32_swSer_begin,
  STM32_swSer_enableRx,
  NULL, /* STM32 has no built-in Bluetooth */
#if defined(USBD_USE_CDC)
  &STM32_USBSerial_ops,
#else
  NULL,
#endif
  NULL,
  STM32_Display_setup,
  STM32_Display_loop,
  STM32_Display_fini,
  STM32_Battery_setup,
  STM32_Battery_param,
  STM32_GNSS_PPS_Interrupt_handler,
  STM32_get_PPS_TimeMarker,
  STM32_Baro_setup,
  STM32_UATSerial_begin,
  STM32_UATModule_restart,
  STM32_WDT_setup,
  STM32_WDT_fini,
  STM32_Button_setup,
  STM32_Button_loop,
  STM32_Button_fini,
  NULL
};

#endif /* ARDUINO_ARCH_STM32 */
