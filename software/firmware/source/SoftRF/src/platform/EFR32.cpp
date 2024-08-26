/*
 * Platform_EFR32.cpp
 * Copyright (C) 2024 Linar Yusupov
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

#if defined(ARDUINO_ARCH_SILABS)

#include <SPI.h>

#include "../system/SoC.h"
#include "../driver/RF.h"
#include "../driver/EEPROM.h"
#include "../driver/LED.h"
#include "../driver/OLED.h"
#include "../driver/Baro.h"
#include "../driver/Sound.h"
#include "../driver/Battery.h"
#include "../driver/Bluetooth.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/GDL90.h"
#include "../protocol/data/D1090.h"

#include <ArduinoLowPower.h>

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

char* ultoa(unsigned long value, char *string, int radix)
{
  char tmp[33];
  char *tp = tmp;
  long i;
  unsigned long v = value;
  char *sp;
  if ( string == NULL )
  {
    return 0;
  }
  if (radix > 36 || radix <= 1)
  {
    return 0;
  }
 
  while (v || tp == tmp)
  {
    i = v % radix;
    v = v / radix;
    if (i < 10)
      *tp++ = i+'0';
    else
      *tp++ = i + 'a' - 10;
  }
  sp = string;
 
  while (tp > tmp)
    *sp++ = *--tp;
  *sp = 0;
  return string;
}

#if (__GNUC__ <= 9)
volatile int __sf;
#endif

#if defined(EXCLUDE_EEPROM)
eeprom_t eeprom_block;
settings_t *settings = &eeprom_block.field.settings;
#endif /* EXCLUDE_EEPROM */

#if defined(USE_SOFTSPI)
#include <SoftSPI.h>
SoftSPI RadioSPI(SOC_GPIO_PIN_MOSI,SOC_GPIO_PIN_MISO, SOC_GPIO_PIN_SCK);
#endif /* USE_SOFTSPI */

#if defined __has_include
#if __has_include(<FlexWire.h>)
#include <FlexWire.h>

FlexWire Wire = FlexWire(SOC_GPIO_PIN_SDA, SOC_GPIO_PIN_SCL, false);
#endif /* FlexWire.h */
#endif /* __has_include */

static void EFR32_setup()
{
  uint32_t reset_cause = get_system_reset_cause();

  if      (reset_cause & EMU_RSTCAUSE_POR) /* Power On Reset */
  {
      reset_info.reason = REASON_DEFAULT_RST;
  }
  else if (reset_cause & EMU_RSTCAUSE_PIN) /* Pin Reset */
  {
      reset_info.reason = REASON_EXT_SYS_RST;
  }
  else if (reset_cause & EMU_RSTCAUSE_EM4) /* EM4 Wakeup Reset */
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
  }
  else if (reset_cause & EMU_RSTCAUSE_WDOG0) /* Watchdog 0 Reset */
  {
      reset_info.reason = REASON_WDT_RST;
  }
#if defined(ARDUINO_NANO_MATTER)
  else if (reset_cause & EMU_RSTCAUSE_WDOG1) /* Watchdog 1 Reset */
  {
      reset_info.reason = REASON_WDT_RST;
  }
#endif /* ARDUINO_NANO_MATTER */
  else if (reset_cause & EMU_RSTCAUSE_LOCKUP) /* M33 Core Lockup Reset */
  {
      reset_info.reason = REASON_EXCEPTION_RST;
  }
  else if (reset_cause & EMU_RSTCAUSE_SYSREQ) /* M33 Core Sys Reset */
  {
      reset_info.reason = REASON_SOFT_RESTART;
  }
  else if (reset_cause & EMU_RSTCAUSE_DVDDBOD) /* HVBOD Reset */
  {
      reset_info.reason = REASON_EXCEPTION_RST;
  }
  else if (reset_cause & EMU_RSTCAUSE_DVDDLEBOD) /* LEBOD Reset */
  {
      reset_info.reason = REASON_EXCEPTION_RST;
  }
  else if (reset_cause & EMU_RSTCAUSE_DECBOD) /* LVBOD Reset */
  {
      reset_info.reason = REASON_EXCEPTION_RST;
  }
  else if (reset_cause & EMU_RSTCAUSE_AVDDBOD) /* LEBOD1 Reset */
  {
      reset_info.reason = REASON_EXCEPTION_RST;
  }
  else if (reset_cause & EMU_RSTCAUSE_IOVDD0BOD) /* LEBOD2 Reset */
  {
      reset_info.reason = REASON_EXCEPTION_RST;
  }
#if defined(ARDUINO_NANO_MATTER)
  else if (reset_cause & EMU_RSTCAUSE_SETAMPER) /* SE Tamper event Reset */
  {
      reset_info.reason = REASON_EXCEPTION_RST;
  }
#endif /* ARDUINO_NANO_MATTER */
#if defined(ARDUINO_SILABS_BGM220EXPLORERKIT)
  else if (reset_cause & EMU_RSTCAUSE_DCI) /* DCI reset */
  {
      reset_info.reason = REASON_EXCEPTION_RST;
  }
#endif /* ARDUINO_SILABS_BGM220EXPLORERKIT */
  else if (reset_cause & EMU_RSTCAUSE_VREGIN) /* DCDC VREGIN comparator */
  {
      reset_info.reason = REASON_EXCEPTION_RST;
  }

#if defined(USE_RADIOLIB)
  lmic_pins.dio[0] = SOC_GPIO_PIN_DIO1;
#endif /* USE_RADIOLIB */

#if SOC_GPIO_RADIO_LED_TX != SOC_UNUSED_PIN
  pinMode(SOC_GPIO_RADIO_LED_TX, OUTPUT);
  digitalWrite(SOC_GPIO_RADIO_LED_TX, ! LED_STATE_ON);
#endif /* SOC_GPIO_RADIO_LED_TX */
#if SOC_GPIO_RADIO_LED_RX != SOC_UNUSED_PIN
  pinMode(SOC_GPIO_RADIO_LED_RX, OUTPUT);
  digitalWrite(SOC_GPIO_RADIO_LED_RX, ! LED_STATE_ON);
#endif /* SOC_GPIO_RADIO_LED_RX */

  Serial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
}

static void EFR32_post_init()
{
  {
    Serial.println();
    Serial.println(F("SoftRF Academy Edition Power-on Self Test"));
    Serial.println();
    Serial.flush();

    Serial.println(F("Built-in components:"));

    Serial.print(F("RADIO   : "));
    Serial.println(hw_info.rf      == RF_IC_SX1276      ? F("PASS") : F("FAIL"));
    Serial.print(F("GNSS    : "));
    Serial.println(hw_info.gnss    != GNSS_MODULE_NONE  ? F("PASS") : F("FAIL"));

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

static void EFR32_loop()
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

static void EFR32_fini(int reason)
{
  switch (reason)
  {
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  case SOFTRF_SHUTDOWN_BUTTON:
  case SOFTRF_SHUTDOWN_LOWBAT:
    /* the GPIO must have EM4 wake functionality */
    pinMode(SOC_GPIO_PIN_BUTTON, INPUT_PULLUP);
    LowPower.attachInterruptWakeup(SOC_GPIO_PIN_BUTTON, nullptr, FALLING);
    break;
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */
  case SOFTRF_SHUTDOWN_NMEA:
    /* the GPIO must have EM4 wake functionality */
    pinMode(SOC_GPIO_PIN_BUTTON, INPUT_PULLUP);
    LowPower.attachInterruptWakeup(SOC_GPIO_PIN_CONS_RX, nullptr, FALLING);
    break;
  default:
    break;
  }

  LowPower.deepSleep();
}

static void EFR32_reset()
{
  NVIC_SystemReset();
}

static uint32_t EFR32_getChipId()
{
#if !defined(SOFTRF_ADDRESS)
  uint32_t id = getDeviceUniqueId();

  return DevID_Mapper(id);
#else
  return (SOFTRF_ADDRESS & 0xFFFFFFFFU );
#endif
}

static void* EFR32_getResetInfoPtr()
{
  return (void *) &reset_info;
}

static String EFR32_getResetInfo()
{
  switch (reset_info.reason)
  {
    default                       : return F("No reset information available");
  }
}

static String EFR32_getResetReason()
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

static uint32_t EFR32_getFreeHeap()
{
  return 0; /* TODO */
}

static long EFR32_random(long howsmall, long howBig)
{
  if(howsmall >= howBig) {
      return howsmall;
  }
  long diff = howBig - howsmall;

  return random(diff) + howsmall;
}

static void EFR32_Sound_test(int var)
{
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && settings->volume != BUZZER_OFF) {
    tone(SOC_GPIO_PIN_BUZZER, 440,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 640,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 840,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 1040, 500); delay(600);
  }
}

static void EFR32_Sound_tone(int hz, uint8_t volume)
{
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && volume != BUZZER_OFF) {
    if (hz > 0) {
      tone(SOC_GPIO_PIN_BUZZER, hz, ALARM_TONE_MS);
    } else {
      noTone(SOC_GPIO_PIN_BUZZER);
    }
  }
}

static void EFR32_WiFi_set_param(int ndx, int value)
{
  /* NONE */
}

static void EFR32_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  /* NONE */
}

static bool EFR32_EEPROM_begin(size_t size)
{
  return true;
}

static void EFR32_EEPROM_extension(int cmd)
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

    if (settings->nmea_out == NMEA_USB ||
        settings->nmea_out == NMEA_UDP ||
        settings->nmea_out == NMEA_TCP ) {
      settings->nmea_out = NMEA_UART;
    }
    if (settings->gdl90 == GDL90_USB || settings->gdl90 == GDL90_UDP) {
      settings->gdl90 = GDL90_UART;
    }
    if (settings->d1090 == D1090_USB || settings->d1090 == D1090_UDP) {
      settings->d1090 = D1090_UART;
    }

    /* AUTO and UK RF bands are deprecated since Release v1.3 */
    if (settings->band == RF_BAND_AUTO || settings->band == RF_BAND_UK) {
      settings->band = RF_BAND_EU;
    }
  }
}

static void EFR32_SPI_begin()
{
#if !defined(EXCLUDE_NRF905) || defined(USE_OGN_RF_DRIVER)
  SPI.begin();
#endif
}

static void EFR32_swSer_begin(unsigned long baud)
{
  Serial_GNSS_In.begin(baud);
}

static void EFR32_swSer_enableRx(boolean arg)
{
  /* NONE */
}

static byte EFR32_Display_setup()
{
  byte rval = DISPLAY_NONE;

#if defined(USE_OLED)
  rval = OLED_setup();
#endif /* USE_OLED */

  return rval;
}

static void EFR32_Display_loop()
{
#if defined(USE_OLED)
  OLED_loop();
#endif /* USE_OLED */
}

static void EFR32_Display_fini(int reason)
{
#if defined(USE_OLED)
  OLED_fini(reason);
#endif /* USE_OLED */
}

static void EFR32_Battery_setup()
{

}

static float EFR32_Battery_param(uint8_t param)
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

void EFR32_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long EFR32_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool EFR32_Baro_setup() {
  return true;
}

static void EFR32_UATSerial_begin(unsigned long baud)
{

}

static void EFR32_UATModule_restart()
{

}

static void EFR32_WDT_setup()
{
  /* TBD */
}

static void EFR32_WDT_fini()
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

static void EFR32_Button_setup()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  if (hw_info.model == SOFTRF_MODEL_ACADEMY) {
    int button_pin = SOC_GPIO_PIN_BUTTON;

    // Button(s) uses external pull up resistor.
    pinMode(button_pin, INPUT_PULLUP);

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

static void EFR32_Button_loop()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  if (hw_info.model == SOFTRF_MODEL_ACADEMY) {
    button_1.check();
  }
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */
}

static void EFR32_Button_fini()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  if (hw_info.model == SOFTRF_MODEL_ACADEMY) {
//  detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_BUTTON));
    while (digitalRead(SOC_GPIO_PIN_BUTTON) == LOW);
  }
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */
}

const SoC_ops_t EFR32_ops = {
  SOC_EFR32,
  "EFR32",
  EFR32_setup,
  EFR32_post_init,
  EFR32_loop,
  EFR32_fini,
  EFR32_reset,
  EFR32_getChipId,
  EFR32_getResetInfoPtr,
  EFR32_getResetInfo,
  EFR32_getResetReason,
  EFR32_getFreeHeap,
  EFR32_random,
  EFR32_Sound_test,
  EFR32_Sound_tone,
  NULL,
  EFR32_WiFi_set_param,
  EFR32_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  EFR32_EEPROM_begin,
  EFR32_EEPROM_extension,
  EFR32_SPI_begin,
  EFR32_swSer_begin,
  EFR32_swSer_enableRx,
#if !defined(EXCLUDE_BLUETOOTH)
  &ArdBLE_Bluetooth_ops,
#else
  NULL,
#endif /* EXCLUDE_BLUETOOTH */
  NULL,
  NULL,
  EFR32_Display_setup,
  EFR32_Display_loop,
  EFR32_Display_fini,
  EFR32_Battery_setup,
  EFR32_Battery_param,
  EFR32_GNSS_PPS_Interrupt_handler,
  EFR32_get_PPS_TimeMarker,
  EFR32_Baro_setup,
  EFR32_UATSerial_begin,
  EFR32_UATModule_restart,
  EFR32_WDT_setup,
  EFR32_WDT_fini,
  EFR32_Button_setup,
  EFR32_Button_loop,
  EFR32_Button_fini,
  NULL
};

#endif /* ARDUINO_ARCH_SILABS */
