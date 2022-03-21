/*
 * Platform_AVR.cpp
 * Copyright (C) 2021-2022 Linar Yusupov
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

#if defined(ARDUINO_ARCH_AVR)

#include <SPI.h>
#include <Wire.h>

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

#include <Adafruit_SleepyDog.h>
#include <avr/boot.h>

// SX127x pin mapping
lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .txe = LMIC_UNUSED_PIN,
    .rxe = LMIC_UNUSED_PIN,
    .rst = SOC_GPIO_PIN_RST,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
    .busy = LMIC_UNUSED_PIN,
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

char UDPpacketBuffer[4]; // Dummy definition to satisfy build sequence

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

static uint32_t bootCount __attribute__ ((section (".noinit")));
static bool wdt_is_active = false;

#define UniqueIDsize 9
uint8_t DeviceID[UniqueIDsize];

static void AVR_setup()
{

  for (size_t i = 0; i < UniqueIDsize; i++)
  {
    DeviceID[i] = boot_signature_byte_get(0x0E + i + (UniqueIDsize == 9 && i > 5 ? 1 : 0));
  }

  Serial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
}

static void AVR_post_init()
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
    case NMEA_UART       :  Serial.println(F("UART")); break;
    case NMEA_USB        :  Serial.println(F("USB"));  break;
    case NMEA_OFF        :
    default              :  Serial.println(F("NULL")); break;
  }

  Serial.print(F("GDL90  - "));
  switch (settings->gdl90)
  {
    case GDL90_UART      :  Serial.println(F("UART")); break;
    case GDL90_USB       :  Serial.println(F("USB"));  break;
    case GDL90_OFF       :
    default              :  Serial.println(F("NULL")); break;
  }

  Serial.print(F("D1090  - "));
  switch (settings->d1090)
  {
    case D1090_UART      :  Serial.println(F("UART")); break;
    case D1090_USB       :  Serial.println(F("USB"));  break;
    case D1090_OFF       :
    default              :  Serial.println(F("NULL")); break;
  }

  Serial.println();
  Serial.flush();

#if defined(USE_OLED)
  OLED_info1();
#endif /* USE_OLED */
}

static bool prev_PPS_state = LOW;

static void AVR_loop()
{
  if (wdt_is_active) {
    Watchdog.reset();
  }

#if SOC_GPIO_PIN_GNSS_PPS != SOC_UNUSED_PIN
  if (digitalPinToInterrupt(SOC_GPIO_PIN_GNSS_PPS) == NOT_AN_INTERRUPT) {
    bool PPS_state = digitalRead(SOC_GPIO_PIN_GNSS_PPS);

    if (PPS_state == HIGH && prev_PPS_state == LOW) {
      PPS_TimeMarker = millis();
    }
    prev_PPS_state = PPS_state;
  }
#endif
}

static void AVR_fini(int reason)
{
  Watchdog.sleep();
}

static void AVR_reset()
{
  void (*reboot)() = 0x0000 ;
  (*reboot)();
}

static uint32_t AVR_getChipId()
{
#if !defined(SOFTRF_ADDRESS)
  uint32_t id = DeviceID[UniqueIDsize-4];
  id = (id << 8) | DeviceID[UniqueIDsize-3];
  id = (id << 8) | DeviceID[UniqueIDsize-2];
  id = (id << 8) | DeviceID[UniqueIDsize-1];

  return DevID_Mapper(id);
#else
  return (SOFTRF_ADDRESS & 0xFFFFFFFFU );
#endif
}

static void* AVR_getResetInfoPtr()
{
  return (void *) &reset_info;
}

static String AVR_getResetInfo()
{
  switch (reset_info.reason)
  {
    default                     : return F("No reset information available");
  }
}

static String AVR_getResetReason()
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


static uint32_t AVR_getFreeHeap()
{
  extern int __heap_start,*__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int) __brkval);
}

static long AVR_random(long howsmall, long howBig)
{
  if(howsmall >= howBig) {
      return howsmall;
  }
  long diff = howBig - howsmall;

  return random(diff) + howsmall;
}

static void AVR_Sound_test(int var)
{
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && settings->volume != BUZZER_OFF) {
    tone(SOC_GPIO_PIN_BUZZER, 440,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 640,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 840,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 1040, 500); delay(600);
  }
}

static void AVR_Sound_tone(int hz, uint8_t volume)
{
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && volume != BUZZER_OFF) {
    if (hz > 0) {
      tone(SOC_GPIO_PIN_BUZZER, hz, ALARM_TONE_MS);
    } else {
      noTone(SOC_GPIO_PIN_BUZZER);
    }
  }
}

static void AVR_WiFi_set_param(int ndx, int value)
{
  /* NONE */
}

static void AVR_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  /* NONE */
}

static bool AVR_EEPROM_begin(size_t size)
{
  EEPROM.begin();

  return true;
}

static void AVR_EEPROM_extension(int cmd)
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
  }
}

static void AVR_SPI_begin()
{
  SPI.begin();
}

static void AVR_swSer_begin(unsigned long baud)
{
  Serial_GNSS_In.begin(baud);
}

static void AVR_swSer_enableRx(boolean arg)
{
  /* NONE */
}

static byte AVR_Display_setup()
{
  byte rval = DISPLAY_NONE;

#if defined(USE_OLED)
  rval = OLED_setup();
#endif /* USE_OLED */

  return rval;
}

static void AVR_Display_loop()
{
#if defined(USE_OLED)
  OLED_loop();
#endif /* USE_OLED */
}

static void AVR_Display_fini(int reason)
{
#if defined(USE_OLED)
  OLED_fini(reason);
#endif /* USE_OLED */
}

static void AVR_Battery_setup()
{

}

static float AVR_Battery_param(uint8_t param)
{
  float rval, voltage;

  switch (param)
  {
  case BATTERY_PARAM_THRESHOLD:
    rval = hw_info.model == SOFTRF_MODEL_ACADEMY ? BATTERY_THRESHOLD_LIPO   :
                                                   BATTERY_THRESHOLD_NIMHX2;
    break;

  case BATTERY_PARAM_CUTOFF:
    rval = hw_info.model == SOFTRF_MODEL_ACADEMY ? BATTERY_CUTOFF_LIPO   :
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

void AVR_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long AVR_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool AVR_Baro_setup() {
  return true;
}

static void AVR_UATSerial_begin(unsigned long baud)
{

}

static void AVR_UATModule_restart()
{

}

static void AVR_WDT_setup()
{
  Watchdog.enable(8000);
  wdt_is_active = true;
}

static void AVR_WDT_fini()
{
  if (wdt_is_active) {
    Watchdog.disable();
    wdt_is_active = false;
  }
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

static void AVR_Button_setup()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  if (hw_info.model == SOFTRF_MODEL_ACADEMY) {
    int button_pin = SOC_GPIO_PIN_BUTTON;

    // Button(s) uses external pull up resistor.
    pinMode(button_pin, INPUT);

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

static void AVR_Button_loop()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  if (hw_info.model == SOFTRF_MODEL_ACADEMY) {
    button_1.check();
  }
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */
}

static void AVR_Button_fini()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  if (hw_info.model == SOFTRF_MODEL_ACADEMY) {
//  detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_BUTTON));
    while (digitalRead(SOC_GPIO_PIN_BUTTON) == LOW);
    pinMode(SOC_GPIO_PIN_BUTTON, ANALOG);
  }
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */
}

static void AVR_USB_setup()
{
  if (USBSerial && USBSerial != Serial) {
    USBSerial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
  }
}

static void AVR_USB_loop()
{

}

static void AVR_USB_fini()
{
  if (USBSerial && USBSerial != Serial) {
    USBSerial.end();
  }
}

static int AVR_USB_available()
{
  int rval = 0;

  if (USBSerial) {
    rval = USBSerial.available();
  }

  return rval;
}

static int AVR_USB_read()
{
  int rval = -1;

  if (USBSerial) {
    rval = USBSerial.read();
  }

  return rval;
}

static size_t AVR_USB_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  if (USBSerial && (size < USBSerial.availableForWrite())) {
    rval = USBSerial.write(buffer, size);
  }

  return rval;
}

IODev_ops_t AVR_USBSerial_ops = {
  "AVR USBSerial",
  AVR_USB_setup,
  AVR_USB_loop,
  AVR_USB_fini,
  AVR_USB_available,
  AVR_USB_read,
  AVR_USB_write
};

const SoC_ops_t AVR_ops = {
  SOC_AVR,
  "AVR",
  AVR_setup,
  AVR_post_init,
  AVR_loop,
  AVR_fini,
  AVR_reset,
  AVR_getChipId,
  AVR_getResetInfoPtr,
  AVR_getResetInfo,
  AVR_getResetReason,
  AVR_getFreeHeap,
  AVR_random,
  AVR_Sound_test,
  AVR_Sound_tone,
  NULL,
  AVR_WiFi_set_param,
  AVR_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  AVR_EEPROM_begin,
  AVR_EEPROM_extension,
  AVR_SPI_begin,
  AVR_swSer_begin,
  AVR_swSer_enableRx,
  NULL, /* AVR has no built-in Bluetooth */
  &AVR_USBSerial_ops,
  NULL,
  AVR_Display_setup,
  AVR_Display_loop,
  AVR_Display_fini,
  AVR_Battery_setup,
  AVR_Battery_param,
  AVR_GNSS_PPS_Interrupt_handler,
  AVR_get_PPS_TimeMarker,
  AVR_Baro_setup,
  AVR_UATSerial_begin,
  AVR_UATModule_restart,
  AVR_WDT_setup,
  AVR_WDT_fini,
  AVR_Button_setup,
  AVR_Button_loop,
  AVR_Button_fini,
  NULL
};

#endif /* ARDUINO_ARCH_AVR */
