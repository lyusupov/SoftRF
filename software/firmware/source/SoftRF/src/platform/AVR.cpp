/*
 * Platform_AVR.cpp
 * Copyright (C) 2021 Linar Yusupov
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


// SX1262 pin mapping
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

static uint32_t bootCount = 0;

static void AVR_setup()
{

}

static void AVR_post_init()
{

}

static void AVR_loop()
{

}

static void AVR_fini(int reason)
{

}

static void AVR_reset()
{

}

static uint32_t AVR_getChipId()
{
#if !defined(SOFTRF_ADDRESS)
  uint32_t id = 0;

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
  return 0;
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

}

static void AVR_Sound_tone(int hz, uint8_t volume)
{
  /* TBD */
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
  swSer.begin(baud);
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

}

static void AVR_WDT_fini()
{

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

static void AVR_UART_loop()
{

}

static size_t AVR_UART_write(const uint8_t *buffer, size_t size)
{
  return size;
}

IODev_ops_t AVR_UART_ops = {
  "AVR UART",
  NULL,
  AVR_UART_loop,
  NULL,
  NULL,
  NULL,
  AVR_UART_write
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
  NULL, /* TBD */
  &AVR_UART_ops,
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
