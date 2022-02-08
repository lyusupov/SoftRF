/*
 * Platform_ASR66.cpp
 * Copyright (C) 2022 Linar Yusupov
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

#if defined(__ASR6601__) || defined(ARDUINO_ARCH_ASR6601)

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
CubeCell_NeoPixel strip = CubeCell_NeoPixel(PIX_NUM, SOC_GPIO_PIN_LED,
                              NEO_GRB + NEO_KHZ800);
#endif /* EXCLUDE_LED_RING */

char UDPpacketBuffer[4]; // Dummy definition to satisfy build sequence

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

static uint32_t bootCount __attribute__ ((section (".noinit")));

typedef enum
{
  ASR66_LOW_POWER,
  ASR66_ACTIVE
} ASR66_states_t;

static ASR66_states_t ASR66_state = ASR66_ACTIVE;

#if defined(BAT_MON_DISABLE)
static bool ASR66_bat_mon_disable = false;
#endif

static int ASR66_probe_pin(uint32_t pin, uint32_t mode)
{
  return 0;
}

static void ASR66_SerialWakeup()
{
  if (hw_info.model == SOFTRF_MODEL_MINI &&
      ASR66_state   == ASR66_LOW_POWER) {
    detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_CONS_RX));
    ASR66_state = ASR66_ACTIVE;
    NVIC_SystemReset();
  }
}

#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
static void ASR66_User_Key_Wakeup()
{
  if (hw_info.model == SOFTRF_MODEL_MINI &&
      ASR66_state   == ASR66_LOW_POWER) {

    delay(10);

    if (digitalRead(SOC_GPIO_PIN_BUTTON) == LOW) {
      detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_BUTTON));
      ASR66_state = ASR66_ACTIVE;

      while (digitalRead(SOC_GPIO_PIN_BUTTON) == LOW);
      delay(100);
      NVIC_SystemReset();
    }
  }
}
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */

static void ASR66_setup()
{

}

static void ASR66_post_init()
{
  {
    Serial.println();
    Serial.println(F("SoftRF Multi Edition Power-on Self Test"));
    Serial.println();
    SERIAL_FLUSH();

    Serial.println(F("Components:"));

    Serial.print(F("RADIO   : "));
    Serial.println(hw_info.rf      == RF_IC_SX1262      ? F("PASS") : F("FAIL"));
    Serial.print(F("GNSS    : "));
    Serial.println(hw_info.gnss    != GNSS_MODULE_NONE  ? F("PASS") : F("N/A"));
    Serial.print(F("BARO    : "));
    Serial.println(hw_info.baro    != BARO_MODULE_NONE  ? F("PASS") : F("N/A"));
    Serial.print(F("DISPLAY : "));
    Serial.println(hw_info.display != DISPLAY_NONE      ? F("PASS") : F("N/A"));

    Serial.println();
    Serial.println(F("Power-on Self Test is complete."));
    Serial.println();
    SERIAL_FLUSH();
  }

  Serial.println(F("Data output device(s):"));

  Serial.print(F("NMEA   - "));
  switch (settings->nmea_out)
  {
    case NMEA_UART       :  Serial.println(F("UART"));    break;
    case NMEA_OFF        :
    default              :  Serial.println(F("NULL"));    break;
  }

  Serial.print(F("GDL90  - "));
  switch (settings->gdl90)
  {
    case GDL90_UART      :  Serial.println(F("UART"));    break;
    case GDL90_OFF       :
    default              :  Serial.println(F("NULL"));    break;
  }

  Serial.print(F("D1090  - "));
  switch (settings->d1090)
  {
    case D1090_UART      :  Serial.println(F("UART"));    break;
    case D1090_OFF       :
    default              :  Serial.println(F("NULL"));    break;
  }

  Serial.println();
  SERIAL_FLUSH();

#if defined(USE_OLED)
  OLED_info1();
#endif /* USE_OLED */
}

static bool prev_PPS_state = LOW;

static void ASR66_loop()
{
  if (ASR66_state == ASR66_LOW_POWER) {
    lowPowerHandler();
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

static void ASR66_fini(int reason)
{

}

static void ASR66_reset()
{
  NVIC_SystemReset();
}

static uint32_t ASR66_getChipId()
{
#if !defined(SOFTRF_ADDRESS)
  uint32_t id = getID();

  return DevID_Mapper(id);
#else
  return (SOFTRF_ADDRESS & 0xFFFFFFFFU );
#endif
}

static void* ASR66_getResetInfoPtr()
{
  return (void *) &reset_info;
}

static String ASR66_getResetInfo()
{
  switch (reset_info.reason)
  {
    default                     : return F("No reset information available");
  }
}

static String ASR66_getResetReason()
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

extern "C" void * _sbrk_r (int);
extern "C" void * _sbrk   (int);
extern int _end;

static uint32_t ASR66_getFreeHeap()
{
//  uint8 *heapend = (uint8 *) _sbrk_r(0);
//  return CYDEV_HEAP_SIZE - (heapend - (uint8 *) &_end);

  char top;
  return &top - reinterpret_cast<char*>(_sbrk(0));
}

static long ASR66_random(long howsmall, long howBig)
{
  if(howsmall >= howBig) {
      return howsmall;
  }
  long diff = howBig - howsmall;

  return random(diff) + howsmall;
}

static void ASR66_Sound_test(int var)
{

}

static void ASR66_Sound_tone(int hz, uint8_t volume)
{
  /* TBD */
}

static void ASR66_WiFi_set_param(int ndx, int value)
{
  /* NONE */
}

static void ASR66_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  /* NONE */
}

static bool ASR66_EEPROM_begin(size_t size)
{
  EEPROM.begin(size);

  return true;
}

static void ASR66_EEPROM_extension(int cmd)
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

    if (settings->nmea_out != NMEA_OFF) {
      settings->nmea_out = NMEA_UART;
    }
    if (settings->gdl90 != GDL90_OFF) {
      settings->gdl90 = GDL90_UART;
    }
    if (settings->d1090 != D1090_OFF) {
      settings->d1090 = D1090_UART;
    }
  }
}

static void ASR66_SPI_begin()
{
#if 0
    LORAC->CR0 = 0x00000200;

    LORAC->SSP_CR0 = 0x07;
    LORAC->SSP_CPSR = 0x02;

    if(LORAC->CR1 != 0x80)
    {
        delayMicroseconds(20);
        LORAC->NSS_CR = 0;
        delayMicroseconds(20);
        LORAC->NSS_CR = 1;
    }

    LORAC->SSP_CR1 = 0x02;
#endif
}

static void ASR66_swSer_begin(unsigned long baud)
{
  Serial_GNSS_In.begin(baud);

  Serial_GNSS_Out.begin(baud);
  iomux(SOC_GPIO_PIN_GNSS_TX, 2); /* RX -> TX */
  iomux(SOC_GPIO_PIN_UART3_RX,2); /* TX -> RX */
}

static void ASR66_swSer_enableRx(boolean arg)
{
  /* NONE */
}

static byte ASR66_Display_setup()
{
  byte rval = DISPLAY_NONE;

#if defined(USE_OLED)
  rval = OLED_setup();
#endif /* USE_OLED */

  return rval;
}

static void ASR66_Display_loop()
{
#if defined(USE_OLED)
  OLED_loop();
#endif /* USE_OLED */
}

static void ASR66_Display_fini(int reason)
{
#if defined(USE_OLED)
  OLED_fini(reason);
#endif /* USE_OLED */
}

static void ASR66_Battery_setup()
{
#if defined(BAT_MON_DISABLE)
  if (hw_info.model == SOFTRF_MODEL_MINI) {
    pinMode(SOC_GPIO_PIN_BMON_DIS, INPUT_PULLDOWN);

    delay(100);

    ASR66_bat_mon_disable = (digitalRead(SOC_GPIO_PIN_BMON_DIS) == HIGH ?
                             true : false);
    pinMode(SOC_GPIO_PIN_BMON_DIS, ANALOG);
  }
#endif
}

static float ASR66_Battery_param(uint8_t param)
{
  float rval, voltage;

  switch (param)
  {
  case BATTERY_PARAM_THRESHOLD:
    rval = hw_info.model == SOFTRF_MODEL_MINI ? BATTERY_THRESHOLD_LIPO   :
                                                BATTERY_THRESHOLD_NIMHX2;
    break;

  case BATTERY_PARAM_CUTOFF:
    rval = hw_info.model == SOFTRF_MODEL_MINI ? BATTERY_CUTOFF_LIPO   :
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

  #if defined(BAT_MON_DISABLE)
    if (ASR66_bat_mon_disable) {
      rval = 0;
    } else
  #endif
    {
      uint16_t mV = 0;

      if (hw_info.model == SOFTRF_MODEL_MINI &&
          ASR66_state   == ASR66_ACTIVE) {
        /* GPIO7 is shared between USER_KEY and VBAT_ADC_CTL functions */
        int user_key_state = digitalRead(SOC_GPIO_PIN_BUTTON);

        /* if the key is not pressed down - activate VBAT_ADC_CTL */
        if (user_key_state == HIGH) {
          pinMode(VBAT_ADC_CTL,OUTPUT);
          digitalWrite(VBAT_ADC_CTL,LOW);
        }

#if !defined(ARDUINO_ARCH_ASR650X)
        /* Heltec CubeCell Arduino Core 1.2.0 or less */
        mV = analogRead(SOC_GPIO_PIN_BATTERY);
#else
        mV = analogReadmV(SOC_GPIO_PIN_BATTERY);
#endif

        /* restore previous state of VBAT_ADC_CTL pin */
        if (user_key_state == HIGH) {
          /*
           * CubeCell-GPS has external 10K VDD pullup resistor
           * connected to GPIO7 (USER_KEY / VBAT_ADC_CTL) pin
           */
          pinMode(VBAT_ADC_CTL, INPUT);
        }
      }

      rval = mV * SOC_ADC_VOLTAGE_DIV / 1000.0;
    }
    break;
  }

  return rval;
}

void ASR66_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long ASR66_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool ASR66_Baro_setup() {
  return true;
}

static void ASR66_UATSerial_begin(unsigned long baud)
{

}

static void ASR66_UATModule_restart()
{

}

static void ASR66_WDT_setup()
{

}

static void ASR66_WDT_fini()
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

static void ASR66_Button_setup()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  if (hw_info.model == SOFTRF_MODEL_MINI) {
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

static void ASR66_Button_loop()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  if (hw_info.model == SOFTRF_MODEL_MINI) {
    button_1.check();
  }
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */
}

static void ASR66_Button_fini()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  if (hw_info.model == SOFTRF_MODEL_MINI) {
//  detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_BUTTON));
    while (digitalRead(SOC_GPIO_PIN_BUTTON) == LOW);
    pinMode(SOC_GPIO_PIN_BUTTON, ANALOG);
  }
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */
}

#if 0
#include "RingBuffer.h"

#define UART1_TX_FIFO_SIZE (MAX_TRACKING_OBJECTS * 65 + 75 + 75 + 42 + 20)

RingBuffer<uint8_t, UART1_TX_FIFO_SIZE> UART_TX_FIFO =
                                    RingBuffer<uint8_t, UART1_TX_FIFO_SIZE>();

static void ASR66_UART_loop()
{
  while (SerialOutput.availableForWrite() > 0) {
//  while (UART_1_SpiUartGetTxBufferSize() < UART_1_TX_BUFFER_SIZE) {
    if (UART_TX_FIFO.empty()) {
      break;
    }
    SerialOutput.write(UART_TX_FIFO.read());
//    UART_1_UartPutChar(UART_TX_FIFO.read());
  }
}

static size_t ASR66_UART_write(const uint8_t *buffer, size_t size)
{
  size_t written;

  for (written=0; written < size; written++) {
    if (!UART_TX_FIFO.full()) {
      UART_TX_FIFO.write(buffer[written]);
    } else {
      break;
    }
  }
  return written;
}

IODev_ops_t ASR66_UART_ops = {
  "ASR66 UART",
  NULL,
  ASR66_UART_loop,
  NULL,
  NULL,
  NULL,
  ASR66_UART_write
};
#endif

const SoC_ops_t ASR66_ops = {
  SOC_ASR66,
  "ASR66",
  ASR66_setup,
  ASR66_post_init,
  ASR66_loop,
  ASR66_fini,
  ASR66_reset,
  ASR66_getChipId,
  ASR66_getResetInfoPtr,
  ASR66_getResetInfo,
  ASR66_getResetReason,
  ASR66_getFreeHeap,
  ASR66_random,
  ASR66_Sound_test,
  ASR66_Sound_tone,
  NULL,
  ASR66_WiFi_set_param,
  ASR66_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  ASR66_EEPROM_begin,
  ASR66_EEPROM_extension,
  ASR66_SPI_begin,
  ASR66_swSer_begin,
  ASR66_swSer_enableRx,
  NULL, /* ASR66 has no built-in Bluetooth */
  NULL, /* ASR66 has no built-in USB */
#if 0
  &ASR66_UART_ops,
#else
  NULL,
#endif
  ASR66_Display_setup,
  ASR66_Display_loop,
  ASR66_Display_fini,
  ASR66_Battery_setup,
  ASR66_Battery_param,
  ASR66_GNSS_PPS_Interrupt_handler,
  ASR66_get_PPS_TimeMarker,
  ASR66_Baro_setup,
  ASR66_UATSerial_begin,
  ASR66_UATModule_restart,
  ASR66_WDT_setup,
  ASR66_WDT_fini,
  ASR66_Button_setup,
  ASR66_Button_loop,
  ASR66_Button_fini,
  NULL
};

#endif /* __ASR6601__ || ARDUINO_ARCH_ASR6601 */
