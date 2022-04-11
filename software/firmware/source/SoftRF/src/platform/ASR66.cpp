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

#include <tremo_wdg.h>
#include <tremo_flash.h>

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
static bool wdt_is_active = false;

typedef enum
{
  ASR66_LOW_POWER,
  ASR66_ACTIVE
} ASR66_states_t;

static ASR66_states_t ASR66_state = ASR66_ACTIVE;

#define UniqueIDsize 2
static uint32_t DeviceID[UniqueIDsize];

static void ASR66_setup()
{
  system_get_chip_id(DeviceID);

  NVIC_DisableIRQ(LORA_IRQn);

  pinMode(SOC_GPIO_PIN_ANT_VDD, OUTPUT);
  digitalWrite(SOC_GPIO_PIN_ANT_VDD, HIGH);
  iomux(SOC_GPIO_PIN_ANT_RXTX, 3);

  Serial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS, SOC_GPIO_PIN_CONS_RX, SOC_GPIO_PIN_CONS_TX);
}

static void ASR66_post_init()
{
  {
    Serial.println();
    Serial.println(F("SoftRF Octave Edition Power-on Self Test"));
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
  if (wdt_is_active) {
    wdg_reload();
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
  Serial_GNSS_In.end();

  digitalWrite(SOC_GPIO_PIN_ANT_VDD, LOW);
  pinMode(SOC_GPIO_PIN_ANT_VDD, INPUT_PULLDOWN);
  iomux(SOC_GPIO_PIN_ANT_RXTX, 0);
  pinMode(SOC_GPIO_PIN_ANT_RXTX, INPUT_PULLDOWN);

  Serial.end();

  switch (reason)
  {
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  case SOFTRF_SHUTDOWN_BUTTON:
  case SOFTRF_SHUTDOWN_LOWBAT:
    pinMode(SOC_GPIO_PIN_BUTTON,  INPUT);
    enableGpioWakeUp(SOC_GPIO_PIN_BUTTON, HIGH);
    break;
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */
  case SOFTRF_SHUTDOWN_NMEA:
    pinMode(SOC_GPIO_PIN_CONS_RX, INPUT);
    enableGpioWakeUp(SOC_GPIO_PIN_CONS_RX, LOW);
    break;
  default:
    break;
  }

  pwr_deepsleep_wfi(PWR_LP_MODE_STOP0);
}

static void ASR66_reset()
{
  system_reset();
}

static uint32_t ASR66_getChipId()
{
#if !defined(SOFTRF_ADDRESS)
  uint32_t id = __builtin_bswap32(DeviceID[UniqueIDsize - 1]);

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

#define _EEPROM_BASE FLASH_BASE+(FLASH_PAGE_SIZE*31) /* ASR6601CB */
static uint8_t ASR66_flash_buf[sizeof(eeprom_t)] __attribute__((aligned(8)));

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
  } else if (cmd == EEPROM_EXT_STORE) {
    for (int i=0; i < sizeof(eeprom_t); i++) {
      ASR66_flash_buf[i] = EEPROM.read(i);
    }

    if (flash_erase_page(_EEPROM_BASE) == ERRNO_OK) {
      flash_program_bytes(_EEPROM_BASE, ASR66_flash_buf, sizeof(eeprom_t));
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

#if defined(GNSS_MASTER_ID) && !defined(EXCLUDE_GNSS_UBLOX)
extern uint8_t GNSSbuf[], setBR[20];
extern uint8_t makeUBXCFG(uint8_t, uint8_t, uint8_t, const uint8_t *);
#endif /* GNSS_MASTER_ID && !EXCLUDE_GNSS_UBLOX */

static void ASR66_swSer_begin(unsigned long baud)
{
  Serial_GNSS_In.begin (baud, SERIAL_8N1, SOC_GPIO_PIN_GNSS_RX,  SOC_GPIO_PIN_UART1_TX);
  Serial_GNSS_Out.begin(baud, SERIAL_8N1, SOC_GPIO_PIN_UART3_RX, SOC_GPIO_PIN_GNSS_TX);

#if defined(GNSS_MASTER_ID)
  if (SoC->getChipId() == GNSS_MASTER_ID) {

#if !defined(EXCLUDE_GNSS_MTK) && (STD_OUT_BR == 38400)
    Serial_GNSS_Out.write("$PMTK251,38400*27\r\n");
    GNSS_FLUSH(); delay(250);
#endif /* EXCLUDE_GNSS_MTK */

#if !defined(EXCLUDE_GNSS_UBLOX)
    unsigned int baudrate = STD_OUT_BR;

    setBR[ 8] = (baudrate      ) & 0xFF;
    setBR[ 9] = (baudrate >>  8) & 0xFF;
    setBR[10] = (baudrate >> 16) & 0xFF;

    uint8_t msglen = makeUBXCFG(0x06, 0x00, sizeof(setBR), setBR);
    Serial_GNSS_Out.write(GNSSbuf, msglen);
    GNSS_FLUSH(); delay(250);
#endif /* EXCLUDE_GNSS_UBLOX */
  }

  Serial_GNSS_In.updateBaudRate (STD_OUT_BR);
  Serial_GNSS_Out.updateBaudRate(STD_OUT_BR);
#endif /* GNSS_MASTER_ID */
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

}

static float ASR66_Battery_param(uint8_t param)
{
  float rval, voltage;

  switch (param)
  {
  case BATTERY_PARAM_THRESHOLD:
    rval = hw_info.model == SOFTRF_MODEL_OCTAVE ? BATTERY_THRESHOLD_LIPO  :
                                                  BATTERY_THRESHOLD_NIMHX2;
    break;

  case BATTERY_PARAM_CUTOFF:
    rval = hw_info.model == SOFTRF_MODEL_OCTAVE ? BATTERY_CUTOFF_LIPO  :
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
#if SOC_GPIO_PIN_BATTERY != SOC_UNUSED_PIN
      mV = analogReadmV(SOC_GPIO_PIN_BATTERY);
#endif
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
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_WDG, true);
  delay(200);
  wdg_start(8 * RCC_FREQ_24M); /* 7 sec */
  wdt_is_active = true;
}

static void ASR66_WDT_fini()
{
  if (wdt_is_active) {
    wdg_stop();
    wdt_is_active = false;
    wdg_deinit();
  }
}

#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
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
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */

static void ASR66_Button_setup()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  {
    int button_pin = SOC_GPIO_PIN_BUTTON;

    pinMode(button_pin, INPUT_PULLDOWN);

    button_1.init(button_pin, LOW);

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
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */
}

static void ASR66_Button_loop()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  button_1.check();
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */
}

static void ASR66_Button_fini()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  while (digitalRead(SOC_GPIO_PIN_BUTTON) == HIGH);
  pinMode(SOC_GPIO_PIN_BUTTON, ANALOG);
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */
}

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
  NULL,
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
