/*
 * Platform_RA4M1.cpp
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

#if defined(ARDUINO_ARCH_RENESAS)

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
#include "../driver/Bluetooth.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/GDL90.h"
#include "../protocol/data/D1090.h"

#include <WDT.h>
#include <r_lpm.h>

#if !defined(ARDUINO_ARCH_RENESAS_UNO)
#error "ARDUINO UNO is the only one supported at this time"
#endif

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
#else
#include "../driver/WiFi.h"
#endif /* EXCLUDE_WIFI */

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

static uint32_t bootCount __attribute__ ((section (".noinit")));
static bool wdt_is_active = false;

const char *RA4M1_Device_Manufacturer = SOFTRF_IDENT;
const char *RA4M1_Device_Model = "Academy Edition";
const uint16_t RA4M1_Device_Version = SOFTRF_USB_FW_VERSION;

#if defined(ARDUINO_UNOR4_WIFI) && (__GNUC__ <= 7)
#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"

ArduinoLEDMatrix matrix;
#endif /* ARDUINO_UNOR4_WIFI */

#if defined(USE_SOFTSPI)
#include <SoftSPI.h>
SoftSPI RadioSPI(SOC_GPIO_PIN_MOSI,SOC_GPIO_PIN_MISO, SOC_GPIO_PIN_SCK);
#endif /* USE_SOFTSPI */

#if defined(ARDUINO_UNOR4_WIFI) && defined(NO_USB)
void __maybe_start_usb() {
#if defined(USE_RA4M1_USB)
  __USBStart();
#endif
}
#endif /* ARDUINO_UNOR4_WIFI */

static void RA4M1_setup()
{
  if (1U == R_SYSTEM->RSTSR0_b.PORF) /* Power on Reset */
  {
      reset_info.reason = REASON_DEFAULT_RST;
  }
  else if (1U == R_SYSTEM->RSTSR1_b.IWDTRF) /* Independent WDT Reset */
  {
      reset_info.reason = REASON_WDT_RST;
  }
  else if (1U == R_SYSTEM->RSTSR1_b.WDTRF) /* Watchdog Timer Reset */
  {
      reset_info.reason = REASON_WDT_RST;
  }
  else if (1U == R_SYSTEM->RSTSR0_b.LVD0RF) /* Voltage monitor Reset */
  {
      reset_info.reason = REASON_WDT_RST;
  }
  else if (1U == R_SYSTEM->RSTSR0_b.DPSRSTF) /* Deep Software Standby Reset */
  {
      reset_info.reason = REASON_DEEP_SLEEP_AWAKE;
  }
  else if (1U == R_SYSTEM->RSTSR1_b.SWRF) /* Software Reset */
  {
      reset_info.reason = REASON_SOFT_RESTART;
  }
  else if (1U == R_SYSTEM->RSTSR2_b.CWSF) /* Warm start */
  {
      reset_info.reason = REASON_EXT_SYS_RST;
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

#if defined(NO_USB)
  Serial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
#else
  SerialOutput.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);

  for (int i=0; i < 20; i++) {if (SerialUSB) break; else delay(100);}
#endif /* NO_USB */

#if defined(ARDUINO_UNOR4_WIFI) && (__GNUC__ <= 7)
  matrix.begin();
  matrix.beginDraw();

  matrix.stroke(0xFFFFFFFF);
  matrix.textScrollSpeed(50);

  const char text[] = "   " SOFTRF_IDENT;
  matrix.textFont(Font_5x7);
  matrix.beginText(0, 1, 0xFFFFFF);
  matrix.println(text);
  matrix.endText(SCROLL_LEFT);
  matrix.endDraw();
#endif /* ARDUINO_UNOR4_WIFI */
}

static void RA4M1_post_init()
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
    case NMEA_UART       :  Serial.println(F("UART"));      break;
    case NMEA_USB        :  Serial.println(F("USB CDC"));   break;
    case NMEA_UDP        :  Serial.println(F("UDP"));       break;
    case NMEA_TCP        :  Serial.println(F("TCP"));       break;
    case NMEA_BLUETOOTH  :  Serial.println(F("Bluetooth")); break;
    case NMEA_OFF        :
    default              :  Serial.println(F("NULL"));      break;
  }

  Serial.print(F("GDL90  - "));
  switch (settings->gdl90)
  {
    case GDL90_UART      :  Serial.println(F("UART"));      break;
    case GDL90_USB       :  Serial.println(F("USB CDC"));   break;
    case GDL90_UDP       :  Serial.println(F("UDP"));       break;
    case GDL90_BLUETOOTH :  Serial.println(F("Bluetooth")); break;
    case GDL90_OFF       :
    default              :  Serial.println(F("NULL"));      break;
  }

  Serial.print(F("D1090  - "));
  switch (settings->d1090)
  {
    case D1090_UART      :  Serial.println(F("UART"));      break;
    case D1090_USB       :  Serial.println(F("USB CDC"));   break;
    case D1090_BLUETOOTH :  Serial.println(F("Bluetooth")); break;
    case D1090_OFF       :
    default              :  Serial.println(F("NULL"));      break;
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

static void RA4M1_loop()
{
  if (wdt_is_active) {
    WDT.refresh();
  }

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

#if SOC_GPIO_PIN_GNSS_PPS != SOC_UNUSED_PIN && defined(NOT_AN_INTERRUPT)
  static bool prev_PPS_state = LOW;

  if (digitalPinToInterrupt(SOC_GPIO_PIN_GNSS_PPS) == NOT_AN_INTERRUPT) {
    bool PPS_state = digitalRead(SOC_GPIO_PIN_GNSS_PPS);

    if (PPS_state == HIGH && prev_PPS_state == LOW) {
      PPS_TimeMarker = millis();
    }
    prev_PPS_state = PPS_state;
  }
#endif
}

static void RA4M1_fini(int reason)
{
#if !defined(NO_USB)
  SerialUSB.end();
#endif /* NO_USB */

#if defined(ARDUINO_UNOR4_WIFI)
  pinMode(SOC_GPIO_PIN_USB_SW, INPUT);
#endif /* ARDUINO_UNOR4_WIFI */

  lpm_instance_ctrl_t p_api_ctrl;
  lpm_cfg_t p_cfg;

  p_cfg.low_power_mode       = LPM_MODE_DEEP; // LPM_MODE_SLEEP LPM_MODE_STANDBY LPM_MODE_STANDBY_SNOOZE LPM_MODE_DEEP
  p_cfg.standby_wake_sources = LPM_STANDBY_WAKE_SOURCE_IRQ0 | LPM_STANDBY_WAKE_SOURCE_RTCALM;
  p_cfg.dtc_state_in_snooze  = LPM_SNOOZE_DTC_DISABLE; // LPM_SNOOZE_DTC_ENABLE LPM_SNOOZE_DTC_DISABLE

  R_LPM_Open(&p_api_ctrl, &p_cfg);

  R_LPM_LowPowerModeEnter(&p_api_ctrl);
}

static void RA4M1_reset()
{
  NVIC_SystemReset();
}

static uint32_t RA4M1_getChipId()
{
#if !defined(SOFTRF_ADDRESS)
  const bsp_unique_id_t* t = R_BSP_UniqueIdGet();
  uint32_t id = t->unique_id_words[3];

  return DevID_Mapper(id);
#else
  return (SOFTRF_ADDRESS & 0xFFFFFFFFU );
#endif
}

static void* RA4M1_getResetInfoPtr()
{
  return (void *) &reset_info;
}

static String RA4M1_getResetInfo()
{
  switch (reset_info.reason)
  {
    default                       : return F("No reset information available");
  }
}

static String RA4M1_getResetReason()
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

static uint32_t RA4M1_getFreeHeap()
{
  char top;
  return &top - reinterpret_cast<char*>(_sbrk(0));
}

static long RA4M1_random(long howsmall, long howBig)
{
  if(howsmall >= howBig) {
      return howsmall;
  }
  long diff = howBig - howsmall;

  return random(diff) + howsmall;
}

static void RA4M1_Sound_test(int var)
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

static void RA4M1_Sound_tone(int hz, uint8_t volume)
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

static void RA4M1_WiFi_set_param(int ndx, int value)
{
  /* NONE */
}

#if !defined(EXCLUDE_WIFI)
static IPAddress broadcastIp_cache = IPAddress(0, 0, 0, 0);

static IPAddress RA4M1_WiFi_get_broadcast()
{
  if (broadcastIp_cache == IPAddress(0, 0, 0, 0)) {
    int status = WiFi.status();

    switch (status)
    {
    case WL_CONNECTED:
      broadcastIp_cache = IPAddress((uint32_t) WiFi.localIP() |
                                  ~((uint32_t) WiFi.subnetMask()));
      break;
    case WL_AP_CONNECTED:
      broadcastIp_cache = IPAddress((uint32_t) WiFi.softAPIP() |
                                  ~((uint32_t) WiFi.subnetMask()));
      break;
    default:
      break;
    }
  }

  return broadcastIp_cache;
}

static IPAddress AP_IP_cache = IPAddress(0, 0, 0, 0);

static IPAddress RA4M1_WiFi_get_AP_IP()
{
  if (AP_IP_cache == IPAddress(0, 0, 0, 0)) {
    AP_IP_cache = WiFi.softAPIP();
  }

  return AP_IP_cache;
}
#endif /* EXCLUDE_WIFI */

static void RA4M1_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
#if !defined(EXCLUDE_WIFI)
  IPAddress ClientIP;
  int status = WiFi.status();

  switch (status)
  {
  case WL_CONNECTED:
    ClientIP = RA4M1_WiFi_get_broadcast();
    if (Uni_Udp && ClientIP != IPAddress(0, 0, 0, 0)) {
      Uni_Udp->beginPacket(ClientIP, port);
      Uni_Udp->write(buf, size);
      Uni_Udp->endPacket();
    }
    break;
  case WL_AP_CONNECTED:
    if (SoC->WiFi_clients_count() > 0) {
      IPAddress APIP = RA4M1_WiFi_get_AP_IP();

      if (APIP != IPAddress(0, 0, 0, 0)) {
        for (int i=0; i < SoC->WiFi_clients_count(); i++) { /* TODO */
          ClientIP = IPAddress(APIP[0], APIP[1], APIP[2], APIP[3] + 1 + i); /* TODO */
          if (Uni_Udp) {
            Uni_Udp->beginPacket(ClientIP, port);
            Uni_Udp->write(buf, size);
            Uni_Udp->endPacket();
          }
        }
      }
    }
    break;
  default:
    break;
  }
#endif /* EXCLUDE_WIFI */
}

static void RA4M1_WiFiUDP_stopAll()
{
#if !defined(EXCLUDE_WIFI)
  /* TODO */
#endif /* EXCLUDE_WIFI */
}

static bool RA4M1_WiFi_hostname(String aHostname)
{
  bool rval = false;
#if !defined(EXCLUDE_WIFI)
  WiFi.setHostname(aHostname.c_str());
  rval = true;
#endif /* EXCLUDE_WIFI */
  return rval;
}

static int RA4M1_WiFi_clients_count()
{
#if !defined(EXCLUDE_WIFI)
  return  1; /* TODO */
#else
  return -1;
#endif /* EXCLUDE_WIFI */
}

static bool RA4M1_EEPROM_begin(size_t size)
{
  return true;
}

static void RA4M1_EEPROM_extension(int cmd)
{
#if defined(ARDUINO_UNOR4_WIFI) && defined(NO_USB)
  if (cmd == EEPROM_EXT_DEFAULTS) {
    settings->nmea_out = NMEA_UART;
#if defined(EXCLUDE_WIFI) && !defined(EXCLUDE_BLUETOOTH)
    settings->bluetooth = BLUETOOTH_LE_HM10_SERIAL;
#endif /* EXCLUDE_BLUETOOTH */
  }
#endif /* ARDUINO_UNOR4_WIFI */

  if (cmd == EEPROM_EXT_LOAD) {
    if (settings->mode != SOFTRF_MODE_NORMAL
#if !defined(EXCLUDE_TEST_MODE)
        &&
        settings->mode != SOFTRF_MODE_TXRX_TEST
#endif /* EXCLUDE_TEST_MODE */
        ) {
      settings->mode = SOFTRF_MODE_NORMAL;
    }

#if defined(ARDUINO_UNOR4_MINIMA)
    if (settings->nmea_out == NMEA_BLUETOOTH ||
        settings->nmea_out == NMEA_UDP       ||
        settings->nmea_out == NMEA_TCP ) {
      settings->nmea_out = NMEA_USB;
    }
    if (settings->gdl90 == GDL90_BLUETOOTH  ||
        settings->gdl90 == GDL90_UDP) {
      settings->gdl90 = GDL90_USB;
    }
    if (settings->d1090 == D1090_BLUETOOTH  ||
        settings->d1090 == D1090_UDP) {
      settings->d1090 = D1090_USB;
    }
#elif defined(ARDUINO_UNOR4_WIFI) && defined(NO_USB) && !defined(USE_RA4M1_USB)
    if (settings->nmea_out == NMEA_USB) {
      settings->nmea_out = NMEA_UART;
    }
    if (settings->gdl90 == GDL90_USB) {
      settings->gdl90 = GDL90_UART;
    }
    if (settings->d1090 == D1090_USB) {
      settings->d1090 = D1090_UART;
    }
#endif

    /* AUTO and UK RF bands are deprecated since Release v1.3 */
    if (settings->band == RF_BAND_AUTO || settings->band == RF_BAND_UK) {
      settings->band = RF_BAND_EU;
    }
  }
}

static void RA4M1_SPI_begin()
{
#if !defined(EXCLUDE_NRF905) || defined(USE_OGN_RF_DRIVER)
  SPI.begin();
#endif
}

static void RA4M1_swSer_begin(unsigned long baud)
{
  Serial_GNSS_In.begin(baud);
}

static void RA4M1_swSer_enableRx(boolean arg)
{
  /* NONE */
}

static byte RA4M1_Display_setup()
{
  byte rval = DISPLAY_NONE;

#if defined(USE_OLED)
  rval = OLED_setup();
#endif /* USE_OLED */

  return rval;
}

static void RA4M1_Display_loop()
{
#if defined(USE_OLED)
  OLED_loop();
#endif /* USE_OLED */
}

static void RA4M1_Display_fini(int reason)
{
#if defined(USE_OLED)
  OLED_fini(reason);
#endif /* USE_OLED */
}

static void RA4M1_Battery_setup()
{

}

static float RA4M1_Battery_param(uint8_t param)
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
      int adc_reading = analogRead(SOC_GPIO_PIN_BATTERY);
      mV = map(adc_reading, 0, 1023, 0, analogReference() * 1000); /* TODO */
#endif
      rval = mV / 1000.0;
    }
    break;
  }

  return rval;
}

void RA4M1_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long RA4M1_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool RA4M1_Baro_setup() {
  return true;
}

static void RA4M1_UATSerial_begin(unsigned long baud)
{

}

static void RA4M1_UATModule_restart()
{

}

static void RA4M1_WDT_setup()
{
  WDT.begin(5000);
  wdt_is_active = true;
}

static void RA4M1_WDT_fini()
{
  if (wdt_is_active) {
    WDT.refresh(); /* TODO */
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

static void RA4M1_Button_setup()
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

static void RA4M1_Button_loop()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  if (hw_info.model == SOFTRF_MODEL_ACADEMY) {
    button_1.check();
  }
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */
}

static void RA4M1_Button_fini()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  if (hw_info.model == SOFTRF_MODEL_ACADEMY) {
//  detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_BUTTON));
    while (digitalRead(SOC_GPIO_PIN_BUTTON) == LOW);
  }
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */
}

static void RA4M1_USB_setup() {
#if defined(NO_USB)
  USBSerial.begin(SERIAL_OUT_BR);
#endif /* NO_USB */
}

static void RA4M1_USB_loop()  {

}

static void RA4M1_USB_fini()  {
#if defined(NO_USB)
  USBSerial.end();
#endif /* NO_USB */
}

static int RA4M1_USB_available()
{
  int rval = 0;

  if (USBSerial) {
    rval = USBSerial.available();
  }

  return rval;
}

static int RA4M1_USB_read()
{
  int rval = -1;

  if (USBSerial) {
    rval = USBSerial.read();
  }

  return rval;
}

static size_t RA4M1_USB_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  if (USBSerial && (size < USBSerial.availableForWrite())) {
    rval = USBSerial.write(buffer, size);
  }

  return rval;
}

IODev_ops_t RA4M1_USBSerial_ops = {
  "RA4M1 USBSerial",
  RA4M1_USB_setup,
  RA4M1_USB_loop,
  RA4M1_USB_fini,
  RA4M1_USB_available,
  RA4M1_USB_read,
  RA4M1_USB_write
};

#if defined(ARDUINO_UNOR4_WIFI)

static void UNOR4W_Serial_setup()     { }
static void UNOR4W_Serial_loop()      { }
static void UNOR4W_Serial_fini()      { }
static int  UNOR4W_Serial_available() { return SerialOutput.available(); }
static int  UNOR4W_Serial_read()      { return SerialOutput.read(); }

static size_t UNOR4W_Serial_write(const uint8_t *buffer, size_t size)
{
/*
        if (nl) {
          if (buf[size-1] == '\r') {
            SerialOutput.write(buf, size-1);
            SerialOutput.write((byte *) "\r\n", 2);
          } else {
            SerialOutput.write(buf, size);
            SerialOutput.write('\n');
          }
        } else {
          SerialOutput.write(buf, size);
        }
*/

  return SerialOutput.write(buffer, size);
}

IODev_ops_t UNOR4W_Serial_ops = {
  "UNOR4W Serial",
  UNOR4W_Serial_setup,
  UNOR4W_Serial_loop,
  UNOR4W_Serial_fini,
  UNOR4W_Serial_available,
  UNOR4W_Serial_read,
  UNOR4W_Serial_write
};
#endif /* ARDUINO_UNOR4_WIFI */

const SoC_ops_t RA4M1_ops = {
  SOC_RA4M1,
  "RA4M1",
  RA4M1_setup,
  RA4M1_post_init,
  RA4M1_loop,
  RA4M1_fini,
  RA4M1_reset,
  RA4M1_getChipId,
  RA4M1_getResetInfoPtr,
  RA4M1_getResetInfo,
  RA4M1_getResetReason,
  RA4M1_getFreeHeap,
  RA4M1_random,
  RA4M1_Sound_test,
  RA4M1_Sound_tone,
  NULL,
  RA4M1_WiFi_set_param,
  RA4M1_WiFi_transmit_UDP,
  RA4M1_WiFiUDP_stopAll,
  RA4M1_WiFi_hostname,
  RA4M1_WiFi_clients_count,
  RA4M1_EEPROM_begin,
  RA4M1_EEPROM_extension,
  RA4M1_SPI_begin,
  RA4M1_swSer_begin,
  RA4M1_swSer_enableRx,
#if !defined(EXCLUDE_BLUETOOTH)
  &ArdBLE_Bluetooth_ops,
#else
  NULL,
#endif /* EXCLUDE_BLUETOOTH */
  &RA4M1_USBSerial_ops,
#if !defined(ARDUINO_UNOR4_WIFI)
  NULL,
#else
  &UNOR4W_Serial_ops,
#endif /* ARDUINO_UNOR4_WIFI */
  RA4M1_Display_setup,
  RA4M1_Display_loop,
  RA4M1_Display_fini,
  RA4M1_Battery_setup,
  RA4M1_Battery_param,
  RA4M1_GNSS_PPS_Interrupt_handler,
  RA4M1_get_PPS_TimeMarker,
  RA4M1_Baro_setup,
  RA4M1_UATSerial_begin,
  RA4M1_UATModule_restart,
  RA4M1_WDT_setup,
  RA4M1_WDT_fini,
  RA4M1_Button_setup,
  RA4M1_Button_loop,
  RA4M1_Button_fini,
  NULL
};

#endif /* ARDUINO_ARCH_RENESAS */
