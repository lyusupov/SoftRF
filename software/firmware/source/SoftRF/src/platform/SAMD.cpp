/*
 * Platform_SAMD.cpp
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

#if defined(ARDUINO_ARCH_SAMD)

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

#if !defined(__SAMD21G18A__)
#error "SAMD21 is the only one supported at this time"
#endif

typedef volatile uint32_t REG32;
#define pREG32 (REG32 *)

#define DEVICE_ID_HIGH    (*(pREG32 (0x0080A044)))
#define DEVICE_ID_LOW     (*(pREG32 (0x0080A048)))

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

//SPIClass SPI1(&PERIPH_SPI1, SOC_GPIO_PIN_MISO, SOC_GPIO_PIN_SCK, SOC_GPIO_PIN_MOSI, PAD_SPI1_TX, PAD_SPI1_RX);

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

const char *SAMD_Device_Manufacturer = SOFTRF_IDENT;
const char *SAMD_Device_Model = "Academy Edition";
const uint16_t SAMD_Device_Version = SOFTRF_USB_FW_VERSION;

#if defined(USE_USB_HOST)

#include <cdcacm.h>
#include <usbhub.h>

class ACMAsyncOper : public CDCAsyncOper
{
public:
    uint8_t OnInit(ACM *pacm);
};

uint8_t ACMAsyncOper::OnInit(ACM *pacm)
{
    uint8_t rcode;
    // Set DTR = 1 RTS=1
    rcode = pacm->SetControlLineState(3);

    if (rcode)
    {
        ErrorMessage<uint8_t>(PSTR("SetControlLineState"), rcode);
        return rcode;
    }

    LINE_CODING lc;
    lc.dwDTERate        = 115200;
    lc.bCharFormat      = 0;
    lc.bParityType      = 0;
    lc.bDataBits        = 8;

    rcode = pacm->SetLineCoding(&lc);

    if (rcode)
        ErrorMessage<uint8_t>(PSTR("SetLineCoding"), rcode);

    return rcode;
}

USBHost       UsbH;
ACMAsyncOper  AsyncOper;
ACM           AcmSerial(&UsbH, &AsyncOper);

#endif /* USE_USB_HOST */

static void SAMD_setup()
{
  uint8_t reset_reason = Watchdog.resetCause();

  if      (reset_reason & PM_RCAUSE_POR)
  {
      reset_info.reason = REASON_DEFAULT_RST;
  }
  else if (reset_reason & PM_RCAUSE_BOD12)
  {
      reset_info.reason = REASON_WDT_RST;
  }
  else if (reset_reason & PM_RCAUSE_BOD33)
  {
      reset_info.reason = REASON_WDT_RST;
  }
  else if (reset_reason & PM_RCAUSE_EXT)
  {
      reset_info.reason = REASON_EXT_SYS_RST;
  }
  else if (reset_reason & PM_RCAUSE_WDT)
  {
      reset_info.reason = REASON_WDT_RST;
  }
  else if (reset_reason & PM_RCAUSE_SYST)
  {
      reset_info.reason = REASON_SOFT_RESTART;
  }

#if SOC_GPIO_RADIO_LED_TX != SOC_UNUSED_PIN
  pinMode(SOC_GPIO_RADIO_LED_TX, OUTPUT);
  digitalWrite(SOC_GPIO_RADIO_LED_TX, ! LED_STATE_ON);
#endif /* SOC_GPIO_RADIO_LED_TX */
#if SOC_GPIO_RADIO_LED_RX != SOC_UNUSED_PIN
  pinMode(SOC_GPIO_RADIO_LED_RX, OUTPUT);
  digitalWrite(SOC_GPIO_RADIO_LED_RX, ! LED_STATE_ON);
#endif /* SOC_GPIO_RADIO_LED_RX */

#if defined(USE_TINYUSB)
  USBDevice.setManufacturerDescriptor(SAMD_Device_Manufacturer);
  USBDevice.setProductDescriptor(SAMD_Device_Model);
  USBDevice.setDeviceVersion(SAMD_Device_Version);
#endif /* USE_TINYUSB */

#if defined(USE_USB_HOST)
  UsbH.Init();
#endif /* USE_USB_HOST */

  Serial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);

#if defined(USBCON)
  for (int i=0; i < 20; i++) {if (Serial) break; else delay(100);}
#endif
}

static void SAMD_post_init()
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
extern uint32_t tx_packets_counter, rx_packets_counter;
static unsigned long tx_led_time_marker = 0;
static unsigned long rx_led_time_marker = 0;

#define	LED_BLINK_TIME 100

static void SAMD_loop()
{
  if (wdt_is_active) {
    Watchdog.reset();
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
}

static void SAMD_fini(int reason)
{
  // Disable USB
  USBDevice.detach();

  Watchdog.sleep();

  NVIC_SystemReset();
}

static void SAMD_reset()
{
  NVIC_SystemReset();
}

static uint32_t SAMD_getChipId()
{
#if !defined(SOFTRF_ADDRESS)
  uint32_t id = DEVICE_ID_LOW;

  return DevID_Mapper(id);
#else
  return (SOFTRF_ADDRESS & 0xFFFFFFFFU );
#endif
}

static void* SAMD_getResetInfoPtr()
{
  return (void *) &reset_info;
}

static String SAMD_getResetInfo()
{
  switch (reset_info.reason)
  {
    default                     : return F("No reset information available");
  }
}

static String SAMD_getResetReason()
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

static uint32_t SAMD_getFreeHeap()
{
  char top;
  return &top - reinterpret_cast<char*>(_sbrk(0));
}

static long SAMD_random(long howsmall, long howBig)
{
  if(howsmall >= howBig) {
      return howsmall;
  }
  long diff = howBig - howsmall;

  return random(diff) + howsmall;
}

static void SAMD_Sound_test(int var)
{
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && settings->volume != BUZZER_OFF) {
    tone(SOC_GPIO_PIN_BUZZER, 440,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 640,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 840,  500); delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 1040, 500); delay(600);
  }
}

static void SAMD_Sound_tone(int hz, uint8_t volume)
{
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && volume != BUZZER_OFF) {
    if (hz > 0) {
      tone(SOC_GPIO_PIN_BUZZER, hz, ALARM_TONE_MS);
    } else {
      noTone(SOC_GPIO_PIN_BUZZER);
    }
  }
}

static void SAMD_WiFi_set_param(int ndx, int value)
{
  /* NONE */
}

static void SAMD_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  /* NONE */
}

static bool SAMD_EEPROM_begin(size_t size)
{
  return true;
}

static void SAMD_EEPROM_extension(int cmd)
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

#if defined(USE_USB_HOST)
    if (settings->nmea_out != NMEA_OFF) {
      settings->nmea_out = NMEA_UART;
    }
    if (settings->gdl90 != GDL90_OFF) {
      settings->gdl90 = GDL90_UART;
    }
    if (settings->d1090 != D1090_OFF) {
      settings->d1090 = D1090_UART;
    }
#endif /* USE_USB_HOST */
  }
}

static void SAMD_SPI_begin()
{
#if USE_ISP_PORT
  SPI.begin();
#else
//  SPI1.begin();
#endif
}

static void SAMD_swSer_begin(unsigned long baud)
{
  Serial_GNSS_In.begin(baud);
}

static void SAMD_swSer_enableRx(boolean arg)
{
  /* NONE */
}

static byte SAMD_Display_setup()
{
  byte rval = DISPLAY_NONE;

#if defined(USE_OLED)
  rval = OLED_setup();
#endif /* USE_OLED */

  return rval;
}

static void SAMD_Display_loop()
{
#if defined(USE_OLED)
  OLED_loop();
#endif /* USE_OLED */
}

static void SAMD_Display_fini(int reason)
{
#if defined(USE_OLED)
  OLED_fini(reason);
#endif /* USE_OLED */
}

static void SAMD_Battery_setup()
{

}

static float SAMD_Battery_param(uint8_t param)
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

void SAMD_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long SAMD_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool SAMD_Baro_setup() {
  return true;
}

static void SAMD_UATSerial_begin(unsigned long baud)
{

}

static void SAMD_UATModule_restart()
{

}

static void SAMD_WDT_setup()
{
  Watchdog.enable(8000);
  wdt_is_active = true;
}

static void SAMD_WDT_fini()
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

static void SAMD_Button_setup()
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

static void SAMD_Button_loop()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  if (hw_info.model == SOFTRF_MODEL_ACADEMY) {
    button_1.check();
  }
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */
}

static void SAMD_Button_fini()
{
#if SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN
  if (hw_info.model == SOFTRF_MODEL_ACADEMY) {
//  detachInterrupt(digitalPinToInterrupt(SOC_GPIO_PIN_BUTTON));
    while (digitalRead(SOC_GPIO_PIN_BUTTON) == LOW);
    pinMode(SOC_GPIO_PIN_BUTTON, ANALOG);
  }
#endif /* SOC_GPIO_PIN_BUTTON != SOC_UNUSED_PIN */
}

static void SAMD_USB_setup()
{
  if (USBSerial && USBSerial != Serial) {
    USBSerial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
  }
}

#include <RingBuffer.h>

#define USB_TX_FIFO_SIZE (MAX_TRACKING_OBJECTS * 65 + 75 + 75 + 42 + 20)
#define USB_RX_FIFO_SIZE (256)

RingBufferN<USB_TX_FIFO_SIZE> USB_TX_FIFO = RingBufferN<USB_TX_FIFO_SIZE>();
RingBufferN<USB_RX_FIFO_SIZE> USB_RX_FIFO = RingBufferN<USB_RX_FIFO_SIZE>();

static void SAMD_USB_loop()
{
#if defined(USE_USB_HOST)

  UsbH.Task();

  if (AcmSerial.isReady()) {
    uint8_t rcode;
    uint8_t data[EPX_SIZE];
    size_t size;

    while ((size = USB_TX_FIFO.available()) != 0) {

      if (size > sizeof(data)) {
        size = sizeof(data);
      }

      for (size_t i=0; i < size; i++) {
        data[i] = USB_TX_FIFO.read_char();
      }

      rcode = AcmSerial.SndData(size, data);
      if (rcode) {
        ErrorMessage<uint8_t>(PSTR("SndData"), rcode);
      }
    }

    uint16_t rcvd = sizeof(data);
    rcode = AcmSerial.RcvData(&rcvd, data);
    if (rcode && rcode != USB_ERRORFLOW) {
      ErrorMessage<uint8_t>(PSTR("RcvData"), rcode);
    } else {
      if( rcvd ) {
#if 1
        SerialOutput.write(data, rcvd);
#else
        size_t written;

        for (written=0; written < rcvd; written++) {
          if (!USB_RX_FIFO.isFull()) {
            USB_RX_FIFO.store_char(data[written]);
          } else {
            break;
          }
        }
#endif
      }
    }
  }

#elif !defined(USE_TINYUSB)

  uint8_t buf[EPX_SIZE];
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

static void SAMD_USB_fini()
{
  if (USBSerial && USBSerial != Serial) {
    USBSerial.end();
  }
}

static int SAMD_USB_available()
{
  int rval = 0;

#if defined(USE_TINYUSB)
  if (USBSerial) {
    rval = USBSerial.available();
  }
#else
  rval = USB_RX_FIFO.available();
#endif /* USE_TINYUSB */

  return rval;
}

static int SAMD_USB_read()
{
  int rval = -1;

#if defined(USE_TINYUSB)
  if (USBSerial) {
    rval = USBSerial.read();
  }
#else
  rval = USB_RX_FIFO.read_char();
#endif /* USE_TINYUSB */

  return rval;
}

static size_t SAMD_USB_write(const uint8_t *buffer, size_t size)
{
#if !defined(USE_TINYUSB)
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

IODev_ops_t SAMD_USBSerial_ops = {
  "SAMD USBSerial",
  SAMD_USB_setup,
  SAMD_USB_loop,
  SAMD_USB_fini,
  SAMD_USB_available,
  SAMD_USB_read,
  SAMD_USB_write
};

const SoC_ops_t SAMD_ops = {
  SOC_SAMD,
  "SAMD",
  SAMD_setup,
  SAMD_post_init,
  SAMD_loop,
  SAMD_fini,
  SAMD_reset,
  SAMD_getChipId,
  SAMD_getResetInfoPtr,
  SAMD_getResetInfo,
  SAMD_getResetReason,
  SAMD_getFreeHeap,
  SAMD_random,
  SAMD_Sound_test,
  SAMD_Sound_tone,
  NULL,
  SAMD_WiFi_set_param,
  SAMD_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  SAMD_EEPROM_begin,
  SAMD_EEPROM_extension,
  SAMD_SPI_begin,
  SAMD_swSer_begin,
  SAMD_swSer_enableRx,
  NULL, /* SAMD has no built-in Bluetooth */
  &SAMD_USBSerial_ops,
  NULL,
  SAMD_Display_setup,
  SAMD_Display_loop,
  SAMD_Display_fini,
  SAMD_Battery_setup,
  SAMD_Battery_param,
  SAMD_GNSS_PPS_Interrupt_handler,
  SAMD_get_PPS_TimeMarker,
  SAMD_Baro_setup,
  SAMD_UATSerial_begin,
  SAMD_UATModule_restart,
  SAMD_WDT_setup,
  SAMD_WDT_fini,
  SAMD_Button_setup,
  SAMD_Button_loop,
  SAMD_Button_fini,
  NULL
};

#endif /* ARDUINO_ARCH_SAMD */
