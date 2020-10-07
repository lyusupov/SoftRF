/*
 * Platform_nRF52.cpp
 * Copyright (C) 2020 Linar Yusupov
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

#if defined(ARDUINO_ARCH_NRF52)

#include <SPI.h>
#include <Wire.h>
#include <pcf8563.h>
#include <SoftSPI.h>
#include <SerialFlash.h>
#include <Adafruit_SleepyDog.h>
#include "nrf_wdt.h"

#include "SoCHelper.h"
#include "RFHelper.h"
#include "EEPROMHelper.h"
#include "GNSSHelper.h"
#include "BaroHelper.h"
#include "LEDHelper.h"
#include "BluetoothHelper.h"

#if defined(USE_EPAPER)
#include "EPDHelper.h"
#endif /* USE_EPAPER */

typedef volatile uint32_t REG32;
#define pREG32 (REG32 *)

#define DEVICE_ID_HIGH    (*(pREG32 (0x10000060)))
#define DEVICE_ID_LOW     (*(pREG32 (0x10000064)))

// RFM95W pin mapping
lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .txe = LMIC_UNUSED_PIN,
    .rxe = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
    .busy = LMIC_UNUSED_PIN,
    .tcxo = LMIC_UNUSED_PIN,
};

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIX_NUM, SOC_GPIO_PIN_LED,
                              NEO_GRB + NEO_KHZ800);

char UDPpacketBuffer[4]; // Dummy definition to satisfy build sequence

static uint32_t prev_tx_packets_counter = 0;
static uint32_t prev_rx_packets_counter = 0;
extern uint32_t tx_packets_counter, rx_packets_counter;

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

static uint32_t bootCount = 0;

PCF8563_Class *rtc = nullptr;
I2CBus        *i2c = nullptr;

static bool nRF52_has_rtc      = false;
static bool nRF52_has_spiflash = false ;

SoftSPI SPI2(SOC_GPIO_PIN_SFL_MOSI, SOC_GPIO_PIN_SFL_MISO, SOC_GPIO_PIN_SFL_SCK);

ui_settings_t ui_settings = {
    .units        = UNITS_METRIC,
    .zoom         = ZOOM_MEDIUM,
    .protocol     = PROTOCOL_NMEA,
    .orientation  = DIRECTION_TRACK_UP,
    .adb          = DB_NONE,
    .idpref       = ID_REG,
    .vmode        = VIEW_MODE_STATUS,
    .voice        = VOICE_OFF,
    .aghost       = ANTI_GHOSTING_OFF,
    .filter       = TRAFFIC_FILTER_OFF,
    .team         = 0
};

ui_settings_t *ui;

#if !defined(PIN_SERIAL2_RX) && !defined(PIN_SERIAL2_TX)
Uart Serial2( NRF_UARTE1, UARTE1_IRQn, SOC_GPIO_PIN_SWSER_RX, SOC_GPIO_PIN_SWSER_TX );

extern "C"
{
  void UARTE1_IRQHandler()
  {
    Serial2.IrqHandler();
  }
}
#endif

static int nRF52_probe_pin(uint32_t pin, uint32_t mode)
{
  return 0;
}

static void nRF52_SerialWakeup() { }

static void nRF52_setup()
{
  ui = &ui_settings;

//  uint32_t u32Reset_reason = NRF_POWER->RESETREAS;
//  reset_info.reason = u32Reset_reason;

  /* supposed to be control of power supply */
  pinMode(SOC_GPIO_PIN_PWR,  OUTPUT);
  digitalWrite(SOC_GPIO_PIN_PWR, HIGH);

  pinMode(SOC_GPIO_LED_GREEN, OUTPUT);
  pinMode(SOC_GPIO_LED_RED,   OUTPUT);
  pinMode(SOC_GPIO_LED_BLUE,  OUTPUT);

  ledOn(SOC_GPIO_LED_GREEN);
  ledOff(SOC_GPIO_LED_RED);
  ledOff(SOC_GPIO_LED_BLUE);

  nRF52_has_spiflash = SerialFlash.begin(SPI2, SOC_GPIO_PIN_SFL_SS);

  if (nRF52_has_spiflash) {
    unsigned char buf[8];

    SerialFlash.readID(buf);

    uint32_t flash_id = (buf[0] << 16) | (buf[1] << 8) | buf[2];

    SerialFlash.sleep();
  }

  Wire.setPins(SOC_GPIO_PIN_SDA, SOC_GPIO_PIN_SCL);

  Wire.begin();
  Wire.beginTransmission(PCF8563_SLAVE_ADDRESS);
  nRF52_has_rtc = (Wire.endTransmission() == 0);
  Wire.end();

  i2c = new I2CBus(Wire);

  if (nRF52_has_rtc && (i2c != nullptr)) {
    rtc = new PCF8563_Class(*i2c);
  }
}

static void nRF52_post_init()
{
  Serial.println();
  Serial.println(F("LilyGO T-xx nRF52840 board Power-on Self Test"));
  Serial.println();
  Serial.flush();

  Serial.println(F("Built-in components:"));

  Serial.print(F("RADIO   : ")); Serial.println(hw_info.rf      != RF_IC_NONE       ? F("PASS") : F("FAIL"));
  Serial.print(F("GNSS    : ")); Serial.println(hw_info.gnss    != GNSS_MODULE_NONE ? F("PASS") : F("FAIL"));
  Serial.print(F("DISPLAY : ")); Serial.println(hw_info.display != DISPLAY_NONE     ? F("PASS") : F("FAIL"));
  Serial.print(F("RTC     : ")); Serial.println(nRF52_has_rtc                       ? F("PASS") : F("FAIL"));
  Serial.print(F("FLASH   : ")); Serial.println(nRF52_has_spiflash                  ? F("PASS") : F("FAIL"));

  Serial.println();
  Serial.println(F("External components:"));
  Serial.print(F("BMx280  : ")); Serial.println(hw_info.baro    != BARO_MODULE_NONE ? F("PASS") : F("FAIL"));

  Serial.println();
  Serial.println(F("Power-on Self Test is completed."));
  Serial.println();
  Serial.flush();

#if defined(USE_EPAPER)
  EPD_info1(nRF52_has_rtc, nRF52_has_spiflash);
#endif /* USE_EPAPER */
}

static void nRF52_loop()
{
  // Reload the watchdog
  if (nrf_wdt_started(NRF_WDT)) {
    Watchdog.reset();
  }
}

static void nRF52_fini()
{
  ledOff(SOC_GPIO_LED_GREEN);
  ledOff(SOC_GPIO_LED_RED);
  ledOff(SOC_GPIO_LED_BLUE);

  pinMode(SOC_GPIO_LED_GREEN, INPUT);
  pinMode(SOC_GPIO_LED_RED,   INPUT);
  pinMode(SOC_GPIO_LED_BLUE,  INPUT);
}

static void nRF52_reset()
{
  NVIC_SystemReset();
}

static uint32_t nRF52_getChipId()
{
#if !defined(SOFTRF_ADDRESS)
  uint32_t id = DEVICE_ID_LOW;

  /* remap address to avoid overlapping with congested FLARM range */
  if (((id & 0x00FFFFFF) >= 0xDD0000) && ((id & 0x00FFFFFF) <= 0xDFFFFF)) {
    id += 0x100000;
  }

  return id;
#else
  return (SOFTRF_ADDRESS & 0xFFFFFFFFU );
#endif
}

static void* nRF52_getResetInfoPtr()
{
  return (void *) &reset_info;
}

static String nRF52_getResetInfo()
{
  switch (reset_info.reason)
  {
    default                     : return F("No reset information available");
  }
}

static String nRF52_getResetReason()
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

static uint32_t nRF52_getFreeHeap()
{
  return dbgHeapTotal() - dbgHeapUsed();
}

static long nRF52_random(long howsmall, long howBig)
{
  return random(howsmall, howBig);
}

static void nRF52_Sound_test(int var)
{
  /* NONE */
}

static void nRF52_WiFi_setOutputPower(int dB)
{
  /* NONE */
}

static void nRF52_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  /* NONE */
}

static bool nRF52_EEPROM_begin(size_t size)
{
  if (size > EEPROM.length()) {
    return false;
  }

  EEPROM.begin();

  return true;
}

static void nRF52_SPI_begin()
{
  SPI.begin();
}

static void nRF52_swSer_begin(unsigned long baud)
{
  swSer.begin(baud);

  /* 'Cold' restart */
//  swSer.write("$PGKC030,3,1*2E\r\n");
//  swSer.flush(); delay(250);

  /* give GOKE GNSS few ms to warm up */
  delay(500);

  /* Firmware version request */
  swSer.write("$PGKC462*2F\r\n");
  swSer.flush(); delay(250);

  /* GPS + GLONASS */
  swSer.write("$PGKC115,1,1,0,0*2A\r\n");
  swSer.flush(); delay(250);

  /* RMC + GGA + GSA */
  swSer.write("$PGKC242,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*36\r\n");
  swSer.flush(); delay(250);

#if SOC_GPIO_PIN_GNSS_PPS != SOC_UNUSED_PIN
  /* Enable 3D fix 1PPS output */
//  swSer.write("$PGKC161,2,100,1000*07\r\n");
//  swSer.flush(); delay(250);
#endif
}

static void nRF52_swSer_enableRx(boolean arg)
{
  /* NONE */
}

static byte nRF52_Display_setup()
{
  byte rval = DISPLAY_NONE;

#if defined(USE_EPAPER)
  if (EPD_setup(true)) {
    rval = DISPLAY_EPD_1_54;
  }

  /* EPD back light off */
  pinMode(SOC_GPIO_PIN_EPD_BLGT, OUTPUT);
  digitalWrite(SOC_GPIO_PIN_EPD_BLGT, LOW);
#endif /* USE_EPAPER */

  return rval;
}

static void nRF52_Display_loop()
{
#if defined(USE_EPAPER)
  EPD_loop();
#endif /* USE_EPAPER */
}

static void nRF52_Display_fini(const char *msg)
{
#if defined(USE_EPAPER)
  /* EPD back light */
  pinMode(SOC_GPIO_PIN_EPD_BLGT, INPUT);

  EPD_fini(msg);
#endif /* USE_EPAPER */
}

static void nRF52_Battery_setup()
{

}

static float nRF52_Battery_voltage()
{
  float raw;

  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(1);

  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(SOC_GPIO_PIN_BATTERY);

  // Set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);

  // Convert the raw value to compensated mv, taking the resistor-
  // divider into account (providing the actual LIPO voltage)
  // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
  return raw * REAL_VBAT_MV_PER_LSB * 0.001;
}

void nRF52_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long nRF52_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool nRF52_Baro_setup() {
  return true;
}

static void nRF52_UATSerial_begin(unsigned long baud)
{

}

static void nRF52_UATModule_restart()
{

}

static void nRF52_WDT_setup()
{
  Watchdog.enable(8000);
}

static void nRF52_WDT_fini()
{
  // cannot disable nRF's WDT
  if (nrf_wdt_started(NRF_WDT)) {
    Watchdog.reset();
  }
}

static void nRF52_Button_setup()
{
  /* TODO */
}

static void nRF52_Button_loop()
{
  /* TODO */
}

static void nRF52_Button_fini()
{
  /* TODO */
}

const SoC_ops_t nRF52_ops = {
  SOC_NRF52,
  "nRF52",
  nRF52_setup,
  nRF52_post_init,
  nRF52_loop,
  nRF52_fini,
  nRF52_reset,
  nRF52_getChipId,
  nRF52_getResetInfoPtr,
  nRF52_getResetInfo,
  nRF52_getResetReason,
  nRF52_getFreeHeap,
  nRF52_random,
  nRF52_Sound_test,
  NULL,
  nRF52_WiFi_setOutputPower,
  nRF52_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  nRF52_EEPROM_begin,
  nRF52_SPI_begin,
  nRF52_swSer_begin,
  nRF52_swSer_enableRx,
  NULL, // &nRF52_Bluetooth_ops,
  NULL,
  nRF52_Display_setup,
  nRF52_Display_loop,
  nRF52_Display_fini,
  nRF52_Battery_setup,
  nRF52_Battery_voltage,
  nRF52_GNSS_PPS_Interrupt_handler,
  nRF52_get_PPS_TimeMarker,
  nRF52_Baro_setup,
  nRF52_UATSerial_begin,
  nRF52_UATModule_restart,
  nRF52_WDT_setup,
  nRF52_WDT_fini,
  nRF52_Button_setup,
  nRF52_Button_loop,
  nRF52_Button_fini
};

#endif /* ARDUINO_ARCH_NRF52 */
