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

#include "SoCHelper.h"
#include "RFHelper.h"
#include "EEPROMHelper.h"
#include "GNSSHelper.h"
#include "BaroHelper.h"
#include "LEDHelper.h"

#include <U8x8lib.h>

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

#if defined(USE_OLED)
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8_i2c(U8X8_PIN_NONE);
#endif /* USE_OLED */

static U8X8_SSD1306_128X64_NONAME_HW_I2C *u8x8 = NULL;

char UDPpacketBuffer[4]; // Dummy definition to satisfy build sequence

static bool OLED_display_probe_status = false;
static bool OLED_display_frontpage = false;
static uint32_t prev_tx_packets_counter = 0;
static uint32_t prev_rx_packets_counter = 0;
extern uint32_t tx_packets_counter, rx_packets_counter;

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

static uint32_t bootCount = 0;

static int nRF52_probe_pin(uint32_t pin, uint32_t mode)
{
  return 0;
}

static void nRF52_SerialWakeup() { }

static void nRF52_setup()
{

}

static void nRF52_loop()
{

}

static void nRF52_fini()
{

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

  return 0;
}

static void nRF52_Display_loop()
{
  /* TBD */
}

static void nRF52_Display_fini(const char *msg)
{
  /* TBD */
}

static void nRF52_Battery_setup()
{
  /* TBD */
}

static float nRF52_Battery_voltage()
{
  /* TBD */
  return 0;
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

}

static void nRF52_WDT_fini()
{

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
  NULL,
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
