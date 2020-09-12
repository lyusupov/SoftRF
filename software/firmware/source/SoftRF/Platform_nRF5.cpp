/*
 * Platform_nRF5.cpp
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

#if defined(ARDUINO_ARCH_NRF5)

#include <SPI.h>
#include <Wire.h>

#include "SoCHelper.h"
#include "RFHelper.h"
#include "EEPROMHelper.h"
#include "GNSSHelper.h"
#include "BaroHelper.h"
#include "LEDHelper.h"

#include <U8x8lib.h>

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

static int nRF5_probe_pin(uint32_t pin, uint32_t mode)
{
  return 0;
}

static void nRF5_SerialWakeup() { }

static void nRF5_setup()
{

}

static void nRF5_loop()
{

}

static void nRF5_fini()
{

}

static void nRF5_reset()
{

}

static uint32_t nRF5_getChipId()
{
  return 0;
}

static void* nRF5_getResetInfoPtr()
{
  return 0;
}

static String nRF5_getResetReason()
{
  return F("DEFAULT");
}

static uint32_t nRF5_getFreeHeap()
{
  return 0 ;
}

static long nRF5_random(long howsmall, long howBig)
{
  return 0;
}

static void nRF5_Sound_test(int var)
{

}

static void nRF5_WiFi_setOutputPower(int dB)
{
  /* NONE */
}

static void nRF5_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  /* NONE */
}

static bool nRF5_EEPROM_begin(size_t size)
{
  return true;
}

static void nRF5_SPI_begin()
{

}

static void nRF5_swSer_begin(unsigned long baud)
{

}

static void nRF5_swSer_enableRx(boolean arg)
{
  /* NONE */
}

static byte nRF5_Display_setup()
{

  return 0;
}

static void nRF5_Display_loop()
{

}

static void nRF5_Display_fini(const char *msg)
{

}

static void nRF5_Battery_setup()
{

}

static float nRF5_Battery_voltage()
{

  return 0;
}

void nRF5_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long nRF5_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool nRF5_Baro_setup() {
  return true;
}

static void nRF5_UATSerial_begin(unsigned long baud)
{

}

static void nRF5_restart()
{

}

static void nRF5_WDT_setup()
{

}

static void nRF5_WDT_fini()
{

}

static void nRF5_Button_setup()
{
  /* TODO */
}

static void nRF5_Button_loop()
{
  /* TODO */
}

static void nRF5_Button_fini()
{
  /* TODO */
}

const SoC_ops_t nRF5_ops = {
  SOC_NRF5,
  "nRF5",
  nRF5_setup,
  nRF5_loop,
  nRF5_fini,
  nRF5_reset,
  nRF5_getChipId,
  nRF5_getResetInfoPtr,
  NULL,
  nRF5_getResetReason,
  nRF5_getFreeHeap,
  nRF5_random,
  nRF5_Sound_test,
  NULL,
  nRF5_WiFi_setOutputPower,
  nRF5_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  nRF5_EEPROM_begin,
  nRF5_SPI_begin,
  nRF5_swSer_begin,
  nRF5_swSer_enableRx,
  NULL,
  nRF5_Display_setup,
  nRF5_Display_loop,
  nRF5_Display_fini,
  nRF5_Battery_setup,
  nRF5_Battery_voltage,
  nRF5_GNSS_PPS_Interrupt_handler,
  nRF5_get_PPS_TimeMarker,
  nRF5_Baro_setup,
  nRF5_UATSerial_begin,
  nRF5_restart,
  nRF5_WDT_setup,
  nRF5_WDT_fini,
  nRF5_Button_setup,
  nRF5_Button_loop,
  nRF5_Button_fini
};

#endif /* ARDUINO_ARCH_NRF5 */
