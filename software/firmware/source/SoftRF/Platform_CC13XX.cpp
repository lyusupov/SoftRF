/*
 * Platform_CC13XX.cpp
 * Copyright (C) 2019 Linar Yusupov
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

#if defined(ENERGIA_ARCH_CC13XX)

#include "SoCHelper.h"
#include "RFHelper.h"
#include "LEDHelper.h"
#include "SoundHelper.h"

#include <easylink/EasyLink.h>

// RFM95W pin mapping
lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .rxtx = { LMIC_UNUSED_PIN, LMIC_UNUSED_PIN },
    .rst = SOC_GPIO_PIN_RST,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
};

static uint8_t ieeeAddr[8];
WS2812 strip(PIX_NUM);
uint8_t LEDs[PIX_NUM][3];

char UDPpacketBuffer[4]; // Dummy definition to satisfy build sequence

static void CC13XX_setup()
{
  EasyLink_getIeeeAddr(ieeeAddr);
}

static void CC13XX_fini()
{

}

static uint32_t CC13XX_getChipId()
{
  return (uint32_t) ieeeAddr[7]        | (ieeeAddr[6] << 8) | \
                   (ieeeAddr[5] << 16) | (ieeeAddr[4] << 24);
}

static long CC13XX_random(long howsmall, long howBig)
{
  return random(howsmall, howBig);
}

static void CC13XX_Sound_test(int var)
{
  if (settings->volume != BUZZER_OFF) {
//    swSer.enableRx(false);

#if 0
    tone(SOC_GPIO_PIN_BUZZER, 440, 500);delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 640, 500);delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 840, 500);delay(500);
    tone(SOC_GPIO_PIN_BUZZER, 1040, 500);

    delay(600);
#endif
//    swSer.enableRx(true);
  }
}

static void CC13XX_WiFi_setOutputPower(int dB)
{
  /* NONE */
}

static void CC13XX_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  /* NONE */
}

static void CC13XX_SPI_begin()
{
  SPI.begin();
}

static void CC13XX_swSer_begin(unsigned long baud)
{
  swSer.begin(baud);
}

static void CC13XX_swSer_enableRx(boolean arg)
{

}

static void CC13XX_Battery_setup()
{

}

static float CC13XX_Battery_voltage()
{
  /* TBD */
  return 0 ;
}

void CC13XX_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long CC13XX_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool CC13XX_Baro_setup() {
  return true;
}

static void CC13XX_UATSerial_begin(unsigned long baud)
{
  UATSerial.begin(baud);
}

static void CC13XX_restart()
{
  /* Nothing to do */
}

static void CC13XX_WDT_setup()
{
  /* TBD */
}

static void CC13XX_WDT_fini()
{
  /* TBD */
}

const SoC_ops_t CC13XX_ops = {
  SOC_CC13XX,
  "CC13XX",
  CC13XX_setup,
  CC13XX_fini,
  CC13XX_getChipId,
  NULL,
  NULL,
  NULL,
  CC13XX_random,
  CC13XX_Sound_test,
  NULL,
  CC13XX_WiFi_setOutputPower,
  CC13XX_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  NULL,
  CC13XX_SPI_begin,
  CC13XX_swSer_begin,
  CC13XX_swSer_enableRx,
  NULL,
  NULL,
  NULL,
  NULL,
  CC13XX_Battery_setup,
  CC13XX_Battery_voltage,
  CC13XX_GNSS_PPS_Interrupt_handler,
  CC13XX_get_PPS_TimeMarker,
  CC13XX_Baro_setup,
  CC13XX_UATSerial_begin,
  CC13XX_restart,
  CC13XX_WDT_setup,
  CC13XX_WDT_fini
};

#endif /* ENERGIA_ARCH_CC13XX */
