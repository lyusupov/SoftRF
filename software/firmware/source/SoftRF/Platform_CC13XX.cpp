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

// RFM95W pin mapping
lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .rxtx = { LMIC_UNUSED_PIN, LMIC_UNUSED_PIN },
    .rst = SOC_GPIO_PIN_RST,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
};

static void CC13XX_setup()
{
  /* TBD */
}

static uint32_t CC13XX_getChipId()
{
  /* TBD */
  return 0;
}

static long CC13XX_random(long howsmall, long howBig)
{
  return random(howsmall, howBig);
}

static void CC13XX_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  /* NONE */
}

static void CC13XX_SPI_begin()
{
  /* TBD */
}

void CC13XX_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long CC13XX_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static void CC13XX_restart()
{
  /* Nothing to do */
}

SoC_ops_t CC13XX_ops = {
  SOC_CC13XX,
  "CC13XX",
  CC13XX_setup,
  CC13XX_getChipId,
  NULL,
  NULL,
  NULL,
  CC13XX_random,
  NULL,
  NULL,
  NULL,
  NULL,
  CC13XX_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  CC13XX_SPI_begin,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  CC13XX_get_PPS_TimeMarker,
  NULL,
  CC13XX_restart
};

#endif /* ENERGIA_ARCH_CC13XX */
