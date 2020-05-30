/*
 * Platform_PSoC4.cpp
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

#if defined(__ASR6501__)

#include <SPI.h>
#include <Wire.h>

#include "SoCHelper.h"
#include "RFHelper.h"
#include "EEPROMHelper.h"
#include "GNSSHelper.h"
#include "LEDHelper.h"

// SX1262 pin mapping
lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .txe = LMIC_UNUSED_PIN,
    .rxe = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
    .busy = LMIC_UNUSED_PIN,
    .tcxo = LMIC_UNUSED_PIN,
};

char UDPpacketBuffer[4]; // Dummy definition to satisfy build sequence

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

static uint32_t bootCount = 0;

static int PSoC4_probe_pin(uint32_t pin, uint32_t mode)
{
  return 0;
}

static void PSoC4_SerialWakeup() { }

static void PSoC4_setup()
{
  boardInitMcu();
}

static void PSoC4_loop()
{

}

static void PSoC4_fini()
{
//  BoardDeInitMcu();
}

static void PSoC4_reset()
{
//  BoardResetMcu();
}

static uint32_t PSoC4_getChipId()
{
#if !defined(SOFTRF_ADDRESS)
  uint32_t id = getID();

  /* remap address to avoid overlapping with congested FLARM range */
  if (((id & 0x00FFFFFF) >= 0xDD0000) && ((id & 0x00FFFFFF) <= 0xDFFFFF)) {
    id += 0x100000;
  }

  return id;
#else
  return (SOFTRF_ADDRESS & 0xFFFFFFFFU );
#endif
}

static void* PSoC4_getResetInfoPtr()
{
  return 0;
}

static String PSoC4_getResetInfo()
{
  switch (reset_info.reason)
  {
    default                     : return F("No reset information available");
  }
}

static String PSoC4_getResetReason()
{
  return F("DEFAULT");
}

static uint32_t PSoC4_getFreeHeap()
{
  return 0; /* TBD */
}

static long PSoC4_random(long howsmall, long howBig)
{
//  return howsmall + BoardGetRandomSeed() % (howBig - howsmall);
  return 0;
}

static void PSoC4_Sound_test(int var)
{

}

static void PSoC4_WiFi_setOutputPower(int dB)
{
  /* NONE */
}

static void PSoC4_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  /* NONE */
}

static bool PSoC4_EEPROM_begin(size_t size)
{
  EEPROM.begin(size);
  return true;
}

static void PSoC4_SPI_begin()
{
  SPI.begin();
}

static void PSoC4_swSer_begin(unsigned long baud)
{
  swSer.begin(baud);
}

static void PSoC4_swSer_enableRx(boolean arg)
{
  /* NONE */
}

static byte PSoC4_Display_setup()
{
  return DISPLAY_NONE;
}

static void PSoC4_Display_loop()
{

}

static void PSoC4_Display_fini(const char *msg)
{

}

static void PSoC4_Battery_setup()
{

}

static float PSoC4_Battery_voltage()
{

  return 0;
}

void PSoC4_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long PSoC4_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static bool PSoC4_Baro_setup() {
  return true;
}

static void PSoC4_UATSerial_begin(unsigned long baud)
{

}

static void PSoC4_UATModule_restart()
{

}

static void PSoC4_WDT_setup()
{

}

static void PSoC4_WDT_fini()
{

}

static void PSoC4_Button_setup()
{
  /* TODO */
}

static void PSoC4_Button_loop()
{
  /* TODO */
}

static void PSoC4_Button_fini()
{
  /* TODO */
}

const SoC_ops_t PSoC4_ops = {
  SOC_PSOC4,
  "PSoC4",
  PSoC4_setup,
  PSoC4_loop,
  PSoC4_fini,
  PSoC4_reset,
  PSoC4_getChipId,
  PSoC4_getResetInfoPtr,
  PSoC4_getResetInfo,
  PSoC4_getResetReason,
  PSoC4_getFreeHeap,
  PSoC4_random,
  PSoC4_Sound_test,
  NULL,
  PSoC4_WiFi_setOutputPower,
  PSoC4_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  PSoC4_EEPROM_begin,
  PSoC4_SPI_begin,
  PSoC4_swSer_begin,
  PSoC4_swSer_enableRx,
  NULL,
  PSoC4_Display_setup,
  PSoC4_Display_loop,
  PSoC4_Display_fini,
  PSoC4_Battery_setup,
  PSoC4_Battery_voltage,
  PSoC4_GNSS_PPS_Interrupt_handler,
  PSoC4_get_PPS_TimeMarker,
  PSoC4_Baro_setup,
  PSoC4_UATSerial_begin,
  PSoC4_UATModule_restart,
  PSoC4_WDT_setup,
  PSoC4_WDT_fini,
  PSoC4_Button_setup,
  PSoC4_Button_loop,
  PSoC4_Button_fini
};

#endif /* __ASR6501__ */
