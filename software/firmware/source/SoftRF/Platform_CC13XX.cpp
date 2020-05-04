/*
 * Platform_CC13XX.cpp
 * Copyright (C) 2019-2020 Linar Yusupov
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

#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2)

#include <Wire.h>

#if defined(ENERGIA_ARCH_CC13XX)
#include <ti/devices/cc13x0/driverlib/sys_ctrl.h>
#endif /* ENERGIA_ARCH_CC13XX */
#if defined(ENERGIA_ARCH_CC13X2)
#include <ti/devices/cc13x2_cc26x2/driverlib/sys_ctrl.h>
#endif /* ENERGIA_ARCH_CC13X2 */

#include "SoCHelper.h"
#include "RFHelper.h"
#include "LEDHelper.h"
#include "SoundHelper.h"
#include "BaroHelper.h"

#include "EasyLink.h"

// RFM95W pin mapping
lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .txe = LMIC_UNUSED_PIN,
    .rxe = LMIC_UNUSED_PIN,
    .rst = SOC_GPIO_PIN_RST,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
    .busy = LMIC_UNUSED_PIN,
    .tcxo = LMIC_UNUSED_PIN,
};

static uint8_t ieeeAddr[8];
WS2812 strip(PIX_NUM);
uint8_t LEDs[PIX_NUM][3];

#if defined(USE_OLED)
#include <U8x8lib.h>

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8_i2c(U8X8_PIN_NONE);

static U8X8_SSD1306_128X64_NONAME_HW_I2C *u8x8 = NULL;

static bool OLED_display_probe_status = false;
static bool OLED_display_frontpage = false;
static uint32_t prev_tx_packets_counter = 0;
static uint32_t prev_rx_packets_counter = 0;
extern uint32_t tx_packets_counter, rx_packets_counter;

const char *OLED_Protocol_ID[] = {
  [RF_PROTOCOL_LEGACY]    = "L",
  [RF_PROTOCOL_OGNTP]     = "O",
  [RF_PROTOCOL_P3I]       = "P",
  [RF_PROTOCOL_ADSB_1090] = "A",
  [RF_PROTOCOL_ADSB_UAT]  = "U",
  [RF_PROTOCOL_FANET]     = "F"
};
#endif /* USE_OLED */

char UDPpacketBuffer[4]; // Dummy definition to satisfy build sequence

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

static uint32_t bootCount = 0;

#if defined(ENERGIA_ARCH_CC13X2)
size_t strnlen (const char *string, size_t length)
{
   char *ret = (char *) memchr ((const void *) string, 0, length);
   return ret ? ret - string : length;
}

char* ltoa( long value, char *string, int radix )
{
  char tmp[33];
  char *tp = tmp;
  long i;
  unsigned long v;
  int sign;
  char *sp;

  if ( string == NULL )
  {
    return 0 ;
  }

  if (radix > 36 || radix <= 1)
  {
    return 0 ;
  }

  sign = (radix == 10 && value < 0);
  if (sign)
  {
    v = -value;
  }
  else
  {
    v = (unsigned long)value;
  }

  while (v || tp == tmp)
  {
    i = v % radix;
    v = v / radix;
    if (i < 10)
      *tp++ = i+'0';
    else
      *tp++ = i + 'a' - 10;
  }

  sp = string;

  if (sign)
    *sp++ = '-';
  while (tp > tmp)
    *sp++ = *--tp;
  *sp = 0;

  return string;
}

char* itoa( int value, char *string, int radix )
{
  return ltoa( value, string, radix ) ;
}
#endif

static void CC13XX_setup()
{
  uint32_t  reset_source = SysCtrlResetSourceGet();

  EasyLink_getIeeeAddr(ieeeAddr);
}

static void CC13XX_loop()
{

}

static void CC13XX_fini()
{

  SysCtrlSystemReset();
}

static void CC13XX_reset()
{
  SysCtrlSystemReset();
}

static uint32_t CC13XX_getChipId()
{
  uint32_t id = (uint32_t) ieeeAddr[7]        | ((uint32_t) ieeeAddr[6] << 8) | \
               ((uint32_t) ieeeAddr[5] << 16) | ((uint32_t) ieeeAddr[4] << 24);

  /* remap address to avoid overlapping with congested FLARM range */
  if (((id & 0x00FFFFFF) >= 0xDD0000) && ((id & 0x00FFFFFF) <= 0xDFFFFF)) {
    id += 0x100000;
  }

  return id;
}

static void* CC13XX_getResetInfoPtr()
{
  return 0;
}

static String CC13XX_getResetInfo()
{
  switch (reset_info.reason)
  {
    default                     : return F("No reset information available");
  }
}

static String CC13XX_getResetReason()
{
  return F("DEFAULT");
}

static uint32_t CC13XX_getFreeHeap()
{
  return 0; /* TBD */
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

static bool CC13XX_EEPROM_begin(size_t size)
{
#if !defined(EXCLUDE_EEPROM)
  EEPROM.begin(size);
#endif
  return true;
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

static byte CC13XX_Display_setup()
{
  byte rval = DISPLAY_NONE;

#if defined(USE_OLED)
/*
 * BUG:
 * return value of Wire.endTransmission() is always '4' with Arduino Core for CCC13X2
 */
#if 0
  /* SSD1306 I2C OLED probing */
  Wire.begin();
  Wire.beginTransmission(SSD1306_OLED_I2C_ADDR);
  if (Wire.endTransmission() == 0)
#endif
  {
    u8x8 = &u8x8_i2c;
    rval = DISPLAY_OLED_TTGO;
  }

  if (u8x8) {
    u8x8->begin();
    u8x8->setFont(u8x8_font_chroma48medium8_r);
    u8x8->clear();

    u8x8->draw2x2String(2, 3, "SoftRF");
  }
#endif /* USE_OLED */

  return rval;
}

static void CC13XX_Display_loop()
{
#if defined(USE_OLED)
  char buf[16];
  uint32_t disp_value;

  if (u8x8) {
    if (!OLED_display_probe_status) {
      u8x8->clear();

      u8x8->draw2x2String(0, 0, "RADIO");
      u8x8->draw2x2String(14, 0, hw_info.rf   != RF_IC_NONE       ? "+" : "-");
      u8x8->draw2x2String(0, 2, "GNSS");
      u8x8->draw2x2String(14, 2, hw_info.gnss != GNSS_MODULE_NONE ? "+" : "-");
      u8x8->draw2x2String(0, 4, "OLED");
      u8x8->draw2x2String(14, 4, hw_info.display != DISPLAY_NONE  ? "+" : "-");
      u8x8->draw2x2String(0, 6, "BARO");
      u8x8->draw2x2String(14, 6, hw_info.baro != BARO_MODULE_NONE ? "+" : "-");

      delay(3000);

      OLED_display_probe_status = true;
    } else if (!OLED_display_frontpage) {

      u8x8->clear();

      u8x8->drawString(1, 1, "ID");

      itoa(ThisAircraft.addr & 0xFFFFFF, buf, 16);
      u8x8->draw2x2String(0, 2, buf);

      u8x8->drawString(8, 1, "PROTOCOL");

      u8x8->draw2x2String(14, 2, OLED_Protocol_ID[ThisAircraft.protocol]);

      u8x8->drawString(1, 5, "RX");

      itoa(rx_packets_counter % 1000, buf, 10);
      u8x8->draw2x2String(0, 6, buf);

      u8x8->drawString(9, 5, "TX");

      if (settings->mode != SOFTRF_MODE_BRIDGE ||
          (settings->mode == SOFTRF_MODE_BRIDGE &&
           settings->txpower == RF_TX_POWER_OFF)) {
        strcpy(buf, "OFF");
      } else {
        itoa(tx_packets_counter % 1000, buf, 10);
      }
      u8x8->draw2x2String(8, 6, buf);

      OLED_display_frontpage = true;
    } else {
      if (rx_packets_counter > prev_rx_packets_counter) {
        disp_value = rx_packets_counter % 1000;
        itoa(disp_value, buf, 10);

        if (disp_value < 10) {
          strcat_P(buf,PSTR("  "));
        } else {
          if (disp_value < 100) {
            strcat_P(buf,PSTR(" "));
          };
        }

        u8x8->draw2x2String(0, 6, buf);
        prev_rx_packets_counter = rx_packets_counter;
      }
      if (tx_packets_counter > prev_tx_packets_counter) {
        disp_value = tx_packets_counter % 1000;
        itoa(disp_value, buf, 10);

        if (disp_value < 10) {
          strcat_P(buf,PSTR("  "));
        } else {
          if (disp_value < 100) {
            strcat_P(buf,PSTR(" "));
          };
        }

        u8x8->draw2x2String(8, 6, buf);
        prev_tx_packets_counter = tx_packets_counter;
      }
    }
  }
#endif /* USE_OLED */
}

static void CC13XX_Display_fini(const char *msg)
{
#if defined(USE_OLED)
  if (u8x8) {
    u8x8->setFont(u8x8_font_chroma48medium8_r);
    u8x8->clear();
    u8x8->draw2x2String(1, 3, msg);
  }
#endif /* USE_OLED */
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
  CC13XX_loop,
  CC13XX_fini,
  CC13XX_reset,
  CC13XX_getChipId,
  CC13XX_getResetInfoPtr,
  CC13XX_getResetInfo,
  CC13XX_getResetReason,
  CC13XX_getFreeHeap,
  CC13XX_random,
  CC13XX_Sound_test,
  NULL,
  CC13XX_WiFi_setOutputPower,
  CC13XX_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  CC13XX_EEPROM_begin,
  CC13XX_SPI_begin,
  CC13XX_swSer_begin,
  CC13XX_swSer_enableRx,
  NULL,
  CC13XX_Display_setup,
  CC13XX_Display_loop,
  CC13XX_Display_fini,
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

#endif /* ENERGIA_ARCH_CC13XX || ENERGIA_ARCH_CC13X2 */
