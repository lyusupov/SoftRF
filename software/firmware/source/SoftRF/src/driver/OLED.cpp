/*
 * OLEDHelper.cpp
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

#include "../SoCHelper.h"

#if defined(USE_OLED)

#include <Wire.h>

#include "OLED.h"

#include "RF.h"
#include "LED.h"
#include "GNSS.h"
#include "Baro.h"

U8X8_OLED_I2C_BUS_TYPE u8x8_i2c(U8X8_PIN_NONE);

U8X8_OLED_I2C_BUS_TYPE *u8x8 = NULL;

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

const char *SoftRF_text   = "SoftRF";
const char *ID_text       = "ID";
const char *PROTOCOL_text = "PROTOCOL";
const char *RX_text       = "RX";
const char *TX_text       = "TX";

byte OLED_setup() {

  byte rval = DISPLAY_NONE;

/*
 * BUG:
 * return value of Wire.endTransmission() is always '4' with Arduino Core for CCC13X2
 */
#if !defined(ENERGIA_ARCH_CC13X2)
  /* SSD1306 I2C OLED probing */
  Wire.begin();
  Wire.beginTransmission(SSD1306_OLED_I2C_ADDR);
  if (Wire.endTransmission() == 0)
#endif /* ENERGIA_ARCH_CC13X2 */
  {
    u8x8 = &u8x8_i2c;
    rval = DISPLAY_OLED_TTGO;
  }

  if (u8x8) {
    u8x8->begin();
    u8x8->setFont(u8x8_font_chroma48medium8_r);
    u8x8->clear();

    if (hw_info.model == SOFTRF_MODEL_DONGLE) {
      u8x8->draw2x2String(2, 1, SoftRF_text);
      u8x8->drawString   (6, 4, "and");
      u8x8->draw2x2String(2, 6, "LilyGO");
    } else {
      u8x8->draw2x2String(2, 3, SoftRF_text);
    }
  }

  return rval;
}

void OLED_loop()
{
  char buf[16];
  uint32_t disp_value;

  if (u8x8) {
    if (!OLED_display_frontpage) {

      u8x8->clear();

      u8x8->drawString(1, 1, ID_text);

      itoa(ThisAircraft.addr & 0xFFFFFF, buf, 16);
      u8x8->draw2x2String(0, 2, buf);

      u8x8->drawString(8, 1, PROTOCOL_text);

      u8x8->draw2x2String(14, 2, OLED_Protocol_ID[ThisAircraft.protocol]);

      u8x8->drawString(1, 5, RX_text);

      itoa(rx_packets_counter % 1000, buf, 10);
      u8x8->draw2x2String(0, 6, buf);

      u8x8->drawString(9, 5, TX_text);

      if (settings->mode        == SOFTRF_MODE_RECEIVER ||
          settings->rf_protocol == RF_PROTOCOL_ADSB_UAT ||
          settings->txpower     == RF_TX_POWER_OFF) {
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
}

void OLED_fini(const char *msg)
{
  if (u8x8) {
    u8x8->setFont(u8x8_font_chroma48medium8_r);
    u8x8->clear();
    u8x8->draw2x2String(1, 3, msg);
  }
}

void OLED_info1()
{
  if (u8x8) {

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
  }
}

#endif /* USE_OLED */
