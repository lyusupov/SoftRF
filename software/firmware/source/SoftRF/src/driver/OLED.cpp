/*
 * OLEDHelper.cpp
 * Copyright (C) 2019-2021 Linar Yusupov
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

#include "../system/SoC.h"

#if defined(USE_OLED)

#include <Wire.h>

#include "OLED.h"

#include "RF.h"
#include "LED.h"
#include "GNSS.h"
#include "Baro.h"
#include "Battery.h"
#include "../TrafficHelper.h"

enum
{
  OLED_PAGE_RADIO,
  OLED_PAGE_OTHER,
  OLED_PAGE_COUNT
};

U8X8_OLED_I2C_BUS_TYPE u8x8_i2c(U8X8_PIN_NONE);

U8X8_OLED_I2C_BUS_TYPE *u8x8 = NULL;

static bool OLED_display_titles = false;
static uint32_t prev_tx_packets_counter = (uint32_t) -1;
static uint32_t prev_rx_packets_counter = (uint32_t) -1;
extern uint32_t tx_packets_counter, rx_packets_counter;

static uint32_t prev_acrfts_counter = (uint32_t) -1;
static uint32_t prev_sats_counter   = (uint32_t) -1;
static uint32_t prev_uptime_minutes = (uint32_t) -1;
static int32_t  prev_voltage        = (uint32_t) -1;
static int8_t   prev_fix            = (uint8_t)  -1;

unsigned long OLEDTimeMarker = 0;

const char *OLED_Protocol_ID[] = {
  [RF_PROTOCOL_LEGACY]    = "L",
  [RF_PROTOCOL_OGNTP]     = "O",
  [RF_PROTOCOL_P3I]       = "P",
  [RF_PROTOCOL_ADSB_1090] = "A",
  [RF_PROTOCOL_ADSB_UAT]  = "U",
  [RF_PROTOCOL_FANET]     = "F"
};

const char *ISO3166_CC[] = {
  [RF_BAND_AUTO] = "--",
  [RF_BAND_EU]   = "EU",
  [RF_BAND_US]   = "US",
  [RF_BAND_AU]   = "AU",
  [RF_BAND_NZ]   = "NZ",
  [RF_BAND_RU]   = "RU",
  [RF_BAND_CN]   = "CN",
  [RF_BAND_UK]   = "UK",
  [RF_BAND_IN]   = "IN",
  [RF_BAND_IL]   = "IL",
  [RF_BAND_KR]   = "KR"
};

const char SoftRF_text[]   = "SoftRF";
const char ID_text[]       = "ID";
const char PROTOCOL_text[] = "PROTOCOL";
const char RX_text[]       = "RX";
const char TX_text[]       = "TX";
const char ACFTS_text[]    = "ACFTS";
const char SATS_text[]     = "SATS";
const char FIX_text[]      = "FIX";
const char UPTIME_text[]   = "UPTIME";
const char BAT_text[]      = "BAT";

static const uint8_t Dot_Tile[] = { 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00 };

static int OLED_current_page = OLED_PAGE_RADIO;

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
    rval = (hw_info.model == SOFTRF_MODEL_MINI ?
            DISPLAY_OLED_HELTEC : DISPLAY_OLED_TTGO);
  }

  if (u8x8) {
    u8x8->begin();
    u8x8->setFont(u8x8_font_chroma48medium8_r);
    u8x8->clear();

    uint8_t shift_y = (hw_info.model == SOFTRF_MODEL_DONGLE ? 1 : 0);

    u8x8->draw2x2String( 2, 2 - shift_y, SoftRF_text);

    if (hw_info.model == SOFTRF_MODEL_DONGLE) {
      u8x8->drawString   ( 6, 3, "and");
      u8x8->draw2x2String( 2, 4, "LilyGO");
    }

    u8x8->drawString   ( 3, 6 + shift_y, SOFTRF_FIRMWARE_VERSION);
    u8x8->drawString   (11, 6 + shift_y, ISO3166_CC[settings->band]);
  }

  OLEDTimeMarker = millis();

  return rval;
}

static void OLED_radio()
{
  char buf[16];
  uint32_t disp_value;

  if (!OLED_display_titles) {

    u8x8->clear();

    u8x8->drawString(1, 1, ID_text);

    snprintf (buf, sizeof(buf), "%06X", ThisAircraft.addr);
    u8x8->draw2x2String(0, 2, buf);

    u8x8->drawString(8, 1, PROTOCOL_text);

    u8x8->draw2x2String(14, 2, OLED_Protocol_ID[ThisAircraft.protocol]);

    u8x8->drawString(1, 5, RX_text);

    u8x8->drawString(9, 5, TX_text);

    if (settings->mode        == SOFTRF_MODE_RECEIVER ||
        settings->rf_protocol == RF_PROTOCOL_ADSB_UAT ||
        settings->txpower     == RF_TX_POWER_OFF) {
      u8x8->draw2x2String(8, 6, "OFF");
      prev_tx_packets_counter = tx_packets_counter;
    } else {
      prev_tx_packets_counter = (uint32_t) -1;
    }

    prev_rx_packets_counter = (uint32_t) -1;

    OLED_display_titles = true;
  }

  if (rx_packets_counter != prev_rx_packets_counter) {
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
  if (tx_packets_counter != prev_tx_packets_counter) {
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

static void OLED_other()
{
  char buf[16];
  uint32_t disp_value;

  if (!OLED_display_titles) {

    u8x8->clear();

    u8x8->drawString( 1, 1, ACFTS_text);

    u8x8->drawString( 7, 1, SATS_text);

    u8x8->drawString(12, 1, FIX_text);

    u8x8->drawString( 1, 5, UPTIME_text);

    u8x8->drawString(12, 5, BAT_text);

    u8x8->drawTile  (4, 6, 1, (uint8_t *) Dot_Tile);
    u8x8->drawTile  (4, 7, 1, (uint8_t *) Dot_Tile);

    u8x8->drawGlyph (13, 7, '.');

    prev_acrfts_counter = (uint32_t) -1;
    prev_sats_counter   = (uint32_t) -1;
    prev_fix            = (uint8_t)  -1;
    prev_uptime_minutes = (uint32_t) -1;
    prev_voltage        = (uint32_t) -1;

    OLED_display_titles = true;
  }

  uint32_t acrfts_counter = Traffic_Count();
  uint32_t sats_counter   = gnss.satellites.value();
  uint8_t  fix            = gnss.location.Quality();
  uint32_t uptime_minutes = millis() / 60000;
  int32_t  voltage        = Battery_voltage() > BATTERY_THRESHOLD_INVALID ?
                              (int) (Battery_voltage() * 10.0) : 0;

  if (prev_acrfts_counter != acrfts_counter) {
    disp_value = acrfts_counter > 99 ? 99 : acrfts_counter;
    itoa(disp_value, buf, 10);

    if (disp_value < 10) {
      strcat_P(buf,PSTR(" "));
    }

    u8x8->draw2x2String(1, 2, buf);
    prev_acrfts_counter = acrfts_counter;
  }

  if (prev_sats_counter != sats_counter) {
    disp_value = sats_counter > 99 ? 99 : sats_counter;
    itoa(disp_value, buf, 10);

    if (disp_value < 10) {
      strcat_P(buf,PSTR(" "));
    }

    u8x8->draw2x2String(7, 2, buf);
    prev_sats_counter = sats_counter;
  }

  if (prev_fix != fix) {
    u8x8->draw2x2Glyph(12, 2, fix > 0 ? '+' : '-');
//    u8x8->draw2x2Glyph(12, 2, '0' + fix);
    prev_fix = fix;
  }

  if (prev_uptime_minutes != uptime_minutes) {
    uint32_t uptime_hours = uptime_minutes / 60;
    disp_value = uptime_hours % 100;
    if (disp_value < 10) {
      buf[0] = '0';
      itoa(disp_value, buf+1, 10);
    } else {
      itoa(disp_value, buf, 10);
    }

    u8x8->draw2x2String(0, 6, buf);

    disp_value = uptime_minutes % 60;
    if (disp_value < 10) {
      buf[0] = '0';
      itoa(disp_value, buf+1, 10);
    } else {
      itoa(disp_value, buf, 10);
    }

    u8x8->draw2x2String(5, 6, buf);

    prev_uptime_minutes = uptime_minutes;
  }

  if (prev_voltage != voltage) {
    if (voltage) {
      disp_value = voltage / 10;
      disp_value = disp_value > 9 ? 9 : disp_value;
      u8x8->draw2x2Glyph(11, 6, '0' + disp_value);

      disp_value = voltage % 10;

      u8x8->draw2x2Glyph(14, 6, '0' + disp_value);
    } else {
      u8x8->draw2x2Glyph(11, 6, 'N');
      u8x8->draw2x2Glyph(14, 6, 'A');
    }
    prev_voltage = voltage;
  }
}

void OLED_loop()
{
  if (u8x8) {
    if (isTimeToOLED()) {
      switch (OLED_current_page)
      {
      case OLED_PAGE_OTHER:
        OLED_other();
        break;
      case OLED_PAGE_RADIO:
      default:
        OLED_radio();
        break;
      }

      OLEDTimeMarker = millis();
    }
  }
}

void OLED_fini(int reason)
{
  if (u8x8) {
    u8x8->setFont(u8x8_font_chroma48medium8_r);
    u8x8->clear();
    u8x8->draw2x2String(1, 3, reason == SOFTRF_SHUTDOWN_LOWBAT ?
                              "LOW BAT" : "  OFF  ");
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

void OLED_Next_Page()
{
  if (u8x8) {
    OLED_current_page = (OLED_current_page + 1) % OLED_PAGE_COUNT;
    OLED_display_titles = false;
  }
}

#endif /* USE_OLED */
