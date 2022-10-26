/*
 * OLEDHelper.cpp
 * Copyright (C) 2019-2022 Linar Yusupov
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
#include "../system/Time.h"

enum
{
  OLED_PAGE_RADIO,
  OLED_PAGE_OTHER,
#if !defined(EXCLUDE_OLED_BARO_PAGE)
  OLED_PAGE_BARO,
#endif /* EXCLUDE_OLED_BARO_PAGE */
  OLED_PAGE_COUNT
};

#if !defined(EXCLUDE_OLED_049)
enum
{
  OLED_049_PAGE_ID,
  OLED_049_PAGE_PROTOCOL,
  OLED_049_PAGE_RX,
  OLED_049_PAGE_SATS_TX,
  OLED_049_PAGE_ACFTS,
  OLED_049_PAGE_UPTIME,
  OLED_049_PAGE_VOLTAGE,
  OLED_049_PAGE_COUNT
};
#endif /* EXCLUDE_OLED_049 */

U8X8_OLED_I2C_BUS_TYPE u8x8_i2c(U8X8_PIN_NONE);

U8X8 *u8x8 = NULL;

static bool OLED_display_titles = false;
static uint32_t prev_tx_packets_counter = (uint32_t) -1;
static uint32_t prev_rx_packets_counter = (uint32_t) -1;
extern uint32_t tx_packets_counter, rx_packets_counter;

static uint32_t prev_acrfts_counter = (uint32_t) -1;
static uint32_t prev_sats_counter   = (uint32_t) -1;
static uint32_t prev_uptime_minutes = (uint32_t) -1;
static int32_t  prev_voltage        = (uint32_t) -1;
static int8_t   prev_fix            = (uint8_t)  -1;

#if !defined(EXCLUDE_OLED_BARO_PAGE)
static int32_t  prev_altitude       = (int32_t)   -10000;
static int32_t  prev_temperature    = (int32_t)   -100;
static uint32_t prev_pressure       = (uint32_t)  -1;
static int32_t  prev_cdr            = (int32_t)   -10000; /* climb/descent rate */
#endif /* EXCLUDE_OLED_BARO_PAGE */

unsigned long OLEDTimeMarker = 0;

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

const char SoftRF_text1[]  = "SoftRF";
const char SoftRF_text2[]  = "and";
const char SoftRF_text3[]  = "LilyGO";
const char ID_text[]       = "ID";
const char PROTOCOL_text[] = "PROTOCOL";
const char RX_text[]       = "RX";
const char TX_text[]       = "TX";
const char ACFTS_text[]    = "ACFTS";
const char SATS_text[]     = "SATS";
const char FIX_text[]      = "FIX";
const char UPTIME_text[]   = "UPTIME";
const char BAT_text[]      = "BAT";

#if !defined(EXCLUDE_OLED_BARO_PAGE)
const char ALT_text[]      = "ALT M";
const char TEMP_text[]     = "TEMP C";
const char PRES_text[]     = "PRES MB";
const char CDR_text[]      = "CDR FPM";
#endif /* EXCLUDE_OLED_BARO_PAGE */

static const uint8_t Dot_Tile[] = { 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00 };

static uint8_t OLED_current_page = OLED_PAGE_RADIO;
static uint8_t page_count        = OLED_PAGE_COUNT;

byte OLED_setup() {

  byte rval = DISPLAY_NONE;
  bool oled_probe = false;

#if defined(plat_oled_probe_func)
  oled_probe = plat_oled_probe_func();
#else
  /* SSD1306 I2C OLED probing */
  Wire.begin();
  Wire.beginTransmission(SSD1306_OLED_I2C_ADDR);
  oled_probe = (Wire.endTransmission() == 0);
#endif /* plat_oled_probe_func */
  if (oled_probe)
  {
    u8x8 = &u8x8_i2c;
    rval = (hw_info.model == SOFTRF_MODEL_MINI     ? DISPLAY_OLED_HELTEC :
            hw_info.model == SOFTRF_MODEL_BRACELET ? DISPLAY_OLED_0_49   :
            DISPLAY_OLED_TTGO);
  }

  if (u8x8) {
    u8x8->begin();
    u8x8->setFont(u8x8_font_chroma48medium8_r);

    switch (rval)
    {
#if !defined(EXCLUDE_OLED_049)
    case DISPLAY_OLED_0_49:

      u8x8->setContrast(255);

      u8x8->draw2x2Glyph(4,  4, SoftRF_text3[0]);
      u8x8->draw2x2Glyph(6,  4, SoftRF_text3[1]);
      u8x8->draw2x2Glyph(8,  4, SoftRF_text3[2]);
      u8x8->draw2x2Glyph(10, 4, SoftRF_text3[3]);
      u8x8->draw2x2Glyph(6,  6, SoftRF_text3[4]);
      u8x8->draw2x2Glyph(8,  6, SoftRF_text3[5]);

      delay(2000);

      u8x8->clear();
      u8x8->draw2x2String( 5, 5, SoftRF_text2);

      delay(2000);

      u8x8->clear();
      u8x8->draw2x2Glyph(4,  4, SoftRF_text1[0]);
      u8x8->draw2x2Glyph(6,  4, SoftRF_text1[1]);
      u8x8->draw2x2Glyph(8,  4, SoftRF_text1[2]);
      u8x8->draw2x2Glyph(10, 4, SoftRF_text1[3]);
      u8x8->draw2x2Glyph(6,  6, SoftRF_text1[4]);
      u8x8->draw2x2Glyph(8,  6, SoftRF_text1[5]);

      OLED_current_page = OLED_049_PAGE_SATS_TX;
      page_count        = OLED_049_PAGE_COUNT;
      break;
#endif /* EXCLUDE_OLED_049 */
    case DISPLAY_OLED_TTGO:
    case DISPLAY_OLED_HELTEC:
    case DISPLAY_OLED_1_3:
    default:
      uint8_t shift_y = (hw_info.model == SOFTRF_MODEL_DONGLE ? 1 : 0);

      u8x8->draw2x2String( 2, 2 - shift_y, SoftRF_text1);

      if (shift_y) {
        u8x8->drawString   ( 6, 3, SoftRF_text2);
        u8x8->draw2x2String( 2, 4, SoftRF_text3);
      }

      u8x8->drawString   ( 3, 6 + shift_y, SOFTRF_FIRMWARE_VERSION);
      u8x8->drawString   (11, 6 + shift_y, ISO3166_CC[settings->band]);

      break;
    }
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

    u8x8->draw2x2Glyph(14, 2, Protocol_ID[ThisAircraft.protocol][0]);

    u8x8->drawString(1, 5, RX_text);

    u8x8->drawString(9, 5, TX_text);

    if (settings->power_save & POWER_SAVE_NORECEIVE &&
        (hw_info.rf == RF_IC_SX1276 || hw_info.rf == RF_IC_SX1262)) {
      u8x8->draw2x2String(0, 6, "OFF");
      prev_rx_packets_counter = rx_packets_counter;
    } else {
      prev_rx_packets_counter = (uint32_t) -1;
    }

    if (settings->mode        == SOFTRF_MODE_RECEIVER ||
        settings->rf_protocol == RF_PROTOCOL_ADSB_UAT ||
        settings->txpower     == RF_TX_POWER_OFF) {
      u8x8->draw2x2String(8, 6, "OFF");
      prev_tx_packets_counter = tx_packets_counter;
    } else {
      prev_tx_packets_counter = (uint32_t) -1;
    }

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
  uint8_t  fix            = (uint8_t) isValidGNSSFix();
  uint32_t uptime_minutes = UpTime.minutes;
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
//  u8x8->draw2x2Glyph(12, 2, '0' + fix);
    prev_fix = fix;
  }

  if (prev_uptime_minutes != uptime_minutes) {
    disp_value = UpTime.hours; /* 0-23 */
    if (disp_value < 10) {
      buf[0] = '0';
      itoa(disp_value, buf+1, 10);
    } else {
      itoa(disp_value, buf, 10);
    }

    u8x8->draw2x2String(0, 6, buf);

    disp_value = uptime_minutes;
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

#if !defined(EXCLUDE_OLED_BARO_PAGE)
static void OLED_baro()
{
  char buf[16];

  if (!OLED_display_titles) {

    u8x8->clear();

    u8x8->drawString( 2, 1, ALT_text);

    u8x8->drawString( 10, 1, TEMP_text);

    u8x8->drawString( 1, 5, PRES_text);

    u8x8->drawString( 9, 5, CDR_text);

    prev_altitude     = (int32_t)   -10000;
    prev_temperature  = prev_altitude;
    prev_pressure     = (uint32_t)  -1;
    prev_cdr          = prev_altitude;

    OLED_display_titles = true;
  }

  int32_t altitude    = Baro_altitude();        /* metres */
  int32_t temperature = Baro_temperature();     /* Celcius */
  uint32_t pressure   = Baro_pressure() / 100;  /* mbar */
  int32_t cdr         = ThisAircraft.vs;        /* feet per minute */

  if (prev_altitude != altitude) {
    snprintf(buf, sizeof(buf), "%4d", altitude);
    u8x8->draw2x2String(0, 2, buf);
    prev_altitude = altitude;
  }

  if (prev_temperature != temperature) {
    snprintf(buf, sizeof(buf), "%3d", temperature);
    u8x8->draw2x2String(10, 2, buf);
    prev_temperature = temperature;
  }

  if (prev_pressure != pressure) {
    snprintf(buf, sizeof(buf), "%4d", pressure);
    u8x8->draw2x2String(0, 6, buf);
    prev_pressure = pressure;
  }

  if (prev_cdr != cdr) {
    int disp_value = constrain(cdr, -999, 999);
    snprintf(buf, sizeof(buf), "%3d", abs(disp_value));
    u8x8->drawGlyph    ( 9, 6, disp_value < 0 ? '_' : ' ');
    u8x8->draw2x2String(10, 6, buf);
    prev_cdr = cdr;
  }
}
#endif /* EXCLUDE_OLED_BARO_PAGE */

#if !defined(EXCLUDE_OLED_049)

void OLED_049_func()
{
  char buf[16];
  uint32_t disp_value;
  uint32_t acrfts_counter;
  uint32_t sats_counter;
  uint32_t uptime_minutes;
  int32_t  voltage;

  switch (OLED_current_page)
  {
  case OLED_049_PAGE_ID:
    if (!OLED_display_titles) {
      u8x8->clear();
      u8x8->drawString(5, 4, ID_text);
      snprintf (buf, sizeof(buf), "%06X", ThisAircraft.addr);
      u8x8->draw2x2Glyph ( 8, 4, buf[0]);
      u8x8->draw2x2Glyph (10, 4, buf[1]);
      u8x8->draw2x2String( 4, 6, buf+2);

      OLED_display_titles = true;
    }

    break;

  case OLED_049_PAGE_PROTOCOL:
    if (!OLED_display_titles) {
      u8x8->clear();
      u8x8->drawString(4, 4, PROTOCOL_text);
      u8x8->draw2x2String(5, 6, Protocol_ID[ThisAircraft.protocol]);

      OLED_display_titles = true;
    }

    break;

  case OLED_049_PAGE_RX:
    if (!OLED_display_titles) {
      u8x8->clear();

      u8x8->drawString(5, 4, RX_text);

      if (settings->power_save & POWER_SAVE_NORECEIVE &&
          (hw_info.rf == RF_IC_SX1276 || hw_info.rf == RF_IC_SX1262)) {
        u8x8->draw2x2String(5, 6, "OFF");
        prev_rx_packets_counter = rx_packets_counter;
      } else {
        prev_rx_packets_counter = (uint32_t) -1;
      }

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

      u8x8->draw2x2String(5, 6, buf);
      prev_rx_packets_counter = rx_packets_counter;
    }

    break;

  case OLED_049_PAGE_SATS_TX:
    if (!OLED_display_titles) {
      u8x8->clear();

      u8x8->drawString( 4, 4, SATS_text);
      prev_sats_counter   = (uint32_t) -1;

      u8x8->drawString(10, 4, TX_text);

      if (settings->mode        == SOFTRF_MODE_RECEIVER ||
          settings->rf_protocol == RF_PROTOCOL_ADSB_UAT ||
          settings->txpower     == RF_TX_POWER_OFF) {
        u8x8->draw2x2String(8, 6, "NA");
        prev_tx_packets_counter = tx_packets_counter;
      } else {
        prev_tx_packets_counter = (uint32_t) -1;
      }

      OLED_display_titles = true;
    }

    sats_counter   = gnss.satellites.value();

    if (prev_sats_counter != sats_counter) {
      disp_value = sats_counter > 9 ? 9 : sats_counter;

      u8x8->draw2x2Glyph(4, 6, '0' + disp_value);
      prev_sats_counter = sats_counter;
    }

    if (tx_packets_counter != prev_tx_packets_counter) {
      disp_value = tx_packets_counter % 100;
      itoa(disp_value, buf, 10);

      if (disp_value < 10) {
        strcat_P(buf,PSTR(" "));
      } else {
      }

      u8x8->draw2x2String(8, 6, buf);
      prev_tx_packets_counter = tx_packets_counter;
    }

    break;

  case OLED_049_PAGE_ACFTS:
    if (!OLED_display_titles) {
      u8x8->clear();
      u8x8->drawString( 5, 4, ACFTS_text);
      prev_acrfts_counter = (uint32_t) -1;

      OLED_display_titles = true;
    }

    acrfts_counter = Traffic_Count();

    if (prev_acrfts_counter != acrfts_counter) {
      disp_value = acrfts_counter > 99 ? 99 : acrfts_counter;
      itoa(disp_value, buf, 10);

      if (disp_value < 10) {
        strcat_P(buf,PSTR(" "));
      }

      u8x8->draw2x2String(5, 6, buf);
      prev_acrfts_counter = acrfts_counter;
    }

    break;

  case OLED_049_PAGE_UPTIME:
    if (!OLED_display_titles) {
      u8x8->clear();
      u8x8->drawString( 5, 4, UPTIME_text);
      u8x8->drawTile  (7, 6, 1, (uint8_t *) Dot_Tile);
      u8x8->drawTile  (7, 7, 1, (uint8_t *) Dot_Tile);
      prev_uptime_minutes = (uint32_t) -1;

      OLED_display_titles = true;
    }

    uptime_minutes = UpTime.minutes;

    if (prev_uptime_minutes != uptime_minutes) {
      disp_value = UpTime.hours % 10; /* 0-9, 0-9, 0-3 */
      itoa(disp_value, buf, 10);

      u8x8->draw2x2String(5, 6, buf);

      disp_value = uptime_minutes;
      if (disp_value < 10) {
        buf[0] = '0';
        itoa(disp_value, buf+1, 10);
      } else {
        itoa(disp_value, buf, 10);
      }

      u8x8->draw2x2String(8, 6, buf);

      prev_uptime_minutes = uptime_minutes;
    }

    break;

  case OLED_049_PAGE_VOLTAGE:
    if (!OLED_display_titles) {
      u8x8->clear();
      u8x8->drawString(5, 4, BAT_text);
      u8x8->drawGlyph (7, 7, '.');
      prev_voltage        = (uint32_t) -1;

      OLED_display_titles = true;
    }

    voltage = Battery_voltage() > BATTERY_THRESHOLD_INVALID ?
              (int) (Battery_voltage() * 10.0) : 0;

    if (prev_voltage != voltage) {
      if (voltage) {
        disp_value = voltage / 10;
        disp_value = disp_value > 9 ? 9 : disp_value;
        u8x8->draw2x2Glyph(5, 6, '0' + disp_value);

        disp_value = voltage % 10;

        u8x8->draw2x2Glyph(8, 6, '0' + disp_value);
      } else {
        u8x8->draw2x2Glyph(5, 6, 'N');
        u8x8->draw2x2Glyph(8, 6, 'A');
      }
      prev_voltage = voltage;
    }

    break;

  default:
    break;
  }
}

#endif /* EXCLUDE_OLED_049 */

void OLED_loop()
{
  if (u8x8) {
    if (isTimeToOLED()) {
#if !defined(EXCLUDE_OLED_049)
      if (hw_info.display == DISPLAY_OLED_0_49) {
        OLED_049_func();
      } else
#endif /* EXCLUDE_OLED_049 */
        switch (OLED_current_page)
        {
        case OLED_PAGE_OTHER:
          OLED_other();
          break;
#if !defined(EXCLUDE_OLED_BARO_PAGE)
        case OLED_PAGE_BARO:
          OLED_baro();
          break;
#endif /* EXCLUDE_OLED_BARO_PAGE */
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
    u8x8->clear();
    switch (hw_info.display)
    {
#if !defined(EXCLUDE_OLED_049)
    case DISPLAY_OLED_0_49:
      u8x8->draw2x2String(5, 5, reason == SOFTRF_SHUTDOWN_LOWBAT ?
                                "BAT" : "OFF");
      delay(2000);
      u8x8->noDisplay();
      break;
#endif /* EXCLUDE_OLED_049 */
    case DISPLAY_OLED_TTGO:
    case DISPLAY_OLED_HELTEC:
    case DISPLAY_OLED_1_3:
    default:
      u8x8->draw2x2String(1, 3, reason == SOFTRF_SHUTDOWN_LOWBAT ?
                                "LOW BAT" : "  OFF  ");
      break;
    }
  }
}

void OLED_info1()
{
  if (u8x8) {

    u8x8->clear();

    switch (hw_info.display)
    {
#if !defined(EXCLUDE_OLED_049)
    case DISPLAY_OLED_0_49:
      {
        u8x8->draw2x2Glyph(  4, 4, 'R');
        u8x8->draw2x2Glyph(  6, 4, hw_info.rf      != RF_IC_NONE       ? '+' : '-');
        u8x8->draw2x2Glyph(  8, 4, 'G');
        u8x8->draw2x2Glyph( 10, 4, hw_info.gnss    != GNSS_MODULE_NONE ? '+' : '-');
        u8x8->draw2x2Glyph(  4, 6, 'O');
        u8x8->draw2x2Glyph(  6, 6, hw_info.display != DISPLAY_NONE     ? '+' : '-');
        u8x8->draw2x2Glyph(  8, 6, 'I');
        u8x8->draw2x2Glyph( 10, 6, hw_info.imu     != IMU_NONE         ? '+' : '-');

        delay(3000);

        const char buf[] = SOFTRF_FIRMWARE_VERSION;
        int ndx = strlen(buf) - 3;
        ndx = ndx < 0 ? 0 : ndx;
        u8x8->clear();
        u8x8->drawString  (4, 4, "VERSION");
        u8x8->draw2x2Glyph(5, 6, toupper(buf[ndx++]));
        u8x8->draw2x2Glyph(7, 6, toupper(buf[ndx++]));
        u8x8->draw2x2Glyph(9, 6, toupper(buf[ndx]));

        delay(2000);

        u8x8->clear();
        u8x8->drawString   (4, 4, "REGION");
        u8x8->draw2x2String(6, 6, ISO3166_CC[settings->band]);
      }
      break;
#endif /* EXCLUDE_OLED_049 */
    case DISPLAY_OLED_TTGO:
    case DISPLAY_OLED_HELTEC:
    case DISPLAY_OLED_1_3:
    default:

      u8x8->draw2x2String( 0, 0, "RADIO");
      u8x8->draw2x2String(14, 0, hw_info.rf   != RF_IC_NONE       ? "+" : "-");
      u8x8->draw2x2String( 0, 2, "GNSS");
      u8x8->draw2x2String(14, 2, hw_info.gnss != GNSS_MODULE_NONE ? "+" : "-");
      u8x8->draw2x2String( 0, 4, "OLED");
      u8x8->draw2x2String(14, 4, hw_info.display != DISPLAY_NONE  ? "+" : "-");
      u8x8->draw2x2String( 0, 6, "BARO");
      u8x8->draw2x2String(14, 6, hw_info.baro != BARO_MODULE_NONE ? "+" : "-");

      break;
    }

    delay(3000);
  }
}

void OLED_info2()
{
  if (u8x8) {

    u8x8->clear();

    switch (hw_info.display)
    {
    case DISPLAY_OLED_TTGO:
    case DISPLAY_OLED_HELTEC:
    case DISPLAY_OLED_1_3:
    default:

      u8x8->draw2x2String( 0, 0, "RTC");
      u8x8->draw2x2String(14, 0, hw_info.rtc != RTC_NONE ? "+" : "-");
      u8x8->draw2x2String( 0, 2, "IMU");
      u8x8->draw2x2String(14, 2, hw_info.imu != IMU_NONE ? "+" : "-");
      u8x8->draw2x2String( 0, 4, "MAG");
      u8x8->draw2x2String(14, 4, hw_info.mag != MAG_NONE ? "+" : "-");
      u8x8->draw2x2String( 0, 6, "CARD");
      u8x8->draw2x2String(14, 6, hw_info.storage == STORAGE_CARD ||
                                 hw_info.storage == STORAGE_FLASH_AND_CARD ?
                                                           "+" : "-");
      break;
    }

    delay(3000);
  }
}

void OLED_info3(int acfts, char *reg, char *mam, char *cn)
{
  if (u8x8) {

    u8x8->clear();

    switch (hw_info.display)
    {
    case DISPLAY_OLED_TTGO:
    case DISPLAY_OLED_HELTEC:
    case DISPLAY_OLED_1_3:
    default:

      if (acfts == -1) {
        u8x8->draw2x2String( 6, 1, "NO");
        u8x8->draw2x2String( 0, 3, "AIRCRAFT");
        u8x8->draw2x2String( 4, 5, "DATA");
      } else {
        char str1[9], str2[9], str3[9], str4[9];

        memset(str1, 0, sizeof(str1));
        memset(str2, 0, sizeof(str2));
        memset(str3, 0, sizeof(str3));
        memset(str4, 0, sizeof(str4));

        snprintf(str1, 6, "%d", acfts);
        strncpy (str2, reg, 8);
        strncpy (str3, mam, 8);
        strncpy (str4,  cn, 8);

        u8x8->draw2x2String( 4, 0, str1);
        u8x8->draw2x2String( 0, 2, str2);
        u8x8->draw2x2String( 0, 4, str3);
        u8x8->draw2x2String( 0, 6, str4);
      }

      break;
    }

    delay(3000);
  }
}

void OLED_Next_Page()
{
  if (u8x8) {
    OLED_current_page = (OLED_current_page + 1) % page_count;

#if !defined(EXCLUDE_OLED_BARO_PAGE)
    if (hw_info.display   != DISPLAY_OLED_0_49 &&
        OLED_current_page == OLED_PAGE_BARO    &&
        hw_info.baro      == BARO_MODULE_NONE) {
      OLED_current_page = (OLED_current_page + 1) % page_count;
    }
#endif /* EXCLUDE_OLED_BARO_PAGE */

#if !defined(EXCLUDE_OLED_049)
    if (hw_info.display   == DISPLAY_OLED_0_49      &&
        OLED_current_page == OLED_049_PAGE_ACFTS    &&
        settings->power_save & POWER_SAVE_NORECEIVE &&
        (hw_info.rf == RF_IC_SX1276 || hw_info.rf == RF_IC_SX1262)) {
      OLED_current_page = (OLED_current_page + 1) % page_count;
    }
#endif /* EXCLUDE_OLED_049 */

    OLED_display_titles = false;
  }
}

#endif /* USE_OLED */
