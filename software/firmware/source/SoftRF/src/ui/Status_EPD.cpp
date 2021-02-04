/*
 * View_Status_EPD.cpp
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

#if defined(USE_EPAPER)

#include "../driver/EPD.h"
#include "../TrafficHelper.h"
#include "../driver/Battery.h"
#include "../driver/EEPROM.h"
#include "../driver/RF.h"
#include <protocol.h>

#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeSerifBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>

extern uint32_t tx_packets_counter, rx_packets_counter;

const char *EPD_Protocol_ID[] = {
  [RF_PROTOCOL_LEGACY]    = "LEG",
  [RF_PROTOCOL_OGNTP]     = "OGN",
  [RF_PROTOCOL_P3I]       = "P3I",
  [RF_PROTOCOL_ADSB_1090] = "ADS",
  [RF_PROTOCOL_ADSB_UAT]  = "UAT",
  [RF_PROTOCOL_FANET]     = "FAN"
};

const char ID_text[]       = "ID";
const char PROTOCOL_text[] = "PROTOCOL";
const char RX_text[]       = "RX";
const char TX_text[]       = "TX";
const char ACFTS_text[]    = "ACFTS";
const char BAT_text[]      = "BAT";

static navbox_t navbox1;
static navbox_t navbox2;
static navbox_t navbox3;
static navbox_t navbox4;
static navbox_t navbox5;
static navbox_t navbox6;

void EPD_status_setup()
{
  memcpy(navbox1.title, NAVBOX1_TITLE, strlen(NAVBOX1_TITLE));
  navbox1.x = 0;
  navbox1.y = 0;
  navbox1.width  = display->width() / 2;
  navbox1.height = display->height() / 3;
  navbox1.value      = 0;
  navbox1.prev_value = navbox1.value;
  navbox1.timestamp  = millis();

  memcpy(navbox2.title, NAVBOX2_TITLE, strlen(NAVBOX2_TITLE));
  navbox2.x = navbox1.width;
  navbox2.y = navbox1.y;
  navbox2.width  = navbox1.width;
  navbox2.height = navbox1.height;
  navbox2.value      = 0;
  navbox2.prev_value = navbox2.value;
  navbox2.timestamp  = millis();

  memcpy(navbox3.title, NAVBOX3_TITLE, strlen(NAVBOX3_TITLE));
  navbox3.x = navbox1.x;
  navbox3.y = navbox1.y + navbox1.height;
  navbox3.width  = navbox1.width;
  navbox3.height = navbox1.height;
  navbox3.value      = ThisAircraft.addr;
  navbox3.prev_value = navbox3.value;
  navbox3.timestamp  = millis();

  memcpy(navbox4.title, NAVBOX4_TITLE, strlen(NAVBOX4_TITLE));
  navbox4.x = navbox3.width;
  navbox4.y = navbox3.y;
  navbox4.width  = navbox3.width;
  navbox4.height = navbox3.height;
  navbox4.value      = settings->rf_protocol;
  navbox4.prev_value = navbox4.value;
  navbox4.timestamp  = millis();

  memcpy(navbox5.title, NAVBOX5_TITLE, strlen(NAVBOX5_TITLE));
  navbox5.x = navbox3.x;
  navbox5.y = navbox3.y + navbox3.height;
  navbox5.width  = navbox3.width;
  navbox5.height = navbox3.height;
  navbox5.value      = rx_packets_counter % 1000;
  navbox5.prev_value = navbox5.value;
  navbox5.timestamp  = millis();

  memcpy(navbox6.title, NAVBOX6_TITLE, strlen(NAVBOX6_TITLE));
  navbox6.x = navbox5.width;
  navbox6.y = navbox5.y;
  navbox6.width  = navbox5.width;
  navbox6.height = navbox5.height;
  navbox6.value      = tx_packets_counter % 1000;
  navbox6.prev_value = navbox6.value;
  navbox6.timestamp  = millis();
}

static void EPD_Draw_NavBoxes()
{
  char buf[16];
  uint32_t disp_value;

  int16_t  tbx, tby;
  uint16_t tbw, tbh;

  if (!EPD_ready_to_display) {

    uint16_t top_navboxes_x = navbox1.x;
    uint16_t top_navboxes_y = navbox1.y;
    uint16_t top_navboxes_w = navbox1.width + navbox2.width;
    uint16_t top_navboxes_h = maxof2(navbox1.height, navbox2.height);

    display->drawRoundRect( navbox1.x + 1, navbox1.y + 1,
                            navbox1.width - 2, navbox1.height - 2,
                            4, GxEPD_BLACK);

    display->drawRoundRect( navbox2.x + 1, navbox2.y + 1,
                            navbox2.width - 2, navbox2.height - 2,
                            4, GxEPD_BLACK);

    display->setFont(&FreeMono9pt7b);

    display->getTextBounds(navbox1.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox1.x + 5, navbox1.y + 5 + tbh);
    display->print(navbox1.title);

    display->getTextBounds(navbox2.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox2.x + 5, navbox2.y + 5 + tbh);
    display->print(navbox2.title);

    display->setFont(&FreeMonoBold18pt7b);

    display->setCursor(navbox1.x + 25, navbox1.y + 52);
    display->print(navbox1.value);

    display->setCursor(navbox2.x + 15, navbox2.y + 52);
    display->print((float) navbox2.value / 10, 1);

    uint16_t middle_navboxes_x = navbox3.x;
    uint16_t middle_navboxes_y = navbox3.y;
    uint16_t middle_navboxes_w = navbox3.width + navbox4.width;
    uint16_t middle_navboxes_h = maxof2(navbox3.height, navbox4.height);

    display->drawRoundRect( navbox3.x + 1, navbox3.y + 1,
                            navbox3.width - 2, navbox3.height - 2,
                            4, GxEPD_BLACK);
    display->drawRoundRect( navbox4.x + 1, navbox4.y + 1,
                            navbox4.width - 2, navbox4.height - 2,
                            4, GxEPD_BLACK);

    display->setFont(&FreeMono9pt7b);

    display->getTextBounds(navbox3.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox3.x + 5, navbox3.y + 5 + tbh);
    display->print(navbox3.title);

    display->getTextBounds(navbox4.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox4.x + 5, navbox4.y + 5 + tbh);
    display->print(navbox4.title);

    display->setFont(&FreeSerifBold12pt7b);

    display->setCursor(navbox3.x + 5, navbox3.y + 50);

    snprintf(buf, sizeof(buf), "%06X", navbox3.value);

    display->print(buf);

    display->setFont(&FreeMonoBold18pt7b);

    display->setCursor(navbox4.x + 15, navbox4.y + 50);
    display->print(EPD_Protocol_ID[navbox4.value]);

    uint16_t bottom_navboxes_x = navbox5.x;
    uint16_t bottom_navboxes_y = navbox5.y;
    uint16_t bottom_navboxes_w = navbox5.width + navbox4.width;
    uint16_t bottom_navboxes_h = maxof2(navbox5.height, navbox6.height);

    display->drawRoundRect( navbox5.x + 1, navbox5.y + 1,
                            navbox5.width - 2, navbox5.height - 2,
                            4, GxEPD_BLACK);
    display->drawRoundRect( navbox6.x + 1, navbox6.y + 1,
                            navbox6.width - 2, navbox6.height - 2,
                            4, GxEPD_BLACK);

    display->setFont(&FreeMono9pt7b);

    display->getTextBounds(navbox5.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox5.x + 5, navbox5.y + 5 + tbh);
    display->print(navbox5.title);

    display->getTextBounds(navbox6.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox6.x + 5, navbox6.y + 5 + tbh);
    display->print(navbox6.title);

    display->setFont(&FreeMonoBold18pt7b);

    display->setCursor(navbox5.x + 25, navbox5.y + 50);
    display->print(navbox5.value);

    display->setCursor(navbox6.x + 25, navbox6.y + 50);
    if (settings->mode        == SOFTRF_MODE_RECEIVER ||
        settings->rf_protocol == RF_PROTOCOL_ADSB_UAT ||
        settings->txpower     == RF_TX_POWER_OFF) {
      display->print("OFF");
    } else {
      display->print(navbox6.value);
    }

    /* a signal to background EPD update task */
    EPD_ready_to_display = true;
  }
}

static void EPD_Update_NavBoxes()
{
  char buf[16];
  int16_t  tbx, tby;
  uint16_t tbw, tbh;

  if (!EPD_ready_to_display) {

    bool updated = false;

    if (navbox1.value != navbox1.prev_value) {

      display->setFont(&FreeMonoBold18pt7b);
      display->getTextBounds("00", 0, 0, &tbx, &tby, &tbw, &tbh);

      display->fillRect(navbox1.x + 25, navbox1.y + 53 - tbh,
                        tbw, tbh + 1, GxEPD_WHITE);
      display->setCursor(navbox1.x + 25, navbox1.y + 52);
      display->print(navbox1.value);

      navbox1.prev_value = navbox1.value;
      updated = true;
    }

    if (navbox2.value != navbox2.prev_value) {

      display->setFont(&FreeMonoBold18pt7b);
      display->getTextBounds("0.0", 0, 0, &tbx, &tby, &tbw, &tbh);

      display->fillRect(navbox2.x + 15, navbox2.y + 53 - tbh,
                        tbw + 5, tbh + 1, GxEPD_WHITE);
      display->setCursor(navbox2.x + 15, navbox2.y + 52);
      display->print((float) navbox2.value / 10, 1);

      navbox2.prev_value = navbox2.value;
      updated = true;
    }

    if (navbox3.value != navbox3.prev_value) {

      display->setFont(&FreeSerifBold12pt7b);
      display->getTextBounds("FFFFFF", 0, 0, &tbx, &tby, &tbw, &tbh);

      display->fillRect(navbox3.x + 5, navbox3.y + 51 - tbh,
                        tbw, tbh + 1, GxEPD_WHITE);
      display->setCursor(navbox3.x + 5, navbox3.y + 50);

      snprintf(buf, sizeof(buf), "%06X", navbox3.value);

      display->print(buf);

      navbox3.prev_value = navbox3.value;
      updated = true;
    }

    if (navbox4.value != navbox4.prev_value) {

      display->setFont(&FreeMonoBold18pt7b);
      display->getTextBounds("UUU", 0, 0, &tbx, &tby, &tbw, &tbh);

      display->fillRect (navbox4.x + 15, navbox4.y + 51 - tbh,
                         tbw, tbh + 1, GxEPD_WHITE);
      display->setCursor(navbox4.x + 15, navbox4.y + 50);
      display->print(EPD_Protocol_ID[navbox4.value]);

      navbox4.prev_value = navbox4.value;
      updated = true;
    }

    if (navbox5.value != navbox5.prev_value) {

      display->setFont(&FreeMonoBold18pt7b);
      display->getTextBounds("000", 0, 0, &tbx, &tby, &tbw, &tbh);

      display->fillRect (navbox5.x + 25, navbox5.y + 51 - tbh,
                         tbw + 5, tbh + 1, GxEPD_WHITE);
      display->setCursor(navbox5.x + 25, navbox5.y + 50);
      display->print(navbox5.value);

      navbox5.prev_value = navbox5.value;
      updated = true;
    }

    if (navbox6.value != navbox6.prev_value) {

      display->setFont(&FreeMonoBold18pt7b);
      display->getTextBounds("000", 0, 0, &tbx, &tby, &tbw, &tbh);

      display->fillRect(navbox6.x + 25, navbox6.y + 51 - tbh,
                        tbw + 5, tbh + 1, GxEPD_WHITE);
      display->setCursor(navbox6.x + 25, navbox6.y + 50);
      display->print(navbox6.value);

      navbox6.prev_value = navbox6.value;
      updated = true;
    }

    /* a signal to background EPD update task */
    if (updated) EPD_ready_to_display = true;
  }
}

void EPD_status_loop()
{
  if (EPD_vmode_updated) {

    EPD_Clear_Screen();

    yield();

    EPD_Draw_NavBoxes();

    EPD_vmode_updated = false;

  } else {

    navbox1.value = Traffic_Count();
    navbox2.value = (int) (Battery_voltage() * 10.0);
    navbox5.value      = rx_packets_counter % 1000;
    navbox6.value      = tx_packets_counter % 1000;

    EPD_Update_NavBoxes();
  }
}

void EPD_status_next()
{

}

void EPD_status_prev()
{

}

#endif /* USE_EPAPER */
