/*
 * View_Status_TFT.cpp
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

#include "SoCHelper.h"

#if !defined(EXCLUDE_TFT)

#include "TFTHelper.h"
#include "TrafficHelper.h"
#include "BatteryHelper.h"
#include <protocol.h>

extern uint32_t tx_packets_counter, rx_packets_counter;

const char *TFT_Protocol_ID[] = {
  [RF_PROTOCOL_LEGACY]    = "L",
  [RF_PROTOCOL_OGNTP]     = "O",
  [RF_PROTOCOL_P3I]       = "P",
  [RF_PROTOCOL_ADSB_1090] = "A",
  [RF_PROTOCOL_ADSB_UAT]  = "U",
  [RF_PROTOCOL_FANET]     = "F"
};

const char ID_text[]       = "ID";
const char PROTOCOL_text[] = "PROTOCOL";
const char RX_text[]       = "RX";
const char TX_text[]       = "TX";
const char ACFTS_text[]    = "ACFTS";
const char BAT_text[]      = "BAT";

void TFT_status_setup()
{

}

void TFT_status_loop()
{
  char buf[16];
  uint32_t disp_value;

  uint16_t tbw;
  uint16_t tbh;

  sprite->createSprite(tft->width(), tft->height());

  sprite->fillSprite(TFT_BLACK);
  sprite->setTextColor(TFT_WHITE);

  sprite->setTextFont(2);
  sprite->setTextSize(2);

  tbw = sprite->textWidth(ID_text);
  tbh = sprite->fontHeight();

  sprite->setCursor(sprite->textWidth(" "), sprite->height()/6 - tbh);
  sprite->print(ID_text);

  tbw = sprite->textWidth(PROTOCOL_text);

  sprite->setCursor(sprite->width() - tbw - sprite->textWidth(" "),
                    sprite->height()/6 - tbh);
  sprite->print(PROTOCOL_text);

  tbw = sprite->textWidth(RX_text);

  sprite->setCursor(sprite->textWidth("   "), sprite->height()/2 - tbh);
  sprite->print(RX_text);

  tbw = sprite->textWidth(TX_text);

  sprite->setCursor(sprite->width()/2 + sprite->textWidth("   "),
                    sprite->height()/2 - tbh);
  sprite->print(TX_text);

  tbw = sprite->textWidth(ACFTS_text);

  sprite->setCursor(sprite->textWidth(" "), (5 * sprite->height()/6) - tbh);
  sprite->print(ACFTS_text);

  tbw = sprite->textWidth(BAT_text);

  sprite->setCursor(sprite->width()/2 + sprite->textWidth("   "),
                    (5 * sprite->height()/6) - tbh);
  sprite->print(BAT_text);

  itoa(ThisDevice.addr & 0xFFFFFF, buf, 16);

  sprite->setTextFont(4);
  sprite->setTextSize(2);

  sprite->setCursor(sprite->textWidth(" "), sprite->height()/6);
  sprite->print(buf);

  tbw = sprite->textWidth(TFT_Protocol_ID[ThisDevice.protocol]);

  sprite->setCursor(sprite->width() - tbw - sprite->textWidth(" "),
                    sprite->height()/6);
  sprite->print(TFT_Protocol_ID[ThisDevice.protocol]);


  disp_value = rx_packets_counter % 1000;
  itoa(disp_value, buf, 10);

  if (disp_value < 10) {
    strcat_P(buf,PSTR("  "));
  } else {
    if (disp_value < 100) {
      strcat_P(buf,PSTR(" "));
    };
  }

  sprite->setCursor(sprite->textWidth(" "), sprite->height()/2);
  sprite->print(buf);

  disp_value = tx_packets_counter % 1000;
  itoa(disp_value, buf, 10);

  if (disp_value < 10) {
    strcat_P(buf,PSTR("  "));
  } else {
    if (disp_value < 100) {
      strcat_P(buf,PSTR(" "));
    };
  }

  sprite->setCursor(sprite->width()/2 + sprite->textWidth(" "), sprite->height()/2);
  sprite->print(buf);

  sprite->setCursor(sprite->textWidth("  "), (5 * sprite->height())/6);
  sprite->print(Traffic_Count());

  sprite->setCursor(sprite->width()/2 + sprite->textWidth("  "), (5 * sprite->height())/6);
  sprite->print(Battery_voltage(), 1);

  tft->setBitmapColor(TFT_WHITE, TFT_NAVY);
  sprite->pushSprite(0, 0);
  sprite->deleteSprite();
}

void TFT_status_next()
{

}

void TFT_status_prev()
{

}
#endif /* EXCLUDE_TFT */
