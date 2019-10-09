/*
 * View_Status_TFT.cpp
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

#include "TFTHelper.h"
#include <protocol.h>

#include "SkyWatch.h"

static uint32_t prev_tx_packets_counter = 0;
static uint32_t prev_rx_packets_counter = 0;
static uint8_t  prev_protocol = 0;
static uint32_t prev_addr = 0;

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

void TFT_status_setup()
{

}

void TFT_status_loop()
{
  char buf[16];
  uint32_t disp_value;

  uint16_t tbw;
  uint16_t tbh;

  if (!TFT_display_frontpage) {
    tft->fillScreen(TFT_NAVY);

    tft->setTextFont(2);
    tft->setTextSize(2);
    tft->setTextColor(TFT_WHITE, TFT_NAVY);

    tbw = tft->textWidth(ID_text);
    tbh = tft->fontHeight();

    tft->setCursor(tft->textWidth(" "), tft->height()/6 - tbh);
    tft->print(ID_text);

    tbw = tft->textWidth(PROTOCOL_text);

    tft->setCursor(tft->width() - tbw - tft->textWidth(" "),
                   tft->height()/6 - tbh);
    tft->print(PROTOCOL_text);

    tbw = tft->textWidth(RX_text);
    tbh = tft->fontHeight();

    tft->setCursor(tft->textWidth("   "), tft->height()/2 - tbh);
    tft->print(RX_text);

    tbw = tft->textWidth(TX_text);

    tft->setCursor(tft->width()/2 + tft->textWidth("   "),
                   tft->height()/2 - tbh);
    tft->print(TX_text);

    tft->setTextFont(4);
    tft->setTextSize(2);

    itoa(ThisDevice.addr & 0xFFFFFF, buf, 16);

    tbw = tft->textWidth(buf);
    tbh = tft->fontHeight();

    tft->setCursor(tft->textWidth(" "), tft->height()/6);
    tft->print(buf);
    prev_addr = ThisDevice.addr;

    tbw = tft->textWidth(TFT_Protocol_ID[ThisDevice.protocol]);

    tft->setCursor(tft->width() - tbw - tft->textWidth(" "),
                   tft->height()/6);
    tft->print(TFT_Protocol_ID[ThisDevice.protocol]);
    prev_protocol = ThisDevice.protocol;

    itoa(rx_packets_counter % 1000, buf, 10);
    tft->setCursor(tft->textWidth(" "), tft->height()/2);
    tft->print(buf);
    prev_rx_packets_counter = rx_packets_counter;

    itoa(tx_packets_counter % 1000, buf, 10);
    tft->setCursor(tft->width()/2 + tft->textWidth(" "), tft->height()/2);
    tft->print(buf);
    prev_tx_packets_counter = tx_packets_counter;

    TFT_display_frontpage = true;

  } else { /* TFT_display_frontpage */

    if (ThisDevice.addr != prev_addr) {
      itoa(ThisDevice.addr & 0xFFFFFF, buf, 16);

      tft->setTextFont(4);
      tft->setTextSize(2);

      tbw = tft->textWidth(buf);
      tbh = tft->fontHeight();

      tft->setCursor(tft->textWidth(" "), tft->height()/6);
      tft->print(buf);
      prev_addr = ThisDevice.addr;
    }

    if (ThisDevice.protocol != prev_protocol) {

      tft->setTextFont(4);
      tft->setTextSize(2);

      tbw = tft->textWidth(TFT_Protocol_ID[ThisDevice.protocol]);

      tft->setCursor(tft->width() - tbw - tft->textWidth(" "),
                     tft->height()/6);
      tft->print(TFT_Protocol_ID[ThisDevice.protocol]);
      prev_protocol = ThisDevice.protocol;
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

      tft->setTextFont(4);
      tft->setTextSize(2);

      tft->setCursor(tft->textWidth(" "), tft->height()/2);
      tft->print(buf);

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

      tft->setTextFont(4);
      tft->setTextSize(2);

      tft->setCursor(tft->width()/2 + tft->textWidth(" "), tft->height()/2);
      tft->print(buf);

      prev_tx_packets_counter = tx_packets_counter;
    }
  }
}

void TFT_status_next()
{

}

void TFT_status_prev()
{

}
