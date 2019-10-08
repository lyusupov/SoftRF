/*
 * TFTHelper.cpp
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

#include <SPI.h>
#include <TFT_eSPI.h>
#include <protocol.h>
#include <FT5206.h>

#include "SoCHelper.h"
#include "TFTHelper.h"
#include "EEPROMHelper.h"
#include "TrafficHelper.h"
#include "BaroHelper.h"

#include "SkyWatch.h"

static TFT_eSPI *tft = NULL;
static FT5206_Class *tp = NULL;

unsigned long TFTTimeMarker = 0;

static bool TFT_display_frontpage = false;
static uint32_t prev_tx_packets_counter = 0;
static uint32_t prev_rx_packets_counter = 0;
static uint8_t  prev_protocol = 0;
static uint32_t prev_addr = 0;

static int FT_view_mode = 0;
extern uint32_t tx_packets_counter, rx_packets_counter;

const char *TFT_Protocol_ID[] = {
  [RF_PROTOCOL_LEGACY]    = "L",
  [RF_PROTOCOL_OGNTP]     = "O",
  [RF_PROTOCOL_P3I]       = "P",
  [RF_PROTOCOL_ADSB_1090] = "A",
  [RF_PROTOCOL_ADSB_UAT]  = "U",
  [RF_PROTOCOL_FANET]     = "F"
};

const char SoftRF_text[]   = "SoftRF";
const char ID_text[]       = "ID";
const char PROTOCOL_text[] = "PROTOCOL";
const char RX_text[]       = "RX";
const char TX_text[]       = "TX";

void TFT_off()
{
    tft->writecommand(TFT_DISPOFF);
    tft->writecommand(TFT_SLPIN);
    if (tp) {
      tp->enterSleepMode();
    }
}

void TFT_sleep()
{
    tft->writecommand(TFT_DISPOFF);
    tft->writecommand(TFT_SLPIN);
    if (tp) {
      tp->enterMonitorMode();
    }
}

void TFT_wakeup()
{
    tft->writecommand(TFT_SLPOUT);
    tft->writecommand(TFT_DISPON);
}

void TFT_backlight_init(void)
{
    ledcAttachPin(SOC_GPIO_PIN_TWATCH_TFT_BL, 1);
    ledcSetup(BACKLIGHT_CHANNEL, 12000, 8);
}

uint8_t TFT_backlight_getLevel()
{
    return ledcRead(BACKLIGHT_CHANNEL);
}

void TFT_backlight_adjust(uint8_t level)
{
    ledcWrite(BACKLIGHT_CHANNEL, level);
}

bool TFT_isBacklightOn()
{
    return (bool)ledcRead(BACKLIGHT_CHANNEL);
}

void TFT_backlight_off()
{
    ledcWrite(BACKLIGHT_CHANNEL, 0);
}

void TFT_backlight_on()
{
    ledcWrite(BACKLIGHT_CHANNEL, 250);
}

byte TFT_setup()
{
  byte rval = DISPLAY_NONE;

  if (hw_info.model == SOFTRF_MODEL_SKYWATCH) {

    SPI.begin(SOC_GPIO_PIN_TWATCH_TFT_SCK, SOC_GPIO_PIN_TWATCH_TFT_MISO,
              SOC_GPIO_PIN_TWATCH_TFT_MOSI, -1);

    tft = new TFT_eSPI(LV_HOR_RES, LV_VER_RES);
    tft->init();
    tft->setRotation(0);

    if (hw_info.baro == BARO_MODULE_NONE) {
      pinMode(SOC_GPIO_PIN_TWATCH_TP_IRQ, INPUT);
      Wire.begin(SOC_GPIO_PIN_TWATCH_TP_SDA, SOC_GPIO_PIN_TWATCH_TP_SCL);
      tp = new FT5206_Class();
      if (! tp->begin(Wire)) {
          Serial.println("Couldn't start FT5206 touchscreen controller");
      }

      bma->begin();
      bma->attachInterrupt();
    }

    tft->fillScreen(TFT_NAVY);

    TFT_backlight_init();

    for (int level = 0; level <= 250; level += 25) {
      TFT_backlight_adjust(level);
      delay(100);
    }

    tft->setTextFont(4);
    tft->setTextSize(2);
    tft->setTextColor(TFT_WHITE, TFT_NAVY);

    uint16_t tbw = tft->textWidth(SoftRF_text);
    uint16_t tbh = tft->fontHeight();
    tft->setCursor((tft->width() - tbw)/2, (tft->height() - tbh)/2);
    tft->println(SoftRF_text);

    rval = DISPLAY_TFT_TTGO;
  }

  return rval;
}

void TFT_loop()
{
  char buf[16];
  uint32_t disp_value;

  uint16_t tbw;
  uint16_t tbh;

  switch (hw_info.display)
  {
  case DISPLAY_TFT_TTGO:
    if (tft) {
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

      bool is_bma_irq = false;

      portENTER_CRITICAL_ISR(&BMA_mutex);
      is_bma_irq = BMA_Irq;
      portEXIT_CRITICAL_ISR(&BMA_mutex);

      if (is_bma_irq) {
        bool  rlst;
        do {
            rlst =  bma->readInterrupt();
        } while (!rlst);

        if (bma->isDoubleClick()) {
          if(TFT_isBacklightOn()) {
            TFT_backlight_off();
          } else {
            TFT_backlight_on();
          }
        }

        portENTER_CRITICAL_ISR(&BMA_mutex);
        BMA_Irq = false;
        portEXIT_CRITICAL_ISR(&BMA_mutex);
      }
    }

    break;

  case DISPLAY_NONE:
  default:
    break;
  }
}

void TFT_fini(const char *msg)
{
  switch (hw_info.display)
  {
  case DISPLAY_TFT_TTGO:
    if (tft) {
        int level;

        for (level = 250; level >= 0; level -= 25) {
          TFT_backlight_adjust(level);
          delay(100);
        }

        tft->fillScreen(TFT_NAVY);

        tft->setTextFont(4);
        tft->setTextSize(2);
        tft->setTextColor(TFT_WHITE, TFT_NAVY);

        uint16_t tbw = tft->textWidth(msg);
        uint16_t tbh = tft->fontHeight();

        tft->setCursor((tft->width() - tbw)/2, (tft->height() - tbh)/2);
        tft->print(msg);

        for (level = 0; level <= 250; level += 25) {
          TFT_backlight_adjust(level);
          delay(100);
        }

        delay(2000);

        for (level = 250; level >= 0; level -= 25) {
          TFT_backlight_adjust(level);
          delay(100);
        }

        TFT_backlight_off();
        TFT_off();
        SPI.end();
    }
    break;

  case DISPLAY_NONE:
  default:
    break;
  }
}
