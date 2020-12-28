/*
 * TFTHelper.cpp
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

#if defined(EXCLUDE_TFT)
byte TFT_setup()                  {return DISPLAY_NONE;}
void TFT_loop()                   {}
void TFT_fini(const char *msg)    {}
#else

#include <SPI.h>
#include <TFT_eSPI.h>

#include <FT5206.h>

#include "TFTHelper.h"
#include "EEPROMHelper.h"
#include "TrafficHelper.h"
#include "BaroHelper.h"

TFT_eSPI *tft = NULL;
TFT_eSprite *sprite = NULL;
static FT5206_Class *tp = NULL;

static unsigned long TFTTimeMarker = 0;

static int TFT_view_mode = 0;
bool TFT_vmode_updated = true;

static Gesture_t gesture = { false, {0,0}, {0,0} };

const char SoftRF_text[]   = "SoftRF";

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

void TFT_Clear_Screen()
{
  tft->fillScreen(TFT_NAVY);
}

byte TFT_setup()
{
  byte rval = DISPLAY_NONE;

  TFT_view_mode = settings->m.vmode;

  if (hw_info.model == SOFTRF_MODEL_SKYWATCH) {

    SPI.begin(SOC_GPIO_PIN_TWATCH_TFT_SCK, SOC_GPIO_PIN_TWATCH_TFT_MISO,
              SOC_GPIO_PIN_TWATCH_TFT_MOSI, -1);

    tft = new TFT_eSPI(LV_HOR_RES, LV_VER_RES);
    tft->init();
    tft->setRotation(0);

    sprite = new TFT_eSprite(tft);
    sprite->setColorDepth(1);

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

    TFT_status_setup();
    TFT_radar_setup();
    TFT_text_setup();
    TFT_time_setup();
  }

  return rval;
}

void TFT_loop()
{
  switch (hw_info.display)
  {
  case DISPLAY_TFT_TTGO:
    if (tft) {

      if (isTimeToDisplay()) {
        switch (TFT_view_mode)
        {
        case VIEW_MODE_RADAR:
          TFT_radar_loop();
          break;
        case VIEW_MODE_TEXT:
          TFT_text_loop();
          break;
        case VIEW_MODE_TIME:
          TFT_time_loop();
          break;
        case VIEW_MODE_STATUS:
        default:
          TFT_status_loop();
          break;
        }

        TFTTimeMarker = millis();
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

      if (tp) {
        int tp_action = NO_GESTURE;

        if (tp->touched()) {
            TP_Point p =  tp->getPoint();
            p.x = map(p.x, 0, TP_MAX_X, 0, LV_HOR_RES);
            p.y = map(p.y, 0, TP_MAX_Y, 0, LV_VER_RES);

            if (gesture.touched) {
              gesture.d_loc = p;
            } else {
              gesture.t_loc = p; gesture.d_loc = p;
              gesture.touched = true;
            }
        } else {
            if (gesture.touched) {
              int16_t threshold_x = tft->width() / 10;
              int16_t threshold_y = tft->height() / 10;
              int16_t limit_xl = tft->width()/2 - threshold_x;
              int16_t limit_xr = tft->width()/2 + threshold_x;
              int16_t limit_yt = tft->height()/2 - threshold_y;
              int16_t limit_yb = tft->height()/2 + threshold_y;

              if (gesture.d_loc.x < limit_xl && gesture.t_loc.x > limit_xr) {
                tp_action = SWIPE_LEFT;
              } else if (gesture.d_loc.x > limit_xr && gesture.t_loc.x < limit_xl) {
                tp_action = SWIPE_RIGHT;
              } else if (gesture.d_loc.y > limit_yb && gesture.t_loc.y < limit_yt) {
                tp_action = SWIPE_DOWN;
              } else if (gesture.d_loc.y < limit_yt && gesture.t_loc.y > limit_yb) {
                tp_action = SWIPE_UP;
              }

              gesture.touched = false;
              gesture.t_loc = gesture.d_loc = {0,0};
            } else {
               /* TBD */
            }
        }

        switch (tp_action)
        {
        case SWIPE_LEFT:
          if (TFT_view_mode < VIEW_MODE_TIME) {
            TFT_view_mode++;
            TFT_vmode_updated = true;
          }
          break;
        case SWIPE_RIGHT:
          if (TFT_view_mode > VIEW_MODE_STATUS) {
            TFT_view_mode--;
            TFT_vmode_updated = true;
          }
          break;
        case SWIPE_DOWN:
          TFT_Up();
          break;
        case SWIPE_UP:
          TFT_Down();
          break;
        case NO_GESTURE:
        default:
          break;
        }
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

void TFT_Up()
{
  if (hw_info.display == DISPLAY_TFT_TTGO) {
    switch (TFT_view_mode)
    {
    case VIEW_MODE_RADAR:
      TFT_radar_unzoom();
      break;
    case VIEW_MODE_TEXT:
      TFT_text_prev();
      break;
    case VIEW_MODE_TIME:
      TFT_time_prev();
      break;
    case VIEW_MODE_STATUS:
    default:
      TFT_status_prev();
      break;
    }
  }
}

void TFT_Down()
{
  if (hw_info.display == DISPLAY_TFT_TTGO) {
    switch (TFT_view_mode)
    {
    case VIEW_MODE_RADAR:
      TFT_radar_zoom();
      break;
    case VIEW_MODE_TEXT:
      TFT_text_next();
      break;
    case VIEW_MODE_TIME:
      TFT_time_next();
      break;
    case VIEW_MODE_STATUS:
    default:
      TFT_status_next();
      break;
    }
  }
}

void TFT_Message(const char *msg1, const char *msg2)
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;
  uint16_t x, y;

  if (msg1 != NULL && strlen(msg1) != 0) {
    tft->setTextFont(4);
    tft->setTextSize(2);

    tft->fillScreen(TFT_NAVY);

    tbw = tft->textWidth(msg1);
    tbh = tft->fontHeight();
    x = (tft->width() - tbw) / 2;
    y = msg2 == NULL ? (tft->height() - tbh) / 2 : tft->height() / 2 - tbh;
    tft->setCursor(x, y);
    tft->print(msg1);

    if (msg2 != NULL && strlen(msg2) != 0) {
      tbw = tft->textWidth(msg2);
      x = (tft->width() - tbw) / 2;
      y = tft->height() / 2;
      tft->setCursor(x, y);
      tft->print(msg2);
    }
  }
}
#endif /* EXCLUDE_TFT */
