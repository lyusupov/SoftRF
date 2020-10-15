/*
 * EPDHelper.cpp
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

#include "../system/SoC.h"

#if defined(USE_EPAPER)

#if defined(ARDUINO_ARCH_NRF52)

#include "EPD.h"
#include "LED.h"
#include "RF.h"
#include "Baro.h"

#include <Fonts/FreeMonoBold24pt7b.h>
#include <Fonts/FreeMono18pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>

GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> epd_ttgo_txx(GxEPD2_154_D67(
                                                            SOC_GPIO_PIN_EPD_SS,
                                                            SOC_GPIO_PIN_EPD_DC,
                                                            SOC_GPIO_PIN_EPD_RST,
                                                            SOC_GPIO_PIN_EPD_BUSY));

GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> *display;

const char EPD_SoftRF_text1[] = "SoftRF";
const char EPD_SoftRF_text2[] = "and";
const char EPD_SoftRF_text3[] = "LilyGO";

const char EPD_Radio_text[]   = "RADIO   ";
const char EPD_GNSS_text[]    = "GNSS    ";
const char EPD_Display_text[] = "DISPLAY ";
const char EPD_RTC_text[]     = "RTC     ";
const char EPD_Flash_text[]   = "FLASH   ";
const char EPD_Baro_text[]    = "BARO  ";

unsigned long EPDTimeMarker = 0;

static int EPD_view_mode = 0;
bool EPD_vmode_updated = true;

volatile bool EPD_ready_to_display = false;

void EPD_Clear_Screen()
{
  while (EPD_ready_to_display) delay(100);

  display->setFullWindow();
  display->fillScreen(GxEPD_WHITE);
  display->display(false);

  EPD_POWEROFF;
}

bool EPD_setup(bool splash_screen)
{
  bool rval = false;

  int16_t  tbx1, tby1;
  uint16_t tbw1, tbh1;
  int16_t  tbx2, tby2;
  uint16_t tbw2, tbh2;
  int16_t  tbx3, tby3;
  uint16_t tbw3, tbh3;
  uint16_t x, y;

  display = &epd_ttgo_txx;

  display->init( /* 38400 */ );

  // first update should be full refresh
  display->setRotation(0);
  display->setFont(&FreeMonoBold24pt7b);
  display->setTextColor(GxEPD_BLACK);
  display->setTextWrap(false);

  display->getTextBounds(EPD_SoftRF_text1, 0, 0, &tbx1, &tby1, &tbw1, &tbh1);
  display->getTextBounds(EPD_SoftRF_text2, 0, 0, &tbx2, &tby2, &tbw2, &tbh2);
  display->getTextBounds(EPD_SoftRF_text3, 0, 0, &tbx3, &tby3, &tbw3, &tbh3);

  display->setFullWindow();

  {
    display->fillScreen(GxEPD_WHITE);

    if (hw_info.model == SOFTRF_MODEL_BADGE) {
      x = (display->width() - tbw1) / 2;
      y = (display->height() + tbh1) / 2 - tbh3;
      display->setCursor(x, y);
      display->print(EPD_SoftRF_text1);
      x = (display->width() - tbw2) / 2;
      y = (display->height() + tbh2) / 2;
      display->setCursor(x, y);
      display->print(EPD_SoftRF_text2);
      x = (display->width() - tbw3) / 2;
      y = (display->height() + tbh3) / 2 + tbh3;
      display->setCursor(x, y);
      display->print(EPD_SoftRF_text3);
    } else {
      x = (display->width() - tbw1) / 2;
      y = (display->height() + tbh1) / 2;
      display->setCursor(x, y);
      display->print(EPD_SoftRF_text1);
    }
  }

  display->display(false);

  EPD_POWEROFF;

  rval = display->epd2.probe();

  EPD_view_mode = ui->vmode;;

  EPD_status_setup();
  EPD_radar_setup();
  EPD_text_setup();
  EPD_time_setup();

  EPDTimeMarker = millis();

  return rval;
}

void EPD_info1(bool rtc, bool spiflash)
{
  switch (hw_info.display)
  {
  case DISPLAY_EPD_1_54:
    int16_t  tbx, tby;
    uint16_t tbw, tbh;

    uint16_t x, y;

    /* EPD back light on */
    digitalWrite(SOC_GPIO_PIN_EPD_BLGT, HIGH);

    display->setFont(&FreeMonoBold18pt7b);
    display->getTextBounds(EPD_Radio_text, 0, 0, &tbx, &tby, &tbw, &tbh);

    display->setFullWindow();

    {
      display->fillScreen(GxEPD_WHITE);

      x = 0;
      y = (tbh + INFO_1_LINE_SPACING);

      display->setCursor(x, y);
      display->print(EPD_Radio_text);
      display->print(hw_info.rf != RF_IC_NONE ? "+" : "-");

      y += (tbh + INFO_1_LINE_SPACING);

      display->setCursor(x, y);
      display->print(EPD_GNSS_text);
      display->print(hw_info.gnss != GNSS_MODULE_NONE ? "+" : "-");

      y += (tbh + INFO_1_LINE_SPACING);

      display->setCursor(x, y);
      display->print(EPD_Display_text);
      display->print(hw_info.display != DISPLAY_NONE ? "+" : "-");

      y += (tbh + INFO_1_LINE_SPACING);

      display->setCursor(x, y);
      display->print(EPD_RTC_text);
      display->print(rtc ? "+" : "-");

      y += (tbh + INFO_1_LINE_SPACING);

      display->setCursor(x, y);
      display->print(EPD_Flash_text);
      display->print(spiflash ? "+" : "-");

      y += (tbh + INFO_1_LINE_SPACING);
      y += (tbh + INFO_1_LINE_SPACING);

      if (hw_info.baro == BARO_MODULE_NONE) {
        display->setFont(&FreeMono18pt7b);
      }

      display->setCursor(x, y);
      display->print(EPD_Baro_text);
      display->print(hw_info.baro != BARO_MODULE_NONE ? "  +" : "N/A");
    }

    display->display(false);

    EPD_POWEROFF;

    delay(4000);

    /* EPD back light off */
    digitalWrite(SOC_GPIO_PIN_EPD_BLGT, LOW);

  case DISPLAY_NONE:
  default:
    break;
  }
}

void EPD_loop()
{
  switch (hw_info.display)
  {
  case DISPLAY_EPD_1_54:

    if (isTimeToDisplay()) {
      switch (EPD_view_mode)
      {
      case VIEW_MODE_RADAR:
        EPD_radar_loop();
        break;
      case VIEW_MODE_TEXT:
        EPD_text_loop();
        break;
      case VIEW_MODE_TIME:
        EPD_time_loop();
        break;
      case VIEW_MODE_STATUS:
      default:
        EPD_status_loop();
        break;
      }

      EPDTimeMarker = millis();
    }
    break;

  case DISPLAY_NONE:
  default:
    break;
  }
}

void EPD_fini(const char *msg)
{
  while (EPD_ready_to_display) delay(100);

  EPD_Message(msg, NULL);

  while (EPD_ready_to_display) delay(100);

  EPD_HIBERNATE;
}

void EPD_Mode()
{
  if (hw_info.display == DISPLAY_EPD_1_54) {
    if (EPD_view_mode == VIEW_MODE_STATUS) {
      EPD_view_mode = VIEW_MODE_RADAR;
      EPD_vmode_updated = true;
    }  else if (EPD_view_mode == VIEW_MODE_RADAR) {
      EPD_view_mode = VIEW_MODE_TEXT;
      EPD_vmode_updated = true;
    }  else if (EPD_view_mode == VIEW_MODE_TEXT) {
      EPD_view_mode = VIEW_MODE_TIME;
      EPD_vmode_updated = true;
    }  else if (EPD_view_mode == VIEW_MODE_TIME) {
      EPD_view_mode = VIEW_MODE_STATUS;
      EPD_vmode_updated = true;
    }
  }
}

void EPD_Up()
{
  if (hw_info.display == DISPLAY_EPD_1_54) {
    switch (EPD_view_mode)
    {
    case VIEW_MODE_RADAR:
      EPD_radar_unzoom();
      break;
    case VIEW_MODE_TEXT:
      EPD_text_prev();
      break;
    case VIEW_MODE_TIME:
      EPD_time_prev();
      break;
    case VIEW_MODE_STATUS:
    default:
      EPD_status_prev();
      break;
    }
  }
}

void EPD_Down()
{
  if (hw_info.display == DISPLAY_EPD_1_54) {
    switch (EPD_view_mode)
    {
    case VIEW_MODE_RADAR:
      EPD_radar_zoom();
      break;
    case VIEW_MODE_TEXT:
      EPD_text_next();
      break;
    case VIEW_MODE_TIME:
      EPD_time_next();
      break;
    case VIEW_MODE_STATUS:
    default:
      EPD_status_next();
      break;
    }
  }
}

void EPD_Message(const char *msg1, const char *msg2)
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;
  uint16_t x, y;

  if (msg1 != NULL && strlen(msg1) != 0 && !EPD_ready_to_display) {

    display->setPartialWindow(0, 0, display->width(), display->height());

    display->setFont(&FreeMonoBold18pt7b);

    {
      display->fillScreen(GxEPD_WHITE);

      if (msg2 == NULL) {

        display->getTextBounds(msg1, 0, 0, &tbx, &tby, &tbw, &tbh);
        x = (display->width() - tbw) / 2;
        y = (display->height() + tbh) / 2;
        display->setCursor(x, y);
        display->print(msg1);

      } else {

        display->getTextBounds(msg1, 0, 0, &tbx, &tby, &tbw, &tbh);
        x = (display->width() - tbw) / 2;
        y = display->height() / 2 - tbh;
        display->setCursor(x, y);
        display->print(msg1);

        display->getTextBounds(msg2, 0, 0, &tbx, &tby, &tbw, &tbh);
        x = (display->width() - tbw) / 2;
        y = display->height() / 2 + tbh;
        display->setCursor(x, y);
        display->print(msg2);
      }
    }

    /* a signal to background EPD update task */
    EPD_ready_to_display = true;
  }
}

void EPD_Task( void * pvParameters )
{
  for( ;; )
  {
    if (EPD_ready_to_display) {

      display->display(true);

      yield();

      EPD_POWEROFF;

      EPD_ready_to_display = false;
    }

    yield();
  }
}

#elif defined(RASPBERRY_PI)

#define ENABLE_GxEPD2_GFX 0

#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold24pt7b.h>

static const uint8_t SS    = 8; //pin 24

GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> display(GxEPD2_270(/*CS=5*/ SS,
                                       /*DC=*/ 25, /*RST=*/ 17, /*BUSY=*/ 24));

const char SoftRF_text[] = "SoftRF";

bool EPD_setup(bool splash_screen)
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;

  display.init();

  // first update should be full refresh
  display.setRotation(0);
  display.setFont(&FreeMonoBold24pt7b);
  display.setTextColor(GxEPD_BLACK);

  display.getTextBounds(SoftRF_text, 0, 0, &tbx, &tby, &tbw, &tbh);
  uint16_t x = (display.width() - tbw) / 2;
  uint16_t y = (display.height() + tbh) / 2;
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, y);
    display.print(SoftRF_text);
  }
  while (display.nextPage());

  delay(1000);

//  display.powerOff();
//  display.hibernate();

  return display.epd2.probe();
}

static bool EPD_display_frontpage = false;

void EPD_loop()
{
  if (!EPD_display_frontpage) {
    int16_t  tbx, tby;
    uint16_t tbw, tbh;

    display.getTextBounds(SoftRF_text, 0, 0, &tbx, &tby, &tbw, &tbh);
    uint16_t x = (display.width() - tbw) / 2;
    uint16_t y = (display.height() + tbh) / 2;
    display.setFullWindow();
//    display.setPartialWindow(x + tbx, y + tby, tbw, tbh);

#if 0
Serial.print("tbx = "); Serial.println(tbx);
Serial.print("tby = "); Serial.println(tby);
Serial.print("tbw = "); Serial.println(tbw);
Serial.print("tbh = "); Serial.println(tbh);
#endif

    display.firstPage();
    do
    {
      display.fillScreen(GxEPD_WHITE);
//      display.fillRect(x + tbx, y + tby, tbw, tbh, GxEPD_WHITE);
    }
    while (display.nextPage());

    uint16_t radar_x = 0;
    uint16_t radar_y = (display.height() - display.width()) / 2;
    uint16_t radar_w = display.width();

    display.setPartialWindow(radar_x, radar_y, radar_w, radar_w);

    uint16_t radar_center_x = radar_w / 2;
    uint16_t radar_center_y = radar_y + radar_w / 2;
    uint16_t radius = radar_w / 2 - 2;

    display.firstPage();
    do
    {
      display.drawCircle(radar_center_x, radar_center_y,
                         radius, GxEPD_BLACK);
      display.drawCircle(radar_center_x, radar_center_y,
                         radius / 2, GxEPD_BLACK);
      display.fillTriangle(radar_center_x - 7, radar_center_y + 5,
                           radar_center_x    , radar_center_y - 5,
                           radar_center_x + 7, radar_center_y + 5,
                           GxEPD_BLACK);
      display.fillTriangle(radar_center_x - 7, radar_center_y + 5,
                           radar_center_x    , radar_center_y + 2,
                           radar_center_x + 7, radar_center_y + 5,
                           GxEPD_WHITE);
    }
    while (display.nextPage());

    uint16_t top_navboxes_x = 0;
    uint16_t top_navboxes_y = 0;
    uint16_t top_navboxes_w = display.width();
    uint16_t top_navboxes_h = radar_y;

    display.setPartialWindow(top_navboxes_x, top_navboxes_y,
                            top_navboxes_w, top_navboxes_h);

    display.firstPage();
    do
    {
      display.drawRoundRect(top_navboxes_x + 2, top_navboxes_y + 2,
                            top_navboxes_w / 2 - 4, top_navboxes_h - 4,
                            4, GxEPD_BLACK);
      display.drawRoundRect(top_navboxes_x + top_navboxes_w / 2 + 2,
                            top_navboxes_y + 2,
                            top_navboxes_w / 2 - 4, top_navboxes_h - 4,
                            4, GxEPD_BLACK);
    }
    while (display.nextPage());

    top_navboxes_y = radar_y + radar_w;

    display.setPartialWindow(top_navboxes_x, top_navboxes_y,
                            top_navboxes_w, top_navboxes_h);

    display.firstPage();
    do
    {
      display.drawRoundRect(top_navboxes_x + 2, top_navboxes_y + 2,
                            top_navboxes_w / 2 - 4, top_navboxes_h - 4,
                            4, GxEPD_BLACK);
      display.drawRoundRect(top_navboxes_x + top_navboxes_w / 2 + 2,
                            top_navboxes_y + 2,
                            top_navboxes_w / 2 - 4, top_navboxes_h - 4,
                            4, GxEPD_BLACK);
    }
    while (display.nextPage());

    display.powerOff();
    EPD_display_frontpage = true;
  }
}

#endif /* RASPBERRY_PI */

#endif /* USE_EPAPER */
