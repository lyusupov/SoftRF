/*
 * EPDHelper.cpp
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

#if defined(USE_EPAPER)

#include "EPD.h"
#include "LED.h"
#include "RF.h"
#include "Baro.h"
#include "../TrafficHelper.h"
#include "../system/Time.h"

#include <Fonts/FreeMonoBold24pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMono18pt7b.h>

#include <Fonts/Org_01.h>
#include <Fonts/FreeMonoBoldOblique9pt7b.h>
#include <Fonts/FreeSerif9pt7b.h>

const char EPD_SoftRF_text1[] = "SoftRF";
const char EPD_SoftRF_text2[] =  "and"  ;
const char EPD_SoftRF_text3[] = "LilyGO";
const char EPD_SoftRF_text4[] = "Author: ";
const char EPD_SoftRF_text5[] = "Linar Yusupov";
const char EPD_SoftRF_text6[] = "(C) 2016-2022";


const char EPD_Radio_text[]   = "RADIO   ";
const char EPD_GNSS_text[]    = "GNSS    ";
const char EPD_Display_text[] = "DISPLAY ";
const char EPD_RTC_text[]     = "RTC     ";
const char EPD_Flash_text[]   = "FLASH   ";
const char EPD_Baro_text[]    = "BARO  ";
const char EPD_IMU_text[]     = "IMU   ";

unsigned long EPDTimeMarker = 0;
static unsigned long EPD_anti_ghosting_timer = 0;
static uint8_t anti_ghosting_minutes = 0;

static int EPD_view_mode = 0;
bool EPD_vmode_updated = true;
uint16_t EPD_pages_mask = (1 << VIEW_MODE_STATUS) |
                          (1 << VIEW_MODE_RADAR ) |
                          (1 << VIEW_MODE_TEXT  ) |
                          (1 << VIEW_MODE_TIME  );

volatile uint8_t EPD_update_in_progress = EPD_UPDATE_NONE;

bool EPD_setup(bool splash_screen)
{
  bool rval = false;

  int16_t  tbx1, tby1;
  uint16_t tbw1, tbh1;
  int16_t  tbx2, tby2;
  uint16_t tbw2, tbh2;
  int16_t  tbx3, tby3;
  uint16_t tbw3, tbh3;
  int16_t  tbx4, tby4;
  uint16_t tbw4, tbh4;
  uint16_t x, y;

  display->init( /* 38400 */ );

  display->setRotation((3 + ui->rotate) & 0x3); /* 270 deg. is default angle */
  display->setTextColor(GxEPD_BLACK);
  display->setTextWrap(false);

  display->setFont(&FreeMonoBold24pt7b);
  display->getTextBounds(EPD_SoftRF_text1, 0, 0, &tbx1, &tby1, &tbw1, &tbh1);
  display->getTextBounds(EPD_SoftRF_text3, 0, 0, &tbx3, &tby3, &tbw3, &tbh3);

  display->setFullWindow();

  display->fillScreen(GxEPD_WHITE);

  if (hw_info.model == SOFTRF_MODEL_BADGE) {

    x = (display->width()  - tbw1) / 2;
    y = (display->height() + tbh1) / 2 - tbh3;
    display->setCursor(x, y);
    display->print(EPD_SoftRF_text1);

    display->setFont(&FreeMono18pt7b);
    display->getTextBounds(EPD_SoftRF_text2, 0, 0, &tbx2, &tby2, &tbw2, &tbh2);

    x = (display->width()  - tbw2) / 2;
    y = (display->height() + tbh2) / 2;
    display->setCursor(x, y);
    display->print(EPD_SoftRF_text2);

    display->setFont(&FreeMonoBold24pt7b);

    x = (display->width()  - tbw3) / 2;
    y = (display->height() + tbh3) / 2 + tbh3;
    display->setCursor(x, y);
    display->print(EPD_SoftRF_text3);

    char buf[32];
    snprintf(buf, sizeof(buf), "HW: %s SW: %s", hw_info.revision > 2 ?
                  Hardware_Rev[3] : Hardware_Rev[hw_info.revision],
                  SOFTRF_FIRMWARE_VERSION);

    display->setFont(&Org_01);
    display->getTextBounds(buf, 0, 0, &tbx4, &tby4, &tbw4, &tbh4);
    x = (display->width() - tbw4) / 2;
    y = display->height() - tbh4;
    display->setCursor(x, y);
    display->print(buf);

  } else {
    x = (display->width()  - tbw1) / 2;
    y = (display->height() + tbh1) / 2;
    display->setCursor(x, y);
    display->print(EPD_SoftRF_text1);
  }

  // first update should be full refresh
  display->display(false);

  EPD_POWEROFF;

  rval = display->probe();

  EPD_status_setup();
  EPD_radar_setup();
  EPD_text_setup();
  EPD_baro_setup();
  EPD_time_setup();
  EPD_imu_setup();

  EPD_view_mode = ui->vmode;
  if (EPD_pages_mask & (1 << EPD_view_mode) == 0) {
    for (int i=0; i < VIEW_MODES_COUNT; i++) {
      int next_view_mode = (EPD_view_mode + i) % VIEW_MODES_COUNT;
      if ((next_view_mode != EPD_view_mode) &&
          (EPD_pages_mask & (1 << next_view_mode))) {
        EPD_view_mode = next_view_mode;
        break;
      }
    }
  }

  switch (ui->aghost)
  {
    case ANTI_GHOSTING_2MIN:
      anti_ghosting_minutes = 2;
      break;
    case ANTI_GHOSTING_5MIN:
    case ANTI_GHOSTING_AUTO:
      anti_ghosting_minutes = 5;
      break;
    case ANTI_GHOSTING_10MIN:
      anti_ghosting_minutes = 10;
      break;
    case ANTI_GHOSTING_OFF:
    default:
      break;
  }

  SoC->ADB_ops && SoC->ADB_ops->setup();

  EPDTimeMarker = millis();
  EPD_anti_ghosting_timer = millis();

  return rval;
}

void EPD_info1()
{
  switch (hw_info.display)
  {
  case DISPLAY_EPD_1_54:
  case DISPLAY_EPD_2_7:
    int16_t  tbx, tby;
    uint16_t tbw, tbh;

    uint16_t x, y;

#if defined(USE_EPD_TASK)
    while (EPD_update_in_progress != EPD_UPDATE_NONE) delay(100);

//    while (!SoC->Display_lock()) { delay(10); }
#endif
    display->setFont(&FreeMonoBold18pt7b);
    display->getTextBounds(EPD_Radio_text, 0, 0, &tbx, &tby, &tbw, &tbh);

    display->fillScreen(GxEPD_WHITE);

    x = 5;
    y = (tbh + INFO_1_LINE_SPACING - 2);

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

    if (hw_info.model == SOFTRF_MODEL_BADGE) {
      y += (tbh + INFO_1_LINE_SPACING);

      display->setCursor(x, y);
      display->print(EPD_RTC_text);
      display->print(hw_info.rtc != RTC_NONE ? "+" : "-");

      y += (tbh + INFO_1_LINE_SPACING);

      display->setCursor(x, y);
      display->print(EPD_Flash_text);
      display->print(hw_info.storage == STORAGE_FLASH ? "+" : "-");

      y += (tbh + INFO_1_LINE_SPACING);

      if (hw_info.baro == BARO_MODULE_NONE) {
        display->setFont(&FreeMono18pt7b);
      }

      display->setCursor(x, y);
      display->print(EPD_Baro_text);
      display->print(hw_info.baro != BARO_MODULE_NONE ? "  +" : "N/A");

      y += (tbh + INFO_1_LINE_SPACING);

      if (hw_info.imu == IMU_NONE) {
        display->setFont(&FreeMono18pt7b);
      } else {
        display->setFont(&FreeMonoBold18pt7b);
      }

      display->setCursor(x, y);
      display->print(EPD_IMU_text);
      display->print(hw_info.imu != IMU_NONE ? "  +" : "N/A");
    }

#if defined(USE_EPD_TASK)
    EPD_update_in_progress = EPD_UPDATE_SLOW;
    while (EPD_update_in_progress != EPD_UPDATE_NONE) { delay(100); }
//    SoC->Display_unlock();
#else
    display->display(false);
#endif

    delay(4000);

#if 0
    display->fillScreen(GxEPD_WHITE);

    EPD_update_in_progress = EPD_UPDATE_SLOW;
    while (EPD_update_in_progress != EPD_UPDATE_NONE) { delay(100); }
#endif

    break;

  case DISPLAY_NONE:
  default:
    break;
  }
}

void EPD_info2(int acfts, char *reg, char *mam, char *cn)
{
  const char *msg_line;

  switch (hw_info.display)
  {
  case DISPLAY_EPD_1_54:
  case DISPLAY_EPD_2_7:
    int16_t  tbx, tby;
    uint16_t tbw, tbh;

    uint16_t x, y;

#if defined(USE_EPD_TASK)
    while (EPD_update_in_progress != EPD_UPDATE_NONE) delay(100);

//    while (!SoC->Display_lock()) { delay(10); }
#endif

    display->fillScreen(GxEPD_WHITE);

    if (acfts == -1) {

      display->setFont(&FreeMonoBold18pt7b);

      msg_line = "WARNING";

      display->getTextBounds(msg_line, 0, 0, &tbx, &tby, &tbw, &tbh);
      x = (display->width() - tbw) / 2;
      y = display->height() / 3;
      display->setCursor(x, y);
      display->print(msg_line);

      display->setFont(&FreeMono18pt7b);

      msg_line = "NO";

      display->getTextBounds(msg_line, 0, 0, &tbx, &tby, &tbw, &tbh);
      x = (display->width() - tbw) / 2;
      y = (2 * display->height()) / 3 - tbh;
      display->setCursor(x, y);
      display->print(msg_line);

      y += (tbh + INFO_1_LINE_SPACING);

      msg_line = "AIRCRAFTS";

      display->getTextBounds(msg_line, 0, 0, &tbx, &tby, &tbw, &tbh);
      x = (display->width() - tbw) / 2;
      display->setCursor(x, y);
      display->print(msg_line);

      y += (tbh + INFO_1_LINE_SPACING);

      msg_line = "DATA";

      display->getTextBounds(msg_line, 0, 0, &tbx, &tby, &tbw, &tbh);
      x = (display->width() - tbw) / 2;
      display->setCursor(x, y);
      display->print(msg_line);

    } else {

      char buf[TEXT_VIEW_LINE_LENGTH+1];

      snprintf(buf, sizeof(buf), "%d", acfts);

      display->setFont(&FreeMonoBold12pt7b);

      display->getTextBounds(buf, 0, 0, &tbx, &tby, &tbw, &tbh);
      x = (display->width() - tbw) / 2;
      y = display->height() / 4 - tbh;
      display->setCursor(x, y);
      display->print(buf);

      y += (tbh + INFO_1_LINE_SPACING);

      msg_line = "ACFTS LOADED";

      display->getTextBounds(msg_line, 0, 0, &tbx, &tby, &tbw, &tbh);
      x = (display->width() - tbw) / 2;
      display->setCursor(x, y);
      display->print(msg_line);

      y += (tbh + INFO_1_LINE_SPACING);
      y += (tbh + INFO_1_LINE_SPACING);
      y += (tbh + INFO_1_LINE_SPACING);

      msg_line = "THIS AIRCRAFT:";

      display->getTextBounds(msg_line, 0, 0, &tbx, &tby, &tbw, &tbh);
      x = (display->width() - tbw) / 2;
      display->setCursor(x, y);
      display->print(msg_line);

      x = 10;
      y += (tbh + INFO_1_LINE_SPACING);

      snprintf(buf, sizeof(buf), "%s", reg);
      display->setCursor(x, y);
      display->print(buf);

      y += (tbh + INFO_1_LINE_SPACING);

      snprintf(buf, sizeof(buf), "%s", mam);
      display->setCursor(x, y);
      display->print(buf);

      y += (tbh + INFO_1_LINE_SPACING);

      snprintf(buf, sizeof(buf), " CN: %s", cn);
      display->setCursor(x, y);
      display->print(buf);
    }

#if defined(USE_EPD_TASK)
    EPD_update_in_progress = EPD_UPDATE_SLOW;
    while (EPD_update_in_progress != EPD_UPDATE_NONE) { delay(100); }
//    SoC->Display_unlock();
#else
    display->display(false);
#endif

    delay(3000);
    break;

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

    if (EPD_vmode_updated) {
#if defined(USE_EPD_TASK)
      if (EPD_update_in_progress == EPD_UPDATE_NONE) {
//      while (!SoC->Display_lock()) { delay(10); }
#else
      {
#endif
        display->fillScreen(GxEPD_BLACK /* GxEPD_WHITE */);

#if defined(USE_EPD_TASK)
        EPD_update_in_progress = EPD_UPDATE_FAST /* EPD_UPDATE_SLOW */;
        while (EPD_update_in_progress != EPD_UPDATE_NONE) { delay(100); }
//      SoC->Display_unlock();
#else
//        display->display(false);
        display->display(true);
#endif
        EPD_vmode_updated = false;
      }

    } else {
      switch (EPD_view_mode)
      {
      case VIEW_MODE_RADAR:
        EPD_radar_loop();
        break;
      case VIEW_MODE_TEXT:
        EPD_text_loop();
        break;
      case VIEW_MODE_BARO:
        EPD_baro_loop();
        break;
      case VIEW_MODE_TIME:
        EPD_time_loop();
        break;
      case VIEW_MODE_IMU:
        EPD_imu_loop();
        break;
      case VIEW_MODE_STATUS:
      default:
        EPD_status_loop();
        break;
      }

      bool auto_ag_condition = ui->aghost == ANTI_GHOSTING_AUTO  &&
                               (EPD_view_mode == VIEW_MODE_RADAR ||
                                EPD_view_mode == VIEW_MODE_TEXT) ?
                               (Traffic_Count() == 0) : true;

      if (anti_ghosting_minutes > 0                                                &&
          (millis() - EPD_anti_ghosting_timer) > (anti_ghosting_minutes * 60000UL) &&
          auto_ag_condition) {
        EPD_vmode_updated = true;
        EPD_anti_ghosting_timer = millis();
      }
    }
    break;

  case DISPLAY_NONE:
  default:
    break;
  }
}

void EPD_fini(int reason, bool screen_saver)
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;
  uint16_t x, y;

  SoC->ADB_ops && SoC->ADB_ops->fini();

  const char *msg = (reason == SOFTRF_SHUTDOWN_LOWBAT ?
                     "LOW BATTERY" : "NORMAL OFF");

  switch (hw_info.display)
  {
  case DISPLAY_EPD_1_54:
#if defined(USE_EPD_TASK)
      while (EPD_update_in_progress != EPD_UPDATE_NONE) delay(100);
//      while (!SoC->Display_lock()) { delay(10); }
#endif
    if (screen_saver) {
      const char *msg_line;

      display->setFont(&FreeMonoBold12pt7b);
      display->fillScreen(GxEPD_WHITE);

      msg_line = "POWER OFF";

      display->getTextBounds(msg_line, 0, 0, &tbx, &tby, &tbw, &tbh);
      x = (display->width() - tbw) / 2;
      y = display->height() / 3;
      display->setCursor(x, y);
      display->print(msg_line);

      msg_line = "SCREEN SAVER";

      display->getTextBounds(msg_line, 0, 0, &tbx, &tby, &tbw, &tbh);
      x = (display->width() - tbw) / 2;
      y = (2 * display->height()) / 3;
      display->setCursor(x, y);
      display->print(msg_line);

#if defined(USE_EPD_TASK)
      /* a signal to background EPD update task */
      EPD_update_in_progress = EPD_UPDATE_SLOW /* EPD_UPDATE_FAST */;
//      SoC->Display_unlock();

//    yield();

      while (EPD_update_in_progress != EPD_UPDATE_NONE) delay(100);
//      while (!SoC->Display_lock()) { delay(10); }
#else
      display->display(false);
#endif

      SoC->loop(); /* reload WDT */

      delay(4000);

      SoC->loop(); /* reload WDT */

      display->fillScreen(GxEPD_WHITE);

    } else {

      display->fillScreen(GxEPD_WHITE);

      display->setFont(&FreeMonoBold12pt7b);
      display->getTextBounds(msg, 0, 0, &tbx, &tby, &tbw, &tbh);
      x = (display->width() - tbw) / 2;
      y = tbh + tbh / 2;
      display->setCursor(x, y);
      display->print(msg);

      x = (display->width()  - 128) / 2;
      y = (display->height() - 128) / 2 - tbh / 2;
      display->drawBitmap(x, y, sleep_icon_128x128, 128, 128, GxEPD_BLACK);

      display->setFont(&Org_01);
      display->getTextBounds(EPD_SoftRF_text4, 0, 0, &tbx, &tby, &tbw, &tbh);
      x =  5;
      y += 128 + 17;
      display->setCursor(x, y);
      display->print(EPD_SoftRF_text4);

      display->setFont(&FreeMonoBoldOblique9pt7b);
      display->print(EPD_SoftRF_text5);

      display->setFont(&FreeSerif9pt7b);
      display->getTextBounds(EPD_SoftRF_text6, 0, 0, &tbx, &tby, &tbw, &tbh);
      x = (display->width() - tbw) / 2;
      y += 21;
      display->setCursor(x, y);
      display->print(EPD_SoftRF_text6);
    }

#if defined(USE_EPD_TASK)
    /* a signal to background EPD update task */
    EPD_update_in_progress = EPD_UPDATE_SLOW /* EPD_UPDATE_FAST */;
//    SoC->Display_unlock();

//    yield();

    while (EPD_update_in_progress != EPD_UPDATE_NONE) delay(100);
//    while (!SoC->Display_lock()) { delay(10); }
#else
    display->display(false);
#endif

    EPD_HIBERNATE;

//    SoC->Display_unlock();
    break;

  case DISPLAY_NONE:
  default:
    break;
  }
}

void EPD_Mode()
{
  if (hw_info.display == DISPLAY_EPD_1_54) {
    for (int i=0; i < VIEW_MODES_COUNT; i++) {
      int next_view_mode = (EPD_view_mode + i) % VIEW_MODES_COUNT;
      if ((next_view_mode != EPD_view_mode) &&
          (EPD_pages_mask & (1 << next_view_mode))) {
        EPD_view_mode = next_view_mode;
        EPD_vmode_updated = true;
        break;
      }
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
      EPD_text_next();
      break;
    case VIEW_MODE_BARO:
      EPD_baro_next();
      break;
    case VIEW_MODE_TIME:
      EPD_time_next();
      break;
    case VIEW_MODE_IMU:
      EPD_imu_next();
      break;
    case VIEW_MODE_STATUS:
    default:
      EPD_status_next();
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
      EPD_text_prev();
      break;
    case VIEW_MODE_BARO:
      EPD_baro_prev();
      break;
    case VIEW_MODE_TIME:
      EPD_time_prev();
      break;
    case VIEW_MODE_IMU:
      EPD_imu_prev();
      break;
    case VIEW_MODE_STATUS:
    default:
      EPD_status_prev();
      break;
    }
  }
}

void EPD_Message(const char *msg1, const char *msg2)
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;
  uint16_t x, y;

#if defined(USE_EPD_TASK)
  if (msg1 != NULL && strlen(msg1) != 0 && EPD_update_in_progress == EPD_UPDATE_NONE) {
//  if (msg1 != NULL && strlen(msg1) != 0 && SoC->Display_lock()) {
#else
  if (msg1 != NULL && strlen(msg1) != 0) {
#endif
    display->setFont(&FreeMonoBold18pt7b);

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

#if defined(USE_EPD_TASK)
    /* a signal to background EPD update task */
    EPD_update_in_progress = EPD_UPDATE_FAST;
//    SoC->Display_unlock();
//    yield();
#else
      display->display(true);
#endif
  }
}

EPD_Task_t EPD_Task( void * pvParameters )
{
//  unsigned long LockTime = millis();

  for( ;; )
  {
    if (EPD_update_in_progress != EPD_UPDATE_NONE) {
//    if (SoC->Display_lock()) {
//Serial.println("EPD_Task: lock"); Serial.flush();

//      LockTime = millis();
      display->display(EPD_update_in_progress == EPD_UPDATE_FAST ? true : false);
//Serial.println("EPD_Task: display"); Serial.flush();
      yield();

      /*
       * SYX 1942 revision of D67 display can use power_off() after partial update,
       * SYX 1948 revision - can not.
       */
      if (EPD_update_in_progress == EPD_UPDATE_FAST) { /* EPD_POWEROFF; */ }

      EPD_update_in_progress = EPD_UPDATE_NONE;
//      SoC->Display_unlock();
//Serial.println("EPD_Task: unlock"); Serial.flush();
//      delay(100);
//    } else {
//      if (millis() - LockTime > 4000) {
//        SoC->Display_unlock();
//Serial.println("EPD_Task: reseet lock"); Serial.flush();
//      }

    }

    yield();
  }
}

#endif /* USE_EPAPER */
