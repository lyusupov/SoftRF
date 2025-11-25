/*
 * TFTHelper.cpp
 * Copyright (C) 2025 Linar Yusupov
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

#include "TFTHelper.h"
#include "EEPROMHelper.h"

#if defined(USE_TFT)

#include <esp_display_panel.hpp>

#include <lvgl.h>
#include "LVGLHelper.h"

using namespace esp_panel::board;

Board *panel = NULL;

const char TFT_SkyView_text1 [] = SKYVIEW_IDENT;
const char TFT_SkyView_text2 [] = "Presented by SoftRF project";
const char TFT_SkyView_text3 [] = "Author:  Linar Yusupov  (C) 2019-2025";
const char TFT_SkyView_text4 [] = "POWER";
const char TFT_SkyView_text5 [] = "OFF";
const char TFT_SkyView_text6 [] = "Screen";
const char TFT_SkyView_text7 [] = "Saver";
const char TFT_SkyView_text8 [] = "VERSION " SKYVIEW_FIRMWARE_VERSION;

unsigned long TFT_TimeMarker = 0;

static int TFT_view_mode = 0;
bool TFT_vmode_updated = true;

static int tp_action = NO_GESTURE;

void gesture_event_handler(lv_event_t * e)
{
#if LVGL_VERSION_MAJOR == 8
  lv_obj_t * screen = lv_event_get_current_target(e);
  lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
  switch(dir) {
    case LV_DIR_LEFT:
      // Serial.println("LV_DIR_LEFT");
      tp_action = SWIPE_LEFT;
      break;
    case LV_DIR_RIGHT:
      // Serial.println("LV_DIR_RIGHT");
      tp_action = SWIPE_RIGHT;
      break;
    case LV_DIR_TOP:
      // Serial.println("LV_DIR_TOP");
      tp_action = SWIPE_UP;
      break;
    case LV_DIR_BOTTOM:
      // Serial.println("LV_DIR_BOTTOM");
      tp_action = SWIPE_DOWN;
      break;
  }
#endif /* LVGL_VERSION_MAJOR == 8 */
}

byte TFT_setup()
{
  byte rval = DISPLAY_NONE;

  TFT_view_mode = VIEW_MODE_STATUS;

  if (panel) {
    lvgl_port_init(panel->getLCD(), panel->getTouch());

    lvgl_port_lock(-1);

#if LVGL_VERSION_MAJOR == 8
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_text_color(lv_scr_act(), lv_color_white(), LV_PART_MAIN);

    lv_obj_t *label_1 = lv_label_create(lv_scr_act());
    lv_label_set_text(label_1, TFT_SkyView_text1);
    lv_obj_set_style_text_font(label_1, &lv_font_montserrat_48, 0);
    lv_obj_align(label_1, LV_ALIGN_CENTER, 0, -60);

    lv_obj_t *label_2 = lv_label_create(lv_scr_act());
    lv_label_set_text(label_2, TFT_SkyView_text8);
    lv_obj_set_style_text_font(label_2, &lv_font_montserrat_24, 0);
    lv_obj_align_to(label_2, label_1, LV_ALIGN_OUT_BOTTOM_MID, 0, 40);

    lv_obj_t *label_3 = lv_label_create(lv_scr_act());
    lv_label_set_text(label_3, TFT_SkyView_text2);
    lv_obj_set_style_text_font(label_3, &lv_font_montserrat_24, 0);
    lv_obj_align_to(label_3, label_2, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

    lv_obj_t *label_4 = lv_label_create(lv_scr_act());
    lv_label_set_text(label_4, TFT_SkyView_text3);
    lv_obj_set_style_text_font(label_4, &lv_font_montserrat_24, 0);
    lv_obj_align_to(label_4, label_3, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

    lv_obj_add_event_cb(lv_scr_act(), gesture_event_handler, LV_EVENT_GESTURE, NULL);
#endif /* LVGL_VERSION_MAJOR == 8 */

    lvgl_port_unlock();

    rval = DISPLAY_TFT_7_0;
  }

  TFT_status_setup();
  TFT_radar_setup();

  TFT_TimeMarker = millis();

  return rval;
}

void TFT_loop()
{
  switch (hw_info.display)
  {
  case DISPLAY_TFT_7_0:
    if (panel) {
      if (isTimeToDisplay()) {
        switch (TFT_view_mode)
        {
        case VIEW_MODE_RADAR:
          TFT_radar_loop();
          break;
        case VIEW_MODE_STATUS:
        default:
          TFT_status_loop();
          break;
        }

        TFT_TimeMarker = millis();
      }

      switch (tp_action)
      {
      case SWIPE_LEFT:
        if (TFT_view_mode == VIEW_MODE_STATUS) {
          TFT_view_mode = VIEW_MODE_RADAR;
          TFT_vmode_updated = true;
        }
        break;
      case SWIPE_RIGHT:
        if (TFT_view_mode == VIEW_MODE_RADAR) {
          TFT_view_mode = VIEW_MODE_STATUS;
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

      tp_action = NO_GESTURE;
    }

    break;

  case DISPLAY_NONE:
  default:
    break;
  }
}

void TFT_fini()
{

}

void TFT_Up()
{
  if (hw_info.display == DISPLAY_TFT_7_0) {
    switch (TFT_view_mode)
    {
    case VIEW_MODE_RADAR:
      TFT_radar_unzoom();
      break;
    case VIEW_MODE_STATUS:
    default:
      break;
    }
  }
}

void TFT_Down()
{
  if (hw_info.display == DISPLAY_TFT_7_0) {
    switch (TFT_view_mode)
    {
    case VIEW_MODE_RADAR:
      TFT_radar_zoom();
      break;
    case VIEW_MODE_STATUS:
    default:
      break;
    }
  }
}

#endif /* USE_TFT */
