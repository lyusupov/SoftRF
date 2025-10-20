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
bool TFT_display_frontpage = false;

static int TFT_view_mode = 0;

void gesture_event_handler(lv_event_t * e)
{
  lv_obj_t * screen = lv_event_get_current_target(e);
  lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
  switch(dir) {
    case LV_DIR_LEFT:
      /* TBD */
      break;
    case LV_DIR_RIGHT:
      /* TBD */
      break;
    case LV_DIR_TOP:
      /* TBD */
      break;
    case LV_DIR_BOTTOM:
      /* TBD */
      break;
  }
}

byte TFT_setup()
{
  byte rval = DISPLAY_NONE;

  TFT_view_mode = VIEW_MODE_STATUS;

  SoC->EPD_setup();

  if (panel) {
    lvgl_port_init(panel->getLCD(), panel->getTouch());

    lvgl_port_lock(-1);

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

    lvgl_port_unlock();

    rval = DISPLAY_TFT_7_0;
  }

  TFT_status_setup();

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
        case VIEW_MODE_STATUS:
        default:
          TFT_status_loop();
          break;
        }

        TFT_TimeMarker = millis();
      }
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

#endif /* USE_TFT */
