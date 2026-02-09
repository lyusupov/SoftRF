/*
 * TFTHelper.cpp
 * Copyright (C) 2025-2026 Linar Yusupov
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
const char TFT_SkyView_text3 [] = "Author:  Linar Yusupov  (C) 2019-2026";
const char TFT_SkyView_text4 [] = "POWER";
const char TFT_SkyView_text5 [] = "OFF";
const char TFT_SkyView_text6 [] = "Screen";
const char TFT_SkyView_text7 [] = "Saver";
const char TFT_SkyView_text8 [] = "VERSION " SKYVIEW_FIRMWARE_VERSION;

unsigned long TFT_TimeMarker = 0;

static int TFT_view_mode = 0;
bool TFT_vmode_updated = true;

static int tp_action = NO_GESTURE;

#if defined(USE_EDPLIB_TOUCH)
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
#endif /* USE_EDPLIB_TOUCH */

#if defined(USE_SENSORLIB_TOUCH)
#include "TouchDrvGT911.hpp"
#include "TouchDrvGT9895.hpp"

TouchDrvInterface *touchDrv = NULL;

int16_t x[5], y[5];

static Gesture_t gesture = { false, {0,0}, {0,0} };

bool setupTouchDrv()
{
    bool result = false;

    switch (hw_info.revision)
    {
    case HW_REV_TDISPLAY_P4_TFT:
      /* TBD */
      break;

    case HW_REV_TDISPLAY_P4_AMOLED:
      touchDrv = new TouchDrvGT9895();
      touchDrv->setPins(-1 /* XL P03 */, -1 /* XL P04 */);
      result = touchDrv->begin(Wire, GT9895_SLAVE_ADDRESS_L,
                               SOC_GPIO_PIN_SDA, SOC_GPIO_PIN_SCL);
      break;

    case HW_REV_DEVKIT:
    default:
      touchDrv = new TouchDrvGT911();
      touchDrv->setPins(-1 /* SOC_GPIO_PIN_TP_RST */, SOC_GPIO_PIN_TP_INT);
      result = touchDrv->begin(Wire, GT911_SLAVE_ADDRESS_L,
                               SOC_GPIO_PIN_SDA, SOC_GPIO_PIN_SCL);
      touchDrv->setMaxCoordinates(panel->getLCD()->getFrameWidth(),
                                  panel->getLCD()->getFrameHeight());
      touchDrv->setMirrorXY(true, true);
      break;
    }

    if (result) {
        Serial.print(touchDrv->getModelName());
        Serial.println(" TP initialized successfully");
        return true;
    }
    delete touchDrv;

    Serial.println("Unable to find touch device.");

    touchDrv = NULL;

    return false;
}
#endif /* USE_SENSORLIB_TOUCH */

byte TFT_setup()
{
  byte rval = DISPLAY_NONE;

  TFT_view_mode = VIEW_MODE_STATUS;

  if (panel) {
#if defined(USE_EDPLIB_TOUCH)
    lvgl_port_init(panel->getLCD(), panel->getTouch());
#else
    lvgl_port_init(panel->getLCD(), nullptr);
#if defined(USE_SENSORLIB_TOUCH)
    setupTouchDrv();
#endif /* USE_SENSORLIB_TOUCH */
#endif /* USE_EDPLIB_TOUCH */

    lvgl_port_lock(-1);

#if LVGL_VERSION_MAJOR == 8
    switch (hw_info.revision)
    {
    case HW_REV_TDISPLAY_P4_TFT:
    case HW_REV_TDISPLAY_P4_AMOLED:
      lv_disp_set_rotation(NULL, LV_DISP_ROT_90);
      break;
    case HW_REV_DEVKIT:
    default:
      break;
    }

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

#if defined(USE_EDPLIB_TOUCH)
    lv_obj_add_event_cb(lv_scr_act(), gesture_event_handler, LV_EVENT_GESTURE, NULL);
#endif /* USE_EDPLIB_TOUCH */
#endif /* LVGL_VERSION_MAJOR == 8 */

    lvgl_port_unlock();

    switch (hw_info.revision)
    {
    case HW_REV_TDISPLAY_P4_TFT:
      rval = DISPLAY_TFT_4_05;
      break;
    case HW_REV_TDISPLAY_P4_AMOLED:
      rval = DISPLAY_AMOLED_4_1;
      break;
    case HW_REV_DEVKIT:
    default:
      rval = DISPLAY_TFT_7_0;
      break;
    }
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
  case DISPLAY_TFT_4_05:
  case DISPLAY_AMOLED_4_1:
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

#if defined(USE_SENSORLIB_TOUCH)
      if (touchDrv) {
        if (touchDrv->isPressed()) {
            uint8_t touched = touchDrv->getPoint(x, y, touchDrv->getSupportTouchPoint());
#if 0
            if (touched) {
                for (int i = 0; i < touched; ++i) {
                    Serial.print("X[");
                    Serial.print(i);
                    Serial.print("]:");
                    Serial.print(x[i]);
                    Serial.print(" ");
                    Serial.print(" Y[");
                    Serial.print(i);
                    Serial.print("]:");
                    Serial.print(y[i]);
                    Serial.print(" ");
                }
                Serial.println();
            }
#endif
            if (touched > 0) {
              TP_Point p;
              p.x = x[0];
              p.y = y[0];

              if (gesture.touched) {
//                Serial.println("touched > 0 , gesture.touched = true");
                gesture.d_loc = p;
              } else {
//                Serial.println("touched > 0 , gesture.touched = flase");
                gesture.t_loc = p; gesture.d_loc = p;
                gesture.touched = true;
              }
            }
        } else {
//            Serial.println("isPressed() = false");
            if (gesture.touched) {
//              Serial.println("isPressed() = false , gesture.touched = true");
              int16_t FrameWidth  = panel->getLCD()->getFrameWidth();
              int16_t FrameHeight = panel->getLCD()->getFrameHeight();
              int16_t threshold_x = FrameWidth  / 10;
              int16_t threshold_y = FrameHeight / 10;
              int16_t limit_xl = FrameWidth/2  - threshold_x;
              int16_t limit_xr = FrameWidth/2  + threshold_x;
              int16_t limit_yt = FrameHeight/2 - threshold_y;
              int16_t limit_yb = FrameHeight/2 + threshold_y;

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
      }
#endif /* USE_SENSORLIB_TOUCH */

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
  if (hw_info.display == DISPLAY_TFT_7_0  ||
      hw_info.display == DISPLAY_TFT_4_05 ||
      hw_info.display == DISPLAY_AMOLED_4_1) {
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
  if (hw_info.display == DISPLAY_TFT_7_0  ||
      hw_info.display == DISPLAY_TFT_4_05 ||
      hw_info.display == DISPLAY_AMOLED_4_1) {
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
