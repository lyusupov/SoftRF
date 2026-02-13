/*
 * DSI.cpp
 * Copyright (C) 2026 Linar Yusupov
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

#include "DSI.h"

#if defined(USE_DSI)

#include "LED.h"
#include "EPD.h"

#include <esp_display_panel.hpp>

#include <lvgl.h>
#include "../system/LVGLHelper.h"

using namespace esp_panel::board;

Board *panel = NULL;

const char DSI_SkyView_text1 [] = SOFTRF_IDENT;
const char DSI_SkyView_text2 [] = "Author:  Linar Yusupov  (C) 2016-2026";
const char DSI_SkyView_text3 [] = "POWER";
const char DSI_SkyView_text4 [] = "OFF";
const char DSI_SkyView_text5 [] = "Screen";
const char DSI_SkyView_text6 [] = "Saver";
const char DSI_SkyView_text7 [] = "VERSION " SOFTRF_FIRMWARE_VERSION;

unsigned long DSI_TimeMarker = 0;

static int DSI_view_mode = 0;
bool DSI_vmode_updated = true;

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

typedef struct TP_Point_struct {
  int16_t x;
  int16_t y;
} TP_Point;

typedef struct Gesture_struct {
  bool     touched;
  TP_Point t_loc;
  TP_Point d_loc;
} Gesture_t;

TouchDrvInterface *touchDrv = NULL;

int16_t x[5], y[5];

static Gesture_t gesture = { false, {0,0}, {0,0} };

bool setupTouchDrv()
{
    bool result = false;

    switch (hw_info.touch)
    {
    case TOUCH_JD9365TG:
      /* TBD */
      break;
    case TOUCH_GT9895:
      touchDrv = new TouchDrvGT9895();
      touchDrv->setPins(-1 /* XL P03 */, -1 /* XL P04 */);
      result = touchDrv->begin(Wire, GT9895_SLAVE_ADDRESS_L,
                               SOC_GPIO_PIN_TDP4_SDA, SOC_GPIO_PIN_TDP4_SCL);
      break;
    case TOUCH_GT911:
    default:
      touchDrv = new TouchDrvGT911();
      touchDrv->setPins(-1 /* SOC_GPIO_PIN_P4_TP_RST */, SOC_GPIO_PIN_P4_TP_INT);
      result = touchDrv->begin(Wire, GT911_SLAVE_ADDRESS_L,
                               SOC_GPIO_PIN_P4_SDA, SOC_GPIO_PIN_P4_SCL);
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

void DSI_setup()
{
  DSI_view_mode = ui->vmode;

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
    switch (hw_info.display)
    {
    case DISPLAY_TFT_LILYGO_4_05:
    case DISPLAY_AMOLED_LILYGO_4_1:
      break;
    case DISPLAY_TFT_WIRELESSTAG_7:
    default:
      lv_disp_set_rotation(NULL, LV_DISP_ROT_90);
      break;
    }

    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_text_color(lv_scr_act(), lv_color_white(), LV_PART_MAIN);

    lv_obj_t *label_1 = lv_label_create(lv_scr_act());
    lv_label_set_text(label_1, DSI_SkyView_text1);
    lv_obj_set_style_text_font(label_1, &lv_font_montserrat_48, 0);
    lv_obj_align(label_1, LV_ALIGN_CENTER, 0, -60);

    lv_obj_t *label_2 = lv_label_create(lv_scr_act());
    lv_label_set_text(label_2, DSI_SkyView_text7);
    lv_obj_set_style_text_font(label_2, &lv_font_montserrat_24, 0);
    lv_obj_align_to(label_2, label_1, LV_ALIGN_OUT_BOTTOM_MID, 0, 40);

    lv_obj_t *label_3 = lv_label_create(lv_scr_act());
    lv_label_set_text(label_3, DSI_SkyView_text2);
    lv_obj_set_style_text_font(label_3, &lv_font_montserrat_24, 0);
    lv_obj_align_to(label_3, label_2, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

#if defined(USE_EDPLIB_TOUCH)
    lv_obj_add_event_cb(lv_scr_act(), gesture_event_handler, LV_EVENT_GESTURE, NULL);
#endif /* USE_EDPLIB_TOUCH */
#endif /* LVGL_VERSION_MAJOR == 8 */

    lvgl_port_unlock();
  }

  DSI_status_setup();
  DSI_radar_setup();

  DSI_TimeMarker = millis();
}

void DSI_loop()
{
  switch (hw_info.display)
  {
  case DISPLAY_TFT_WIRELESSTAG_7:
  case DISPLAY_TFT_LILYGO_4_05:
  case DISPLAY_AMOLED_LILYGO_4_1:
    if (panel) {
      if (isTimeToDisplay()) {
        switch (DSI_view_mode)
        {
        case VIEW_MODE_RADAR:
          DSI_radar_loop();
          break;
        case VIEW_MODE_STATUS:
        default:
          DSI_status_loop();
          break;
        }

        DSI_TimeMarker = millis();
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
        if (DSI_view_mode == VIEW_MODE_STATUS) {
          DSI_view_mode = VIEW_MODE_RADAR;
          DSI_vmode_updated = true;
        }
        break;
      case SWIPE_RIGHT:
        if (DSI_view_mode == VIEW_MODE_RADAR) {
          DSI_view_mode = VIEW_MODE_STATUS;
          DSI_vmode_updated = true;
        }
        break;
      case SWIPE_DOWN:
        DSI_Up();
        break;
      case SWIPE_UP:
        DSI_Down();
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

void DSI_fini()
{

}

void DSI_Up()
{
  if (hw_info.display == DISPLAY_TFT_WIRELESSTAG_7 ||
      hw_info.display == DISPLAY_TFT_LILYGO_4_05   ||
      hw_info.display == DISPLAY_AMOLED_LILYGO_4_1) {
    switch (DSI_view_mode)
    {
    case VIEW_MODE_RADAR:
      DSI_radar_unzoom();
      break;
    case VIEW_MODE_STATUS:
    default:
      break;
    }
  }
}

void DSI_Down()
{
  if (hw_info.display == DISPLAY_TFT_WIRELESSTAG_7 ||
      hw_info.display == DISPLAY_TFT_LILYGO_4_05   ||
      hw_info.display == DISPLAY_AMOLED_LILYGO_4_1) {
    switch (DSI_view_mode)
    {
    case VIEW_MODE_RADAR:
      DSI_radar_zoom();
      break;
    case VIEW_MODE_STATUS:
    default:
      break;
    }
  }
}

void DSI_Next_Page()
{
  if (hw_info.display == DISPLAY_TFT_WIRELESSTAG_7 ||
      hw_info.display == DISPLAY_TFT_LILYGO_4_05   ||
      hw_info.display == DISPLAY_AMOLED_LILYGO_4_1) {
    switch (DSI_view_mode)
    {
    case VIEW_MODE_RADAR:
      DSI_view_mode = VIEW_MODE_STATUS;
      DSI_vmode_updated = true;
      break;
    case VIEW_MODE_STATUS:
    default:
      DSI_view_mode = VIEW_MODE_RADAR;
      DSI_vmode_updated = true;
      break;
    }
  }
}

#endif /* USE_DSI */
