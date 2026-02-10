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

    switch (hw_info.model)
    {
    case SOFTRF_MODEL_CONCORDE:
      /* TBD */
      break;
#if 0
    case SOFTRF_MODEL_CONCORDE:
      touchDrv = new TouchDrvGT9895();
      touchDrv->setPins(-1 /* XL P03 */, -1 /* XL P04 */);
      result = touchDrv->begin(Wire, GT9895_SLAVE_ADDRESS_L,
                               SOC_GPIO_PIN_SDA, SOC_GPIO_PIN_SCL);
      break;
#endif
    case SOFTRF_MODEL_STANDALONE:
    default:
      touchDrv = new TouchDrvGT911();
      touchDrv->setPins(-1 /* SOC_GPIO_PIN_P4_TP_RST */, SOC_GPIO_PIN_P4_TP_INT);
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

void DSI_setup()
{
  byte rval = DISPLAY_NONE;

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
    switch (hw_info.model)
    {
    case SOFTRF_MODEL_CONCORDE:
      break;
    case SOFTRF_MODEL_STANDALONE:
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

  DSI_TimeMarker = millis();
}

#endif /* USE_DSI */
