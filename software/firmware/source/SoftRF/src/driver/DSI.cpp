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
#include "RF.h"
#include "Baro.h"
#include "../system/Time.h"

#include <esp_display_panel.hpp>

#include <lvgl.h>
#include "../system/LVGLHelper.h"

using namespace esp_panel::board;

Board *panel = NULL;

const char DSI_SoftRF_text1 [] = SOFTRF_IDENT;
const char DSI_SoftRF_text2 [] = "Author:  Linar Yusupov  (C) 2016-2026";
const char DSI_SoftRF_text3 [] = "POWER OFF";
const char DSI_SoftRF_text4 [] = "LOW BATTERY";
const char DSI_SoftRF_text5 [] = "Screen";
const char DSI_SoftRF_text6 [] = "Saver";
const char DSI_SoftRF_text7 [] = "VERSION " SOFTRF_FIRMWARE_VERSION;

static unsigned long DSI_TimeMarker = 0;
static unsigned long TP_TimeMarker  = 0;

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
#include <SensorLib_Version.h>
#include "TouchDrvGT911.hpp"
#include "TouchDrvGT9895.hpp"
#if SENSORLIB_VERSION >= SENSORLIB_VERSION_VAL(0, 4, 0)
#include "TouchDrvHI8561.hpp"
#endif /* (0, 4, 0) */

TouchDrvInterface *touchDrv = NULL;

#define USE_GESTURE_V1 1
#define USE_GESTURE_V2 0

#if USE_GESTURE_V1
typedef struct TP_Point_struct {
  int16_t x;
  int16_t y;
} TP_Point;

typedef struct Gesture_struct {
  bool     touched;
  TP_Point t_loc;
  TP_Point d_loc;
} Gesture_t;

int16_t x[5], y[5];

static Gesture_t gesture = { false, {0,0}, {0,0} };
#endif /* USE_GESTURE_V1 */

#if USE_GESTURE_V2
enum GestureType {
    GESTURE_NONE,
    GESTURE_TAP,
    GESTURE_LEFT,
    GESTURE_RIGHT,
    GESTURE_UP,
    GESTURE_DOWN
};

struct GestureState {
    bool active = false;
    uint8_t id = 0xFF;
    int16_t startX = 0;
    int16_t startY = 0;
    int16_t lastX = 0;
    int16_t lastY = 0;
    uint32_t startTime = 0;
    uint32_t lastTime = 0;
    uint16_t samples = 0;
};

GestureState gesture;

static const int SWIPE_THRESHOLD = 70;  // Minimum movement for swipe gesture
static const int TAP_MOVE_THRESHOLD = 18;   // Maximum movement for tap gesture
static const uint32_t TAP_TIME_THRESHOLD = 250; // Maximum time for tap gesture
static const uint32_t MAX_GESTURE_TIME = 1200; // Maximum gesture duration
static const float DOMINANCE_RATIO = 1.2f;  // Dominance ratio for swipe gestures
static const bool enableTap = false;        // Disable print tap event

// only swipe gesture is limited by cooldown
static const uint32_t SWIPE_COOLDOWN_MS = 600;

uint32_t lastSwipeEventTime = 0;
bool swipeEventLocked = false;

const char *gestureToString(GestureType g)
{
    switch (g) {
    case GESTURE_TAP:   return "TAP";
    case GESTURE_LEFT:  return "LEFT";
    case GESTURE_RIGHT: return "RIGHT";
    case GESTURE_UP:    return "UP";
    case GESTURE_DOWN:  return "DOWN";
    default:            return "NONE";
    }
}

bool isSwipeGesture(GestureType g)
{
    return g == GESTURE_LEFT ||
           g == GESTURE_RIGHT ||
           g == GESTURE_UP ||
           g == GESTURE_DOWN;
}

void resetGesture()
{
    gesture.active = false;
    gesture.id = 0xFF;
    gesture.startX = 0;
    gesture.startY = 0;
    gesture.lastX = 0;
    gesture.lastY = 0;
    gesture.startTime = 0;
    gesture.lastTime = 0;
    gesture.samples = 0;
}

GestureType classifyGesture(int dx, int dy, uint32_t dt)
{
    int adx = abs(dx);
    int ady = abs(dy);

    if (dt <= TAP_TIME_THRESHOLD && adx <= TAP_MOVE_THRESHOLD && ady <= TAP_MOVE_THRESHOLD) {
        return GESTURE_TAP;
    }

    if (dt > MAX_GESTURE_TIME) {
        return GESTURE_NONE;
    }

    if (adx >= SWIPE_THRESHOLD && (float)adx > (float)ady * DOMINANCE_RATIO) {
        return dx > 0 ? GESTURE_RIGHT : GESTURE_LEFT;
    }

    if (ady >= SWIPE_THRESHOLD && (float)ady > (float)adx * DOMINANCE_RATIO) {
        return dy > 0 ? GESTURE_DOWN : GESTURE_UP;
    }

    return GESTURE_NONE;
}

void unlockSwipeIfNeeded(uint32_t now)
{
    if (swipeEventLocked && (now - lastSwipeEventTime) >= SWIPE_COOLDOWN_MS) {
        swipeEventLocked = false;
    }
}

bool canEmitSwipe(uint32_t now)
{
    if (!swipeEventLocked) {
        return true;
    }
    return (now - lastSwipeEventTime) >= SWIPE_COOLDOWN_MS;
}

void lockSwipeEvent(uint32_t now)
{
    lastSwipeEventTime = now;
    swipeEventLocked = true;
}

void printGestureResult(GestureType type, int dx, int dy, uint32_t dt)
{
    Serial.println("===== Gesture Result =====");
    Serial.printf("id      : %u\n", gesture.id);
    Serial.printf("start   : (%d, %d)\n", gesture.startX, gesture.startY);
    Serial.printf("end     : (%d, %d)\n", gesture.lastX, gesture.lastY);
    Serial.printf("delta   : dx=%d dy=%d\n", dx, dy);
    Serial.printf("time    : %lu ms\n", (unsigned long)dt);
    Serial.printf("samples : %u\n", gesture.samples);
    Serial.printf("gesture : %s\n", gestureToString(type));
    if (isSwipeGesture(type)) {
        Serial.printf("swipe cooldown: %lu ms\n", (unsigned long)SWIPE_COOLDOWN_MS);
    }
    Serial.println();
}
#endif /* USE_GESTURE_V2 */

bool setupTouchDrv()
{
    bool result = false;

    switch (hw_info.touch)
    {
    case TOUCH_JD9365TG:
#if SENSORLIB_VERSION >= SENSORLIB_VERSION_VAL(0, 4, 0)
      touchDrv = new TouchDrvHI8561();
      touchDrv->setPins(-1 /* XL P03 */, -1 /* XL P04 */);
      result = touchDrv->begin(TDP4_IIC_1, HI8561_ADDRESS,
                               SOC_GPIO_PIN_TDP4_SDA_1, SOC_GPIO_PIN_TDP4_SCL_1);
#endif /* (0, 4, 0) */
      break;
    case TOUCH_GT9895:
      touchDrv = new TouchDrvGT9895();
      touchDrv->setPins(-1 /* XL P03 */, -1 /* XL P04 */);
      result = touchDrv->begin(TDP4_IIC_1, GT9895_SLAVE_ADDRESS_L,
                               SOC_GPIO_PIN_TDP4_SDA_1, SOC_GPIO_PIN_TDP4_SCL_1);
#if SENSORLIB_VERSION >= SENSORLIB_VERSION_VAL(0, 4, 0)
      touchDrv->setResolution(1060, 2400);
      touchDrv->setTargetResolution(panel->getLCD()->getFrameWidth(),
                                    panel->getLCD()->getFrameHeight());
      // touchDrv->setMaxCoordinates(panel->getLCD()->getFrameWidth(),
      //                             panel->getLCD()->getFrameHeight());
#endif /* (0, 4, 0) */
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
    lv_label_set_text(label_1, DSI_SoftRF_text1);
    lv_obj_set_style_text_font(label_1, &lv_font_montserrat_48, 0);
    lv_obj_align(label_1, LV_ALIGN_CENTER, 0, -60);

    lv_obj_t *label_2 = lv_label_create(lv_scr_act());
    lv_label_set_text(label_2, DSI_SoftRF_text7);
    lv_obj_set_style_text_font(label_2, &lv_font_montserrat_24, 0);
    lv_obj_align_to(label_2, label_1, LV_ALIGN_OUT_BOTTOM_MID, 0, 40);

    lv_obj_t *label_3 = lv_label_create(lv_scr_act());
    lv_label_set_text(label_3, DSI_SoftRF_text2);
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
  TP_TimeMarker  = millis();
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
#if USE_GESTURE_V1
      if (touchDrv && isTimeToTPSense()) {
#if SENSORLIB_VERSION == SENSORLIB_VERSION_VAL(0, 3, 1)
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
                gesture.d_loc = p;
              } else {
                gesture.t_loc = p; gesture.d_loc = p;
                gesture.touched = true;
              }
            }
#endif /* (0, 3, 1) */

#if SENSORLIB_VERSION >= SENSORLIB_VERSION_VAL(0, 4, 0)
        TouchPoints touch_points = touchDrv->getTouchPoints();
        if (touch_points.hasPoints()) {
            int i = 0;
            const TouchPoint &point = touch_points.getPoint(i);
#if 0
            Serial.print("X[");
            Serial.print(i);
            Serial.print("]:");
            Serial.print(point.x);
            Serial.print(" ");
            Serial.print(" Y[");
            Serial.print(i);
            Serial.print("]:");
            Serial.print(point.y);
            Serial.println();
#endif
            TP_Point p;
            p.x = point.x;
            p.y = point.y;

            if (gesture.touched) {
              gesture.d_loc = p;
            } else {
              gesture.t_loc = p; gesture.d_loc = p;
              gesture.touched = true;
            }
#endif /* (0, 4, 0) */
        } else {
            if (gesture.touched) {
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

        TP_TimeMarker = millis();
      }
#endif /* USE_GESTURE_V1 */

#if USE_GESTURE_V2
      uint32_t now = millis();
      unlockSwipeIfNeeded(now);

      TouchPoints touch_points = touchDrv->getTouchPoints();

      if (touch_points.hasPoints()) {
          const TouchPoint &point = touch_points.getPoint(0);

          if (!gesture.active) {
              gesture.active = true;
              gesture.id = point.id;
              gesture.startX = point.x;
              gesture.startY = point.y;
              gesture.lastX = point.x;
              gesture.lastY = point.y;
              gesture.startTime = now;
              gesture.lastTime = now;
              gesture.samples = 1;
          } else {
              gesture.lastX = point.x;
              gesture.lastY = point.y;
              gesture.lastTime = now;
              gesture.samples++;
          }
      } else {
          if (gesture.active) {
              int dx = gesture.lastX - gesture.startX;
              int dy = gesture.lastY - gesture.startY;
              uint32_t dt = gesture.lastTime - gesture.startTime;

              GestureType type = classifyGesture(dx, dy, dt);

              if (type == GESTURE_TAP) {
                  if (enableTap) {
                      // printGestureResult(type, dx, dy, dt);
                  }
              } else if (isSwipeGesture(type)) {
                  if (canEmitSwipe(now)) {
                      // printGestureResult(type, dx, dy, dt);
                      lockSwipeEvent(now);
                  }
              }

              switch (type)
              {
                GESTURE_LEFT:   tp_action = SWIPE_LEFT;  break;
                GESTURE_RIGHT:  tp_action = SWIPE_RIGHT; break;
                GESTURE_UP:     tp_action = SWIPE_UP;    break;
                GESTURE_DOWN:   tp_action = SWIPE_DOWN;  break;
                default:        tp_action = NO_GESTURE;  break;
              }

              resetGesture();
          }
      }
#endif /* USE_GESTURE_V2 */
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

void DSI_fini(int reason)
{
  if (panel) {
    lvgl_port_lock(-1);

#if LVGL_VERSION_MAJOR == 8
    lv_obj_clean(lv_scr_act());

    lv_obj_t *label_1 = lv_label_create(lv_scr_act());
    lv_label_set_text(label_1, reason == SOFTRF_SHUTDOWN_LOWBAT ?
                      DSI_SoftRF_text4 : DSI_SoftRF_text3);
    lv_obj_set_style_text_font(label_1, &lv_font_montserrat_48, 0);
    lv_obj_align(label_1, LV_ALIGN_CENTER, 0, -60);

#if defined(USE_EDPLIB_TOUCH)
    lv_obj_remove_event_cb(lv_scr_act(), gesture_event_handler);
#endif /* USE_EDPLIB_TOUCH */
#endif /* LVGL_VERSION_MAJOR == 8 */

    lvgl_port_unlock();

    delay(3000); /* Keep shutdown message on DSI display for 3 seconds */

    auto lcd = panel->getLCD();
    esp_lcd_panel_t *handle = lcd->getRefreshPanelHandle();

    switch (hw_info.display)
    {
    case DISPLAY_TFT_LILYGO_4_05:
      /* TBD */
      break;
    case DISPLAY_AMOLED_LILYGO_4_1:
      /* TBD */
      break;
    case DISPLAY_TFT_WIRELESSTAG_7:
    default:
      /* TBD */
      break;
    }
  }
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

void DSI_info1()
{
  if (panel) {
    switch (hw_info.display)
    {
    case DISPLAY_TFT_WIRELESSTAG_7:
    case DISPLAY_TFT_LILYGO_4_05:
    case DISPLAY_AMOLED_LILYGO_4_1:
      lvgl_port_lock(-1);

#if LVGL_VERSION_MAJOR == 8
      lv_obj_clean(lv_scr_act());

      {
      lv_obj_t *label_1 = lv_label_create(lv_scr_act());
      lv_label_set_text(label_1, "          SELF TEST");
      lv_obj_set_style_text_font(label_1, &lv_font_montserrat_48, 0);
      lv_obj_align(label_1, LV_ALIGN_OUT_TOP_LEFT, 40, 20);

      lv_obj_t *data_1 = lv_label_create(lv_scr_act());
      lv_label_set_text(data_1, " ");
      lv_obj_set_style_text_font(data_1, &lv_font_montserrat_48, 0);
      lv_obj_align(data_1, LV_ALIGN_TOP_RIGHT, -40, 20);

      lv_obj_t *label_2 = lv_label_create(lv_scr_act());
      lv_label_set_text(label_2, "RADIO");
      lv_obj_set_style_text_font(label_2, &lv_font_montserrat_48, 0);
      lv_obj_align_to(label_2, label_1, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 40);

      lv_obj_t *data_2 = lv_label_create(lv_scr_act());
      lv_label_set_text_fmt(data_2, "%s", hw_info.rf != RF_IC_NONE ?
                            "PASS" : "FAIL");
      lv_obj_set_style_text_font(data_2, &lv_font_montserrat_48, 0);
      lv_obj_set_style_text_color(data_2, hw_info.rf != RF_IC_NONE ?
                                  lv_palette_main(LV_PALETTE_GREEN) :
                                  lv_palette_main(LV_PALETTE_RED), 0);
      lv_obj_align_to(data_2, data_1, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 40);

      lv_obj_t *label_3 = lv_label_create(lv_scr_act());
      lv_label_set_text(label_3, "GNSS");
      lv_obj_set_style_text_font(label_3, &lv_font_montserrat_48, 0);
      lv_obj_align_to(label_3, label_2, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

      lv_obj_t *data_3 = lv_label_create(lv_scr_act());
      lv_label_set_text_fmt(data_3, "%s", hw_info.gnss != GNSS_MODULE_NONE ?
                            "PASS" : "FAIL");
      lv_obj_set_style_text_font(data_3, &lv_font_montserrat_48, 0);
      lv_obj_set_style_text_color(data_3, hw_info.gnss != GNSS_MODULE_NONE ?
                                  lv_palette_main(LV_PALETTE_GREEN) :
                                  lv_palette_main(LV_PALETTE_RED), 0);
      lv_obj_align_to(data_3, data_2, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

      lv_obj_t *label_4 = lv_label_create(lv_scr_act());
      lv_label_set_text(label_4, "RTC");
      lv_obj_set_style_text_font(label_4, &lv_font_montserrat_48, 0);
      lv_obj_align_to(label_4, label_3, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

      lv_obj_t *data_4 = lv_label_create(lv_scr_act());
      lv_label_set_text_fmt(data_4, "%s", hw_info.rtc == RTC_PCF8563 ?
                            "PASS" : "FAIL");
      lv_obj_set_style_text_font(data_4, &lv_font_montserrat_48, 0);
      lv_obj_set_style_text_color(data_4, hw_info.rtc == RTC_PCF8563 ?
                                  lv_palette_main(LV_PALETTE_GREEN) :
                                  lv_palette_main(LV_PALETTE_RED), 0);
      lv_obj_align_to(data_4, data_3, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

      lv_obj_t *label_5 = lv_label_create(lv_scr_act());
      lv_label_set_text(label_5, "IMU");
      lv_obj_set_style_text_font(label_5, &lv_font_montserrat_48, 0);
      lv_obj_align_to(label_5, label_4, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

      lv_obj_t *data_5 = lv_label_create(lv_scr_act());
      lv_label_set_text_fmt(data_5, "%s", hw_info.imu != IMU_NONE ?
                            "PASS" : "FAIL");
      lv_obj_set_style_text_font(data_5, &lv_font_montserrat_48, 0);
      lv_obj_set_style_text_color(data_5, hw_info.imu != IMU_NONE ?
                                  lv_palette_main(LV_PALETTE_GREEN) :
                                  lv_palette_main(LV_PALETTE_RED), 0);
      lv_obj_align_to(data_5, data_4, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

      lv_obj_t *label_6 = lv_label_create(lv_scr_act());
      lv_label_set_text(label_6, "AUDIO");
      lv_obj_set_style_text_font(label_6, &lv_font_montserrat_48, 0);
      lv_obj_align_to(label_6, label_5, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

      lv_obj_t *data_6 = lv_label_create(lv_scr_act());
      lv_label_set_text_fmt(data_6, "%s", hw_info.audio == AUDIO_ES8311 ?
                            "PASS" : "FAIL");
      lv_obj_set_style_text_font(data_6, &lv_font_montserrat_48, 0);
      lv_obj_set_style_text_color(data_6, hw_info.audio == AUDIO_ES8311 ?
                                  lv_palette_main(LV_PALETTE_GREEN) :
                                  lv_palette_main(LV_PALETTE_RED), 0);
      lv_obj_align_to(data_6, data_5, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

      lv_obj_t *label_7 = lv_label_create(lv_scr_act());
      lv_label_set_text(label_7, "TOUCH");
      lv_obj_set_style_text_font(label_7, &lv_font_montserrat_48, 0);
      lv_obj_align_to(label_7, label_6, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

      lv_obj_t *data_7 = lv_label_create(lv_scr_act());
      lv_label_set_text_fmt(data_7, "%s", hw_info.touch != TOUCH_NONE ?
                            "PASS" : "FAIL");
      lv_obj_set_style_text_font(data_7, &lv_font_montserrat_48, 0);
      lv_obj_set_style_text_color(data_7, hw_info.touch != TOUCH_NONE ?
                                  lv_palette_main(LV_PALETTE_GREEN) :
                                  lv_palette_main(LV_PALETTE_RED), 0);
      lv_obj_align_to(data_7, data_6, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

      lv_obj_t *label_8 = lv_label_create(lv_scr_act());
      lv_label_set_text(label_8, "HAPTIC");
      lv_obj_set_style_text_font(label_8, &lv_font_montserrat_48, 0);
      lv_obj_align_to(label_8, label_7, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

      lv_obj_t *data_8 = lv_label_create(lv_scr_act());
      lv_label_set_text_fmt(data_8, "%s", hw_info.haptic == HAPTIC_AW86224 ?
                            "PASS" : "FAIL");
      lv_obj_set_style_text_font(data_8, &lv_font_montserrat_48, 0);
      lv_obj_set_style_text_color(data_8, hw_info.haptic == HAPTIC_AW86224 ?
                                  lv_palette_main(LV_PALETTE_GREEN) :
                                  lv_palette_main(LV_PALETTE_RED), 0);
      lv_obj_align_to(data_8, data_7, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

      lv_obj_t *label_9 = lv_label_create(lv_scr_act());
      lv_label_set_text(label_9, "CARD");
      lv_obj_set_style_text_font(label_9, &lv_font_montserrat_48, 0);
      lv_obj_align_to(label_9, label_8, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

      lv_obj_t *data_9 = lv_label_create(lv_scr_act());
      lv_label_set_text_fmt(data_9, "%s", hw_info.storage == STORAGE_CARD ?
                            "PASS" : "N/A");
      lv_obj_set_style_text_font(data_9, &lv_font_montserrat_48, 0);
      lv_obj_set_style_text_color(data_9, hw_info.storage == STORAGE_CARD ?
                                  lv_palette_main(LV_PALETTE_GREEN) :
                                  lv_palette_main(LV_PALETTE_YELLOW), 0);
      lv_obj_align_to(data_9, data_8, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

      lv_obj_t *label_10 = lv_label_create(lv_scr_act());
      lv_label_set_text(label_10, "BARO");
      lv_obj_set_style_text_font(label_10, &lv_font_montserrat_48, 0);
      lv_obj_align_to(label_10, label_9, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

      lv_obj_t *data_10 = lv_label_create(lv_scr_act());
      lv_label_set_text_fmt(data_10, "%s", hw_info.baro != BARO_MODULE_NONE ?
                            "PASS" : "N/A");
      lv_obj_set_style_text_font(data_10, &lv_font_montserrat_48, 0);
      lv_obj_set_style_text_color(data_10, hw_info.baro != BARO_MODULE_NONE ?
                                  lv_palette_main(LV_PALETTE_GREEN) :
                                  lv_palette_main(LV_PALETTE_YELLOW), 0);
      lv_obj_align_to(data_10, data_9, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);
      }
#endif /* LVGL_VERSION_MAJOR == 8 */

      lvgl_port_unlock();

      delay(4000);

      break;
    case DISPLAY_NONE:
    default:
      break;
    }
  }
}

void DSI_info2(int acfts, char *reg, char *mam, char *cn)
{
  if (panel) {
    switch (hw_info.display)
    {
    case DISPLAY_TFT_WIRELESSTAG_7:
    case DISPLAY_TFT_LILYGO_4_05:
    case DISPLAY_AMOLED_LILYGO_4_1:
      lvgl_port_lock(-1);

#if LVGL_VERSION_MAJOR == 8
      lv_obj_clean(lv_scr_act());

      if (acfts == -1) {
        lv_obj_t *label_1 = lv_label_create(lv_scr_act());
        lv_label_set_text(label_1, "WARNING");
        lv_obj_set_style_text_font(label_1, &lv_font_montserrat_48, 0);
        lv_obj_set_style_text_color(label_1, lv_palette_main(LV_PALETTE_YELLOW), 0);
        lv_obj_align(label_1, LV_ALIGN_CENTER, 0, -60);

        lv_obj_t *label_2 = lv_label_create(lv_scr_act());
        lv_label_set_text(label_2, "NO AIRCRAFTS");
        lv_obj_set_style_text_font(label_2, &lv_font_montserrat_48, 0);
        lv_obj_align_to(label_2, label_1, LV_ALIGN_OUT_BOTTOM_MID, 0, 60);

        lv_obj_t *label_3 = lv_label_create(lv_scr_act());
        lv_label_set_text(label_3, "DATA");
        lv_obj_set_style_text_font(label_3, &lv_font_montserrat_48, 0);
        lv_obj_align_to(label_3, label_2, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
      } else {
        lv_obj_t *label_1 = lv_label_create(lv_scr_act());
        lv_label_set_text_fmt(label_1, "%d ACFTS LOADED", acfts);
        lv_obj_set_style_text_font(label_1, &lv_font_montserrat_48, 0);
        lv_obj_align(label_1, LV_ALIGN_CENTER, 0, -100);

        lv_obj_t *label_2 = lv_label_create(lv_scr_act());
        lv_label_set_text(label_2, "THIS AIRCRAFT");
        lv_obj_set_style_text_font(label_2, &lv_font_montserrat_48, 0);
        lv_obj_align_to(label_2, label_1, LV_ALIGN_OUT_BOTTOM_MID, 0, 100);

        lv_obj_t *label_3 = lv_label_create(lv_scr_act());
        lv_label_set_text(label_3, reg);
        lv_obj_set_style_text_font(label_3, &lv_font_montserrat_48, 0);
        lv_obj_align_to(label_3, label_2, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

        lv_obj_t *label_4 = lv_label_create(lv_scr_act());
        lv_label_set_text(label_4, mam);
        lv_obj_set_style_text_font(label_4, &lv_font_montserrat_48, 0);
        lv_obj_align_to(label_4, label_3, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

        lv_obj_t *label_5 = lv_label_create(lv_scr_act());
        lv_label_set_text_fmt(label_5, "CN: %s", cn);
        lv_obj_set_style_text_font(label_5, &lv_font_montserrat_48, 0);
        lv_obj_align_to(label_5, label_4, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
      }
#endif /* LVGL_VERSION_MAJOR == 8 */

      lvgl_port_unlock();

      delay(3000);

      break;
    case DISPLAY_NONE:
    default:
      break;
    }
  }
}

#endif /* USE_DSI */
