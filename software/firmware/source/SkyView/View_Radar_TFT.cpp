/*
 * View_Radar_TFT.cpp
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

#if defined(USE_TFT)

#include <lvgl.h>
#include <TimeLib.h>

#include "TFTHelper.h"
#include "LVGLHelper.h"
#include "EEPROMHelper.h"
#include "TrafficHelper.h"
#include "NMEAHelper.h"
#include "GDL90Helper.h"

static int TFT_zoom = ZOOM_MEDIUM;

void TFT_radar_setup()
{
  TFT_zoom = settings->zoom;
}

void TFT_radar_loop()
{
  bool hasData = settings->protocol == PROTOCOL_NMEA  ? NMEA_isConnected()  :
                 settings->protocol == PROTOCOL_GDL90 ? GDL90_isConnected() :
                 false;
  bool hasFix  = false;

  if (hasData) {
    hasFix = settings->protocol == PROTOCOL_NMEA  ? isValidGNSSFix()   :
             settings->protocol == PROTOCOL_GDL90 ? GDL90_hasOwnShip() :
             false;
  }

  /* divider is a half of full scale */
  int32_t divider = 2000;

  uint16_t radar_x = 0;
  uint16_t radar_y = 0;
  uint16_t radar_w;

#if LVGL_VERSION_MAJOR == 8
  radar_w = min(lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
#endif /* LVGL_VERSION_MAJOR == 8 */

  uint16_t radar_center_x = radar_w / 2;
  uint16_t radar_center_y = radar_y + radar_w / 2;

  if (settings->units == UNITS_METRIC || settings->units == UNITS_MIXED) {
    switch(TFT_zoom)
    {
    case ZOOM_LOWEST:
      divider = 10000; /* 20 KM */
      break;
    case ZOOM_LOW:
      divider =  5000; /* 10 KM */
      break;
    case ZOOM_HIGH:
      divider =  1000; /*  2 KM */
      break;
    case ZOOM_MEDIUM:
    default:
      divider =  2000;  /* 4 KM */
      break;
    }
  } else {
    switch(TFT_zoom)
    {
    case ZOOM_LOWEST:
      divider = 9260;  /* 10 NM */
      break;
    case ZOOM_LOW:
      divider = 4630;  /*  5 NM */
      break;
    case ZOOM_HIGH:
      divider =  926;  /*  1 NM */
      break;
    case ZOOM_MEDIUM:  /*  2 NM */
    default:
      divider = 1852;
      break;
    }
  }

  lvgl_port_lock(-1);

#if LVGL_VERSION_MAJOR == 8
  lv_obj_clean(lv_scr_act());

  lv_obj_t *circle_1 = lv_obj_create(lv_scr_act());
  lv_obj_set_size(circle_1, radar_w, radar_w);
  lv_obj_set_pos(circle_1, 0, 0);
  lv_obj_set_style_radius(circle_1, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_color(circle_1, lv_palette_main(LV_PALETTE_GREEN), 0);
  lv_obj_set_style_border_width(circle_1, 2, 0);
  lv_obj_set_style_bg_color(circle_1, lv_color_black(), 0);

  lv_obj_t *circle_2 = lv_obj_create(lv_scr_act());
  lv_obj_set_size(circle_2, radar_w / 2, radar_w / 2);
  lv_obj_set_pos(circle_2, radar_w / 4, radar_w / 4);
  lv_obj_set_style_radius(circle_2, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_color(circle_2, lv_palette_main(LV_PALETTE_GREEN), 0);
  lv_obj_set_style_border_width(circle_2, 2, 0);
  lv_obj_set_style_bg_color(circle_2, lv_color_black(), 0);

  lv_obj_t *rect = lv_obj_create(lv_scr_act());
  lv_obj_set_scrollbar_mode(rect, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_size(rect, lv_disp_get_hor_res(NULL) - radar_w, radar_w);
  lv_obj_set_pos(rect, radar_w, 0);
  lv_obj_set_style_radius(rect, 10, 0);
  lv_obj_set_style_bg_color(rect, lv_color_black(), 0);
  lv_obj_set_style_border_color(rect, lv_palette_main(LV_PALETTE_GREY), 0);
  lv_obj_set_style_border_width(rect, 2, 0);

  if (hasData == false || hasFix == false) {
    lv_obj_t *label_1 = lv_label_create(lv_scr_act());
    lv_label_set_text(label_1, hasData == false ? NO_DATA_TEXT :
                               hasFix  == false ? NO_FIX_TEXT : "");
    lv_obj_set_style_text_font(label_1, &lv_font_montserrat_48, 0);
    lv_obj_align_to(label_1, rect, LV_ALIGN_CENTER, 0, 0);
  }

  /* little airplane */
  static lv_point_t line1_points[] = { {20, 0}, {20,40} };
  static lv_point_t line2_points[] = { { 0,20}, {40,20} };
  static lv_point_t line3_points[] = { { 4,18}, {36,18} };
  static lv_point_t line4_points[] = { {12,40}, {28,40} };

  static lv_style_t style_line;
  lv_style_init(&style_line);
  lv_style_set_line_width(&style_line, 4);
  lv_style_set_line_color(&style_line, lv_color_white());
  lv_style_set_line_rounded(&style_line, false);

  lv_obj_t * line1;
  line1 = lv_line_create(lv_scr_act());
  lv_obj_set_pos(line1, radar_w / 2 - 20, radar_w / 2 - 20);
  lv_line_set_points(line1, line1_points, 2);
  lv_obj_add_style(line1, &style_line, 0);

  lv_obj_t * line2;
  line2 = lv_line_create(lv_scr_act());
  lv_obj_set_pos(line2, radar_w / 2 - 20, radar_w / 2 - 20);
  lv_line_set_points(line2, line2_points, 2);
  lv_obj_add_style(line2, &style_line, 0);

  lv_obj_t * line3;
  line3 = lv_line_create(lv_scr_act());
  lv_obj_set_pos(line3, radar_w / 2 - 20, radar_w / 2 - 20);
  lv_line_set_points(line3, line3_points, 2);
  lv_obj_add_style(line3, &style_line, 0);

  lv_obj_t * line4;
  line4 = lv_line_create(lv_scr_act());
  lv_obj_set_pos(line4, radar_w / 2 - 20, radar_w / 2 - 20);
  lv_line_set_points(line4, line4_points, 2);
  lv_obj_add_style(line4, &style_line, 0);
#endif /* LVGL_VERSION_MAJOR == 8 */

  switch (settings->orientation)
  {
  case DIRECTION_NORTH_UP:
    {
#if LVGL_VERSION_MAJOR == 8
      lv_obj_t *rose_1 = lv_label_create(lv_scr_act());
      lv_label_set_text(rose_1, "N");
      lv_obj_set_style_text_font(rose_1, &lv_font_montserrat_24, 0);
      lv_obj_set_style_text_color(rose_1, lv_palette_main(LV_PALETTE_GREEN), 0);
      lv_obj_set_style_bg_color(rose_1, lv_color_black(), 0);
      lv_obj_align_to(rose_1, circle_1, LV_ALIGN_TOP_MID, 0, 0);

      lv_obj_t *rose_2 = lv_label_create(lv_scr_act());
      lv_label_set_text(rose_2, "W");
      lv_obj_set_style_text_font(rose_2, &lv_font_montserrat_24, 0);
      lv_obj_set_style_text_color(rose_2, lv_palette_main(LV_PALETTE_GREEN), 0);
      lv_obj_set_style_bg_color(rose_2, lv_color_black(), 0);
      lv_obj_align_to(rose_2, circle_1, LV_ALIGN_LEFT_MID, 0, 0);

      lv_obj_t *rose_3 = lv_label_create(lv_scr_act());
      lv_label_set_text(rose_3, "E");
      lv_obj_set_style_text_font(rose_3, &lv_font_montserrat_24, 0);
      lv_obj_set_style_text_color(rose_3, lv_palette_main(LV_PALETTE_GREEN), 0);
      lv_obj_set_style_bg_color(rose_3, lv_color_black(), 0);
      lv_obj_align_to(rose_3, circle_1, LV_ALIGN_RIGHT_MID, 0, 0);

      lv_obj_t *rose_4 = lv_label_create(lv_scr_act());
      lv_label_set_text(rose_4, "S");
      lv_obj_set_style_text_font(rose_4, &lv_font_montserrat_24, 0);
      lv_obj_set_style_text_color(rose_4, lv_palette_main(LV_PALETTE_GREEN), 0);
      lv_obj_set_style_bg_color(rose_4, lv_color_black(), 0);
      lv_obj_align_to(rose_4, circle_1, LV_ALIGN_BOTTOM_MID, 0, 0);
#endif /* LVGL_VERSION_MAJOR == 8 */
    }
    break;
  case DIRECTION_TRACK_UP:
    {
      char cog_text[6];
      snprintf(cog_text, sizeof(cog_text), "%03d", ThisAircraft.Track);

#if LVGL_VERSION_MAJOR == 8
      lv_obj_t *rose_1 = lv_label_create(lv_scr_act());
      lv_label_set_text(rose_1, cog_text);
      lv_obj_set_style_text_font(rose_1, &lv_font_montserrat_24, 0);
      lv_obj_set_style_text_color(rose_1, lv_palette_main(LV_PALETTE_GREEN), 0);
      lv_obj_set_style_bg_color(rose_1, lv_color_black(), 0);
      lv_obj_align_to(rose_1, circle_1, LV_ALIGN_TOP_MID, 0, 0);

      lv_obj_t *rose_2 = lv_label_create(lv_scr_act());
      lv_label_set_text(rose_2, "L");
      lv_obj_set_style_text_font(rose_2, &lv_font_montserrat_24, 0);
      lv_obj_set_style_text_color(rose_2, lv_palette_main(LV_PALETTE_GREEN), 0);
      lv_obj_set_style_bg_color(rose_2, lv_color_black(), 0);
      lv_obj_align_to(rose_2, circle_1, LV_ALIGN_LEFT_MID, 0, 0);

      lv_obj_t *rose_3 = lv_label_create(lv_scr_act());
      lv_label_set_text(rose_3, "R");
      lv_obj_set_style_text_font(rose_3, &lv_font_montserrat_24, 0);
      lv_obj_set_style_text_color(rose_3, lv_palette_main(LV_PALETTE_GREEN), 0);
      lv_obj_set_style_bg_color(rose_3, lv_color_black(), 0);
      lv_obj_align_to(rose_3, circle_1, LV_ALIGN_RIGHT_MID, 0, 0);

      lv_obj_t *rose_4 = lv_label_create(lv_scr_act());
      lv_label_set_text(rose_4, "B");
      lv_obj_set_style_text_font(rose_4, &lv_font_montserrat_24, 0);
      lv_obj_set_style_text_color(rose_4, lv_palette_main(LV_PALETTE_GREEN), 0);
      lv_obj_set_style_bg_color(rose_4, lv_color_black(), 0);
      lv_obj_align_to(rose_4, circle_1, LV_ALIGN_BOTTOM_MID, 0, 0);
#endif /* LVGL_VERSION_MAJOR == 8 */
    }
#if 0
    sprite->drawRoundRect( x - 2, y - tbh - 2,
                            tbw + 8, tbh + 6,
                            4, TFT_WHITE);
#endif
    break;
  default:
    /* TBD */
    break;
  }

  const char *scale_lbl_1_txt;

  if (settings->units == UNITS_METRIC || settings->units == UNITS_MIXED) {
    scale_lbl_1_txt = TFT_zoom == ZOOM_LOWEST ? "10 KM" :
                      TFT_zoom == ZOOM_LOW    ? " 5 KM" :
                      TFT_zoom == ZOOM_MEDIUM ? " 2 KM" :
                      TFT_zoom == ZOOM_HIGH   ? " 1 KM" : "";
  } else { /* TODO */
    scale_lbl_1_txt = TFT_zoom == ZOOM_LOWEST ? "10 NM" :
                      TFT_zoom == ZOOM_LOW    ? " 5 NM" :
                      TFT_zoom == ZOOM_MEDIUM ? " 2 NM" :
                      TFT_zoom == ZOOM_HIGH   ? " 1 NM" : "";
  }

#if LVGL_VERSION_MAJOR == 8
  lv_obj_t *pad_1 = lv_obj_create(lv_scr_act());
  lv_obj_set_size(pad_1, 80, 30);
  lv_obj_set_pos(pad_1, (radar_w * 8) / 10, (radar_w * 8) / 10);
  lv_obj_invalidate(pad_1);

  lv_obj_t *scale_lbl_1 = lv_label_create(lv_scr_act());
  lv_label_set_text(scale_lbl_1, scale_lbl_1_txt);
  lv_obj_set_style_text_font(scale_lbl_1, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(scale_lbl_1, lv_palette_main(LV_PALETTE_GREEN), 0);
  lv_obj_set_pos(scale_lbl_1, (radar_w * 8) / 10, (radar_w * 8) / 10);
#endif /* LVGL_VERSION_MAJOR == 8 */

  const char *scale_lbl_2_txt;

  if (settings->units == UNITS_METRIC || settings->units == UNITS_MIXED) {
    scale_lbl_2_txt = TFT_zoom == ZOOM_LOWEST ? " 5 KM" :
                      TFT_zoom == ZOOM_LOW    ? "2.5 K" :
                      TFT_zoom == ZOOM_MEDIUM ? " 1 KM" :
                      TFT_zoom == ZOOM_HIGH   ? "500 M" : "";
  } else { /* TODO */
    scale_lbl_2_txt = TFT_zoom == ZOOM_LOWEST ? "10 NM" :
                      TFT_zoom == ZOOM_LOW    ? " 5 NM" :
                      TFT_zoom == ZOOM_MEDIUM ? " 2 NM" :
                      TFT_zoom == ZOOM_HIGH   ? " 1 NM" : "";
  }

#if LVGL_VERSION_MAJOR == 8
  lv_obj_t *pad_2 = lv_obj_create(lv_scr_act());
  lv_obj_set_size(pad_2, 80, 30);
  lv_obj_set_pos(pad_2, (radar_w * 6) / 10 + 20, (radar_w * 6) / 10);
  lv_obj_invalidate(pad_2);

  lv_obj_t *scale_lbl_2 = lv_label_create(lv_scr_act());
  lv_label_set_text(scale_lbl_2, scale_lbl_2_txt);
  lv_obj_set_style_text_font(scale_lbl_2, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(scale_lbl_2, lv_palette_main(LV_PALETTE_GREEN), 0);
  lv_obj_set_pos(scale_lbl_2, (radar_w * 6) / 10 + 20, (radar_w * 6) / 10);
#endif /* LVGL_VERSION_MAJOR == 8 */

#if 0
  if (settings->units == UNITS_METRIC || settings->units == UNITS_MIXED) {
    sprite->print(TFT_zoom == ZOOM_LOWEST ? "20 KM" :
                  TFT_zoom == ZOOM_LOW    ? "10 KM" :
                  TFT_zoom == ZOOM_MEDIUM ? " 4 KM" :
                  TFT_zoom == ZOOM_HIGH   ? " 2 KM" : "");
  } else {
    sprite->print(TFT_zoom == ZOOM_LOWEST ? "10 NM" :
                  TFT_zoom == ZOOM_LOW    ? " 5 NM" :
                  TFT_zoom == ZOOM_MEDIUM ? " 2 NM" :
                  TFT_zoom == ZOOM_HIGH   ? " 1 NM" : "");
  }
#endif

  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].ID && (now() - Container[i].timestamp) <= TFT_EXPIRATION_TIME) {

      int16_t rel_x;
      int16_t rel_y;
      float distance;
      float bearing;

      switch (settings->orientation)
      {
      case DIRECTION_NORTH_UP:
        rel_x = Container[i].RelativeEast;
        rel_y = Container[i].RelativeNorth;
        break;
      case DIRECTION_TRACK_UP:
        distance = sqrtf(Container[i].RelativeNorth * Container[i].RelativeNorth +
                         Container[i].RelativeEast  * Container[i].RelativeEast);

        bearing = atan2f(Container[i].RelativeNorth,
                         Container[i].RelativeEast) * 180.0 / PI;  /* -180 ... 180 */

        /* convert from math angle into course relative to north */
        bearing = (bearing <= 90.0 ? 90.0 - bearing :
                                    450.0 - bearing);

        bearing -= ThisAircraft.Track;

        rel_x = constrain(distance * sin(radians(bearing)),
                                     -32768, 32767);
        rel_y = constrain(distance * cos(radians(bearing)),
                                     -32768, 32767);
        break;
      default:
        /* TBD */
        break;
      }

      int16_t x = ((int32_t) rel_x * (int32_t) radar_w / 2) / divider;
      int16_t y = ((int32_t) rel_y * (int32_t) radar_w / 2) / divider;

      lv_color_t color = Container[i].AlarmLevel == ALARM_LEVEL_URGENT ?
                         lv_palette_main(LV_PALETTE_RED) :
                        (Container[i].AlarmLevel == ALARM_LEVEL_IMPORTANT ?
                         lv_palette_main(LV_PALETTE_YELLOW) :
                         lv_palette_main(LV_PALETTE_GREEN));

      static lv_point_t triangle_down_pts[] = { {0,0}, {20,0}, {10,20}, {0,0} };
      static lv_point_t triangle_up_pts[] = { {0,20}, {20,20}, {10,0}, {0,20} };

#if LVGL_VERSION_MAJOR == 8
      static lv_style_t style_line;
      lv_style_init(&style_line);
      lv_style_set_line_width(&style_line, 4);
      lv_style_set_line_color(&style_line, color);
      lv_style_set_line_rounded(&style_line, false);

      if        (Container[i].RelativeVertical >   TFT_RADAR_V_THRESHOLD) {
        lv_obj_t * triangle_up;
        triangle_up = lv_line_create(lv_scr_act());
        lv_obj_set_pos(triangle_up, radar_w / 2 + x - 10, radar_w / 2 + y - 10);
        lv_line_set_points(triangle_up, triangle_up_pts, 4);
        lv_obj_add_style(triangle_up, &style_line, 0);
      } else if (Container[i].RelativeVertical < - TFT_RADAR_V_THRESHOLD) {
        lv_obj_t * triangle_down;
        triangle_down = lv_line_create(lv_scr_act());
        lv_obj_set_pos(triangle_down, radar_w / 2 + x - 10, radar_w / 2 + y - 10);
        lv_line_set_points(triangle_down, triangle_down_pts, 4);
        lv_obj_add_style(triangle_down, &style_line, 0);
      } else {
        lv_obj_t *circle_3 = lv_obj_create(lv_scr_act());
        lv_obj_set_size(circle_3, 20, 20);
        lv_obj_set_pos(circle_3, radar_w / 2 + x - 10, radar_w / 2 + y - 10);
        lv_obj_set_style_radius(circle_3, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_color(circle_3, color, 0);
        lv_obj_set_style_border_width(circle_3, 4, 0);
        lv_obj_set_style_bg_color(circle_3, lv_color_black(), 0);
      }
#endif /* LVGL_VERSION_MAJOR == 8 */
    }
  }

  lvgl_port_unlock();
}

void TFT_radar_zoom()
{
  if (TFT_zoom < ZOOM_HIGH) TFT_zoom++;
}

void TFT_radar_unzoom()
{
  if (TFT_zoom > ZOOM_LOWEST) TFT_zoom--;
}
#endif /* USE_TFT */
