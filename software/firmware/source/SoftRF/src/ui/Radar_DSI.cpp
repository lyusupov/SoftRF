/*
 * View_Radar_DSI.cpp
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

#if defined(USE_DSI)

#include <lvgl.h>
#include <TimeLib.h>

#include "../driver/DSI.h"
#include "../driver/EPD.h"
#include "../driver/LED.h"
#include "../system/LVGLHelper.h"
#include "../driver/EEPROM.h"
#include "../TrafficHelper.h"
#include "../driver/RF.h"

#define TEXT_VIEW_LINE_LENGTH   16      /* characters */

static int DSI_zoom = ZOOM_MEDIUM;

static int DSI_current = 1;
static int prev_j      = 0;

static lv_style_t style_line_own_position;
static lv_style_t style_line_traffic;

const char *DSI_Aircraft_Type[] = {
  [AIRCRAFT_TYPE_UNKNOWN]    = "Unknown",
  [AIRCRAFT_TYPE_GLIDER]     = "Glider",
  [AIRCRAFT_TYPE_TOWPLANE]   = "Towplane",
  [AIRCRAFT_TYPE_HELICOPTER] = "Helicopter",
  [AIRCRAFT_TYPE_PARACHUTE]  = "Parachute",
  [AIRCRAFT_TYPE_DROPPLANE]  = "Dropplane",
  [AIRCRAFT_TYPE_HANGGLIDER] = "Hangglider",
  [AIRCRAFT_TYPE_PARAGLIDER] = "Paraglider",
  [AIRCRAFT_TYPE_POWERED]    = "Powered",
  [AIRCRAFT_TYPE_JET]        = "Jet",
#if defined(EXCLUDE_AIR7)
  [AIRCRAFT_TYPE_UFO]        = "UFO",
#else
  [AIRCRAFT_TYPE_GYROCOPTER] = "Gyrocopter",
#endif /* EXCLUDE_AIR7 */
  [AIRCRAFT_TYPE_BALLOON]    = "Balloon",
  [AIRCRAFT_TYPE_ZEPPELIN]   = "Zeppelin",
  [AIRCRAFT_TYPE_UAV]        = "UAV",
#if defined(EXCLUDE_AIR7)
  [AIRCRAFT_TYPE_RESERVED]   = "Reserved",
#else
  [AIRCRAFT_TYPE_AIRFIELD]   = "Airfield",
#endif /* EXCLUDE_AIR7 */
  [AIRCRAFT_TYPE_STATIC]     = "Static"
};

void DSI_radar_setup()
{
  DSI_zoom = ui->zoom;

  lvgl_port_lock(-1);
#if LVGL_VERSION_MAJOR == 8
  lv_style_init(&style_line_own_position);
  lv_style_set_line_width(&style_line_own_position, 4);
  lv_style_set_line_color(&style_line_own_position, lv_color_white());
  lv_style_set_line_rounded(&style_line_own_position, false);

  lv_style_init(&style_line_traffic);
  lv_style_set_line_width(&style_line_traffic, 4);
  lv_style_set_line_color(&style_line_traffic, lv_color_white());
  lv_style_set_line_rounded(&style_line_traffic, false);
#endif /* LVGL_VERSION_MAJOR == 8 */
  lvgl_port_unlock();
}

void DSI_radar_loop()
{
  bool hasFix = isValidGNSSFix() || (settings->mode == SOFTRF_MODE_TXRX_TEST);
//  bool hasTraffic = (Traffic_Count() > 0);

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

  if (ui->units == UNITS_METRIC || ui->units == UNITS_MIXED) {
    switch(DSI_zoom)
    {
    case ZOOM_LOWEST:
      divider = 75000; /* 75 KM */
      break;
    case ZOOM_LOW:
      divider =  5000; /*  5 KM */
      break;
    case ZOOM_HIGH:
      divider =  1000; /*  1 KM */
      break;
    case ZOOM_MEDIUM:
    default:
      divider =  2000;  /* 2 KM */
      break;
    }
  } else {
    switch(DSI_zoom)
    {
    case ZOOM_LOWEST:
      divider = 74080;  /* 40 NM */
      break;
    case ZOOM_LOW:
      divider =  9260;  /*  5 NM */
      break;
    case ZOOM_HIGH:
      divider = 1852;   /*  1 NM */
      break;
    case ZOOM_MEDIUM:
    default:
      divider = 3704;   /*  2 NM */
      break;
    }
  }

  int dx = 0;
  switch (hw_info.display)
  {
  case DISPLAY_AMOLED_LILYGO_4_1:
    dx = 3;
    break;
  case DISPLAY_TFT_WIRELESSTAG_7:
  case DISPLAY_TFT_LILYGO_4_05:
  default:
    break;
  }

  lvgl_port_lock(-1);

#if LVGL_VERSION_MAJOR == 8
  lv_obj_clean(lv_scr_act());

  lv_obj_t *circle_1 = lv_obj_create(lv_scr_act());
  lv_obj_set_size(circle_1, radar_w - 2*dx, radar_w);
  lv_obj_set_pos(circle_1, dx, 0);
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

  /* little airplane */
  static lv_point_t line1_points[] = { {20, 0}, {20,40} };
  static lv_point_t line2_points[] = { { 0,20}, {40,20} };
  static lv_point_t line3_points[] = { { 4,18}, {36,18} };
  static lv_point_t line4_points[] = { {12,40}, {28,40} };

  lv_obj_t * line1;
  line1 = lv_line_create(lv_scr_act());
  lv_obj_set_pos(line1, radar_w / 2 - 20, radar_w / 2 - 20);
  lv_line_set_points(line1, line1_points, 2);
  lv_obj_add_style(line1, &style_line_own_position, 0);

  lv_obj_t * line2;
  line2 = lv_line_create(lv_scr_act());
  lv_obj_set_pos(line2, radar_w / 2 - 20, radar_w / 2 - 20);
  lv_line_set_points(line2, line2_points, 2);
  lv_obj_add_style(line2, &style_line_own_position, 0);

  lv_obj_t * line3;
  line3 = lv_line_create(lv_scr_act());
  lv_obj_set_pos(line3, radar_w / 2 - 20, radar_w / 2 - 20);
  lv_line_set_points(line3, line3_points, 2);
  lv_obj_add_style(line3, &style_line_own_position, 0);

  lv_obj_t * line4;
  line4 = lv_line_create(lv_scr_act());
  lv_obj_set_pos(line4, radar_w / 2 - 20, radar_w / 2 - 20);
  lv_line_set_points(line4, line4_points, 2);
  lv_obj_add_style(line4, &style_line_own_position, 0);
#endif /* LVGL_VERSION_MAJOR == 8 */

  switch (ui->orientation)
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
      snprintf(cog_text, sizeof(cog_text), "%03d", ThisAircraft.course);

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
    break;
  default:
    /* TBD */
    break;
  }

  const char *scale_lbl_1_txt;

  if (ui->units == UNITS_METRIC || ui->units == UNITS_MIXED) {
    scale_lbl_1_txt = DSI_zoom == ZOOM_LOWEST ? "75 KM" :
                      DSI_zoom == ZOOM_LOW    ? " 5 KM" :
                      DSI_zoom == ZOOM_MEDIUM ? " 2 KM" :
                      DSI_zoom == ZOOM_HIGH   ? " 1 KM" : "";
  } else { /* TODO */
    scale_lbl_1_txt = DSI_zoom == ZOOM_LOWEST ? "40 NM" :
                      DSI_zoom == ZOOM_LOW    ? " 5 NM" :
                      DSI_zoom == ZOOM_MEDIUM ? " 2 NM" :
                      DSI_zoom == ZOOM_HIGH   ? " 1 NM" : "";
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

  if (ui->units == UNITS_METRIC || ui->units == UNITS_MIXED) {
    scale_lbl_2_txt = DSI_zoom == ZOOM_LOWEST ? "37 KM" :
                      DSI_zoom == ZOOM_LOW    ? "2.5 K" :
                      DSI_zoom == ZOOM_MEDIUM ? " 1 KM" :
                      DSI_zoom == ZOOM_HIGH   ? "500 M" : "";
  } else { /* TODO */
    scale_lbl_2_txt = DSI_zoom == ZOOM_LOWEST ? "20 NM" :
                      DSI_zoom == ZOOM_LOW    ? "2.5 M" :
                      DSI_zoom == ZOOM_MEDIUM ? " 1 NM" :
                      DSI_zoom == ZOOM_HIGH   ? "0.5 M" : "";
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

  int32_t radius = radar_w / 2;

  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (hasFix && Container[i].addr &&
       (now() - Container[i].timestamp) <= DSI_EXPIRATION_TIME) {

      int16_t rel_x;
      int16_t rel_y;
      float distance;
      float bearing_f;

      bool isTeam = (Container[i].addr == ui->team) ;

      distance  = Container[i].distance;
      bearing_f = Container[i].bearing;

      switch (ui->orientation)
      {
      case DIRECTION_NORTH_UP:
        break;
      case DIRECTION_TRACK_UP:
        bearing_f -= ThisAircraft.course;
        break;
      default:
        /* TBD */
        break;
      }

      rel_x = constrain(distance * sin(radians(bearing_f)),
                                   -32768, 32767);
      rel_y = constrain(distance * cos(radians(bearing_f)),
                                   -32768, 32767);

      int16_t x = ((int32_t) rel_x * (int32_t) radius) / divider;
      int16_t y = ((int32_t) rel_y * (int32_t) radius) / divider;

      float RelativeVertical = Container[i].altitude - ThisAircraft.altitude;

      lv_color_t color = Container[i].alarm_level == ALARM_LEVEL_URGENT ?
                         lv_palette_main(LV_PALETTE_RED) :
                        (Container[i].alarm_level == ALARM_LEVEL_IMPORTANT ?
                         lv_palette_main(LV_PALETTE_YELLOW) :
                         lv_palette_main(LV_PALETTE_BLUE));

      static lv_point_t triangle_down_pts[] = { {0,0}, {20,0}, {10,20}, {0,0} };
      static lv_point_t triangle_up_pts[] = { {0,20}, {20,20}, {10,0}, {0,20} };

#if LVGL_VERSION_MAJOR == 8
      lv_style_set_line_color(&style_line_traffic, color);

      if        (RelativeVertical >   DSI_RADAR_V_THRESHOLD) {
        lv_obj_t * triangle_up;
        triangle_up = lv_line_create(lv_scr_act());
        lv_obj_set_pos(triangle_up, radar_w / 2 + x - 10, radar_w / 2 - y - 10);
        lv_line_set_points(triangle_up, triangle_up_pts, 4);
        lv_obj_add_style(triangle_up, &style_line_traffic, 0);
      } else if (RelativeVertical < - DSI_RADAR_V_THRESHOLD) {
        lv_obj_t * triangle_down;
        triangle_down = lv_line_create(lv_scr_act());
        lv_obj_set_pos(triangle_down, radar_w / 2 + x - 10, radar_w / 2 - y - 10);
        lv_line_set_points(triangle_down, triangle_down_pts, 4);
        lv_obj_add_style(triangle_down, &style_line_traffic, 0);
      } else {
        lv_obj_t *circle_3 = lv_obj_create(lv_scr_act());
        lv_obj_set_size(circle_3, 20, 20);
        lv_obj_set_pos(circle_3, radar_w / 2 + x - 10, radar_w / 2 - y - 10);
        lv_obj_set_style_radius(circle_3, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_color(circle_3, color, 0);
        lv_obj_set_style_border_width(circle_3, 4, 0);
        lv_obj_set_style_bg_color(circle_3, lv_color_black(), 0);
      }
#endif /* LVGL_VERSION_MAJOR == 8 */
    }
  }

#if LVGL_VERSION_MAJOR == 8
  int dy = 10;
  dx += 5;

  lv_obj_t *rect = lv_obj_create(lv_scr_act());
  lv_obj_set_scrollbar_mode(rect, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_size(rect, radar_w - 2*dx, lv_disp_get_ver_res(NULL) - radar_w - dy);
  lv_obj_set_pos(rect, dx, radar_w);
  lv_obj_set_style_radius(rect, 80, 0);
  lv_obj_set_style_bg_color(rect, lv_color_black(), 0);
  lv_obj_set_style_border_color(rect, lv_palette_main(LV_PALETTE_GREY), 0);
  lv_obj_set_style_border_width(rect, 2, 0);
#endif /* LVGL_VERSION_MAJOR == 8 */

  int j=0;
  int bearing_i;
  char info_line [TEXT_VIEW_LINE_LENGTH];
  char id_text   [TEXT_VIEW_LINE_LENGTH];

  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].addr && (now() - Container[i].timestamp) <= DSI_EXPIRATION_TIME) {

      traffic_by_dist[j].fop = &Container[i];
      traffic_by_dist[j].distance = Container[i].distance;
      j++;
    }
  }

  if (hasFix && j > 0) {
    const char *u_dist, *u_alt, *u_spd;
    float disp_dist;
    int   disp_alt, disp_spd;

    qsort(traffic_by_dist, j, sizeof(traffic_by_dist_t), traffic_cmp_by_distance);

    if (DSI_current > j) {
      if (prev_j > j) {
        DSI_current = j;
      } else {
        DSI_current = 1;
      }
    }
    prev_j = j;

    bearing_i = (int) traffic_by_dist[DSI_current - 1].fop->bearing;

    /* This bearing is always relative to current ground track */
//  if (ui->orientation == DIRECTION_TRACK_UP) {
      bearing_i -= ThisAircraft.course;
//  }

    if (bearing_i < 0) {
      bearing_i += 360;
    }

    int oclock = ((bearing_i + 15) % 360) / 30;
    float Relative_Vertical = traffic_by_dist[DSI_current - 1].fop->altitude -
                              ThisAircraft.altitude;

    switch (ui->units)
    {
    case UNITS_IMPERIAL:
      u_dist = "nm";
      u_alt  = "f";
      u_spd  = "kts";
      disp_dist = (traffic_by_dist[DSI_current - 1].distance * _GPS_MILES_PER_METER) /
                  _GPS_MPH_PER_KNOT;
      disp_alt  = abs((int) (Relative_Vertical * _GPS_FEET_PER_METER));
      disp_spd  = traffic_by_dist[DSI_current - 1].fop->speed;
      break;
    case UNITS_MIXED:
      u_dist = "km";
      u_alt  = "f";
      u_spd  = "kph";
      disp_dist = traffic_by_dist[DSI_current - 1].distance / 1000.0;
      disp_alt  = abs((int) (Relative_Vertical * _GPS_FEET_PER_METER));
      disp_spd  = traffic_by_dist[DSI_current - 1].fop->speed * _GPS_KMPH_PER_KNOT;
      break;
    case UNITS_METRIC:
    default:
      u_dist = "km";
      u_alt  = "m";
      u_spd  = "kph";
      disp_dist = traffic_by_dist[DSI_current - 1].distance / 1000.0;
      disp_alt  = abs((int) Relative_Vertical);
      disp_spd  = traffic_by_dist[DSI_current - 1].fop->speed * _GPS_KMPH_PER_KNOT;
      break;
    }

    if (ui->idpref == ID_TYPE) {
      uint8_t acft_type = traffic_by_dist[DSI_current - 1].fop->aircraft_type;
      acft_type = acft_type > AIRCRAFT_TYPE_STATIC ? AIRCRAFT_TYPE_UNKNOWN : acft_type;
      strncpy(id_text, DSI_Aircraft_Type[acft_type], sizeof(id_text));
    } else {
      uint32_t id = traffic_by_dist[DSI_current - 1].fop->addr;

      if (!(SoC->ADB_ops && SoC->ADB_ops->query(DB_OGN, id, id_text, sizeof(id_text)))) {
        snprintf(id_text, sizeof(id_text), "ID: %06X", id);
      }
    }

#if LVGL_VERSION_MAJOR == 8
    snprintf(info_line, sizeof(info_line), "Traffic %d of %d", DSI_current, j);

    lv_obj_t *line_1 = lv_label_create(lv_scr_act());
    lv_label_set_text(line_1, info_line);
    lv_obj_set_style_text_font(line_1, &lv_font_montserrat_48, 0);
    lv_obj_align_to(line_1, rect, LV_ALIGN_TOP_MID, 0, 10);

    if (oclock == 0) {
      strcpy(info_line, "ahead");
    } else {
      snprintf(info_line, sizeof(info_line), " %2d o'clock", oclock);
    }

    lv_obj_t *line_2 = lv_label_create(lv_scr_act());
    lv_label_set_text(line_2, info_line);
    lv_obj_set_style_text_font(line_2, &lv_font_montserrat_48, 0);
    lv_obj_align_to(line_2, line_1, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    snprintf(info_line, sizeof(info_line), "%4.1f %s out", disp_dist, u_dist);

    lv_obj_t *line_3 = lv_label_create(lv_scr_act());
    lv_label_set_text(line_3, info_line);
    lv_obj_set_style_text_font(line_3, &lv_font_montserrat_48, 0);
    lv_obj_align_to(line_3, line_2, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    snprintf(info_line, sizeof(info_line), "%4d %s ", disp_alt, u_alt);

    if ((int) Relative_Vertical > 50) {
      strcat(info_line, "above");
    } else if ((int) Relative_Vertical < -50) {
      strcat(info_line, "below");
    } else {
      strcpy(info_line, "same alt.");
    }

    lv_obj_t *line_4 = lv_label_create(lv_scr_act());
    lv_label_set_text(line_4, info_line);
    lv_obj_set_style_text_font(line_4, &lv_font_montserrat_48, 0);
    lv_obj_align_to(line_4, line_3, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    snprintf(info_line, sizeof(info_line), "CoG %3d deg",
             (int) traffic_by_dist[DSI_current - 1].fop->course);

    lv_obj_t *line_5 = lv_label_create(lv_scr_act());
    lv_label_set_text(line_5, info_line);
    lv_obj_set_style_text_font(line_5, &lv_font_montserrat_48, 0);
    lv_obj_align_to(line_5, line_4, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    snprintf(info_line, sizeof(info_line), "GS  %3d %s", disp_spd, u_spd);

    lv_obj_t *line_6 = lv_label_create(lv_scr_act());
    lv_label_set_text(line_6, info_line);
    lv_obj_set_style_text_font(line_6, &lv_font_montserrat_48, 0);
    lv_obj_align_to(line_6, line_5, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    lv_obj_t *line_7 = lv_label_create(lv_scr_act());
    lv_label_set_text(line_7, id_text);
    lv_obj_set_style_text_font(line_7, &lv_font_montserrat_48, 0);
    lv_obj_align_to(line_7, line_6, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
#endif /* LVGL_VERSION_MAJOR == 8 */
  } else {
#if LVGL_VERSION_MAJOR == 8
    lv_obj_t *label_1 = lv_label_create(lv_scr_act());
    lv_label_set_text(label_1, hasFix == false ? NO_FIX_TEXT : NO_TRAFFIC_TEXT);
    lv_obj_set_style_text_font(label_1, &lv_font_montserrat_48, 0);
    lv_obj_align_to(label_1, rect, LV_ALIGN_CENTER, 0, 0);
#endif /* LVGL_VERSION_MAJOR == 8 */
  }

  lvgl_port_unlock();
}

void DSI_radar_zoom()
{
  if (DSI_zoom < ZOOM_HIGH) DSI_zoom++;
}

void DSI_radar_unzoom()
{
  if (DSI_zoom > ZOOM_LOWEST) DSI_zoom--;
}
#endif /* USE_DSI */
