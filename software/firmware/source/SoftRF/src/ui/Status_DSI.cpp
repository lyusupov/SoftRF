/*
 * View_Status_DSI.cpp
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

#include "../driver/DSI.h"
#include "../system/LVGLHelper.h"
#include "../driver/EEPROM.h"
#include "../driver/Battery.h"
#include "../driver/GNSS.h"
#include "../driver/RF.h"
#include "../driver/Baro.h"
#if !defined(EXCLUDE_MAVLINK)
#include "../protocol/data/MAVLink.h"
#endif /* EXCLUDE_MAVLINK */

void DSI_status_setup()
{

}

void DSI_status_loop()
{
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  float vdd = Battery_voltage() ;
  bool low_voltage = (vdd <= Battery_threshold());

  char str_lat[16];
  char str_lon[16];
  char str_alt[16];
  char str_Vcc[8];

  dtostrf(ThisAircraft.latitude,  8, 4, str_lat);
  dtostrf(ThisAircraft.longitude, 8, 4, str_lon);
  dtostrf(ThisAircraft.altitude,  7, 1, str_alt);
  dtostrf(vdd, 4, 2, str_Vcc);

  unsigned int timestamp = (unsigned int) ThisAircraft.timestamp;
  unsigned int sats;

#if !defined(EXCLUDE_MAVLINK)
  if (settings->mode == SOFTRF_MODE_UAV)
    sats = the_aircraft.gps.num_sats;
  else
#endif /* EXCLUDE_MAVLINK */
    sats = gnss.satellites.value(); // Number of satellites in use (u32)

  lvgl_port_lock(-1);

#if LVGL_VERSION_MAJOR == 8
  lv_obj_clean(lv_scr_act());

  lv_obj_t *label_1 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_1, "Device Id");
  lv_obj_set_style_text_font(label_1, &lv_font_montserrat_40, 0);
  lv_obj_align(label_1, LV_ALIGN_OUT_TOP_LEFT, 40, 20);

  lv_obj_t *data_1 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_1, "%06X", SoC->getChipId() & 0xFFFFFF);
  lv_obj_set_style_text_font(data_1, &lv_font_montserrat_40, 0);
  lv_obj_align(data_1, LV_ALIGN_TOP_RIGHT, -40, 20);

  lv_obj_t *label_2 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_2, "Software Version");
  lv_obj_set_style_text_font(label_2, &lv_font_montserrat_40, 0);
  lv_obj_align_to(label_2, label_1, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_2 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_2, "%s", SOFTRF_FIRMWARE_VERSION);
  lv_obj_set_style_text_font(data_2, &lv_font_montserrat_40, 0);
  lv_obj_align_to(data_2, data_1, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

  lv_obj_t *label_3 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_3, "SoC");
  lv_obj_set_style_text_font(label_3, &lv_font_montserrat_40, 0);
  lv_obj_align_to(label_3, label_2, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_3 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_3, "%s", SoC->name);
  lv_obj_set_style_text_font(data_3, &lv_font_montserrat_40, 0);
  lv_obj_align_to(data_3, data_2, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

  lv_obj_t *label_4 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_4, "GNSS");
  lv_obj_set_style_text_font(label_4, &lv_font_montserrat_40, 0);
  lv_obj_align_to(label_4, label_3, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_4 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_4, "%s", GNSS_name[hw_info.gnss]);
  lv_obj_set_style_text_font(data_4, &lv_font_montserrat_40, 0);
  lv_obj_align_to(data_4, data_3, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

  lv_obj_t *label_5 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_5, "Radio");
  lv_obj_set_style_text_font(label_5, &lv_font_montserrat_40, 0);
  lv_obj_align_to(label_5, label_4, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_5 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_5, "%s", rf_chip   == NULL ? "NONE" : rf_chip->name);
  lv_obj_set_style_text_font(data_5, &lv_font_montserrat_40, 0);
  lv_obj_align_to(data_5, data_4, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

  lv_obj_t *label_6 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_6, "Baro");
  lv_obj_set_style_text_font(label_6, &lv_font_montserrat_40, 0);
  lv_obj_align_to(label_6, label_5, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_6 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_6, "%s", baro_chip == NULL ? "NONE" : baro_chip->name);
  lv_obj_set_style_text_font(data_6, &lv_font_montserrat_40, 0);
  lv_obj_align_to(data_6, data_5, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

  lv_obj_t *label_7 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_7, "Uptime");
  lv_obj_set_style_text_font(label_7, &lv_font_montserrat_40, 0);
  lv_obj_align_to(label_7, label_6, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_7 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_7, "%02d:%02d:%02d", hr, min % 60, sec % 60);
  lv_obj_set_style_text_font(data_7, &lv_font_montserrat_40, 0);
  lv_obj_align_to(data_7, data_6, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

  lv_obj_t *label_8 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_8, "Free memory");
  lv_obj_set_style_text_font(label_8, &lv_font_montserrat_40, 0);
  lv_obj_align_to(label_8, label_7, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_8 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_8, "%u", SoC->getFreeHeap());
  lv_obj_set_style_text_font(data_8, &lv_font_montserrat_40, 0);
  lv_obj_align_to(data_8, data_7, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

  lv_obj_t *label_9 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_9, "Battery voltage");
  lv_obj_set_style_text_font(label_9, &lv_font_montserrat_40, 0);
  lv_obj_align_to(label_9, label_8, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_9 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_9, "%s", str_Vcc);
  lv_obj_set_style_text_font(data_9, &lv_font_montserrat_40, 0);
  lv_obj_set_style_text_color(data_9, low_voltage ?
                              lv_palette_main(LV_PALETTE_RED) :
                              lv_palette_main(LV_PALETTE_GREEN), 0);
  lv_obj_align_to(data_9, data_8, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

  lv_obj_t *label_10 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_10, "            Packets");
  lv_obj_set_style_text_font(label_10, &lv_font_montserrat_40, 0);
  lv_obj_align_to(label_10, label_9, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_10 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_10, " ");
  lv_obj_set_style_text_font(data_10, &lv_font_montserrat_40, 0);
  lv_obj_align_to(data_10, data_9, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

  lv_obj_t *label_11 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_11, "Transmitted");
  lv_obj_set_style_text_font(label_11, &lv_font_montserrat_40, 0);
  lv_obj_align_to(label_11, label_10, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_11 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_11, "%u", tx_packets_counter);
  lv_obj_set_style_text_font(data_11, &lv_font_montserrat_40, 0);
  lv_obj_align_to(data_11, data_10, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

  lv_obj_t *label_12 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_12, "Received");
  lv_obj_set_style_text_font(label_12, &lv_font_montserrat_40, 0);
  lv_obj_align_to(label_12, label_11, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_12 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_12, "%u", rx_packets_counter);
  lv_obj_set_style_text_font(data_12, &lv_font_montserrat_40, 0);
  lv_obj_align_to(data_12, data_11, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

  lv_obj_t *label_13 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_13, "            Fix");
  lv_obj_set_style_text_font(label_13, &lv_font_montserrat_40, 0);
  lv_obj_align_to(label_13, label_12, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_13 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_13, " ");
  lv_obj_set_style_text_font(data_13, &lv_font_montserrat_40, 0);
  lv_obj_align_to(data_13, data_12, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

  lv_obj_t *label_14 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_14, "Time");
  lv_obj_set_style_text_font(label_14, &lv_font_montserrat_40, 0);
  lv_obj_align_to(label_14, label_13, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_14 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_14, "%u", timestamp);
  lv_obj_set_style_text_font(data_14, &lv_font_montserrat_40, 0);
  lv_obj_align_to(data_14, data_13, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

  lv_obj_t *label_15 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_15, "Satellites");
  lv_obj_set_style_text_font(label_15, &lv_font_montserrat_40, 0);
  lv_obj_align_to(label_15, label_14, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_15 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_15, "%u", sats);
  lv_obj_set_style_text_font(data_15, &lv_font_montserrat_40, 0);
  lv_obj_align_to(data_15, data_14, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

  lv_obj_t *label_16 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_16, "Latitude");
  lv_obj_set_style_text_font(label_16, &lv_font_montserrat_40, 0);
  lv_obj_align_to(label_16, label_15, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_16 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_16, "%s", str_lat);
  lv_obj_set_style_text_font(data_16, &lv_font_montserrat_40, 0);
  lv_obj_align_to(data_16, data_15, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

  lv_obj_t *label_17 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_17, "Longitude");
  lv_obj_set_style_text_font(label_17, &lv_font_montserrat_40, 0);
  lv_obj_align_to(label_17, label_16, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_17 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_17, "%s", str_lon);
  lv_obj_set_style_text_font(data_17, &lv_font_montserrat_40, 0);
  lv_obj_align_to(data_17, data_16, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

  lv_obj_t *label_18 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_18, "Altitude");
  lv_obj_set_style_text_font(label_18, &lv_font_montserrat_40, 0);
  lv_obj_align_to(label_18, label_17, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_18 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_18, "%s", str_alt);
  lv_obj_set_style_text_font(data_18, &lv_font_montserrat_40, 0);
  lv_obj_align_to(data_18, data_17, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);
#endif /* LVGL_VERSION_MAJOR == 8 */

  lvgl_port_unlock();
}

#endif /* USE_DSI */
