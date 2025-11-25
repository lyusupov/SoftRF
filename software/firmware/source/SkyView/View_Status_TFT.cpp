/*
 * View_Status_TFT.cpp
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

#include "TFTHelper.h"
#include "LVGLHelper.h"
#include "EEPROMHelper.h"
#include "BatteryHelper.h"

void TFT_status_setup()
{

}

void TFT_status_loop()
{
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  float vdd = Battery_voltage() ;
  bool low_voltage = (vdd <= Battery_threshold());
  char str_Vcc[8];
  dtostrf(vdd, 4, 2, str_Vcc);

  lvgl_port_lock(-1);

#if LVGL_VERSION_MAJOR == 8
  lv_obj_clean(lv_scr_act());

  lv_obj_t *label_1 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_1, "Device Id");
  lv_obj_set_style_text_font(label_1, &lv_font_montserrat_48, 0);
  lv_obj_align(label_1, LV_ALIGN_OUT_TOP_LEFT, 40, 20);

  lv_obj_t *data_1 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_1, "%06X", SoC->getChipId() & 0xFFFFFF);
  lv_obj_set_style_text_font(data_1, &lv_font_montserrat_48, 0);
  lv_obj_align(data_1, LV_ALIGN_TOP_RIGHT, -40, 20);

  lv_obj_t *label_2 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_2, "Software Version");
  lv_obj_set_style_text_font(label_2, &lv_font_montserrat_48, 0);
  lv_obj_align_to(label_2, label_1, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_2 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_2, "%s  %s", SKYVIEW_FIRMWARE_VERSION, SoC->name);
  lv_obj_set_style_text_font(data_2, &lv_font_montserrat_48, 0);
  lv_obj_align_to(data_2, data_1, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

  lv_obj_t *label_3 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_3, "Uptime");
  lv_obj_set_style_text_font(label_3, &lv_font_montserrat_48, 0);
  lv_obj_align_to(label_3, label_2, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_3 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_3, "%02d:%02d:%02d", hr, min % 60, sec % 60);
  lv_obj_set_style_text_font(data_3, &lv_font_montserrat_48, 0);
  lv_obj_align_to(data_3, data_2, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

  lv_obj_t *label_4 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_4, "Free memory");
  lv_obj_set_style_text_font(label_4, &lv_font_montserrat_48, 0);
  lv_obj_align_to(label_4, label_3, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_4 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_4, "%u", SoC->getFreeHeap());
  lv_obj_set_style_text_font(data_4, &lv_font_montserrat_48, 0);
  lv_obj_align_to(data_4, data_3, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

  lv_obj_t *label_5 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_5, "Battery voltage");
  lv_obj_set_style_text_font(label_5, &lv_font_montserrat_48, 0);
  lv_obj_align_to(label_5, label_4, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_5 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_5, "%s", str_Vcc);
  lv_obj_set_style_text_font(data_5, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(data_5, low_voltage ?
                              lv_palette_main(LV_PALETTE_RED) :
                              lv_palette_main(LV_PALETTE_GREEN), 0);
  lv_obj_align_to(data_5, data_4, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

  lv_obj_t *label_6 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_6, "Display");
  lv_obj_set_style_text_font(label_6, &lv_font_montserrat_48, 0);
  lv_obj_align_to(label_6, label_5, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_6 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_6, "%s",
    hw_info.display      == DISPLAY_EPD_2_7   ||
    hw_info.display      == DISPLAY_EPD_4_7   ? "e-Paper" :
    hw_info.display      == DISPLAY_TFT_7_0   ? "TFT" :
    hw_info.display      == DISPLAY_OLED_2_4  ? "OLED" : "NONE");
  lv_obj_set_style_text_font(data_6, &lv_font_montserrat_48, 0);
  lv_obj_align_to(data_6, data_5, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);

  lv_obj_t *label_7 = lv_label_create(lv_scr_act());
  lv_label_set_text(label_7, "Connection type");
  lv_obj_set_style_text_font(label_7, &lv_font_montserrat_48, 0);
  lv_obj_align_to(label_7, label_6, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

  lv_obj_t *data_7 = lv_label_create(lv_scr_act());
  lv_label_set_text_fmt(data_7, "%s",
    settings->connection == CON_SERIAL_MAIN   ? "Serial" :
    settings->connection == CON_BLUETOOTH_SPP ? "Bluetooth SPP" :
    settings->connection == CON_BLUETOOTH_LE  ? "Bluetooth LE" :
    settings->connection == CON_USB           ? "USB" :
    settings->connection == CON_WIFI_UDP      ? "WiFi" : "NONE");
  lv_obj_set_style_text_font(data_7, &lv_font_montserrat_48, 0);
  lv_obj_align_to(data_7, data_6, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);
#endif /* LVGL_VERSION_MAJOR == 8 */

  lvgl_port_unlock();
}

#endif /* USE_TFT */
