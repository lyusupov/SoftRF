/*
 * TFTHelper.h
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

#ifndef TFTHELPER_H
#define TFTHELPER_H

#define TFT_EXPIRATION_TIME     5 /* seconds */

#define NO_DATA_TEXT            "NO DATA"
#define NO_FIX_TEXT             "NO FIX"

#define isTimeToDisplay()       (millis() - TFT_TimeMarker > 1000)

#define TFT_RADAR_V_THRESHOLD   50      /* metres */

enum {
  NO_GESTURE,
  SWIPE_LEFT,
  SWIPE_RIGHT,
  SWIPE_UP,
  SWIPE_DOWN,
};

byte TFT_setup();
void TFT_loop();
void TFT_fini();

void TFT_Up();
void TFT_Down();

void TFT_status_setup();
void TFT_status_loop();

void TFT_radar_setup();
void TFT_radar_loop();
void TFT_radar_zoom();
void TFT_radar_unzoom();

extern bool TFT_vmode_updated;

#endif /* TFTHELPER_H */
