/*
 * DSI.h
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

#ifndef DSIHELPER_H
#define DSIHELPER_H

#define DSI_EXPIRATION_TIME     5 /* seconds */

#define NO_DATA_TEXT            "NO DATA"
#define NO_FIX_TEXT             "NO FIX"
#define NO_TRAFFIC_TEXT         "NO TRAFFIC"

#define isTimeToDisplay()       (millis() - DSI_TimeMarker > 1000)

#define DSI_RADAR_V_THRESHOLD   50      /* metres */

enum {
  NO_GESTURE,
  SWIPE_LEFT,
  SWIPE_RIGHT,
  SWIPE_UP,
  SWIPE_DOWN,
};

void DSI_setup();
void DSI_loop();
void DSI_fini();

void DSI_Up();
void DSI_Down();
void DSI_Next_Page();

void DSI_status_setup();
void DSI_status_loop();

void DSI_radar_setup();
void DSI_radar_loop();
void DSI_radar_zoom();
void DSI_radar_unzoom();

extern bool DSI_vmode_updated;

#endif /* DSIHELPER_H */
