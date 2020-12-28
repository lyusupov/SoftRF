/*
 * TFTHelper.h
 * Copyright (C) 2019-2021 Linar Yusupov
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

#include <TFT_eSPI.h>
#include <FT5206.h>

#define TFT_EXPIRATION_TIME     5 /* seconds */

#define TP_MAX_X                320
#define TP_MAX_Y                320

#define NO_DATA_TEXT            "NO DATA"
#define NO_FIX_TEXT             "NO FIX"

#define isTimeToDisplay()       (millis() - TFTTimeMarker > 1000)
#define maxof2(a,b)             (a > b ? a : b)

#define TFT_RADAR_V_THRESHOLD   50      /* metres */

#define TEXT_VIEW_LINE_LENGTH   13      /* characters */
#define TEXT_VIEW_LINE_SPACING  8      /* pixels */

typedef struct Gesture_struct {
  bool     touched;
  TP_Point t_loc;
  TP_Point d_loc;
} Gesture_t;

enum {
  NO_GESTURE,
  SWIPE_LEFT,
  SWIPE_RIGHT,
  SWIPE_UP,
  SWIPE_DOWN
};

void TFT_Clear_Screen();
byte TFT_setup();
void TFT_loop();
void TFT_fini(const char *);
void TFT_Up();
void TFT_Down();
void TFT_Message(const char *, const char *);

void TFT_status_setup();
void TFT_status_loop();
void TFT_status_next();
void TFT_status_prev();

void TFT_radar_setup();
void TFT_radar_loop();
void TFT_radar_zoom();
void TFT_radar_unzoom();

void TFT_text_setup();
void TFT_text_loop();
void TFT_text_next();
void TFT_text_prev();

void TFT_time_setup();
void TFT_time_loop();
void TFT_time_next();
void TFT_time_prev();

extern TFT_eSPI *tft;
extern TFT_eSprite *sprite;
extern bool TFT_vmode_updated;

#endif /* TFTHELPER_H */
