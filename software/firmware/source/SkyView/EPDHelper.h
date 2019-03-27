/*
 * EPDHelper.h
 * Copyright (C) 2019 Linar Yusupov
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

#ifndef EPDHELPER_H
#define EPDHELPER_H

#define ENABLE_GxEPD2_GFX 0

#include <GxEPD2_BW.h>

#define EPD_EXPIRATION_TIME     5 /* seconds */

#define NO_DATA_TEXT "NO DATA"
#define NO_FIX_TEXT  "NO FIX"

#define NAVBOX1_TITLE "ACFTS"
#define NAVBOX2_TITLE "DATA"
#define NAVBOX3_TITLE "SCALE"
#define NAVBOX4_TITLE "BAT"

#define maxof2(a,b)   (a > b ? a : b)

typedef struct navbox_struct
{
  char      title[8];
  uint16_t  x;
  uint16_t  y;
  uint16_t  width;
  uint16_t  height;
  int       value;
  int       prev_value;
  uint32_t  timestamp;
} navbox_t;

enum
{
	EPD_SCALE_2KM,
	EPD_SCALE_4KM,
	EPD_SCALE_8KM
};

byte EPD_setup();
void EPD_loop();

extern GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> *display;

#endif /* EPDHELPER_H */
