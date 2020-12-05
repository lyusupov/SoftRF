/*
 * EPDHelper.h
 * Copyright (C) 2019-2020 Linar Yusupov
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

#define ENABLE_GxEPD2_GFX       0

#define EPD_EXPIRATION_TIME     5 /* seconds */

#define NO_DATA_TEXT            "NO DATA"
#define NO_FIX_TEXT             "NO FIX"

#define NAVBOX1_TITLE           "ACFTS"
#define NAVBOX2_TITLE           "BAT"
#define NAVBOX3_TITLE           "ID"
#define NAVBOX4_TITLE           "PROTOCOL"
#define NAVBOX5_TITLE           "RX"
#define NAVBOX6_TITLE           "TX"

#define isTimeToEPD()           (millis() - EPDTimeMarker > 1000)
#define maxof2(a,b)             (a > b ? a : b)

#define EPD_RADAR_V_THRESHOLD   50      /* metres */

#define TEXT_VIEW_LINE_LENGTH   13     /* characters */
#define TEXT_VIEW_LINE_SPACING  12     /* pixels */
#define INFO_1_LINE_SPACING     7      /* pixels */


//#define EPD_HIBERNATE         {}
#define EPD_HIBERNATE           display->hibernate()
//#define EPD_HIBERNATE         display->powerOff()

#define EPD_POWEROFF            display->powerOff()

enum
{
	VIEW_MODE_STATUS,
	VIEW_MODE_RADAR,
	VIEW_MODE_TEXT,
	VIEW_MODE_TIME
};

/*
 * 'Radar view' scale factor (outer circle diameter)
 *
 * Metric and Mixed:
 *  LOWEST - 20 KM diameter (10 KM radius)
 *  LOW    - 10 KM diameter ( 5 KM radius)
 *  MEDIUM -  4 KM diameter ( 2 KM radius)
 *  HIGH   -  2 KM diameter ( 1 KM radius)
 *
 * Imperial:
 *  LOWEST - 10 NM diameter (  5 NM radius)
 *  LOW    -  5 NM diameter (2.5 NM radius)
 *  MEDIUM -  2 NM diameter (  1 NM radius)
 *  HIGH   -  1 NM diameter (0.5 NM radius)
 */
enum
{
	ZOOM_LOWEST,
	ZOOM_LOW,
	ZOOM_MEDIUM,
	ZOOM_HIGH
};

enum
{
	UNITS_METRIC,
	UNITS_IMPERIAL,
	UNITS_MIXED     // almost the same as metric, but all the altitudes are in feet
};

enum
{
	PROTOCOL_NONE,
	PROTOCOL_NMEA, /* FTD-12 */
	PROTOCOL_GDL90,
	PROTOCOL_MAVLINK_1,
	PROTOCOL_MAVLINK_2,
	PROTOCOL_D1090,
	PROTOCOL_UATRADIO
};

enum
{
	ID_REG,
	ID_TAIL,
	ID_MAM
};

enum
{
	VOICE_OFF,
	VOICE_1,
	VOICE_2,
	VOICE_3
};

enum
{
	ANTI_GHOSTING_OFF,
	ANTI_GHOSTING_AUTO,
	ANTI_GHOSTING_2MIN,
	ANTI_GHOSTING_5MIN,
	ANTI_GHOSTING_10MIN
};

enum
{
	TRAFFIC_FILTER_OFF,
	TRAFFIC_FILTER_500M,
	TRAFFIC_FILTER_1500M
};

enum
{
	DB_NONE,
	DB_AUTO,
	DB_FLN,
	DB_OGN,
	DB_ICAO
};

typedef struct navbox_struct
{
  char      title[9];
  uint16_t  x;
  uint16_t  y;
  uint16_t  width;
  uint16_t  height;
  int32_t   value;
  int32_t   prev_value;
  uint32_t  timestamp;
} navbox_t;

void EPD_Clear_Screen();
bool EPD_setup(bool);
void EPD_loop();
void EPD_fini(int);
void EPD_info1(bool, bool);

void EPD_Mode();
void EPD_Up();
void EPD_Down();
void EPD_Message(const char *, const char *);
EPD_Task_t EPD_Task(void *);

void EPD_status_setup();
void EPD_status_loop();
void EPD_status_next();
void EPD_status_prev();

void EPD_radar_setup();
void EPD_radar_loop();
void EPD_radar_zoom();
void EPD_radar_unzoom();

void EPD_text_setup();
void EPD_text_loop();
void EPD_text_next();
void EPD_text_prev();

void EPD_time_setup();
void EPD_time_loop();
void EPD_time_next();
void EPD_time_prev();

extern unsigned long EPDTimeMarker;
extern bool EPD_vmode_updated;
extern volatile bool EPD_ready_to_display;

#endif /* EPDHELPER_H */
