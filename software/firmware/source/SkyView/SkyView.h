/*
 * SkyView.h
 * Copyright (C) 2019-2022 Linar Yusupov
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

#ifndef SKYVIEW_H
#define SKYVIEW_H

#define SKYVIEW_FIRMWARE_VERSION  "0.12"
#define SKYVIEW_IDENT     "SkyView-"

#define DEFAULT_AP_SSID   "SoftRF-abc123"
#define DEFAULT_AP_PSK    "12345678"

#define RELAY_DST_PORT    12390
#define RELAY_SRC_PORT    (RELAY_DST_PORT - 1)

/* SoftRF serial output defaults */
#define SERIAL_OUT_BR     38400
#define SERIAL_OUT_BITS   SERIAL_8N1

#define DATA_TIMEOUT      2000 /* 2.0 seconds */

#define MAX_FILENAME_LEN  64
#define WAV_FILE_SUFFIX   ".wav"
#define VOICE1_SUBDIR     "voice1/"
#define VOICE2_SUBDIR     "voice2/"
#define VOICE3_SUBDIR     "voice3/"

typedef struct hardware_info {
    byte  model;
    byte  revision;
    byte  soc;
    byte  display;

} hardware_info_t;

enum
{
	SOFTRF_MODEL_UNKNOWN,
	SOFTRF_MODEL_STANDALONE,
	SOFTRF_MODEL_PRIME,
	SOFTRF_MODEL_UAV,
	SOFTRF_MODEL_PRIME_MK2,
	SOFTRF_MODEL_RASPBERRY,
	SOFTRF_MODEL_UAT,
	SOFTRF_MODEL_SKYVIEW,
	SOFTRF_MODEL_RETRO,
	SOFTRF_MODEL_SKYWATCH,
	SOFTRF_MODEL_DONGLE,
	SOFTRF_MODEL_OCTAVE,
	SOFTRF_MODEL_UNI,
	SOFTRF_MODEL_WEBTOP_SERIAL,
	SOFTRF_MODEL_MINI,
	SOFTRF_MODEL_BADGE,
	SOFTRF_MODEL_ES,
	SOFTRF_MODEL_BRACELET,
	SOFTRF_MODEL_ACADEMY,
	SOFTRF_MODEL_LEGO,
	SOFTRF_MODEL_WEBTOP_USB,
	SOFTRF_MODEL_PRIME_MK3,
	SOFTRF_MODEL_BALKAN,
};

enum
{
	HW_REV_UNKNOWN,
	HW_REV_DEVKIT,
	HW_REV_T5S_1_9,
	HW_REV_T5S_2_8,
	HW_REV_T8_1_8,
	HW_REV_T5_1
};

enum
{
	DISPLAY_NONE,
	DISPLAY_EPD_2_7,
	DISPLAY_OLED_2_4,
	DISPLAY_EPD_4_7
};

enum
{
	ADAPTER_WAVESHARE_PI_HAT_2_7,
	ADAPTER_WAVESHARE_ESP8266,
	ADAPTER_WAVESHARE_ESP32,
	ADAPTER_TTGO_T5S,
	ADAPTER_NODEMCU,
	ADAPTER_OLED,
	ADAPTER_TTGO_T5_4_7
};

enum
{
	CON_NONE,
	CON_SERIAL,
	CON_WIFI_UDP,
	CON_WIFI_TCP,
	CON_BLUETOOTH_SPP,
	CON_BLUETOOTH_LE
};

enum
{
	B4800,
	B9600,
	B19200,
	B38400,
	B57600,
	B115200,
	B2000000
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
	UNITS_METRIC,
	UNITS_IMPERIAL,
	UNITS_MIXED     // almost the same as metric, but all the altitudes are in feet
};

enum
{
	VIEW_MODE_RADAR,
	VIEW_MODE_TABLE,
	VIEW_MODE_TEXT
};

enum
{
	DIRECTION_TRACK_UP,
	DIRECTION_NORTH_UP
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
	POWER_SAVE_NONE = 0,
	POWER_SAVE_WIFI = 1,
	POWER_SAVE_GNSS = 2
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

extern hardware_info_t hw_info;

extern void shutdown(const char *);
extern void Input_loop(void);

#endif /* SKYVIEW_H */
