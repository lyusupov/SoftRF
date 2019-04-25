/*
 * SkyView.h
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

#ifndef SKYVIEW_H
#define SKYVIEW_H

#define SKYVIEW_FIRMWARE_VERSION  "0.8"

#define DEFAULT_AP_SSID       "SoftRF-abc123"
#define DEFAULT_AP_PSK        "12345678"
#define DEFAULT_BT_NAME       "SoftRF-abc123"
#define DEFAULT_BT_KEY        "1234"

#define RELAY_DST_PORT  12390
#define RELAY_SRC_PORT  (RELAY_DST_PORT - 1)

#define DATA_TIMEOUT  2000 /* 2.0 seconds */

typedef struct hardware_info {
    byte  model;
    byte  revision;
    byte  soc;
    byte  display;

} hardware_info_t;

enum
{
	SOFTRF_MODEL_STANDALONE,
	SOFTRF_MODEL_PRIME,
	SOFTRF_MODEL_UAV,
	SOFTRF_MODEL_PRIME_MK2,
	SOFTRF_MODEL_RASPBERRY,
	SOFTRF_MODEL_UAT,
	SOFTRF_MODEL_SKYVIEW
};

enum
{
	DISPLAY_NONE,
	DISPLAY_EPD_2_7,
	DISPLAY_OLED_2_4
};

enum
{
	ADAPTER_WAVESHARE_PI_HAT_2_7,
	ADAPTER_WAVESHARE_ESP8266,
	ADAPTER_WAVESHARE_ESP32,
	ADAPTER_TTGO_T5S,
	ADAPTER_NODEMCU
};

enum
{
	CON_NONE,
	CON_SERIAL,
	CON_WIFI_UDP,
	CON_WIFI_TCP,
	CON_BLUETOOTH
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
	DIRECTION_TRACK_UP,
	DIRECTION_NORTH_UP
};

enum
{
	BLUETOOTH_OFF,
	BLUETOOTH_SPP
};

enum
{
	DB_FLN,
	DB_OGN,
	DB_PAW
};

extern hardware_info_t hw_info;

extern void shutdown(void);

#endif /* SKYVIEW_H */
