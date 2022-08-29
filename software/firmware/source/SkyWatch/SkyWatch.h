/*
 * SkyWatch.h
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

#ifndef SKYWATCH_H
#define SKYWATCH_H

#if defined(ARDUINO)
#include <Arduino.h>
#endif /* ARDUINO */

#define SKYWATCH_FIRMWARE_VERSION  "1.1"
#define SKYWATCH_IDENT    "SkyWatch"
#define SOFTRF_IDENT      "SoftRF"

#define DEFAULT_AP_SSID   "SoftRF-abc123"
#define DEFAULT_AP_PSK    "12345678"
#define DEFAULT_BT_NAME   "SoftRF-abc123"
#define DEFAULT_BT_KEY    "1234"

/*
 * If you need for SkyWatch to operate in wireless
 * client mode - specify your local AP's SSID/PSK:
 *
 * #define MY_ACCESSPOINT_SSID "My_AP_SSID"
 * #define MY_ACCESSPOINT_PSK  "My_AP_PSK"
 *
 * If SkyWatch's built-in AP is not stable enough for you, consider
 * to use "reverse" operation when your smartphone is acting
 * as an AP for the SkyWatch unit as a client:
 *
 * #define MY_ACCESSPOINT_SSID "AndroidAP"
 * #define MY_ACCESSPOINT_PSK  "12345678"
 */

// Default mode is AP with
// SSID: SkyWatch-XXXXXX
// KEY:  12345678
// IP: 192.168.1.1
// NETMASK: 255.255.255.0
#define MY_ACCESSPOINT_SSID ""
#define MY_ACCESSPOINT_PSK  ""

#define RELAY_DST_PORT    12390
#define RELAY_SRC_PORT    (RELAY_DST_PORT - 1)

/* S76G (STM32) AN3155 BR & BITS */
#define SERIAL_IN_BR      115200
#define SERIAL_IN_BITS    SERIAL_8E1

/* SoftRF serial output defaults */
#define SERIAL_OUT_BR     38400
#define SERIAL_OUT_BITS   SERIAL_8N1

#define DATA_TIMEOUT      2000 /* 2.0 seconds */

#define MAX_FILENAME_LEN  64
#define WAV_FILE_SUFFIX   ".wav"
#define VOICE1_SUBDIR     "voice1/"
#define VOICE2_SUBDIR     "voice2/"
#define VOICE3_SUBDIR     "voice3/"

typedef struct UFO {
    uint8_t   raw[34];
    time_t    timestamp;

    uint8_t   protocol;

    uint32_t  addr;
    uint8_t   addr_type;
    float     latitude;
    float     longitude;
    float     altitude;
    float     pressure_altitude;
    float     course;     /* CoG */
    float     speed;      /* ground speed in knots */
    uint8_t   aircraft_type;

    float     vs; /* feet per minute */

    bool      stealth;
    bool      no_track;

    int8_t    ns[4];
    int8_t    ew[4];

    float     geoid_separation; /* metres */
    uint16_t  hdop; /* cm */
    int8_t    rssi; /* SX1276 only */

    /* 'legacy' specific data */
    float     distance;
    float     bearing;
    int8_t    alarm_level;

    /* ADS-B (ES, UAT, GDL90) specific data */
    uint8_t   callsign[8];
} ufo_t;

typedef struct hardware_info {
    byte  model;
    byte  revision;
    byte  soc;
    byte  rf;
    byte  gnss;
    byte  baro;
    byte  display;
    byte  storage;
    byte  rtc;
    byte  imu;
    byte  pmu;
    byte  slave;
} hardware_info_t;

typedef struct IODev_ops_struct {
  const char name[16];
  void (*setup)();
  void (*loop)();
  void (*fini)();
  int (*available)(void);
  int (*read)(void);
  size_t (*write)(const uint8_t *buffer, size_t size);
} IODev_ops_t;

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
	HW_REV_T_WATCH_19,
	HW_REV_T_WATCH_20,
	HW_REV_DEVKIT,
	HW_REV_T8,
	HW_REV_TDONGLE,
};

enum
{
	DISPLAY_NONE,
	DISPLAY_EPD_2_7,
	DISPLAY_OLED_2_4,
	DISPLAY_TFT_TTGO_240,
	DISPLAY_TFT_TTGO_135,
};

enum
{
	ADAPTER_NONE,
	ADAPTER_WAVESHARE_PI_HAT_2_7,
	ADAPTER_WAVESHARE_ESP8266,
	ADAPTER_WAVESHARE_ESP32,
	ADAPTER_TTGO_T5S,
	ADAPTER_NODEMCU,
	ADAPTER_OLED,
};

enum
{
	CON_NONE,
	CON_SERIAL_MAIN,
	CON_SERIAL_AUX,
	CON_USB,
	CON_WIFI_UDP,
	CON_WIFI_TCP,
	CON_BLUETOOTH,
};

enum
{
	B4800,
	B9600,
	B19200,
	B38400,
	B57600,
	B115200,
	B2000000,
};

enum
{
	PROTOCOL_NONE,
	PROTOCOL_NMEA, /* FTD-12 */
	PROTOCOL_GDL90,
	PROTOCOL_MAVLINK_1,
	PROTOCOL_MAVLINK_2,
	PROTOCOL_D1090,
	PROTOCOL_UATRADIO,
};

enum
{
	UNITS_METRIC,
	UNITS_IMPERIAL,
	UNITS_MIXED,    // almost the same as metric, but all the altitudes are in feet
};

enum
{
	VIEW_MODE_STATUS,
	VIEW_MODE_RADAR,
	VIEW_MODE_TEXT,
	VIEW_MODE_TIME,
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
	ZOOM_HIGH,
};

enum
{
	ID_REG,
	ID_TAIL,
	ID_MAM,
	ID_TYPE,
};

enum
{
	VOICE_OFF,
	VOICE_1,
	VOICE_2,
	VOICE_3,
};

enum
{
	ANTI_GHOSTING_OFF,
	ANTI_GHOSTING_AUTO,
	ANTI_GHOSTING_2MIN,
	ANTI_GHOSTING_5MIN,
	ANTI_GHOSTING_10MIN,
};

enum
{
	STORAGE_NONE,
	STORAGE_SD,
	STORAGE_FLASH,
};

enum
{
	RTC_NONE,
	RTC_PCF8563,
};

enum
{
	IMU_NONE,
	IMU_BNO080,
	IMU_BMA423,
	IMU_ICM20948,
	IMU_MPU9250,
};

enum
{
	TRAFFIC_FILTER_OFF,
	TRAFFIC_FILTER_500M,
	TRAFFIC_FILTER_1500M,
};

enum
{
	DB_NONE,
	DB_AUTO,
	DB_FLN,
	DB_OGN,
	DB_ICAO,
};

enum
{
	ROTATE_0,
	ROTATE_90,
	ROTATE_180,
	ROTATE_270,
};

/* SoftRF enumerations */

enum
{
	SOFTRF_MODE_NORMAL,
	SOFTRF_MODE_WATCHOUT,
	SOFTRF_MODE_BRIDGE,
	SOFTRF_MODE_RELAY,
	SOFTRF_MODE_TXRX_TEST,
	SOFTRF_MODE_LOOPBACK,
	SOFTRF_MODE_UAV,
	SOFTRF_MODE_RECEIVER,
	SOFTRF_MODE_CASUAL,
};

enum
{
	RF_IC_NONE,
	RF_IC_NRF905,
	RF_IC_SX1276,
	RF_IC_UATM,
	RF_IC_CC13XX,
	RF_DRV_OGN,
	RF_IC_SX1262,
	RF_IC_MAX2837,
};

enum
{
	RF_TX_POWER_FULL,
	RF_TX_POWER_LOW,
	RF_TX_POWER_OFF,
};

enum
{
	TRAFFIC_ALARM_NONE,
	TRAFFIC_ALARM_DISTANCE,
	TRAFFIC_ALARM_VECTOR,
	TRAFFIC_ALARM_LEGACY,
};

enum
{
	BUZZER_VOLUME_FULL,
	BUZZER_VOLUME_LOW,
	BUZZER_OFF,
};

enum
{
	DIRECTION_TRACK_UP,
	DIRECTION_NORTH_UP,
	LED_OFF,
};

enum
{
	NMEA_OFF,
	NMEA_UART,
	NMEA_UDP,
	NMEA_TCP,
	NMEA_USB,
	NMEA_BLUETOOTH,
};

enum
{
	GDL90_OFF,
	GDL90_UART,
	GDL90_UDP,
	GDL90_TCP,
	GDL90_USB,
	GDL90_BLUETOOTH,
};

enum
{
	D1090_OFF,
	D1090_UART,
	D1090_UDP,
	D1090_TCP,
	D1090_USB,
	D1090_BLUETOOTH,
};

enum
{
	JSON_OFF,
	JSON_PING,
};

/* end of SoftRF enumerations */

extern ufo_t ThisDevice;
extern hardware_info_t hw_info;
extern bool inServiceMode;

extern void shutdown(const char *);

#endif /* SKYWATCH_H */
