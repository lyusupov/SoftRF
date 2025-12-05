/*
 * SoftRF.h
 * Copyright (C) 2016-2025 Linar Yusupov
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

#ifndef SOFTRF_H
#define SOFTRF_H

#if defined(ARDUINO) || defined(HACKRF_ONE)
#include <Arduino.h>
#endif /* ARDUINO */

#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2) || \
    defined(HACKRF_ONE)          || defined(ARDUINO_ARCH_AVR)    || \
    defined(ARDUINO_ARCH_SILABS)
#include <TimeLib.h>
#endif /* CC13XX || CC13X2 || HACKRF_ONE || AVR || SILABS */

#if defined(RASPBERRY_PI) || defined(LUCKFOX_LYRA)
#include <raspi/raspi.h>
#endif /* RASPBERRY_PI */

#define SOFTRF_IDENT            "SoftRF"
#define SOFTRF_FIRMWARE_VERSION "1.7.1"
#define SOFTRF_USB_FW_VERSION   0x0107

#define ENTRY_EXPIRATION_TIME   10 /* seconds */
#define LED_EXPIRATION_TIME     5  /* seconds */
#define EXPORT_EXPIRATION_TIME  5  /* seconds */

#define SETTINGS_JSON_PATH      "/settings.json"

/*
 * If you need for SoftRF to operate in wireless
 * client mode - specify your local AP's SSID/PSK:
 *
 * #define MY_ACCESSPOINT_SSID "My_AP_SSID"
 * #define MY_ACCESSPOINT_PSK  "My_AP_PSK"
 *
 * If SoftRF's built-in AP is not stable enough for you, consider
 * to use "reverse" operation when your smartphone is acting
 * as an AP for the SoftRF unit as a client:
 *
 * #define MY_ACCESSPOINT_SSID "AndroidAP"
 * #define MY_ACCESSPOINT_PSK  "12345678"
 */

// Default mode is AP with
// SSID: SoftRF-XXXXXX
// KEY:  12345678
// IP: 192.168.1.1
// NETMASK: 255.255.255.0
#define MY_ACCESSPOINT_SSID ""
#define MY_ACCESSPOINT_PSK  ""

#define RELAY_DST_PORT  12390
#define RELAY_SRC_PORT  (RELAY_DST_PORT - 1)

#define GDL90_DST_PORT    4000
#define NMEA_UDP_PORT     10110
#define NMEA_TCP_PORT     2000

/*
 * Serial I/O default values.
 * Can be overridden by platfrom-specific code.
 */
#if !defined(SERIAL_IN_BR)
/*
 * 9600 is default value of NMEA baud rate
 * for most of GNSS modules
 * being used in SoftRF project
 */
#define SERIAL_IN_BR      9600
#endif
#if !defined(SERIAL_IN_BITS)
#define SERIAL_IN_BITS    SERIAL_8N1
#endif

/*
 * 38400 is known as maximum baud rate
 * that HC-05 Bluetooth module
 * can handle without symbols loss.
 *
 * Applicable for Standalone Edition. Inherited by most of other SoftRF platforms.
 */
#define STD_OUT_BR        38400
#define STD_OUT_BITS      SERIAL_8N1

#if !defined(SERIAL_OUT_BR)
#define SERIAL_OUT_BR     STD_OUT_BR
#endif
#if !defined(SERIAL_OUT_BITS)
#define SERIAL_OUT_BITS   STD_OUT_BITS
#endif

#define UAT_RECEIVER_BR   2000000

#if defined(PREMIUM_PACKAGE) && !defined(RASPBERRY_PI)
#define ENABLE_AHRS
#endif /* PREMIUM_PACKAGE */

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

    /* bitmap of issued voice/tone/ble/... alerts */
    uint8_t   alert;

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
    byte  mag;
    byte  pmu;
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

typedef struct DB_ops_struct {
  bool (*setup)();
  bool (*fini)();
  bool (*query)(uint8_t, uint32_t, char *, size_t);
} DB_ops_t;

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
	SOFTRF_MODEL_HAM,
	SOFTRF_MODEL_MIDI,
	SOFTRF_MODEL_ECO,
	SOFTRF_MODEL_INK,
	SOFTRF_MODEL_CARD,
	SOFTRF_MODEL_COZY,
	SOFTRF_MODEL_NEO,
	SOFTRF_MODEL_HANDHELD,
	SOFTRF_MODEL_GIZMO,
	SOFTRF_MODEL_NANO,
	SOFTRF_MODEL_DECENT,
	SOFTRF_MODEL_LYRA,
	SOFTRF_MODEL_AIRVENTURE,
	SOFTRF_MODEL_SOLARIS,
	SOFTRF_MODEL_POCKET,
	SOFTRF_MODEL_LABUBU,
	SOFTRF_MODEL_CONCORDE,
};

enum
{
	STD_EDN_REV_DEFAULT    = 0,
	STD_EDN_REV_T3_1_1     = 11,
	STD_EDN_REV_T3_1_6     = 16,
	STD_EDN_REV_S3_DEVKIT  = 203,
	STD_EDN_REV_BPICOW     = 4,
	STD_EDN_REV_EHUB       = 5,
	STD_EDN_REV_T3S3_OLED  = 6,
	STD_EDN_REV_WT99P4C5   = 7,
	STD_EDN_REV_P4_EVB     = 8,
	STD_EDN_REV_LYRA       = 9,
	STD_EDN_REV_C5_DEVKIT  = 10,
};

enum
{
	SOFTRF_SHUTDOWN_NONE,
	SOFTRF_SHUTDOWN_DEFAULT,
	SOFTRF_SHUTDOWN_DEBUG,
	SOFTRF_SHUTDOWN_ABORT,
	SOFTRF_SHUTDOWN_WATCHDOG,
	SOFTRF_SHUTDOWN_NMEA,
	SOFTRF_SHUTDOWN_BUTTON,
	SOFTRF_SHUTDOWN_LOWBAT,
	SOFTRF_SHUTDOWN_SENSOR,
};

enum
{
	STORAGE_NONE,
	STORAGE_FLASH,
	STORAGE_CARD,
	STORAGE_FLASH_AND_CARD,
};

enum
{
	IMU_NONE,
	ACC_BMA423,
	ACC_ADXL362,
	ACC_QMA6100P,
	ACC_SC7A20H,
	IMU_MPU6886,
	IMU_MPU9250,
	IMU_BNO080,
	IMU_ICM20948,
	IMU_QMI8658,
	IMU_BHI260AP,
};

enum
{
	MAG_NONE,
	MAG_AK8963,
	MAG_AK09916,
	MAG_IIS2MDC,
	MAG_QMC6310,
	MAG_BMM150,
};

extern ufo_t ThisAircraft;
extern hardware_info_t hw_info;
extern const float txrx_test_positions[90][2] PROGMEM;

extern void shutdown(int);

#define TXRX_TEST_NUM_POSITIONS (sizeof(txrx_test_positions) / sizeof(float) / 2)
#define TXRX_TEST_ALTITUDE    438.0
#define TXRX_TEST_COURSE      280.0
#define TXRX_TEST_SPEED       50.0
#define TXRX_TEST_VS          -300.0

//#define ENABLE_TTN
//#define TEST_PAW_ON_NICERF_SV610_FW466
#define  DO_GDL90_FF_EXT

#define StdOut  Serial

#endif /* SOFTRF_H */
