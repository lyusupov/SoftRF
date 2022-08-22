/*
 * SoftRF.h
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

#ifndef SOFTRF_H
#define SOFTRF_H

/*
 * PART 1. SoftRF 'radio' component.
 */

/*

  TO RETREIVE SETTINGS
  --------------------

  Query:    "$PSRFC,?*47"

  Response: "$PSRFC,<version>,
                    <mode>,<protocol>,<band>,<aircraft type>,
                    <alarm>,<tx power>,<buzzer>,<leds>,
                    <NMEA GNSS>,<NMEA private>,<NMEA legacy>,<NMEA sensors>,
                    <NMEA output>,<GDL90 output>,<D1090 output>,
                    <stealth>,<no track>,<power save>*<checksum><CR><LF>"

  TO APPLY SETTINGS
  -----------------

  Sentence: "$PSRFC,1,
                    <mode>,<protocol>,<band>,<aircraft type>,
                    <alarm>,<tx power>,<buzzer>,<leds>,
                    <NMEA GNSS>,<NMEA private>,<NMEA legacy>,<NMEA sensors>,
                    <NMEA output>,<GDL90 output>,<D1090 output>,
                    <stealth>,<no track>,<power save>*<checksum><CR><LF>"

  Response: dump of new settings followed by system restart

  EXAMPLE OF NMEA SENTENCE
  ------------------------

  $PSRFC,1,0,1,1,1,1,0,0,0,1,0,1,1,4,0,0,0,0,0*4C


  LIST OF SETTINGS AVAILABLE
  --------------------------

  Version:        1
  Mode:           Normal, UAV
  Protocol:       Legacy, OGNTP, P3I, FANET, UAT, 1090ES
  Band:           EU, US/CA, AU, NZ, RU, CN, UK, IN, IL, KR
  Aircraft:       Glider, Towplane, Powered, Helicopter, UAV, Hangglider,
                  Paraglider, Balloon, Static
  Alarm:          None, Distance, Vector
  Tx Power:       Full, Low, Off
  Buzzer volume:  Off
  LEDs:           Off

  NMEA:
        GNSS      On, Off
        Private   On, Off
        Legacy    On, Off
        Sensors   On, Off

  NMEA  output:   Serial, USB, Bluetooth
  GDL90 output:   Off
  D1090 output:   Off

  Stealth:        On, Off
  No track:       On, Off

  Power save:     Off, GNSS, No Receive

  RECOMMENDED DEFAULT SETTINGS
  ----------------------------

  Mode:           Normal
  Protocol:       OGNTP
  Band:           EU
  Aircraft:       Glider
  Alarm:          Distance
  Tx Power:       Full      

  NMEA:
        GNSS      On
        Private   Off
        Legacy    On
        Sensors   On

  NMEA  output:   USB

  Stealth:        Off
  No track:       Off

  Power save:     Off

  OTHER INFORMATION
  -----------------

  https://github.com/lyusupov/SoftRF/wiki/Dongle-settings
  https://github.com/lyusupov/SoftRF/wiki/Settings

 */


/*
 *  C/C++ enumerations
 */

enum
{
	SOFTRF_MODE_NORMAL,
	SOFTRF_MODE_WATCHOUT,
	SOFTRF_MODE_BRIDGE,
	SOFTRF_MODE_RELAY,
	SOFTRF_MODE_TXRX_TEST,
	SOFTRF_MODE_LOOPBACK,
	SOFTRF_MODE_UAV,
	SOFTRF_MODE_RECEIVER
};

enum
{
	SOFTRF_PROTOCOL_LEGACY,    /* Air V6 */
	SOFTRF_PROTOCOL_OGNTP,     /* Open Glider Network tracker */
	SOFTRF_PROTOCOL_P3I,       /* PilotAware */
	SOFTRF_PROTOCOL_ADSB_1090, /* ADS-B 1090ES */
	SOFTRF_PROTOCOL_ADSB_UAT,  /* ADS-B UAT */
	SOFTRF_PROTOCOL_FANET,     /* Skytraxx */
	/* Volunteer contributors are welcome */
	SOFTRF_PROTOCOL_EID,       /* UAS eID */
	SOFTRF_PROTOCOL_GOTENNA    /* goTenna Mesh */
};

enum
{
	SOFTRF_BAND_AUTO = 0,
	SOFTRF_BAND_EU   = 1, /* 868.4 MHz band */
	SOFTRF_BAND_US   = 2, /* 915 MHz band */
	SOFTRF_BAND_AU   = 3, /* 921 MHz band */
	SOFTRF_BAND_NZ   = 4, /* 869.250 MHz band */
	SOFTRF_BAND_RU   = 5, /* 868.8 MHz band */
	SOFTRF_BAND_CN   = 6, /* 470 MHz band */
	SOFTRF_BAND_UK   = 7, /* 869.52 MHz band */
	SOFTRF_BAND_IN   = 8, /* 866.0 MHz band */
	SOFTRF_BAND_IL   = 9, /* 916.2 MHz band */
	SOFTRF_BAND_KR   = 10 /* 920.9 MHz band */
};

enum
{
	SOFTRF_AIRCRAFT_TYPE_UNKNOWN,
	SOFTRF_AIRCRAFT_TYPE_GLIDER,
	SOFTRF_AIRCRAFT_TYPE_TOWPLANE,
	SOFTRF_AIRCRAFT_TYPE_HELICOPTER,
	SOFTRF_AIRCRAFT_TYPE_PARACHUTE,
	SOFTRF_AIRCRAFT_TYPE_DROPPLANE,
	SOFTRF_AIRCRAFT_TYPE_HANGGLIDER,
	SOFTRF_AIRCRAFT_TYPE_PARAGLIDER,
	SOFTRF_AIRCRAFT_TYPE_POWERED,
	SOFTRF_AIRCRAFT_TYPE_JET,
	SOFTRF_AIRCRAFT_TYPE_UFO,
	SOFTRF_AIRCRAFT_TYPE_BALLOON,
	SOFTRF_AIRCRAFT_TYPE_ZEPPELIN,
	SOFTRF_AIRCRAFT_TYPE_UAV,
	SOFTRF_AIRCRAFT_TYPE_RESERVED,
	SOFTRF_AIRCRAFT_TYPE_STATIC
};

enum
{
	SOFTRF_TX_POWER_FULL,
	SOFTRF_TX_POWER_LOW,
	SOFTRF_TX_POWER_OFF
};

enum
{
	SOFTRF_ALARM_NONE,
	SOFTRF_ALARM_DISTANCE,
	SOFTRF_ALARM_VECTOR,
	SOFTRF_ALARM_LEGACY
};

enum
{
	SOFTRF_VOLUME_FULL,
	SOFTRF_VOLUME_LOW,
	SOFTRF_VOLUME_OFF
};

enum
{
	SOFTRF_TRACK_UP,
	SOFTRF_NORTH_UP,
	SOFTRF_LED_OFF
};

enum
{
	SOFTRF_NMEA_GNSS_OFF,
	SOFTRF_NMEA_GNSS_ON
};

enum
{
	SOFTRF_NMEA_PRIVATE_OFF,
	SOFTRF_NMEA_PRIVATE_ON
};

enum
{
	SOFTRF_NMEA_LEGACY_OFF,
	SOFTRF_NMEA_LEGACY_ON
};

enum
{
	SOFTRF_NMEA_SENSORS_OFF,
	SOFTRF_NMEA_SENSORS_ON
};

enum
{
	SOFTRF_NMEA_OUTPUT_OFF,
	SOFTRF_NMEA_OUTPUT_UART,
	SOFTRF_NMEA_OUTPUT_UDP,
	SOFTRF_NMEA_OUTPUT_TCP,
	SOFTRF_NMEA_OUTPUT_USB,
	SOFTRF_NMEA_OUTPUT_BLUETOOTH
};

enum
{
	SOFTRF_GDL90_OFF,
	SOFTRF_GDL90_UART,
	SOFTRF_GDL90_UDP,
	SOFTRF_GDL90_TCP,
	SOFTRF_GDL90_USB,
	SOFTRF_GDL90_BLUETOOTH
};

enum
{
	SOFTRF_D1090_OFF,
	SOFTRF_D1090_UART,
	SOFTRF_D1090_UDP,
	SOFTRF_D1090_TCP,
	SOFTRF_D1090_USB,
	SOFTRF_D1090_BLUETOOTH
};

enum
{
	SOFTRF_STEALTH_OFF,
	SOFTRF_STEALTH_ON
};

enum
{
	SOFTRF_NOTRACK_OFF,
	SOFTRF_NOTRACK_ON
};

enum
{
	SOFTRF_POWER_SAVE_NONE      = 0,
	SOFTRF_POWER_SAVE_WIFI      = 1,
	SOFTRF_POWER_SAVE_GNSS      = 2,
	SOFTRF_POWER_SAVE_NORECEIVE = 4
};

/* ------------------------------------------------------------------------- */

/*
 * PART 2. SoftRF 'Badge Edition' e-paper extension.
 */

/*

  TO RETREIVE SETTINGS
  --------------------

  Query:    "$PSKVC,?*4E"

  Response: "$PSKVC,<version>,
                    <mode>,<adapter>,<connection method>,<units>,
                    <radar zoom level>,<data protocol>,<baud rate>,
                    <server name>,<server key>,
                    <screen rotation>,<radar orientation>,
                    <aircrafts database>,<aircraft ID preference>,
                    <view mode>,<voice alarm>,
                    <e-paper anti-ghosting>,<traffic filter>,
                    <power save>,<team member ID>*<checksum><CR><LF>"

  TO APPLY SETTINGS
  -----------------

  Sentence: "$PSKVC,1,
                    <mode>,<adapter>,<connection method>,<units>,
                    <radar zoom level>,<data protocol>,<baud rate>,
                    <server name>,<server key>,
                    <screen rotation>,<radar orientation>,
                    <aircrafts database>,<aircraft ID preference>,
                    <view mode>,<voice alarm>,
                    <e-paper anti-ghosting>,<traffic filter>,
                    <power save>,<team member ID>*<checksum><CR><LF>"

  Response: dump of new settings followed by system restart

  EXAMPLE OF NMEA SENTENCE
  ------------------------

  $PSKVC,1,0,0,0,2,1,0,,,0,0,0,3,0,0,0,0,0,AABBCC*70

  LIST OF SETTINGS AVAILABLE
  --------------------------

  TBD

  RECOMMENDED DEFAULT SETTINGS
  ----------------------------

  TBD

  OTHER INFORMATION
  -----------------

  https://github.com/lyusupov/SoftRF/wiki/SkyView-settings

 */

/*
 *  C/C++ enumerations
 */

enum
{
	SKYVIEW_MODE_STATUS,
	SKYVIEW_MODE_RADAR,
	SKYVIEW_MODE_TEXT,
	SKYVIEW_MODE_BARO,
	SKYVIEW_MODE_TIME,
	SKYVIEW_MODE_IMU,
};

/*
 * 'Radar view' scale factor (outer circle diameter)
 *
 * Metric and Mixed:
 *  LOWEST - 60 KM diameter (30 KM radius)
 *  LOW    - 10 KM diameter ( 5 KM radius)
 *  MEDIUM -  4 KM diameter ( 2 KM radius)
 *  HIGH   -  2 KM diameter ( 1 KM radius)
 *
 * Imperial:
 *  LOWEST - 30 NM diameter ( 15 NM radius)
 *  LOW    -  5 NM diameter (2.5 NM radius)
 *  MEDIUM -  2 NM diameter (  1 NM radius)
 *  HIGH   -  1 NM diameter (0.5 NM radius)
 */
enum
{
	SKYVIEW_ZOOM_LOWEST,
	SKYVIEW_ZOOM_LOW,
	SKYVIEW_ZOOM_MEDIUM,
	SKYVIEW_ZOOM_HIGH
};

enum
{
	SKYVIEW_UNITS_METRIC,
	SKYVIEW_UNITS_IMPERIAL,
	SKYVIEW_UNITS_MIXED     // almost the same as metric, but all the altitudes are in feet
};

enum
{
	SKYVIEW_PROTOCOL_NONE,
	SKYVIEW_PROTOCOL_NMEA, /* FTD-12 */
	SKYVIEW_PROTOCOL_GDL90,
	SKYVIEW_PROTOCOL_MAVLINK_1,
	SKYVIEW_PROTOCOL_MAVLINK_2,
	SKYVIEW_PROTOCOL_D1090,
	SKYVIEW_PROTOCOL_UATRADIO
};

enum
{
	SKYVIEW_ID_REG,
	SKYVIEW_ID_TAIL,
	SKYVIEW_ID_MAM,
	SKYVIEW_ID_TYPE
};

enum
{
	SKYVIEW_VOICE_OFF,
	SKYVIEW_VOICE_1,
	SKYVIEW_VOICE_2,
	SKYVIEW_VOICE_3
};

enum
{
	SKYVIEW_ANTI_GHOSTING_OFF,
	SKYVIEW_ANTI_GHOSTING_AUTO,
	SKYVIEW_ANTI_GHOSTING_2MIN,
	SKYVIEW_ANTI_GHOSTING_5MIN,
	SKYVIEW_ANTI_GHOSTING_10MIN
};

enum
{
	SKYVIEW_TRAFFIC_FILTER_OFF,
	SKYVIEW_TRAFFIC_FILTER_500M,
	SKYVIEW_TRAFFIC_FILTER_1500M
};

enum
{
	SKYVIEW_DB_NONE,
	SKYVIEW_DB_AUTO,
	SKYVIEW_DB_FLN,
	SKYVIEW_DB_OGN,
	SKYVIEW_DB_ICAO
};

enum
{
	SKYVIEW_ROTATE_0,
	SKYVIEW_ROTATE_90,
	SKYVIEW_ROTATE_180,
	SKYVIEW_ROTATE_270
};

/* ------------------------------------------------------------------------- */

/*
 * PART 3. SoftRF 'security' message.
 */

/*

  APPLICABLE
  ----------

  FIRMWARE: 1.1 or newer
  MODEL(S): Badge Edition
            Uni Edition
            Lego Edition
            Prime Edition Mark II

  TO RETREIVE SETTINGS
  --------------------

  Query:    "$PSRFS,?*57"

  Response: "$PSRFS,<version>,<IGC key>*<checksum><CR><LF>"

  TO APPLY SETTINGS
  -----------------

  Sentence: "$PSRFS,1,<IGC key>*<checksum><CR><LF>"

  Response: dump of new settings followed by system restart

  EXAMPLE OF NMEA SENTENCE
  ------------------------

  $PSRFS,1,00000000000000000000000000000000*75

  LIST OF SETTINGS AVAILABLE
  --------------------------

  Version:        1
  IGC key:        128-bit HEX integer

  RECOMMENDED DEFAULT SETTINGS
  ----------------------------

  IGC key:        00000000000000000000000000000000

  OTHER INFORMATION
  -----------------

  https://github.com/lyusupov/SoftRF/wiki/Dongle-settings
  https://github.com/lyusupov/SoftRF/wiki/Settings

 */

/* ------------------------------------------------------------------------- */

/*
 * PART 4. SoftRF 'heartbeat' message.
 */

/*

  Sentence: "$PSRFH,
                    <device ID>,<protocol>,<RX packets>,<TX packets>,
                    <battery voltage>*<checksum><CR><LF>"

  APPLICABLE
  ----------

  FIRMWARE: 1.1 or newer
  MODEL(S): Dongle Edition
            Uni Edition
            Mini Edition
            Academy Edition
            Lego Edition
            ES Edition

  DESCRIPTION
  -----------

  Device ID:       24-bit HEX integer unique ident of SoftRF device
  Protocol:        integer (Legacy, OGNTP, P3I, FANET, UAT, 1090ES)
  RX packets:      integer
  TX packets:      integer
  Battery voltage: in centi-Volt units (0.01 V)

  EXAMPLE OF NMEA SENTENCE
  ------------------------

  $PSRFH,AABBCC,1,0,0,370*76

  INTERVAL
  --------

  1 second

 */

#endif /* SOFTRF_H */
