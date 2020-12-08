/*
 * SoftRF.h
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

#ifndef SOFTRF_H
#define SOFTRF_H

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
  Protocol:       Legacy, OGNTP, P3I, FANET
  Band:           EU, US/CA, AU, NZ, RU, CN, UK, IN
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

  Power save:     Off, GNSS

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
	SOFTRF_POWER_SAVE_NONE = 0,
	SOFTRF_POWER_SAVE_WIFI = 1,
	SOFTRF_POWER_SAVE_GNSS = 2
};

#endif /* SOFTRF_H */
