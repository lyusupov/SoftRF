/*
 * Platform_RPi.cpp
 * Copyright (C) 2018-2021 Linar Yusupov
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

/*
 * Usage example:
 *
 *  pi@raspberrypi $ make -f Makefile.RPi
 *
 *     < ... skipped ... >
 *
 *  pi@raspberrypi $ { echo "{class:SOFTRF,protocol:OGNTP}" ; cat /dev/ttyUSB0 ; } | sudo ./SoftRF
 *  SX1276 RFIC is detected.
 *  $GPGSA,A,3,02,30,05,06,07,09,,,,,,,5.09,3.19,3.97*04
 *  $GPRMC,145750.00,A,5XXX.XXX68,N,03XXX.XXX33,E,0.701,,051118,,,A*7E
 *  $GPGGA,145750.00,5XXX.XXX68,N,03XXX.XXX33,E,1,06,3.19,179.2,M,12.5,M,,*5E
 *  $PFLAA,3,0,0,0,2,C5D804!OGN_C5D804,0,,0,00000.0,1*60
 *  $PFLAU,1,1,2,1,3,-30,2,0,0*4E
 *
 *     < ... skipped ... >
 *
 *  Concurrent shell session:
 *
 *  pi@raspberrypi $ ps -ax | grep dump1090
 *  2381 pts/1    Sl+    1:53 dump1090 --interactive --net
 *
 *  pi@raspberrypi $ wget -q -O - http://localhost:8080/data/aircraft.json | nc -N localhost 30007
 *
 */

#if defined(RASPBERRY_PI)

#include "../system/SoC.h"
#include "../driver/EEPROM.h"
#include <TinyGPS++.h>
#if !defined(EXCLUDE_MAVLINK)
#include <aircraft.h>
#endif /* EXCLUDE_MAVLINK */
#include "../driver/RF.h"
#include "../driver/LED.h"
#include "../driver/Sound.h"
#include "../driver/Baro.h"
#include "../TrafficHelper.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/GDL90.h"
#include "../protocol/data/D1090.h"
#include "../protocol/data/JSON.h"
#include "../driver/WiFi.h"
#include "../driver/EPD.h"
#include "../driver/Battery.h"
#include "../driver/Bluetooth.h"

#include "TCPServer.h"

#include <stdio.h>
#include <sys/select.h>

#include <iostream>

#include <ArduinoJson.h>

// Dragino LoRa/GPS HAT or compatible SX1276 pin mapping
lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .txe = LMIC_UNUSED_PIN,
    .rxe = LMIC_UNUSED_PIN,
    .rst = SOC_GPIO_PIN_RST,
#if !defined(USE_OGN_RF_DRIVER)
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
#else
    .dio = {SOC_GPIO_PIN_DIO0, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
#endif
    .busy = SOC_GPIO_PIN_DIO0,
    .tcxo = LMIC_UNUSED_PIN,
};

TTYSerial Serial1("/dev/ttyAMA0");
TTYSerial Serial2("/dev/ttyUSB0");

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
#if defined(USE_BASICMAC)
void os_getJoinEui (u1_t* buf) { }
//void os_getDevEui (u1_t* buf) { }
void os_getNwkKey (u1_t* buf) { }
//u1_t os_getRegion (void) { return REGCODE_EU868; }
#else
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
#endif

#if defined(USE_BASICMAC)
extern "C" void onLmicEvent (ev_t ev);
void onLmicEvent (ev_t ev) {
#else
void onEvent (ev_t ev) {
#endif
}

eeprom_t eeprom_block;
settings_t *settings = &eeprom_block.field.settings;
ufo_t ThisAircraft;

#if !defined(EXCLUDE_MAVLINK)
aircraft the_aircraft;
#endif /* EXCLUDE_MAVLINK */

char UDPpacketBuffer[UDP_PACKET_BUFSIZE]; // buffer to hold incoming and outgoing packets

static struct rst_info reset_info = {
  .reason = REASON_DEFAULT_RST,
};

hardware_info_t hw_info = {
  .model    = DEFAULT_SOFTRF_MODEL,
  .revision = 0,
  .soc      = SOC_NONE,
  .rf       = RF_IC_NONE,
  .gnss     = GNSS_MODULE_NONE,
  .baro     = BARO_MODULE_NONE,
  .display  = DISPLAY_NONE
};

#define isTimeToExport() (millis() - ExportTimeMarker > 1000)
unsigned long ExportTimeMarker = 0;

std::string input_line;

TCPServer Traffic_TCP_Server;

#if defined(USE_EPAPER)
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> __attribute__ ((common)) epd_waveshare(GxEPD2_270(/*CS=5*/ 8,
                                       /*DC=*/ 25, /*RST=*/ 17, /*BUSY=*/ 24));
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> *display;
#endif /* USE_EPAPER */

ui_settings_t ui_settings = {
    .adapter      = 0,
    .connection   = 0,
    .units        = UNITS_METRIC,
    .zoom         = ZOOM_MEDIUM,
    .protocol     = PROTOCOL_NMEA,
    .baudrate     = 0,
    .server       = { 0 },
    .key          = { 0 },
    .resvd1       = 0,
    .orientation  = DIRECTION_TRACK_UP,
    .adb          = DB_NONE,
    .idpref       = ID_REG,
    .vmode        = VIEW_MODE_STATUS,
    .voice        = VOICE_OFF,
    .aghost       = ANTI_GHOSTING_OFF,
    .filter       = TRAFFIC_FILTER_OFF,
    .power_save   = 0,
    .team         = 0
};

ui_settings_t *ui;

//-------------------------------------------------------------------------
//
// The MIT License (MIT)
//
// Copyright (c) 2015 Andrew Duncan
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
// CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//-------------------------------------------------------------------------

#include <fcntl.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/ioctl.h>

static uint32_t SerialNumber = 0;

void RPi_SerialNumber(void)
{
    int fd = open("/dev/vcio", 0);
    if (fd == -1)
    {
        perror("open /dev/vcio");
        exit(EXIT_FAILURE);
    }

    uint32_t property[32] =
    {
        0x00000000,
        0x00000000,
        0x00010004,
        0x00000010,
        0x00000000,
        0x00000000,
        0x00000000,
        0x00000000,
        0x00000000,
        0x00000000
    };

    property[0] = 10 * sizeof(property[0]);

    if (ioctl(fd, _IOWR(100, 0, char *), property) == -1)
    {
        perror("ioctl");
        exit(EXIT_FAILURE);
    }

    close(fd);

    SerialNumber = property[5];
}

//----- end of MIT License ------------------------------------------------

static void RPi_setup()
{
  eeprom_block.field.magic                  = SOFTRF_EEPROM_MAGIC;
  eeprom_block.field.version                = SOFTRF_EEPROM_VERSION;
  eeprom_block.field.settings.mode          = SOFTRF_MODE_NORMAL;
  eeprom_block.field.settings.rf_protocol   = RF_PROTOCOL_OGNTP;
  eeprom_block.field.settings.band          = RF_BAND_EU;
  eeprom_block.field.settings.aircraft_type = AIRCRAFT_TYPE_GLIDER;
  eeprom_block.field.settings.txpower       = RF_TX_POWER_FULL;
  eeprom_block.field.settings.volume        = BUZZER_VOLUME_FULL;
  eeprom_block.field.settings.pointer       = DIRECTION_NORTH_UP;
  eeprom_block.field.settings.bluetooth     = BLUETOOTH_OFF;
  eeprom_block.field.settings.alarm         = TRAFFIC_ALARM_DISTANCE;

  eeprom_block.field.settings.nmea_g        = true;
  eeprom_block.field.settings.nmea_p        = false;
  eeprom_block.field.settings.nmea_l        = true;
  eeprom_block.field.settings.nmea_s        = true;
  eeprom_block.field.settings.nmea_out      = NMEA_UART;
  eeprom_block.field.settings.gdl90         = GDL90_OFF;
  eeprom_block.field.settings.d1090         = D1090_OFF;
  eeprom_block.field.settings.json          = JSON_OFF;
  eeprom_block.field.settings.stealth       = false;
  eeprom_block.field.settings.no_track      = false;
  eeprom_block.field.settings.power_save    = POWER_SAVE_NONE;
  eeprom_block.field.settings.freq_corr     = 0;

  ui = &ui_settings;

  RPi_SerialNumber();
}

static void RPi_post_init()
{

#if 0
  Serial.println();
  Serial.println(F("Raspberry Pi Power-on Self Test"));
  Serial.println();
  Serial.flush();

  Serial.println(F("Built-in components:"));

  Serial.print(F("RADIO   : ")); Serial.println(hw_info.rf      != RF_IC_NONE       ? F("PASS") : F("FAIL"));
  Serial.print(F("GNSS    : ")); Serial.println(hw_info.gnss    != GNSS_MODULE_NONE ? F("PASS") : F("FAIL"));
  Serial.print(F("DISPLAY : ")); Serial.println(hw_info.display != DISPLAY_NONE     ? F("PASS") : F("FAIL"));

  Serial.println();
  Serial.println(F("External components:"));
  Serial.print(F("BMx280  : ")); Serial.println(hw_info.baro    != BARO_MODULE_NONE ? F("PASS") : F("N/A"));

  Serial.println();
  Serial.println(F("Power-on Self Test is completed."));
  Serial.println();
  Serial.flush();
#endif

#if defined(USE_EPAPER)

  EPD_info1(false, false);

#endif /* USE_EPAPER */
}

static void RPi_loop()
{

}

static void RPi_fini(int reason)
{

}

static void RPi_reset()
{

}

static uint32_t RPi_getChipId()
{
  uint32_t id = SerialNumber ? SerialNumber : gethostid();

  return DevID_Mapper(id);
}

static void* RPi_getResetInfoPtr()
{
  return (void *) &reset_info;
}

static long RPi_random(long howsmall, long howBig)
{
  return howsmall + random() % (howBig - howsmall);
}

static void RPi_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  /* TBD */
}

static void RPi_SPI_begin()
{
  SPI.begin();
}

static void RPi_swSer_begin(unsigned long baud)
{
  swSer.begin(baud);
}

pthread_t RPi_EPD_update_thread;

static byte RPi_Display_setup()
{
  byte rval = DISPLAY_NONE;

#if defined(USE_EPAPER)
// GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> *epd_waveshare = new GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT>(GxEPD2_270(/*CS=5*/ 8,
//                                       /*DC=*/ 25, /*RST=*/ 17, /*BUSY=*/ 24));

  display = &epd_waveshare;

  if (EPD_setup(true)) {

    if ( pthread_create(&RPi_EPD_update_thread, NULL, &EPD_Task, (void *)0) != 0) {
      fprintf( stderr, "pthread_create(EPD_Task) Failed\n\n" );
      exit(EXIT_FAILURE);
    }

#if 0
    struct sched_param  param;
    param.sched_priority = 50;
    pthread_setschedparam(RPi_EPD_update_thread, SCHED_RR, &param);
#endif

    rval = DISPLAY_EPD_2_7;
  }
#endif /* USE_EPAPER */

  return rval;
}

static void RPi_Display_loop()
{
#if defined(USE_EPAPER)
  if (hw_info.display == DISPLAY_EPD_2_7) {
    EPD_loop();
  }
#endif /* USE_EPAPER */
}

static void RPi_Display_fini(int reason)
{
#if defined(USE_EPAPER)

  EPD_Clear_Screen();
  EPD_fini(reason);

  if ( RPi_EPD_update_thread != (pthread_t) 0)
  {
    pthread_cancel( RPi_EPD_update_thread );
  }
#endif /* USE_EPAPER */
}

static void RPi_Battery_setup()
{
  /* TBD */
}

static float RPi_Battery_param(uint8_t param)
{
  float rval;

  switch (param)
  {
  case BATTERY_PARAM_THRESHOLD:
    rval = BATTERY_THRESHOLD_USB;
    break;

  case BATTERY_PARAM_CUTOFF:
    rval = BATTERY_CUTOFF_USB;
    break;

  case BATTERY_PARAM_CHARGE:
    /* TBD */

    rval = 100;
    break;

  case BATTERY_PARAM_VOLTAGE:
  default:
    rval = BATTERY_THRESHOLD_USB + 0.05;
    break;
  }

  return rval;
}

void RPi_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long RPi_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
}

static void RPi_UATSerial_begin(unsigned long baud)
{
  UATSerial.begin(baud);
  UATSerial.dtr(false);
  UATSerial.rts(false);
}

static void RPi_UATModule_restart()
{
  UATSerial.dtr(false);

  delay(100);

#if DEBUG
  Serial.println("RTS on");
#endif

  UATSerial.rts(true);

  delay(100);

#if DEBUG
  Serial.println("RTS off");
#endif

  UATSerial.rts(false);
}

static void RPi_WDT_setup()
{
  /* TBD */
}

static void RPi_WDT_fini()
{
  /* TBD */
}

static void RPi_Button_setup()
{
  /* TODO */
}

static void RPi_Button_loop()
{
  /* TODO */
}

static void RPi_Button_fini()
{
  /* TODO */
}

const SoC_ops_t RPi_ops = {
  SOC_RPi,
  "RPi",
  RPi_setup,
  RPi_post_init,
  RPi_loop,
  RPi_fini,
  RPi_reset,
  RPi_getChipId,
  RPi_getResetInfoPtr,
  NULL,
  NULL,
  NULL,
  RPi_random,
  NULL,
  NULL,
  NULL,
  NULL,
  RPi_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  RPi_SPI_begin,
  RPi_swSer_begin,
  NULL,
  NULL,
  NULL,
  NULL,
  RPi_Display_setup,
  RPi_Display_loop,
  RPi_Display_fini,
  RPi_Battery_setup,
  RPi_Battery_param,
  NULL,
  RPi_get_PPS_TimeMarker,
  NULL,
  RPi_UATSerial_begin,
  RPi_UATModule_restart,
  RPi_WDT_setup,
  RPi_WDT_fini,
  RPi_Button_setup,
  RPi_Button_loop,
  RPi_Button_fini
};

static bool inputAvailable()
{
  struct timeval tv;
  fd_set fds;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
  return (FD_ISSET(0, &fds));
}

static void parseNMEA(const char *str, int len)
{
  // NMEA input
  for (int i=0; i < len; i++) {
    gnss.encode(str[i]);
  }
  if (settings->nmea_g) {
    NMEA_Out(settings->nmea_out, (byte *) str, len, true);
  }

  GNSSTimeSync();

  if (isValidGNSSFix()) {
    ThisAircraft.latitude = gnss.location.lat();
    ThisAircraft.longitude = gnss.location.lng();
    ThisAircraft.altitude = gnss.altitude.meters();
    ThisAircraft.course = gnss.course.deg();
    ThisAircraft.speed = gnss.speed.knots();
    ThisAircraft.hdop = (uint16_t) gnss.hdop.value();
    ThisAircraft.geoid_separation = gnss.separation.meters();

    /*
     * When geoidal separation is zero or not available - use approx. EGM96 value
     */
    if (ThisAircraft.geoid_separation == 0.0) {
      ThisAircraft.geoid_separation = (float) LookupSeparation(
                                                ThisAircraft.latitude,
                                                ThisAircraft.longitude
                                              );
      /* we can assume the GPS unit is giving ellipsoid height */
      ThisAircraft.altitude -= ThisAircraft.geoid_separation;
    }
  }
}

static void RPi_PickGNSSFix()
{
  if (inputAvailable()) {
    std::getline(std::cin, input_line);
    const char *str = input_line.c_str();
    int len = input_line.length();

    if (str[0] == '$' && str[1] == 'G') {
      // NMEA input
      parseNMEA(str, len);

    } else if (str[0] == '{') {
      // JSON input

      JsonObject& root = jsonBuffer.parseObject(str);

      JsonVariant msg_class = root["class"];

      if (msg_class.success()) {
        const char *msg_class_s = msg_class.as<char*>();

        if (!strcmp(msg_class_s,"TPV")) { // "TPV"
          parseTPV(root);
        } else if (!strcmp(msg_class_s,"SOFTRF")) {
          parseSettings(root);

          RF_setup();
          Traffic_setup();
        }
      }

      if (root.containsKey("now") &&
          root.containsKey("messages") &&
          root.containsKey("aircraft")) {
        /* 'aircraft.json' output from 'dump1090' application */
        parseD1090(root);
      } else if (root.containsKey("aircraft")) {
        /* uAvionix PingStation */
        parsePING(root);
      }

      jsonBuffer.clear();

      if ((time(NULL) - now()) > 3) {
        hasValidGPSDFix = false;
      }
    }
  }
}

static void RPi_ReadTraffic()
{
  string traffic_input = Traffic_TCP_Server.getMessage();
  if (traffic_input != "") {
    const char *str = traffic_input.c_str();
    int len = traffic_input.length();

    if (str[0] == '{') {
      // JSON input

//    cout << "Traffic message:" << traffic_input << endl;

      JsonObject& root = jsonBuffer.parseObject(str);

      JsonVariant msg_class = root["class"];

      if (msg_class.success()) {
        const char *msg_class_s = msg_class.as<char*>();

        if (!strcmp(msg_class_s,"SOFTRF")) {
          parseSettings(root);

          RF_setup();
          Traffic_setup();
        }
      }

      if (root.containsKey("now") &&
          root.containsKey("messages") &&
          root.containsKey("aircraft")) {
        /* 'aircraft.json' output from 'dump1090' application */
        if (isValidFix()) {
          parseD1090(root);
        }
      } else if (root.containsKey("aircraft")) {
        /* uAvionix PingStation */
        if (isValidFix()) {
          parsePING(root);
        }
      }

      JsonVariant rawdata = root["rawdata"];
      if (rawdata.success()) {
        parseRAW(root);
      }

      jsonBuffer.clear();
    } else if (str[0] == 'q') {
      if (len >= 4 && str[1] == 'u' && str[2] == 'i' && str[3] == 't') {
        Traffic_TCP_Server.detach();
        fprintf( stderr, "Program termination.\n" );
        exit(EXIT_SUCCESS);
      }
    }

    Traffic_TCP_Server.clean();
  }
}

void normal_loop()
{
    /* Read GNSS data from standard input */
    RPi_PickGNSSFix();

    /* Read NMEA data from GNSS module on GPIO pins */
//    PickGNSSFix();

    RPi_ReadTraffic();

    RF_loop();

    ThisAircraft.timestamp = now();

    if (isValidFix()) {
      RF_Transmit(RF_Encode(&ThisAircraft), true);
    }

    bool success = RF_Receive();

    if (success && isValidFix()) ParseData();

    if (isValidFix()) {
      Traffic_loop();
    }

    if (isTimeToExport()) {
      NMEA_Export();

      if (isValidFix()) {
        GDL90_Export();
        D1090_Export();
        JSON_Export();
      }
      ExportTimeMarker = millis();
    }

    // Handle Air Connect
    NMEA_loop();

    SoC->Display_loop();

    ClearExpired();
}

void relay_loop()
{
    /* Read GNSS data from standard input */
    RPi_PickGNSSFix();

    /* Read NMEA data from GNSS module on GPIO pins */
//    PickGNSSFix();

    RPi_ReadTraffic();

    RF_loop();

    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
      size_t size = RF_Payload_Size(settings->rf_protocol);
      size = size > sizeof(Container[i].raw) ? sizeof(Container[i].raw) : size;

      if (memcmp (Container[i].raw, EmptyFO.raw, size) != 0) {
        // Raw data
        size_t tx_size = sizeof(TxBuffer) > size ? size : sizeof(TxBuffer);
        memcpy(TxBuffer, Container[i].raw, tx_size);

        if (tx_size > 0) {
          /* Follow duty cycle rule */
          if (RF_Transmit(tx_size, true /* false */)) {
#if 0
            String str = Bin2Hex(TxBuffer, tx_size);
            printf("%s\n", str.c_str());
#endif
            Container[i] = EmptyFO;
          }
        }
      } else if (isValidFix() &&
                 Container[i].addr &&
                 Container[i].latitude  != 0.0 &&
                 Container[i].longitude != 0.0 &&
                 Container[i].altitude  != 0.0 &&
                 Container[i].distance < (ALARM_ZONE_NONE * 2) ) {

        fo = Container[i];
        fo.timestamp = now(); /* GNSS date&time */

        /* Follow duty cycle rule */
        if (RF_Transmit(RF_Encode(&fo), true /* false */)) {
#if 0
          printf("%06X %f %f %f %d %d %d\n",
              fo.addr,
              fo.latitude,
              fo.longitude,
              fo.altitude,
              fo.addr_type,
              (int) fo.vs,
              fo.aircraft_type);
#endif
          Container[i] = EmptyFO;
        }
      }
    }
}

unsigned int pos_ndx = 0;
unsigned long TxPosUpdMarker = 0;

void txrx_test_loop()
{
  bool success = false;
#if DEBUG_TIMING
  unsigned long baro_start_ms, baro_end_ms;
  unsigned long tx_start_ms, tx_end_ms, rx_start_ms, rx_end_ms;
  unsigned long parse_start_ms, parse_end_ms, led_start_ms, led_end_ms;
  unsigned long export_start_ms, export_end_ms;
  unsigned long oled_start_ms, oled_end_ms;
#endif

  setTime(time(NULL));

  RPi_ReadTraffic();

  RF_loop();

  ThisAircraft.timestamp = now();

  if (TxPosUpdMarker == 0 || (millis() - TxPosUpdMarker) > 4000 ) {
    ThisAircraft.latitude =  pgm_read_float( &txrx_test_positions[pos_ndx][0]);
    ThisAircraft.longitude =  pgm_read_float( &txrx_test_positions[pos_ndx][1]);
    pos_ndx = (pos_ndx + 1) % TXRX_TEST_NUM_POSITIONS;
    TxPosUpdMarker = millis();
  }

  ThisAircraft.altitude = TXRX_TEST_ALTITUDE;
  ThisAircraft.course = TXRX_TEST_COURSE;
  ThisAircraft.speed = TXRX_TEST_SPEED;
  ThisAircraft.vs = TXRX_TEST_VS;

#if DEBUG_TIMING
  tx_start_ms = millis();
#endif

  RF_Transmit(RF_Encode(&ThisAircraft), true);

#if DEBUG_TIMING
  tx_end_ms = millis();
  rx_start_ms = millis();
#endif
  success = RF_Receive();
#if DEBUG_TIMING
  rx_end_ms = millis();
#endif

#if DEBUG_TIMING
  parse_start_ms = millis();
#endif
  if (success) ParseData();
#if DEBUG_TIMING
  parse_end_ms = millis();
#endif

  Traffic_loop();

#if DEBUG_TIMING
  export_start_ms = millis();
#endif
  if (isTimeToExport()) {
    NMEA_Position();
    NMEA_Export();
    GDL90_Export();
    D1090_Export();
    ExportTimeMarker = millis();
  }
#if DEBUG_TIMING
  export_end_ms = millis();
#endif

#if DEBUG_TIMING
  if (tx_end_ms - tx_start_ms) {
    Serial.print(F("TX start: "));
    Serial.print(tx_start_ms);
    Serial.print(F(" TX stop: "));
    Serial.println(tx_end_ms);
  }
  if (rx_end_ms - rx_start_ms) {
    Serial.print(F("RX start: "));
    Serial.print(rx_start_ms);
    Serial.print(F(" RX stop: "));
    Serial.println(rx_end_ms);
  }
  if (parse_end_ms - parse_start_ms) {
    Serial.print(F("Parse start: "));
    Serial.print(parse_start_ms);
    Serial.print(F(" Parse stop: "));
    Serial.println(parse_end_ms);
  }

  if (export_end_ms - export_start_ms) {
    Serial.print(F("Export start: "));
    Serial.print(export_start_ms);
    Serial.print(F(" Export stop: "));
    Serial.println(export_end_ms);
  }
#endif

  // Handle Air Connect
  NMEA_loop();

  ClearExpired();
}


void * traffic_tcpserv_loop(void * m)
{
  pthread_detach(pthread_self());
  Traffic_TCP_Server.receive();
}

int main()
{
  // Init GPIO bcm
  if (!bcm2835_init()) {
      fprintf( stderr, "bcm2835_init() Failed\n\n" );
      exit(EXIT_FAILURE);
  }

  Serial.begin(SERIAL_OUT_BR);

  hw_info.soc = SoC_setup(); // Has to be very first procedure in the execution order

  Serial.println();
  Serial.print(F(SOFTRF_IDENT));
  Serial.print(SoC->name);
  Serial.print(F(" FW.REV: " SOFTRF_FIRMWARE_VERSION " DEV.ID: "));
  Serial.println(String(SoC->getChipId(), HEX));
  Serial.println(F("Copyright (C) 2015-2021 Linar Yusupov. All rights reserved."));
  Serial.flush();

  hw_info.rf = RF_setup();

  if (hw_info.rf == RF_IC_NONE) {
      exit(EXIT_FAILURE);
  }

  Serial.print("Intializing E-ink display module (may take up to 10 seconds)... ");
  Serial.flush();
  hw_info.display = SoC->Display_setup();
  if (hw_info.display != DISPLAY_NONE) {
    Serial.println(" done.");
  } else {
    Serial.println(" failed!");
  }

  ThisAircraft.addr = SoC->getChipId() & 0x00FFFFFF;
  ThisAircraft.aircraft_type = settings->aircraft_type;
  ThisAircraft.protocol = settings->rf_protocol;
  ThisAircraft.stealth  = settings->stealth;
  ThisAircraft.no_track = settings->no_track;

//  hw_info.gnss = GNSS_setup();

  Traffic_setup();
  NMEA_setup();

  Traffic_TCP_Server.setup(JSON_SRV_TCP_PORT);

  pthread_t traffic_tcpserv_thread;
  if ( pthread_create(&traffic_tcpserv_thread, NULL, traffic_tcpserv_loop, (void *)0) != 0) {
    fprintf( stderr, "pthread_create(traffic_tcpserv_thread) Failed\n\n" );
    exit(EXIT_FAILURE);
  }

  SoC->post_init();

  SoC->WDT_setup();

  while (true) {
    switch (settings->mode)
    {
    case SOFTRF_MODE_TXRX_TEST:
      txrx_test_loop();
      break;
    case SOFTRF_MODE_RELAY:
      relay_loop();
      break;
    case SOFTRF_MODE_NORMAL:
    default:
      normal_loop();
      break;
    }

#if defined(TAKE_CARE_OF_MILLIS_ROLLOVER)
    /* take care of millis() rollover on a long term run */
    if (millis() > (47 * 24 * 3600 * 1000UL)) {
      time_t current_time = time(NULL);
      struct tm timebuf;

      if (current_time == ((time_t)-1) ||
          localtime_r(&current_time, &timebuf) == NULL) {
        Traffic_TCP_Server.detach();
        fprintf(stderr, "Failure to obtain the current time.\n");
        exit(EXIT_FAILURE);
      }

      /* shut SoftRF down at night time only */
      if (timebuf.tm_hour >= 2 && timebuf.tm_hour <= 5) {
        Traffic_TCP_Server.detach();
        fprintf( stderr, "Program termination: millis() rollover prevention.\n" );
        exit(EXIT_SUCCESS);
      }
    }
#endif /* TAKE_CARE_OF_MILLIS_ROLLOVER */
  }

  Traffic_TCP_Server.detach();
  return 0;
}

void shutdown(int reason)
{
  SoC->WDT_fini();

  if (hw_info.display != DISPLAY_NONE) {
    SoC->Display_fini(reason);
  }

  Traffic_TCP_Server.detach();
  fprintf( stderr, "Program termination. Reason code: %d.\n", reason );
  exit(EXIT_SUCCESS);
}

#endif /* RASPBERRY_PI */
