/*
 * Platform_RPi.cpp
 * Copyright (C) 2018-2019 Linar Yusupov
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

#include "Platform_RPi.h"
#include "SoCHelper.h"
#include "EEPROMHelper.h"
#include <TinyGPS++.h>
#include <aircraft.h>
#include "RFHelper.h"
#include "LEDHelper.h"
#include "SoundHelper.h"
#include "BaroHelper.h"
#include "TrafficHelper.h"
#include "NMEAHelper.h"
#include "GDL90Helper.h"
#include "D1090Helper.h"
#include "JSONHelper.h"
#include "WiFiHelper.h"
#include "EPDHelper.h"
#include "BatteryHelper.h"

#include "TCPServer.h"

#include <stdio.h>
#include <sys/select.h>

#include <iostream>

#include <ArduinoJson.h>

// Dragino LoRa/GPS HAT or compatible SX1276 pin mapping
lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .rxtx = { LMIC_UNUSED_PIN, LMIC_UNUSED_PIN },
    .rst = SOC_GPIO_PIN_RST,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
};

TTYSerial Serial1("/dev/ttyAMA0");
TTYSerial Serial2("/dev/ttyUSB0");

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

void onEvent (ev_t ev) { }

eeprom_t eeprom_block;
settings_t *settings = &eeprom_block.field.settings;
ufo_t ThisAircraft;
aircraft the_aircraft;

char UDPpacketBuffer[UDP_PACKET_BUFSIZE]; // buffer to hold incoming and outgoing packets

hardware_info_t hw_info = {
  .model    = SOFTRF_MODEL_RASPBERRY,
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

static void RPi_setup()
{
  eeprom_block.field.magic = SOFTRF_EEPROM_MAGIC;
  eeprom_block.field.version = SOFTRF_EEPROM_VERSION;
  eeprom_block.field.settings.mode = SOFTRF_MODE_NORMAL;
  eeprom_block.field.settings.rf_protocol = RF_PROTOCOL_OGNTP;
  eeprom_block.field.settings.band = RF_BAND_EU;
  eeprom_block.field.settings.aircraft_type = AIRCRAFT_TYPE_GLIDER;
  eeprom_block.field.settings.txpower = RF_TX_POWER_FULL;
  eeprom_block.field.settings.volume = BUZZER_VOLUME_FULL;
  eeprom_block.field.settings.pointer = DIRECTION_NORTH_UP;
  eeprom_block.field.settings.bluetooth = BLUETOOTH_OFF;
  eeprom_block.field.settings.alarm = TRAFFIC_ALARM_DISTANCE;

  eeprom_block.field.settings.nmea_g     = true;
  eeprom_block.field.settings.nmea_p     = false;
  eeprom_block.field.settings.nmea_l     = true;
  eeprom_block.field.settings.nmea_s     = true;
  eeprom_block.field.settings.nmea_out   = NMEA_UART;
  eeprom_block.field.settings.gdl90      = GDL90_OFF;
  eeprom_block.field.settings.d1090      = D1090_OFF;
  eeprom_block.field.settings.json       = JSON_OFF;
  eeprom_block.field.settings.stealth    = false;
  eeprom_block.field.settings.no_track   = false;
  eeprom_block.field.settings.power_save = POWER_SAVE_NONE;
}

static void RPi_fini()
{

}

static uint32_t RPi_getChipId()
{
  return gethostid();
}

static long RPi_random(long howsmall, long howBig)
{
  return random(howsmall, howBig);
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

static byte RPi_Display_setup()
{
  byte rval = DISPLAY_NONE;

  if (EPD_setup()) {
    rval = DISPLAY_EPD_2_7;
  }

  return rval;
}

static void RPi_Display_loop()
{
  if (hw_info.display == DISPLAY_EPD_2_7) {
    EPD_loop();
  }
}

static void RPi_Display_fini(const char *msg)
{

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
}

static void RPi_CC13XX_restart()
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

const SoC_ops_t RPi_ops = {
  SOC_RPi,
  "RPi",
  RPi_setup,
  RPi_fini,
  RPi_getChipId,
  NULL,
  NULL,
  NULL,
  RPi_random,
  NULL,
  NULL,
  NULL,
  RPi_WiFi_transmit_UDP,
  NULL,
  NULL,
  NULL,
  NULL,
  RPi_SPI_begin,
  RPi_swSer_begin,
  NULL,
  NULL,
  RPi_Display_setup,
  RPi_Display_loop,
  RPi_Display_fini,
  NULL,
  NULL,
  NULL,
  RPi_get_PPS_TimeMarker,
  NULL,
  RPi_UATSerial_begin,
  RPi_CC13XX_restart,
  RPi_WDT_setup,
  RPi_WDT_fini
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
    NMEA_Out((byte *) str, len, true);
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

    if (isTimeToExport() && isValidFix()) {
      NMEA_Export();
      GDL90_Export();
      D1090_Export();
      JSON_Export();
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
      String str = Bin2Hex(Container[i].raw, size);
      size_t str_len = str.length();

      if (str_len > 0) {
        // Raw data
        char hexdata[2 * MAX_PKT_SIZE + 1];

        str.toCharArray(hexdata, sizeof(hexdata));

        if (str_len > 2 * MAX_PKT_SIZE) {
          str_len = 2 * MAX_PKT_SIZE;
        }

        for(int j = 0; j < str_len ; j+=2)
        {
          TxBuffer[j>>1] = getVal(hexdata[j+1]) + (getVal(hexdata[j]) << 4);
        }

        size_t tx_size = str_len / 2;

        if (tx_size > 0) {
          /* Follow duty cycle rule */
          if (RF_Transmit(tx_size, true /* false */)) {
#if 0
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

  Serial.begin(38400);

  hw_info.soc = SoC_setup(); // Has to be very first procedure in the execution order

  hw_info.rf = RF_setup();

  if (hw_info.rf == RF_IC_NONE) {
      exit(EXIT_FAILURE);
  }

#if 0
  Serial.print("Intializing E-ink display module (may take up to 10 seconds)... ");
  Serial.flush();
  hw_info.display = SoC->Display_setup();
  if (hw_info.display != DISPLAY_NONE) {
    Serial.println(" done.");
  } else {
    Serial.println(" failed!");
  }
#endif

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
	if( pthread_create(&traffic_tcpserv_thread, NULL, traffic_tcpserv_loop, (void *)0) != 0) {
    fprintf( stderr, "pthread_create(traffic_tcpserv_thread) Failed\n\n" );
    exit(EXIT_FAILURE);
  }

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
  }

  Traffic_TCP_Server.detach();
  return 0;
}

#endif /* RASPBERRY_PI */
