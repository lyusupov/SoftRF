/*
 * UATbridge(.ino) firmware
 * Copyright (C) 2019 Linar Yusupov
 *
 * Author: Linar Yusupov, linar.r.yusupov@gmail.com
 *
 * Web: http://github.com/lyusupov/SoftRF
 *
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

#include "TimeHelper.h"
#include "LEDHelper.h"
#include "GNSSHelper.h"
#include "RFHelper.h"
#include "SoundHelper.h"
#include "EEPROMHelper.h"
#include "GDL90Helper.h"
#include "NMEAHelper.h"
#include "D1090Helper.h"
#include "BaroHelper.h"
#include "SoCHelper.h"
#include "TrafficHelper.h"
#include "MAVLinkHelper.h"

#include "SoftRF.h"
#include "EasyLink.h"
#include "Protocol_UAT978.h"

#include <uat.h>
#include <fec/char.h>
#include <fec.h>
#include <uat_decode.h>

/*
 * Built-in 128K flash memory of the CC1310F128 (7x7)
 * does fit for either:
 * - RECEIVER & BRIDGE modes, or
 * - NORMAL mode
 * but not both at the same time.
 * Hope that the things will change with appearance of production CC1312 silicon.
 */
//#define NORMAL_MODE

//#define DEBUG_UAT

#define isValidFix()      isValidGNSSFix()
#define isTimeToExport()  (millis() - ExportTimeMarker > 1000)

EasyLink_RxPacket rxPacket;
EasyLink myLink;

eeprom_t eeprom_block;
settings_t *settings = &eeprom_block.field.settings;

ufo_t ThisAircraft;

hardware_info_t hw_info = {
  .model    = SOFTRF_MODEL_UAT,
  .revision = 0,
  .soc      = SOC_NONE,
  .rf       = RF_IC_NONE,
  .gnss     = GNSS_MODULE_NONE,
  .baro     = BARO_MODULE_NONE,
  .display  = DISPLAY_NONE
};

unsigned long ExportTimeMarker = 0;

Stratux_frame_t LPUATRadio_frame = {
  .magic1     = STRATUX_UATRADIO_MAGIC_1,
  .magic2     = STRATUX_UATRADIO_MAGIC_2,
  .magic3     = STRATUX_UATRADIO_MAGIC_3,
  .magic4     = STRATUX_UATRADIO_MAGIC_4,

  .msgLen     = LONG_FRAME_BYTES,
  .rssi       = 0,
  .timestamp  = 0UL,
};

struct uat_adsb_mdb mdb;

#if defined(NORMAL_MODE)
#include <WS2812.h>

#define uni_Color(r,g,b)  {r, g, b}

WS2812 strip(RING_LED_NUM);

uint8_t ring  [RING_LED_NUM][3] = { LED_COLOR_GREEN, LED_COLOR_GREEN,
                                    LED_COLOR_GREEN, LED_COLOR_GREEN,
                                    LED_COLOR_GREEN, LED_COLOR_GREEN,
                                    LED_COLOR_GREEN, LED_COLOR_GREEN };
#endif /* NORMAL_MODE */

#if defined(DEBUG_UAT)
#include <xdc/std.h>

#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Types.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/utils/Load.h>
#include <ti/sysbios/hal/Hwi.h>

static void printUtilization()
{
    xdc_UInt i;
    Memory_Stats memStat;
    Hwi_StackInfo hwiStackStat;
    Load_Stat loadStat;
    Task_Handle tsk;
    float idleLoad;
    uint32_t idleLoadInt, idleLoadFrac;

    /* collect current stats */
    Load_update();

    /* use time NOT spent in idle task for Total CPU Load */
    Load_getTaskLoad(Task_getIdleTask(), &loadStat);
    idleLoad = 100.0 - 100.0*(float)loadStat.threadTime/(float)loadStat.totalTime;
    idleLoadInt = idleLoad;
    idleLoadFrac = 10.0*idleLoad - 10.0*idleLoadInt;

    Serial.write("Total CPU Load: ");
    Serial.print(idleLoadInt);
    Serial.print(".");
    Serial.println(idleLoadFrac);
    Serial.println("");
#if 0
    /* collect stats on all statically Created tasks */
    Serial.println("Task info:");
    for (i = 0; i < Task_Object_count(); i++) {
        tsk = Task_Object_get(NULL, i);
        printTaskInfo(tsk);
    }

    /* collect stats on all dynamically Created tasks */
    tsk = Task_Object_first();
    while (tsk) {
        printTaskInfo(tsk);
        tsk = Task_Object_next(tsk);
    }
    Serial.println("");
#endif
    Hwi_getStackInfo(&hwiStackStat, TRUE);
    Serial.print(F("Hwi stack usage: "));
    Serial.print(hwiStackStat.hwiStackPeak);
    Serial.print("/");
    Serial.println(hwiStackStat.hwiStackSize);
    Serial.println("");

    Memory_getStats(NULL, &memStat);
    Serial.print(F("Heap usage: "));
    Serial.print(memStat.totalSize - memStat.totalFreeSize);
    Serial.print("/");
    Serial.println(memStat.totalSize);
}
#endif /* DEBUG_UAT */

void setup() {
  hw_info.soc = SoC_setup(); // Has to be very first procedure in the execution order

  eeprom_block.field.magic                  = SOFTRF_EEPROM_MAGIC;
  eeprom_block.field.version                = SOFTRF_EEPROM_VERSION;
#if !defined(NORMAL_MODE)
  eeprom_block.field.settings.mode          = SOFTRF_MODE_RECEIVER;
#else /* NORMAL_MODE */
  eeprom_block.field.settings.mode          = SOFTRF_MODE_NORMAL;
#endif /* NORMAL_MODE */
  eeprom_block.field.settings.rf_protocol   = RF_PROTOCOL_OGNTP;
  eeprom_block.field.settings.band          = RF_BAND_EU;
  eeprom_block.field.settings.aircraft_type = AIRCRAFT_TYPE_GLIDER;
  eeprom_block.field.settings.txpower       = RF_TX_POWER_LOW;
  eeprom_block.field.settings.volume        = BUZZER_OFF;
  eeprom_block.field.settings.pointer       = LED_OFF;
  eeprom_block.field.settings.bluetooth     = BLUETOOTH_OFF;
  eeprom_block.field.settings.alarm         = TRAFFIC_ALARM_NONE;

  eeprom_block.field.settings.nmea_g   = false;
  eeprom_block.field.settings.nmea_p   = false;
  eeprom_block.field.settings.nmea_l   = false;
  eeprom_block.field.settings.nmea_s   = false;
  eeprom_block.field.settings.nmea_out = NMEA_OFF;
  eeprom_block.field.settings.gdl90    = GDL90_OFF;
  eeprom_block.field.settings.d1090    = D1090_OFF;
  eeprom_block.field.settings.stealth  = false;
  eeprom_block.field.settings.no_track = false;

  switch (settings->mode)
  {
  case SOFTRF_MODE_UAV:
    MAVLink_setup();
    break;

  case SOFTRF_MODE_NORMAL:
  case SOFTRF_MODE_BRIDGE:
  case SOFTRF_MODE_RECEIVER:
  default:
    Serial.begin( 9600 );
    break;
  }

  Serial.println();
  Serial.print(F(SOFTRF_UAT_IDENT " FW.REV: " SOFTRF_FIRMWARE_VERSION " DEV.ID: "));
  Serial.println(String(SoC->getChipId(), HEX));

  switch (settings->mode)
  {

#if defined(NORMAL_MODE)

  case SOFTRF_MODE_NORMAL:

    ThisAircraft.addr = SoC->getChipId() & 0x00FFFFFF;
    ThisAircraft.aircraft_type = settings->aircraft_type;
    ThisAircraft.protocol = settings->rf_protocol;
    ThisAircraft.stealth  = settings->stealth;
    ThisAircraft.no_track = settings->no_track;

    hw_info.baro = Baro_setup();

#if defined(DEBUG_UAT)
    Serial.print("Baro module ID: ");
    Serial.println(hw_info.baro);
#endif

    hw_info.gnss = GNSS_setup();

#if defined(DEBUG_UAT)
    Serial.print("GNSS module ID: ");
    Serial.println(hw_info.gnss);
#endif

    if (hw_info.gnss == GNSS_MODULE_NONE) {
      Serial.println("WARNING! GNSS module is not detected!");
    } else {
      settings->nmea_g   = true;
      settings->nmea_l   = true;
      settings->nmea_s   = true;
      settings->nmea_out = NMEA_UART;
    }

    init_fec();
    Traffic_setup();
    NMEA_setup();

    strip.begin();
    strip.sendBuffer(ring, RING_LED_NUM);
    delay(1000);
    for(int i=0; i < RING_LED_NUM; i++) {
      ring[i][0] = 0;
      ring[i][1] = 0;
      ring[i][2] = 0;
    }
    strip.sendBuffer(ring,   RING_LED_NUM);

    Serial.println("Normal mode.");
    break;

#else /* NORMAL_MODE */

  case SOFTRF_MODE_UAV:

    ThisAircraft.addr = SoC->getChipId() & 0x00FFFFFF;
    ThisAircraft.aircraft_type = AIRCRAFT_TYPE_UAV;
    ThisAircraft.protocol = settings->rf_protocol;

    init_fec();
    Traffic_setup();

    Serial.println("UAV mode.");
    break;

  case SOFTRF_MODE_RECEIVER:

    hw_info.rf = RF_setup();

#if defined(DEBUG_UAT)
    Serial.print("SPI radio ID: ");
    Serial.println(hw_info.rf);
#endif

    if (hw_info.rf != RF_IC_NONE) {
      settings->mode = SOFTRF_MODE_BRIDGE;

      init_fec();

      if (settings->rf_protocol == RF_PROTOCOL_LEGACY) {
        hw_info.gnss = GNSS_setup();

#if defined(DEBUG_UAT)
        if (hw_info.gnss != GNSS_MODULE_NONE) {
          settings->nmea_g   = true;
          settings->nmea_out = NMEA_UART;
        }
#endif
      }

      Serial.print("Protocol: ");
      Serial.println(
        settings->rf_protocol == RF_PROTOCOL_LEGACY ? legacy_proto_desc.name :
        settings->rf_protocol == RF_PROTOCOL_OGNTP  ? ogntp_proto_desc.name  :
        settings->rf_protocol == RF_PROTOCOL_P3I    ? p3i_proto_desc.name    :
        settings->rf_protocol == RF_PROTOCOL_FANET  ? fanet_proto_desc.name  :
        "UNK"
      );

      Serial.print("GNSS: ");
      Serial.println(GNSS_name[hw_info.gnss]);

      Serial.println("Bridge mode.");
    } else {
      Serial.println("Receiver mode.");
      Serial.flush();
      Serial.end();
      Serial.begin( 2000000 );
    }
    break;

#endif /* NORMAL_MODE */

  default:
    Serial.println("Receiver mode.");
    break;
  }

  myLink.begin(EasyLink_Phy_Custom);
  Serial.println("Listening...");
}

void loop() {

    switch (settings->mode)
    {
#if defined(NORMAL_MODE)
    case SOFTRF_MODE_NORMAL:
      normal();
      break;
#else /* NORMAL_MODE */
    case SOFTRF_MODE_UAV:
      uav();
      break;
    case SOFTRF_MODE_BRIDGE:
      bridge();
      break;
    case SOFTRF_MODE_RECEIVER:
    default:
      receiver();
      break;
#endif /* NORMAL_MODE */
    }
}

static bool UAT_Receive_Sync()
{
  // rxTimeout is in Radio time and needs to be converted from miliseconds to Radio Time
  rxPacket.rxTimeout = EasyLink_ms_To_RadioTime(2000);

  // Turn the receiver on immediately
  rxPacket.absTime = EasyLink_ms_To_RadioTime(0);

  EasyLink_Status status = myLink.receive(&rxPacket);

  if (status == EasyLink_Status_Success) {

#if defined(DEBUG_UAT)
    Serial.print("Packet received with length ");
    Serial.print(rxPacket.len);
    Serial.print(" RSSI ");
    Serial.print(rxPacket.rssi);
#if 0
    Serial.println(" and value:");
    Serial.println(Bin2Hex((byte *) rxPacket.payload, rxPacket.len));
#else
    Serial.println();
#endif
#endif
    return(true);
  } else {
#if defined(DEBUG_UAT)
    Serial.print("Error receiving packet with status code: ");
    Serial.print(status);
    Serial.print(" (");
    Serial.print(myLink.getStatusString(status));
    Serial.println(")");
#endif
    return(false);
  }
}

static bool UAT_receive_complete  = false;
static bool UAT_receive_active    = false;

void UAT_Receive_callback(EasyLink_RxPacket *rxPacket_ptr, EasyLink_Status status)
{
  UAT_receive_active = false;

  if (status == EasyLink_Status_Success) {
    memcpy(&rxPacket, rxPacket_ptr, sizeof(rxPacket));
    UAT_receive_complete  = true;
  }
}

static bool UAT_Receive_Async()
{
  bool success = false;
  EasyLink_Status status;

  if (!UAT_receive_active) {
    status = myLink.receive(&UAT_Receive_callback);

    if (status == EasyLink_Status_Success) {
      UAT_receive_active = true;
    }
  }

  if (UAT_receive_complete == true) {

    success = true;
    UAT_receive_complete = false;
  }

  return success;
}

#define UAT_Receive UAT_Receive_Async

void receiver()
{
  bool success = UAT_Receive();

  if (success) {
    /*
     * If SoftRF-LoRa radio is not found -
     * fallback to Stratux LowPower UAT radio compatible
     * data output
     */
    LPUATRadio_frame.timestamp = now();
    LPUATRadio_frame.rssi = rxPacket.rssi;
    LPUATRadio_frame.msgLen = rxPacket.len;

    memcpy(LPUATRadio_frame.data, rxPacket.payload, rxPacket.len);

    Serial.write((uint8_t *) &LPUATRadio_frame, sizeof(LPUATRadio_frame));
  }
}

void bridge()
{
  RF_loop();

  if (settings->rf_protocol == RF_PROTOCOL_LEGACY) {
    PickGNSSFix();
    GNSSTimeSync();
  }

  bool success = UAT_Receive();

  if (success) {

    int rs_errors;
    ThisAircraft.timestamp = now();

    int frame_type = correct_adsb_frame(rxPacket.payload, &rs_errors);

    if (frame_type != -1 &&
        uat978_decode((void *) rxPacket.payload, &ThisAircraft, &fo) ) {

#if defined(DEBUG_UAT)
      Serial.print(fo.addr, HEX);
      Serial.print(',');
      Serial.print(fo.aircraft_type, HEX);
      Serial.print(',');
      Serial.print(fo.latitude, 6);
      Serial.print(',');
      Serial.print(fo.longitude, 6);
      Serial.print(',');
      Serial.print(fo.altitude);
      Serial.print(',');
      Serial.print(fo.speed);
      Serial.print(',');
      Serial.print(fo.course);
      Serial.print(',');
      Serial.print(fo.vs);
      Serial.println();
      Serial.flush();
#endif

      if (settings->rf_protocol == RF_PROTOCOL_LEGACY) {
        /*
         * "Legacy" needs some accurate timing for proper operation
         */
        if (isValidFix()) {
          RF_Transmit(RF_Encode(&fo), false /* true */);
        }
      } else {
        RF_Transmit(RF_Encode(&fo), false /* true */);
      }
    } else {
#if defined(DEBUG_UAT)
      Serial.println("FEC error");
#endif
    }
  }
}

void normal()
{
  bool success;

  Baro_loop();

  PickGNSSFix();

  GNSSTimeSync();

  ThisAircraft.timestamp = now();

  if (isValidGNSSFix()) {
    ThisAircraft.latitude = gnss.location.lat();
    ThisAircraft.longitude = gnss.location.lng();
    ThisAircraft.altitude = gnss.altitude.meters();
    ThisAircraft.course = gnss.course.deg();
    ThisAircraft.speed = gnss.speed.knots();
    ThisAircraft.hdop = (uint16_t) gnss.hdop.value();
    ThisAircraft.geoid_separation = gnss.separation.meters();
  }

  success = UAT_Receive();

  if (success) {
    int rs_errors;
    ThisAircraft.timestamp = now();

    int frame_type = correct_adsb_frame(rxPacket.payload, &rs_errors);

    if (frame_type != -1 &&
        uat978_decode((void *) rxPacket.payload, &ThisAircraft, &fo) ) {

#if defined(DEBUG_UAT)
      Serial.print(fo.addr, HEX);
      Serial.print(',');
      Serial.print(fo.aircraft_type, HEX);
      Serial.print(',');
      Serial.print(fo.latitude, 6);
      Serial.print(',');
      Serial.print(fo.longitude, 6);
      Serial.print(',');
      Serial.print(fo.altitude);
      Serial.print(',');
      Serial.print(fo.speed);
      Serial.print(',');
      Serial.print(fo.course);
      Serial.print(',');
      Serial.print(fo.vs);
      Serial.println();
      Serial.flush();
#endif

      fo.rssi = rxPacket.rssi;

      for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {

        if (Container[i].addr == fo.addr) {
          Container[i] = fo;
          Traffic_Update(i);
          break;
        } else {
          if (now() - Container[i].timestamp > ENTRY_EXPIRATION_TIME) {
            Container[i] = fo;
            Traffic_Update(i);
            break;
          }
        }
      }

    } else {
#if defined(DEBUG_UAT)
      Serial.println("FEC error");
#endif
    }
  }

  if (isValidGNSSFix()) {
    Traffic_loop();
  }

  if (isTimeToExport() && isValidGNSSFix()) {
    NMEA_Export();
    GDL90_Export();
    D1090_Export();
    ExportTimeMarker = millis();
  }

  NMEA_loop();

  ClearExpired();
}

void uav()
{
  bool success;

  PickMAVLinkFix();

  MAVLinkTimeSync();
  MAVLinkSetWiFiPower();

  ThisAircraft.timestamp = now();

  if (isValidMAVFix()) {
    ThisAircraft.latitude = the_aircraft.location.gps_lat / 1e7;
    ThisAircraft.longitude = the_aircraft.location.gps_lon / 1e7;
    ThisAircraft.altitude = the_aircraft.location.gps_alt / 1000.0;
    ThisAircraft.course = the_aircraft.location.gps_cog;
    ThisAircraft.speed = (the_aircraft.location.gps_vog / 100.0) / _GPS_MPS_PER_KNOT;
    ThisAircraft.pressure_altitude = the_aircraft.location.baro_alt;
    ThisAircraft.hdop = the_aircraft.location.gps_hdop;
  }

  success = UAT_Receive();

  if (success) {
    int rs_errors;
    ThisAircraft.timestamp = now();

    int frame_type = correct_adsb_frame(rxPacket.payload, &rs_errors);

    if (frame_type != -1 &&
        uat978_decode((void *) rxPacket.payload, &ThisAircraft, &fo) ) {

#if defined(DEBUG_UAT)
      Serial.print(fo.addr, HEX);
      Serial.print(',');
      Serial.print(fo.aircraft_type, HEX);
      Serial.print(',');
      Serial.print(fo.latitude, 6);
      Serial.print(',');
      Serial.print(fo.longitude, 6);
      Serial.print(',');
      Serial.print(fo.altitude);
      Serial.print(',');
      Serial.print(fo.speed);
      Serial.print(',');
      Serial.print(fo.course);
      Serial.print(',');
      Serial.print(fo.vs);
      Serial.println();
      Serial.flush();
#endif

      fo.rssi = rxPacket.rssi;

      for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {

        if (Container[i].addr == fo.addr) {
          Container[i] = fo;
          Traffic_Update(i);
          break;
        } else {
          if (now() - Container[i].timestamp > ENTRY_EXPIRATION_TIME) {
            Container[i] = fo;
            Traffic_Update(i);
            break;
          }
        }
      }

    } else {
#if defined(DEBUG_UAT)
      Serial.println("FEC error");
#endif
    }
  }

  if (isValidMAVFix()) {
    Traffic_loop();
  }

  if (isTimeToExport() && isValidMAVFix()) {
    MAVLinkShareTraffic();
    ExportTimeMarker = millis();
  }

  ClearExpired();
}
