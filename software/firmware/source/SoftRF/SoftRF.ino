/*
 * SoftRF(.ino) firmware
 * Copyright (C) 2016-2017 Linar Yusupov
 *
 * Author: Linar Yusupov, linar.r.yusupov@gmail.com
 *
 * Web: http://github.com/lyusupov/SoftRF
 *
 * Arduino core for ESP8266 is developed/supported by ESP8266 Community (support-esp8266@esp8266.com)
 * AVR/Arduino nRF905 Library/Driver is developed by Zak Kemble, contact@zakkemble.co.uk
 * flarm_decode is developed by Stanislaw Pusep, http://github.com/creaktive
 * Arduino Time Library is developed by Paul Stoffregen, http://github.com/PaulStoffregen
 * "Aircraft" and MAVLink Libraries are developed by Andy Little
 * TinyGPS++ and PString Libraries are developed by Mikal Hart
 * Adafruit NeoPixel Library is developed by Phil Burgess, Michael Miller and others
 * TrueRandom Library is developed by Peter Knight
 * IBM LMIC framework is maintained by Matthijs Kooijman
 * ESP8266FtpServer is developed by David Paiva
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

   NodeMCU 1.0 GPIO pins:
   2 -> CE
   4 -> PWR
   16 -> TXE
   0 -> CD
   5 -> DR
   15 -> CSN
   12 -> SO
   13 -> SI
   14 -> SCK
*/

#include <SoftwareSerial.h>
extern "C" {
#include <user_interface.h>
}

#include "WiFiHelper.h"
#include "OTAHelper.h"
#include "WebHelper.h"
#include "TimeHelper.h"
#include "LEDHelper.h"
#include "GNSSHelper.h"
#include "RFHelper.h"
#include "SoundHelper.h"
#include "EEPROMHelper.h"
#include "BatteryHelper.h"
#include "MAVLinkHelper.h"
#include "GDL90Helper.h"

#include "SoftRF.h"

#if LOGGER_IS_ENABLED
#include "LogHelper.h"
#endif /* LOGGER_IS_ENABLED */

#define DEBUG 0


#define isTimeToDisplay() (millis() - LEDTimeMarker > 1000)
#define isTimeToExport() (millis() - ExportTimeMarker > 1000)
#define isValidFix() (gnss.location.isValid() && (gnss.location.age() <= 3000))

ufo_t ThisAircraft;

unsigned long LEDTimeMarker = 0;
unsigned long ExportTimeMarker = 0;

//ADC_MODE(ADC_VCC);

SoftwareSerial swSer(D3 /* 0 */, /* 5 */ 9 , false, 256);

void setup()
{
  rst_info *resetInfo;

  resetInfo = ESP.getResetInfoPtr();

  Serial.begin(38400);  
  //Misc_info();

#if LOGGER_IS_ENABLED
  Logger_setup();
#endif /* LOGGER_IS_ENABLED */

  Serial.println(""); Serial.print(F("Reset reason: ")); Serial.println(resetInfo->reason);
  Serial.println(ESP.getResetReason());
  Serial.print(F("Free heap size: ")); Serial.println(ESP.getFreeHeap());
  Serial.println(ESP.getResetInfo()); Serial.println("");

  EEPROM_setup();
  Battery_setup();

  ThisAircraft.addr = ESP.getChipId() & 0x00FFFFFF;

  RF_setup();
  delay(100);


  if (settings->mode == SOFTRF_MODE_UAV_BEACON) {
    Serial.begin(57600);
    MAVLink_setup();
    ThisAircraft.aircraft_type = AIRCRAFT_TYPE_UAV;  
  }  else {
    GNSS_setup();
    ThisAircraft.aircraft_type = AIRCRAFT_TYPE_GLIDER;  
  }
  
  LED_setup();

  WiFi_setup();
  OTA_setup();
  Web_setup();

  GDL90_setup();

#if 0
  GNSSserver.begin();
  GNSSserver.setNoDelay(true);
#endif

  LED_test();
  Sound_test(resetInfo->reason);
 
  if (settings->mode == SOFTRF_MODE_TX_TEST || settings->mode == SOFTRF_MODE_RX_TEST) {
    Time_setup();  
  }

}


void loop()
{
  switch (settings->mode)
  {
  case SOFTRF_MODE_TX_TEST:
    tx_test_loop();
    break;
  case SOFTRF_MODE_RX_TEST:
    rx_test_loop();
    break;
  case SOFTRF_MODE_UAV_BEACON:
    uav_loop();
    break;
  case SOFTRF_MODE_BRIDGE:
    bridge_loop();
    break;
  case SOFTRF_MODE_NORMAL:
  default:
    normal_loop();
    break;
  }   

  // Handle OTA update.
  OTA_loop();

  // Handle DNS
  WiFi_loop();

  // Handle Web
  Web_loop();

#if LOGGER_IS_ENABLED
  Logger_loop();
#endif /* LOGGER_IS_ENABLED */

  delay(0);
}

void normal_loop()
{
  unsigned long startTime = millis();
  bool success;

  PickGNSSFix();

  GNSSTimeSync();

  ThisAircraft.timestamp = now();
  if (isValidFix()) {
    ThisAircraft.latitude = gnss.location.lat();
    ThisAircraft.longitude = gnss.location.lng();
    ThisAircraft.altitude = gnss.altitude.meters();
    ThisAircraft.course = gnss.course.deg();

    RF_Transmit();
  }

  if (Import()) success = true;

  success = RF_Receive();

#if DEBUG
  success = true;
#endif

  if (success && isValidFix()) ParseData();

  if (isTimeToDisplay()) {
    if (isValidFix()) {
      LED_DisplayTraffic();  
    } else {
      LED_Clear();      
    }
    LEDTimeMarker = millis();  
  }

  if (isTimeToExport() && isValidFix()) {
    Export();
    ExportTimeMarker = millis(); 
  }

  ClearExpired();

}

#define MAVisValidFix() (the_aircraft.gps.fix_type == 3 /* 3D fix */ )

void uav_loop()
{
  bool success = false;

  PickMAVLinkFix();

  MAVLinkTimeSync();

  ThisAircraft.timestamp = now();

  if (MAVisValidFix()) {
    ThisAircraft.latitude = the_aircraft.location.gps_lat / 1e7;
    ThisAircraft.longitude = the_aircraft.location.gps_lon / 1e7;
    ThisAircraft.altitude = the_aircraft.location.gps_alt / 1000.0;
    ThisAircraft.course = the_aircraft.location.gps_cog;

    RF_Transmit();
  }

  success = RF_Receive();

  if (success && MAVisValidFix()) ParseData();

  if (isTimeToExport() && MAVisValidFix()) {
    MAVLinkShareTraffic();
    ExportTimeMarker = millis(); 
  }

  ClearExpired();
}

//const float tx_test_positions[][2] PROGMEM = { { 56.0092, 38.3710 } };
#if 1 /* 2017 */
const float tx_test_positions[90][2] PROGMEM = {
         { 56.014028944788 , 38.353422195622 } ,
         { 56.01443675978 , 38.354237587162 } ,
         { 56.014484737731 , 38.355439216801 } ,
         { 56.014172879982 , 38.356168777653 } ,
         { 56.013669104455 , 38.356383354374 } ,
         { 56.013117342767 , 38.356168777653 } ,
         { 56.012805473978 , 38.355353386113 } ,
         { 56.012469612467 , 38.354409248539 } ,
         { 56.012133748035 , 38.353851349064 } ,
         { 56.011845861913 , 38.353422195622 } ,
         { 56.011773890047 , 38.354065925786 } ,
         { 56.011809875996 , 38.354387790865 } ,
         { 56.011917833645 , 38.354881317326 } ,
         { 56.011893843082 , 38.355696708866 } ,
         { 56.011917833645 , 38.356383354374 } ,
         { 56.012277690292 , 38.356984169193 } ,
         { 56.012613553472 , 38.357370407291 } ,
         { 56.013141332569 , 38.357542068668 } ,
         { 56.013765062205 , 38.357584984013 } ,
         { 56.01419686913 , 38.357542068668 } ,
         { 56.014532715622 , 38.357971222111 } ,
         { 56.014700637774 , 38.358700782962 } ,
         { 56.014772604187 , 38.359430343815 } ,
         { 56.014388781769 , 38.359902412601 } ,
         { 56.014004955537 , 38.359945327946 } ,
         { 56.013621125491 , 38.359988243289 } ,
         { 56.013141332569 , 38.359945327946 } ,
         { 56.012685523774 , 38.359773666568 } ,
         { 56.012325670926 , 38.35968783588 } ,
         { 56.012013795745 , 38.358743698307 } ,
         { 56.012277690292 , 38.357971222111 } ,
         { 56.013309260769 , 38.357842476078 } ,
         { 56.013861019717 , 38.358014137454 } ,
         { 56.014340803698 , 38.358185798832 } ,
         { 56.014484737731 , 38.358743698307 } ,
         { 56.014412770782 , 38.359344513126 } ,
         { 56.01443675978 , 38.360331566044 } ,
         { 56.014580693455 , 38.360975296207 } ,
         { 56.014820581721 , 38.361747772403 } ,
         { 56.014340803698 , 38.361833603091 } ,
         { 56.013765062205 , 38.361833603091 } ,
         { 56.013261281358 , 38.361790687747 } ,
         { 56.0127335039 , 38.361833603091 } ,
         { 56.012397641763 , 38.361876518436 } ,
         { 56.012061776706 , 38.361833603091 } ,
         { 56.012085767164 , 38.362219841189 } ,
         { 56.012685523774 , 38.362305671878 } ,
         { 56.013693093915 , 38.362348587222 } ,
         { 56.01443675978 , 38.362219841189 } ,
         { 56.01489254791 , 38.363249809451 } ,
         { 56.014868559196 , 38.36445143909 } ,
         { 56.014964513965 , 38.36526683063 } ,
         { 56.01489254791 , 38.366082222171 } ,
         { 56.014484737731 , 38.365481407351 } ,
         { 56.014220858262 , 38.365095169253 } ,
         { 56.013789051605 , 38.364494354434 } ,
         { 56.013381229775 , 38.364193947024 } ,
         { 56.013093352949 , 38.364236862369 } ,
         { 56.012637543587 , 38.364065200991 } ,
         { 56.0122297096 , 38.364279777712 } ,
         { 56.012661533688 , 38.364623100467 } ,
         { 56.01318931213 , 38.364623100467 } ,
         { 56.01364511498 , 38.364880592532 } ,
         { 56.01419686913 , 38.365695984073 } ,
         { 56.01467664894 , 38.366210968203 } ,
         { 56.014670651729 , 38.36485913486 } ,
         { 56.014634668445 , 38.363560945697 } ,
         { 56.014436759779 , 38.362809927172 } ,
         { 56.014736620997 , 38.362058908649 } ,
         { 56.014952519632 , 38.361179144092 } ,
         { 56.014652660091 , 38.360106260486 } ,
         { 56.013513172602 , 38.360127718157 } ,
         { 56.012589563341 , 38.360063345141 } ,
         { 56.012091764775 , 38.359215767091 } ,
         { 56.01207976955 , 38.358250171846 } ,
         { 56.012565573195 , 38.357863933748 } ,
         { 56.012061776705 , 38.35687688083 } ,
         { 56.012013795744 , 38.356061489291 } ,
         { 56.011797880682 , 38.353679687685 } ,
         { 56.012241704777 , 38.354366333193 } ,
         { 56.01248160757 , 38.354752571291 } ,
         { 56.012649538638 , 38.355310470766 } ,
         { 56.012829463973 , 38.355739624209 } ,
         { 56.013333250451 , 38.356061489291 } ,
         { 56.013777056906 , 38.356190235323 } ,
         { 56.014196869128 , 38.355846912569 } ,
         { 56.01435279822 , 38.355203182405 } ,
         { 56.014388781768 , 38.354495079226 } ,
         { 56.014184874557 , 38.354001552767 } ,
         { 56.014016950163 , 38.353615314669 }
      };
#endif

#define NUM_POSITIONS (sizeof(tx_test_positions) / sizeof(float) / 2) 
#define ALTITUDE    438
unsigned int pos_ndx = 0;
unsigned long TxPosUpdMarker = 0;

void tx_test_loop()
{
  bool success = false;

  ThisAircraft.timestamp = now();

  if (TxPosUpdMarker == 0 || (millis() - TxPosUpdMarker) > 4000 ) {
    ThisAircraft.latitude =  pgm_read_float( &tx_test_positions[pos_ndx][0]);
    ThisAircraft.longitude =  pgm_read_float( &tx_test_positions[pos_ndx][1]);
    pos_ndx = (pos_ndx + 1) % NUM_POSITIONS;
    TxPosUpdMarker = millis();
  }
  ThisAircraft.altitude = ALTITUDE;
  ThisAircraft.course = 0;

  RF_Transmit();

  success = RF_Receive();

  if(success)
  {
    fo.raw = Bin2Hex(RxBuffer);

    if (settings->nmea_p) {
      StdOut.print(F("$PSRFI,")); StdOut.print(now()); StdOut.print(F(",")); StdOut.println(fo.raw);
    }
  }

  if (isTimeToExport()) {
    GDL90_Export();
    ExportTimeMarker = millis(); 
  }
}

const float rx_test_positions[][2] PROGMEM = { { 56.0092, 38.3710 } };

void rx_test_loop()
{
  bool success = false;

  ThisAircraft.timestamp = now();

  ThisAircraft.latitude = pgm_read_float( &rx_test_positions[0][0]);;
  ThisAircraft.longitude = pgm_read_float( &rx_test_positions[0][1]);
  ThisAircraft.altitude = ALTITUDE;
  ThisAircraft.course = 0;

  //RF_Transmit();

  success = RF_Receive();

  if (success) ParseData();

  if (isTimeToDisplay()) {
    LED_DisplayTraffic();  
    LEDTimeMarker = millis();  
  }

  if (isTimeToExport()) {
    Export();
    ExportTimeMarker = millis(); 
  }

  ClearExpired();
}


void bridge_loop()
{
  unsigned long startTime = millis();
  bool success;

  void *answer = WiFi_relay_from_android();
  if ((answer != NULL) && (settings->txpower != NRF905_TX_PWR_OFF) )
  {
    memcpy(TxBuffer, (unsigned char*) answer, PKT_SIZE);

    // Make data
    char *data = (char *) TxBuffer;

    // Set address of device to send to
    byte addr[] = TXADDR;
    nRF905_setTXAddress(addr);

    // Set payload data
    nRF905_setData(data, NRF905_PAYLOAD_SIZE );

    // Send payload (send fails if other transmissions are going on, keep trying until success)
    while (!nRF905_send()) {
      delay(0);
    } ;
    if (settings->nmea_p) {
      StdOut.print(F("$PSRFO,")); StdOut.print(now()); StdOut.print(F(",")); StdOut.println(Bin2Hex((byte *) data));
    }

    tx_packets_counter++;
    TxTimeMarker = millis();
  }

  success = RF_Receive();

  if(success)
  {

    fo.raw = Bin2Hex(RxBuffer);

    if (settings->nmea_p) {
      StdOut.print(F("$PSRFI,")); StdOut.print(now()); StdOut.print(F(",")); StdOut.println(fo.raw);
    }

    WiFi_relay_to_android();
  }

  if (isTimeToDisplay()) {
    LED_Clear();  
    LEDTimeMarker = millis();  
  }
}
