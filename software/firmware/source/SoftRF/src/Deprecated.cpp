/*
 * Deprecated.cpp
 * Copyright (C) 2016-2021 Linar Yusupov
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

#if 0
//how many clients should be able to telnet to this ESP8266
#define MAX_SRV_CLIENTS 1
WiFiServer GNSSserver(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];
#endif


#if 0
void WiFi_forward_to_argus()
{
    char str_lat[16];
    char str_lon[16];

    dtostrf(fo.latitude, 8, 4, str_lat);
    dtostrf(fo.longitude, 8, 4, str_lon);

    Udp.beginPacket(ARGUS_HOSTNAME, ARGUS_PORT);

    snprintf(UDPpacketBuffer, sizeof(UDPpacketBuffer), "%u %X %s %s %d %s %s %u", \
      fo.timestamp * 1000, fo.addr, str_lat, str_lon, fo.altitude, "0" , "0" , TypeADSB );

#ifdef SERIAL_VERBOSE
    Serial.println(UDPpacketBuffer);
#endif

    Udp.write(UDPpacketBuffer, strlen(UDPpacketBuffer));
    Udp.endPacket();
}
#endif

#if 0
// FLRDDE626>APRS,qAS,EGHL:/074548h5111.32N/00102.04W'086/007/A=000607 id0ADDE626 -019fpm +0.0rot 5.5dB 3e -4.3kHz
void WiFi_forward_to_cloud() {
  tmElements_t tm;
  char str_lat[8];
  char str_lon[8];

  breakTime(fo.timestamp, tm);

  dtostrf_workaround(take_minutes(fo.latitude), 5, 2, str_lat);
  dtostrf_workaround(take_minutes(fo.longitude), 5, 2, str_lon);

  //Serial.print(fo.latitude); Serial.print(" "); Serial.println(fo.longitude); 

  snprintf(UDPpacketBuffer, sizeof(UDPpacketBuffer), \
    "FLR%X>APRS,qAS,%s:/%02d%02d%02dh%02d%s%s/%03d%s%s/A=%05u TS:%d RAW:%s", \
    fo.addr, STATION_ID, tm.Hour, tm.Minute, tm.Second, \
    abs(take_degrees(fo.latitude)), str_lat, (fo.latitude < 0 ? "S" : "N"), \
    abs(take_degrees(fo.longitude)), str_lon, (fo.longitude < 0 ? "W" : "E"), \
    fo.altitude, fo.timestamp, fo.raw.c_str() );

#ifdef SERIAL_VERBOSE
  Serial.println(UDPpacketBuffer);
#endif

  client.println(UDPpacketBuffer);
}
#endif

#if 0
bool Import()
{
  void *answer = WiFi_relay_from_android();
  if (answer != NULL)
  {
    memcpy(RxBuffer, (unsigned char*) answer, PKT_SIZE);
    return true;
  } else {
    return false;
  }
}
#endif

#if 0
void *WiFi_relay_from_android()
{
  int noBytes = Uni_Udp.parsePacket();
  if ( noBytes ) {
#if 0
    Serial.print(millis() / 1000);
    Serial.print(":Packet of ");
    Serial.print(noBytes);
    Serial.print(" received from ");
    Serial.print(Uni_Udp.remoteIP());
    Serial.print(":");
    Serial.println(Udp.remotePort());
#endif
    // We've received a packet, read the data from it
    Uni_Udp.read(UDPpacketBuffer,noBytes); // read the packet into the buffer
#if 0
    // display the packet contents in HEX
    for (int i=1;i<=noBytes;i++){
      Serial.print(UDPpacketBuffer[i-1],HEX);
      if (i % 32 == 0){
        Serial.println();
      }
      else Serial.print(' ');
    } // end for
    Serial.println();
#endif
    return UDPpacketBuffer;
  } else {
    return NULL;
  }  // end if
}
#endif


/* bridge_loop */
#if 0
  void *answer = WiFi_relay_from_android();
  if ((answer != NULL) && (settings->txpower != RF_TX_POWER_OFF) )
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
      yield();
    } ;
    if (settings->nmea_p) {
      StdOut.print(F("$PSRFO,")); StdOut.print(now()); StdOut.print(F(",")); StdOut.println(Bin2Hex((byte *) data));
    }

    tx_packets_counter++;
    TxTimeMarker = millis();
  }
#endif


#if 0
unsigned int pos_ndx = 0;
unsigned long TxPosUpdMarker = 0;

void tx_test_loop()
{
  bool success = false;
#if DEBUG_TIMING
  unsigned long tx_start_ms, tx_end_ms, rx_start_ms, rx_end_ms;
#endif
  ThisAircraft.timestamp = now();

  if (TxPosUpdMarker == 0 || (millis() - TxPosUpdMarker) > 4000 ) {
    ThisAircraft.latitude =  pgm_read_float( &tx_test_positions[pos_ndx][0]);
    ThisAircraft.longitude =  pgm_read_float( &tx_test_positions[pos_ndx][1]);
    pos_ndx = (pos_ndx + 1) % TX_TEST_NUM_POSITIONS;
    TxPosUpdMarker = millis();
  }
  ThisAircraft.altitude = TEST_ALTITUDE;
  ThisAircraft.course = TEST_COURSE;
  ThisAircraft.speed = TEST_SPEED;
#if DEBUG_TIMING
  tx_start_ms = millis();
#endif
  RF_Transmit(RF_Encode());
#if DEBUG_TIMING
  tx_end_ms = millis();
  rx_start_ms = millis();
#endif
  success = RF_Receive();
#if DEBUG_TIMING
  rx_end_ms = millis();

  Serial.print(F("TX start: "));
  Serial.print(tx_start_ms);
  Serial.print(F(" TX stop: "));
  Serial.print(tx_end_ms);
  Serial.print(F(" RX start: "));
  Serial.print(rx_start_ms);
  Serial.print(F(" RX stop: "));
  Serial.println(rx_end_ms);
#endif

  if(success)
  {
    fo.raw = Bin2Hex(RxBuffer);

    if (settings->nmea_p) {
      StdOut.print(F("$PSRFI,")); StdOut.print(now()); StdOut.print(F(",")); StdOut.println(fo.raw);
    }
  }

  if (isTimeToExport()) {
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
  ThisAircraft.altitude = TEST_ALTITUDE;
  ThisAircraft.course = TEST_COURSE;
  ThisAircraft.speed = TEST_SPEED;

  //RF_Transmit(RF_Encode());

  success = RF_Receive();

  if (success) ParseData();

  if (isTimeToDisplay()) {
    LED_DisplayTraffic();
    LEDTimeMarker = millis();
  }

  if (isTimeToExport()) {
    NMEA_Position();
    NMEA_Export();
    GDL90_Export();
    D1090_Export();
    ExportTimeMarker = millis();
  }

  ClearExpired();
}
#endif

#if 0
#define take_degrees(x) ( (int) x )
#define take_minutes(x) ( fabs(x - (float) take_degrees(x)) * 60.00)

char * dtostrf_workaround(double number, signed char width, unsigned char prec, char *s) {
  char * rval = dtostrf(number, width, prec, s);
  if (number < 10.0) {
    s[0] = '0';
  }
  return rval;
}
#endif

#if 0

void Misc_info()
{
  Serial.println("\r\n");
  Serial.print(F("Chip ID: 0x"));
  Serial.println(SoC->getChipId(), HEX);

  uint32_t realSize = SoC->getFlashChipRealSize();
  uint32_t ideSize = ESP.getFlashChipSize();
  FlashMode_t ideMode = ESP.getFlashChipMode();

  Serial.printf("Flash real id:   %08X\n", SoC->getFlashChipId());
  Serial.printf("Flash real size: %u\n\n", realSize);

  Serial.printf("Flash ide  size: %u\n", ideSize);
  Serial.printf("Flash ide speed: %u\n", ESP.getFlashChipSpeed());
  Serial.printf("Flash ide mode:  %s\n", (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"));

  if (ideSize != realSize) {
    Serial.println(F("Flash Chip configuration wrong!\n"));
  } else {
    Serial.println(F("Flash Chip configuration ok.\n"));
  }
}

#endif

#if 0
#include <ets_sys.h>
#include <osapi.h>
#include <gpio.h>
#include <os_type.h>
extern "C" {
#include <user_interface.h>
}

void ICACHE_FLASH_ATTR user_init()
{
  // init gpio subsytem
  gpio_init();
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA3_U, FUNC_GPIO10);
}
#endif

#if 0
void Sound_test(int var)
{
  if (settings->volume != BUZZER_OFF) {
    swSer.enableRx(false);

    if (var == REASON_DEFAULT_RST ||
        var == REASON_EXT_SYS_RST ||
        var == REASON_SOFT_RESTART) {
      tone(10, 440, 500);delay(500);
      tone(10, 640, 500);delay(500);
      tone(10, 840, 500);delay(500);
      tone(10, 1040, 500);
    } else if (var == REASON_WDT_RST) {
      tone(10, 440, 500);delay(500);
      tone(10, 1040, 500);delay(500);
      tone(10, 440, 500);delay(500);
      tone(10, 1040, 500);
    } else {
      tone(10, 1040, 500);delay(500);
      tone(10, 840, 500);delay(500);
      tone(10, 640, 500);delay(500);
      tone(10, 440, 500);
    }
    delay(600);

    swSer.enableRx(true);
  }
}
#endif

/* Platform_ESP32.h */

//#define USE_S7XG_DRIVER

/* RFHelper.cpp */

#if defined(USE_S7XG_DRIVER)

#include <s7xg.h>

S7XG_Class s7xg;

const rfchip_ops_t s7xg_ops = {
  RF_IC_S7XG,
  "S7XG",
  s7xg_probe,
  s7xg_setup,
  s7xg_channel,
  s7xg_receive,
  s7xg_transmit,
  s7xg_shutdown
};
#endif /* USE_S7XG_DRIVER */

#if defined(USE_S7XG_DRIVER)
    else
    {
      if (s7xg_ops.probe()) {
        rf_chip = &s7xg_ops;
        Serial.println(F("S7XG SoC is detected."));
      }
    }
#endif /* USE_S7XG_DRIVER */

#if defined(USE_S7XG_DRIVER)
/*
 * S7XG-specific code
 *
 *
 */

static uint8_t s7xg_channel_prev = (uint8_t) -1;
bool s7xg_receive_active         = false;

#define Serial_S7XG         swSer
#define S7XG_UART_BAUD_RATE 115200

extern byte getVal(char);

bool s7xg_probe()
{
  bool success = false;

  /* Do not probe on CC13XX, ESP8266, RPi and STM32 */
  if (SoC->id == SOC_CC13XX ||
      SoC->id == SOC_RPi    ||
      SoC->id == SOC_STM32  ||
      SoC->id == SOC_ESP8266) {
    return success;
  }

  SoC->swSer_begin(S7XG_UART_BAUD_RATE);

  s7xg.begin(Serial_S7XG);

  s7xg.loraReceiveContinuous(false);
  s7xg.reset();

  delay(1000);

  /* get rid of residual input garbage */
  while (Serial_S7XG.available() > 0) {
    Serial_S7XG.read();
  }

  String model = s7xg.getHardWareModel();

  if ( model == "S76G" || model == "S78G") {
      success = true;
  }

  /* Current ESP32 Core has a bug with Serial1.end()+Serial1.begin() cycle */
  if (SoC->id != SOC_ESP32) {
    Serial_S7XG.end();
  }

  return success;
}

void s7xg_channel(uint8_t channel)
{
  if (channel != s7xg_channel_prev) {
    uint32_t frequency = RF_FreqPlan.getChanFrequency(channel);

    if (s7xg_receive_active) {
      s7xg.loraReceiveContinuous(false);

      /* restart Rx upon a channel switch */
      s7xg_receive_active = false;
    }

    s7xg.loraSetFrequency(frequency);

    s7xg_channel_prev = channel;
  }
}

void s7xg_setup()
{
  /* Current ESP32 Core has a bug with Serial1.end()+Serial1.begin() cycle */
  if (SoC->id != SOC_ESP32) {
    SoC->swSer_begin(S7XG_UART_BAUD_RATE);
  }

  /* Enforce radio settings to follow FANET protocol's RF specs */
  settings->rf_protocol = RF_PROTOCOL_FANET;

  LMIC.protocol   = &fanet_proto_desc;

  protocol_encode = &fanet_encode;
  protocol_decode = &fanet_decode;

  switch(settings->txpower)
  {
  case RF_TX_POWER_FULL:

    /* Load regional max. EIRP at first */
    LMIC.txpow = RF_FreqPlan.MaxTxPower;

    /* SX1276 is unable to give more than 20 dBm */
    if (LMIC.txpow > 20)
      LMIC.txpow = 20;
#if 1
    /*
     * Enforce Tx power limit until confirmation
     * that RFM95W is doing well
     * when antenna is not connected
     */
    if (LMIC.txpow > 17)
      LMIC.txpow = 17;
#endif
    break;
  case RF_TX_POWER_OFF:
  case RF_TX_POWER_LOW:
  default:
    LMIC.txpow = 2; /* 2 dBm is minimum for RFM95W on PA_BOOST pin */
    break;
  }

  s7xg.loraSetPower(LMIC.txpow);

  s7xg.loraSetSpreadingFactor(7);   /* SF_7 */
  s7xg.loraSetBandWidth      (250); /* BW_250 */

  /* for only a few nodes around, increase the coding rate to ensure a more robust transmission */
  s7xg.loraSetCodingRate     (8);   /* CR_4_8 */

  s7xg.loraSetPreambleLength(12); /* default value */
  s7xg.loraSetSyncWord(LMIC.protocol->syncword[0]);
  s7xg.loraSetCRC(true);
  s7xg.loraSetIQInvert(false);
}

char hex_rx_buf[2 * MAX_PKT_SIZE + 1];

bool s7xg_receive()
{
  bool success = false;
  const char *s;
  int8_t rssi, snr;


  if (!s7xg_receive_active) {
    s7xg.loraReceiveContinuous(true);
    s7xg_receive_active = true;
  }

  if (Serial_S7XG.available()) {
    s =  Serial_S7XG.readStringUntil('\r').c_str();

    if (strlen(s) < 3) {
      s =  Serial_S7XG.readStringUntil('\r').c_str();
    }

    sscanf(s, ">> radio_rx %s %d %d\n", hex_rx_buf, &rssi, &snr);  /* overflow - TBD */
    if (strlen(hex_rx_buf) > 0) {
      for(int j = 0; j < LMIC.protocol->payload_size * 2 ; j+=2)
      {
        RxBuffer[j>>1] = getVal(hex_rx_buf[j+1]) + (getVal(hex_rx_buf[j]) << 4);
      }

      success = true;
    }
  }

  if (success) {
    RF_last_rssi = rssi;
    rx_packets_counter++;
  }

  if (SoC->Bluetooth) {
    SoC->Bluetooth->loop();
  }

  return success;
}

void s7xg_transmit()
{
  char hex_tx_buf[2 * MAX_PKT_SIZE + 1];
  uint8_t tmp;
  int i;

  if (RF_tx_size > 0) {

    if (s7xg_receive_active) {
      s7xg.loraReceiveContinuous(false);
      s7xg_receive_active = false;
    }

    for (i=0; i < RF_tx_size; i++) {
      tmp = (TxBuffer[i] >> 4) & 0x0F;
      hex_tx_buf[i+i]   = tmp > 9 ? 'a' + (tmp - 10) : '0' + tmp;
      tmp =  TxBuffer[i]       & 0x0F;
      hex_tx_buf[i+i+1] = tmp > 9 ? 'a' + (tmp - 10) : '0' + tmp;
    }
    hex_tx_buf[i+i] = 0;

    s7xg.loraTransmit(hex_tx_buf);
  }
}

void s7xg_shutdown()
{
  s7xg.loraReceiveContinuous(false);
  s7xg.gpsSetMode(GPS_MODE_IDLE);
}
#endif /* USE_S7XG_DRIVER */

/* GNSSHelper.cpp */

#if defined(USE_S7XG_DRIVER)

#include <s7xg.h>

extern S7XG_Class s7xg;
extern bool s7xg_receive_active;
extern void NMEA_RMCGGA(char *, GPS_Class);

unsigned long S7XG_Time_Marker = 0;
#endif /* USE_S7XG_DRIVER */

#if defined(USE_S7XG_DRIVER)
  if (hw_info.model == SOFTRF_MODEL_SKYWATCH &&
      hw_info.rf    == RF_IC_S7XG ) {

    s7xg.gpsStop();
    s7xg.gpsReset();
    s7xg.gpsSetLevelShift(true);
    s7xg.gpsSetSystem(GPS_STATE_SYS_GPS_GLONASS);
    s7xg.gpsSetPositioningCycle(1000);
    s7xg.gpsSetMode(GPS_MODE_MANUAL);

    S7XG_Time_Marker = millis();

    return GNSS_MODULE_SONY;
  }
#endif /* USE_S7XG_DRIVER */

#if defined(USE_S7XG_DRIVER)
  if (hw_info.model == SOFTRF_MODEL_SKYWATCH &&
      hw_info.rf    == RF_IC_S7XG ) {

    if ((millis() - S7XG_Time_Marker) > 1000 ) {
      if (s7xg_receive_active) {
        s7xg.loraReceiveContinuous(false);
      }

      GPS_Class gnss_data = s7xg.gpsGetData(GPS_DATA_TYPE_DD);

      if (s7xg_receive_active) {
        s7xg.loraReceiveContinuous(true);
      }

      if (gnss_data.isVaild()) {
        NMEA_RMCGGA((char *) GNSSbuf, gnss_data);
        for (int i=0; i < strlen((char *) GNSSbuf); i++) {
          gnss.encode(GNSSbuf[i]);
        }
        NMEA_Out(GNSSbuf, strlen((char *) GNSSbuf), false);
      }

      S7XG_Time_Marker = millis();
    }

    return;
  }
#endif /* USE_S7XG_DRIVER */

/* NMEAHelper.cpp */

#if defined(USE_S7XG_DRIVER)

#include <s7xg.h>

void NMEA_RMCGGA(char *dest, GPS_Class data)
{
  NmeaInfo info;

  float latitude  = data.lat();
  float longitude = data.lng();

  nmeaInfoClear(&info);

  info.utc.year   = data.year();
  info.utc.mon    = data.month();
  info.utc.day    = data.day();
  info.utc.hour   = data.hour();
  info.utc.min    = data.minute();
  info.utc.sec    = data.second();
  info.utc.hsec   = 0;

  info.latitude   = ((int) latitude) * 100.0;
  info.latitude  += (latitude - (int) latitude) * 60.0;
  info.longitude  = ((int) longitude) * 100.0;
  info.longitude += (longitude - (int) longitude) * 60.0;

  info.elevation  = 0;
  info.height     = LookupSeparation(latitude, longitude);
  info.sig        = (NmeaSignal) 1;

  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_UTCDATE);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_UTCTIME);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_LAT);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_LON);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_ELV);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SIG);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_HEIGHT);

  size_t gen_sz = nmeaSentenceFromInfo(&nmealib_buf, &info, (NmeaSentence)
                                        (NMEALIB_SENTENCE_GPRMC |
                                         NMEALIB_SENTENCE_GPGGA));

  if (gen_sz) {
    memcpy(dest, nmealib_buf.buffer, gen_sz);
    dest[gen_sz] = 0;
  }
}
#endif /* USE_S7XG_DRIVER */

#if 0
static void play_memory(const unsigned char *data, int size)
{
  headerState_t state = HEADER_RIFF;
  wavRiff_t *wavRiff;
  wavProperties_t *props;

  while (size > 0) {
    switch(state){
    case HEADER_RIFF:
      wavRiff = (wavRiff_t *) data;

      if(wavRiff->chunkID == CCCC('R', 'I', 'F', 'F') && wavRiff->format == CCCC('W', 'A', 'V', 'E')){
        state = HEADER_FMT;
      }
      data += sizeof(wavRiff_t);
      size -= sizeof(wavRiff_t);
      break;

    case HEADER_FMT:
      props = (wavProperties_t *) data;
      state = HEADER_DATA;
      data += sizeof(wavProperties_t);
      size -= sizeof(wavProperties_t);
      break;

    case HEADER_DATA:
      uint32_t chunkId, chunkSize;
      chunkId = *((uint32_t *) data);
      data += sizeof(uint32_t);
      size -= sizeof(uint32_t);
      chunkSize = *((uint32_t *) data);
      state = DATA;
      data += sizeof(uint32_t);
      size -= sizeof(uint32_t);

      //initialize i2s with configurations above
      i2s_driver_install((i2s_port_t)i2s_num, &i2s_config, 0, NULL);
      i2s_set_pin((i2s_port_t)i2s_num, &pin_config);
      //set sample rates of i2s to sample rate of wav file
      i2s_set_sample_rates((i2s_port_t)i2s_num, props->sampleRate);
      break;

      /* after processing wav file, it is time to process music data */
    case DATA:
      i2s_write_sample_nb(*((uint32_t *) data));
      data += sizeof(uint32_t);
      size -= sizeof(uint32_t);
      break;
    }
  }

  if (state == DATA) {
    i2s_driver_uninstall((i2s_port_t)i2s_num); //stop & destroy i2s driver
  }
}
#endif
