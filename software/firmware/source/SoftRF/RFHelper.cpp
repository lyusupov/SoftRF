/*
 * RFHelper.cpp
 * Copyright (C) 2016-2019 Linar Yusupov
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
#if defined(ARDUINO)
#include <SPI.h>
#endif /* ARDUINO */

#include "RFHelper.h"
#include "Protocol_Legacy.h"
#include "Protocol_OGNTP.h"
#include "Protocol_P3I.h"
#include "Protocol_FANET.h"
#include "Protocol_UAT978.h"
#include "SoCHelper.h"
#include "EEPROMHelper.h"
#include "WebHelper.h"
#include "MAVLinkHelper.h"
#include <fec.h>

#if LOGGER_IS_ENABLED
#include "LogHelper.h"
#endif /* LOGGER_IS_ENABLED */

byte RxBuffer[MAX_PKT_SIZE];

unsigned long TxTimeMarker = 0;
byte TxBuffer[MAX_PKT_SIZE];

uint32_t tx_packets_counter = 0;
uint32_t rx_packets_counter = 0;

int8_t RF_last_rssi = 0;

static FreqPlan RF_FreqPlan;
static bool RF_ready = false;

static size_t RF_tx_size = 0;
static long TxRandomValue = 0;

const rfchip_ops_t *rf_chip = NULL;
bool RF_SX1276_RST_is_connected = true;

size_t (*protocol_encode)(void *, ufo_t *);
bool (*protocol_decode)(void *, ufo_t *, ufo_t *);

const rfchip_ops_t nrf905_ops = {
  RF_IC_NRF905,
  "NRF905",
  nrf905_probe,
  nrf905_setup,
  nrf905_channel,
  nrf905_receive,
  nrf905_transmit,
  nrf905_shutdown
};

const rfchip_ops_t sx1276_ops = {
  RF_IC_SX1276,
  "SX1276",
  sx1276_probe,
  sx1276_setup,
  sx1276_channel,
  sx1276_receive,
  sx1276_transmit,
  sx1276_shutdown
};

const rfchip_ops_t cc13xx_ops = {
  RF_IC_CC13XX,
  "CC13XX",
  cc13xx_probe,
  cc13xx_setup,
  cc13xx_channel,
  cc13xx_receive,
  cc13xx_transmit,
  cc13xx_shutdown
};

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

#if defined(USE_OGN_RF_DRIVER)

#define vTaskDelay  delay

#if defined(WITH_SI4X32)
#include <rf/si4x32/rfm.h>
#else
#include <rf/combo/rfm.h>
#endif /* WITH_SI4X32 */

const rfchip_ops_t ognrf_ops = {
  RF_DRV_OGN,
  "OGNDRV",
  ognrf_probe,
  ognrf_setup,
  ognrf_channel,
  ognrf_receive,
  ognrf_transmit,
  ognrf_shutdown
};
#endif /* USE_OGN_RF_DRIVER */

String Bin2Hex(byte *buffer, size_t size)
{
  String str = "";
  for (int i=0; i < size; i++) {
    byte c = buffer[i];
    str += (c < 0x10 ? "0" : "") + String(c, HEX);
  }
  return str;
}

uint8_t parity(uint32_t x) {
    uint8_t parity=0;
    while (x > 0) {
      if (x & 0x1) {
          parity++;
      }
      x >>= 1;
    }
    return (parity % 2);
}
 
byte RF_setup(void)
{

  if (rf_chip == NULL) {
#if !defined(USE_OGN_RF_DRIVER)
    if (hw_info.model != SOFTRF_MODEL_SKYWATCH) {
      if (sx1276_ops.probe()) {
        rf_chip = &sx1276_ops;
        Serial.println(F("SX1276 RFIC is detected."));
      } else if (nrf905_ops.probe()) {
        rf_chip = &nrf905_ops;
        Serial.println(F("NRF905 RFIC is detected."));
      } else if (cc13xx_ops.probe()) {
        rf_chip = &cc13xx_ops;
        Serial.println(F("CC13XX RFIC is detected."));
      } else {
        Serial.println(F("WARNING! Neither SX1276"
#if !defined(ENERGIA_ARCH_CC13XX)
        ", CC13XX"
#endif /* ENERGIA_ARCH_CC13XX */
          " or NRF905 RFIC is detected!"));
      }
    }
#if defined(USE_S7XG_DRIVER)
    else
    {
      if (s7xg_ops.probe()) {
        rf_chip = &s7xg_ops;
        Serial.println(F("S7XG SoC is detected."));
      }
    }
#endif /* USE_S7XG_DRIVER */
#else /* USE_OGN_RF_DRIVER */
    if (ognrf_ops.probe()) {
      rf_chip = &ognrf_ops;
      Serial.println(F("OGN_DRV: RFIC is detected."));
    } else {
      Serial.println(F("WARNING! RFIC is NOT detected."));
    }
#endif /* USE_OGN_RF_DRIVER */
  }

  /* "AUTO" freq. will set the plan upon very first valid GNSS fix */
  if (settings->band == RF_BAND_AUTO) {
    /* Supersede EU plan with UK when PAW is selected */
    if (rf_chip && rf_chip != &nrf905_ops && settings->rf_protocol == RF_PROTOCOL_P3I) {
      RF_FreqPlan.setPlan(RF_BAND_UK);
    }
  } else {
    RF_FreqPlan.setPlan(settings->band);
  }

  if (rf_chip) {
    rf_chip->setup();
    return rf_chip->type;
  } else {
    return RF_IC_NONE;
  }
}

#define DELAY_PPS_GPSTIME 200  // approx msec between PPS and time in NMEA sentence
#define SLOT1_START       400  // slot1 start msec after PPS
#define SLOT1_ADVANCE     100  // advance slot1 to mid of dead time between slots
#define SLOT2_START       800  // slot2 start msec after PPS
#define SLOT_DURATION     400  // slot msec duration

static long TimeReference =   0;  // Hop reference timing
static long TimeReference_2 = 0; 
static long Now_millis =      0;
static long prev_TimeCommit = 0;
uint8_t Slot = 0;
time_t slotTime = 0;

void RF_SetChannel(void)
{
  tmElements_t tm;
  time_t Time;

  switch (settings->mode)
  {
  case SOFTRF_MODE_TXRX_TEST:
    Time = now();
    break;
  case SOFTRF_MODE_UAV:
    Time = the_aircraft.location.gps_time_stamp / 1000000;
    break;
  case SOFTRF_MODE_NORMAL:
  default:
    unsigned long pps_btime_ms = SoC->get_PPS_TimeMarker();
//    unsigned long time_corr_pos = 0;
    unsigned long time_corr_neg = 0;
    unsigned long timeAge = 0;
    unsigned long lastCommitTime = (Now_millis = millis()) - (timeAge = gnss.time.age());
	
    // HOP Testing - NMEA sentence time commit
    //Serial.printf("Commit: %d, %d, %d\r\n", lastCommitTime, prev_TimeCommit, pps_btime_ms);
	
	// Time could be in GGA or RMC. For consistency must pick only first one
	// problem is that the second commit is 450 msec after the first or only 550 before next !
	// not needed if PPS is available
	if (lastCommitTime - prev_TimeCommit < 500) {
	  lastCommitTime = prev_TimeCommit;
      timeAge = Now_millis - lastCommitTime;
	} else {
	  prev_TimeCommit = lastCommitTime;
	}
    
    // if PPS available, reference time is PPS relative for accuracy
    if (pps_btime_ms) {
      // calculate delta time from millis() to PPS reference
      if (pps_btime_ms <= lastCommitTime) {
        time_corr_neg = (lastCommitTime - pps_btime_ms) % 1000;
      } else {
        time_corr_neg = 1000 - ((pps_btime_ms - lastCommitTime) % 1000);
      }
    } else {  // no PPS, approximate reference delay
      time_corr_neg = DELAY_PPS_GPSTIME;
    }

    // only frequency hop with legacy and OGN protocols
    switch (settings->rf_protocol) {
      case RF_PROTOCOL_LEGACY: 
      case RF_PROTOCOL_OGNTP: 
        if ((Now_millis - TimeReference) >= 1000) {   
	      if (pps_btime_ms) {
	        TimeReference = pps_btime_ms +SLOT1_START -SLOT1_ADVANCE -0; // allow for latency ?
		  } else {
            TimeReference = lastCommitTime -time_corr_neg +SLOT1_START -SLOT1_ADVANCE;
		  }
          Slot = 0;
          if ((Now_millis - TimeReference) >= 1000) { // has PPS stopped ?
		    TxTimeMarker = Now_millis;                // if so no Tx
			return;
		  } else {
            TxTimeMarker = TimeReference;
		  }
          TxRandomValue = SoC->random(0, SLOT_DURATION -10) +SLOT1_ADVANCE;  // allow some margin
        } else {
          if ((Now_millis - TimeReference_2) >= 1000) {
	        TimeReference_2 = TimeReference +SLOT_DURATION +SLOT1_ADVANCE;
            Slot = 1;
            TxTimeMarker = TimeReference_2;
            TxRandomValue = SoC->random(10, SLOT_DURATION -0);  //  allow some margin
          } else {
 	        return;	  
          }
	    }
        break;
      default:
        /* FANET uses 868.2 MHz. Bandwidth is 250kHz  */
        Slot = 0;
        break;
    }

    // HOP Testing - slot timing 400 and 800 msec after PPS
    //Serial.printf("Timing: %d, %d, %d, %d, %d, %d, %d\r\n", Now_millis, pps_btime_ms, timeAge, time_corr_neg, TimeReference, TxRandomValue, Slot);

    // latest time from GPS
    int yr = gnss.date.year();
    if( yr > 99)
        yr = yr - 1970;
    else
        yr += 30;
    tm.Year = yr;
    tm.Month = gnss.date.month();
    tm.Day = gnss.date.day();
    tm.Hour = gnss.time.hour();
    tm.Minute = gnss.time.minute();
    tm.Second = gnss.time.second();

    // time right now is:
    slotTime = Time = makeTime(tm) + (timeAge + time_corr_neg)/ 1000;
    break;
  }

  uint8_t OGN = (settings->rf_protocol == RF_PROTOCOL_OGNTP ? 1 : 0);

  uint8_t chan = RF_FreqPlan.getChannel(Time, Slot, OGN);

  // HOP Testing - time and channel
  //Serial.printf("Time: %d, %d\r\n", Time,chan);
  
#if DEBUG
  Serial.print("Plan: "); Serial.println(RF_FreqPlan.Plan);
  Serial.print("Slot: "); Serial.println(Slot);
  Serial.print("OGN: "); Serial.println(OGN);
  Serial.print("Channel: "); Serial.println(chan);
#endif

  if (RF_ready && rf_chip) {
    rf_chip->channel(chan);
  }
}

void RF_loop()
{
  if (!RF_ready) {
    if (RF_FreqPlan.Plan == RF_BAND_AUTO) {
      if (ThisAircraft.latitude || ThisAircraft.longitude) {
        RF_FreqPlan.setPlan((int32_t)(ThisAircraft.latitude  * 600000),
                            (int32_t)(ThisAircraft.longitude * 600000));
        RF_ready = true;
      }
    } else {
      RF_ready = true;
    }
  }

  if (RF_ready) {
    RF_SetChannel();
  }
}

size_t RF_Encode(ufo_t *fop)
{
  size_t size = 0;
  if (RF_ready && protocol_encode) {

    if (settings->txpower == RF_TX_POWER_OFF ) {
      return size;
    }

    if ((millis() - TxTimeMarker) > TxRandomValue) {
      switch (settings->rf_protocol) {
        case RF_PROTOCOL_LEGACY: 
        case RF_PROTOCOL_OGNTP: 
          fop->timestamp = slotTime;
		  break;  
	    }
      size = (*protocol_encode)((void *) &TxBuffer[0], fop);
    }
  }
  return size;
}

bool RF_Transmit(size_t size, bool wait)
{
  if (RF_ready && rf_chip && (size > 0)) {
    RF_tx_size = size;

    if (settings->txpower == RF_TX_POWER_OFF ) {
      return true;
    }

    if (!wait || (millis() - TxTimeMarker) > TxRandomValue) {

      time_t timestamp = now();

      rf_chip->transmit();

      if (settings->nmea_p) {
        StdOut.print(F("$PSRFO,"));
        StdOut.print((unsigned long) timestamp);
        StdOut.print(F(","));
        StdOut.println(Bin2Hex((byte *) &TxBuffer[0],
                               RF_Payload_Size(settings->rf_protocol)));
      }
      tx_packets_counter++;
      RF_tx_size = 0;

      switch (settings->rf_protocol) {
        case RF_PROTOCOL_LEGACY: 
        case RF_PROTOCOL_OGNTP: 
          TxRandomValue = 2000;   // HOP - stop any re-trigger for now until next channel change
          TxTimeMarker = millis();
          break;
      default:
        TxRandomValue = (LMIC.protocol ?
          SoC->random(LMIC.protocol->tx_interval_min, LMIC.protocol->tx_interval_max) :
          SoC->random(LEGACY_TX_INTERVAL_MIN, LEGACY_TX_INTERVAL_MAX));
        TxTimeMarker = millis();
        break;
      }
      
      // HOP testing - transmit time
      //Serial.printf("Tx: %d, %d, %d\r\n", millis(), TxTimeMarker, TxRandomValue);

      return true;
    }
  }
  return false;
}

bool RF_Receive(void)
{
  bool rval = false;

  if (RF_ready && rf_chip) {
    rval = rf_chip->receive();
  }
  
  return rval;
}

void RF_Shutdown(void)
{
  if (rf_chip) {
    rf_chip->shutdown();
  }
}

uint8_t RF_Payload_Size(uint8_t protocol)
{
  switch (protocol)
  {
    case RF_PROTOCOL_LEGACY:    return legacy_proto_desc.payload_size;
    case RF_PROTOCOL_OGNTP:     return ogntp_proto_desc.payload_size;
    case RF_PROTOCOL_P3I:       return p3i_proto_desc.payload_size;
    case RF_PROTOCOL_FANET:     return fanet_proto_desc.payload_size;
    case RF_PROTOCOL_ADSB_UAT:  return uat978_proto_desc.payload_size;
    default:                    return 0;
  }
}

/*
 * NRF905-specific code
 *
 *
 */

static uint8_t nrf905_channel_prev = (uint8_t) -1;
static bool nrf905_receive_active  = false;

bool nrf905_probe()
{
  uint8_t addr[4];
  uint8_t ref[] = TXADDR;

  digitalWrite(CSN, HIGH);
  pinMode(CSN, OUTPUT);

  SoC->SPI_begin();

#if defined(ARDUINO)
  SPI.setClockDivider(SPI_CLOCK_DIV2);
#endif /* ARDUINO */

  digitalWrite(CSN, LOW);

  SPI.transfer(NRF905_CMD_R_TX_ADDRESS);
  for(uint8_t i=4;i--;) {
    addr[i] = SPI.transfer(NRF905_CMD_NOP);
  }

  digitalWrite(CSN, HIGH);

  SPI.end();

#if 0
  delay(3000);
  Serial.print("NRF905 probe: ");
  Serial.print(addr[0], HEX); Serial.print(" ");
  Serial.print(addr[1], HEX); Serial.print(" ");
  Serial.print(addr[2], HEX); Serial.print(" ");
  Serial.print(addr[3], HEX); Serial.print(" ");
  Serial.println();
#endif

  /* Cold start state */
  if ((addr[0] == 0xE7) && (addr[1] == 0xE7) && (addr[2] == 0xE7) && (addr[3] == 0xE7)) {
    return true;
  }

  /* Warm restart state */
  if ((addr[0] == 0xE7) && (addr[1] == ref[0]) && (addr[2] == ref[1]) && (addr[3] == ref[2])) {
    return true;
  }

  return false;
}

void nrf905_channel(uint8_t channel)
{
  if (channel != nrf905_channel_prev) {

    uint32_t frequency;
    nRF905_band_t band;

    frequency = RF_FreqPlan.getChanFrequency(channel);
    band = (frequency >= 868000000UL ? NRF905_BAND_868 : NRF905_BAND_433);

    nRF905_setFrequency(band , frequency);

    nrf905_channel_prev = channel;
    /* restart Rx upon a channel switch */
    nrf905_receive_active = false;
  }
}

void nrf905_setup()
{
  SoC->SPI_begin();

  // Start up
  nRF905_init();

  /* Channel selection is now part of RF_loop() */
//  nrf905_channel(channel);

  //nRF905_setTransmitPower(NRF905_PWR_10);
  //nRF905_setTransmitPower(NRF905_PWR_n10);

  switch(settings->txpower)
  {
  case RF_TX_POWER_FULL:

    /*
     * NRF905 is unable to give more than 10 dBm
     * 10 dBm is legal everywhere in the world
     */

    nRF905_setTransmitPower((nRF905_pwr_t)NRF905_PWR_10);
    break;
  case RF_TX_POWER_OFF:
  case RF_TX_POWER_LOW:
  default:
    nRF905_setTransmitPower((nRF905_pwr_t)NRF905_PWR_n10);
    break;
  }

  nRF905_setCRC(NRF905_CRC_16);
  //nRF905_setCRC(NRF905_CRC_DISABLE);

  // Set address of this device
  byte addr[] = RXADDR;
  nRF905_setRXAddress(addr);

  /* Enforce radio settings to follow "Legacy" protocol's RF specs */
  settings->rf_protocol = RF_PROTOCOL_LEGACY;

  /* Enforce encoder and decoder to process "Legacy" frames only */
  protocol_encode = &legacy_encode;
  protocol_decode = &legacy_decode;

  /* Put IC into receive mode */
  nRF905_receive();
}

bool nrf905_receive()
{
  bool success = false;

  // Put into receive mode
  if (!nrf905_receive_active) {
    nRF905_receive();
    nrf905_receive_active = true;
  }

  success = nRF905_getData(RxBuffer, LEGACY_PAYLOAD_SIZE);
  if (success) { // Got data
    rx_packets_counter++;
  }

  if (SoC->Bluetooth) {
    SoC->Bluetooth->loop();
  }

  return success;
}

void nrf905_transmit()
{
    nrf905_receive_active = false;

    // Set address of device to send to
    byte addr[] = TXADDR;
    nRF905_setTXAddress(addr);

    // Set payload data
    nRF905_setData(&TxBuffer[0], LEGACY_PAYLOAD_SIZE );

    // Send payload (send fails if other transmissions are going on, keep trying until success)
    while (!nRF905_send()) {
      yield();
    } ;
}

void nrf905_shutdown()
{
  nRF905_powerDown();
  SPI.end();
}

/*
 * SX1276-specific code
 *
 *
 */

#if !defined(DISABLE_INVERT_IQ_ON_RX)
#error This example requires DISABLE_INVERT_IQ_ON_RX to be set. Update \
       config.h in the lmic library to set it.
#endif

osjob_t sx1276_txjob;
osjob_t sx1276_timeoutjob;

static void sx1276_tx_func (osjob_t* job);
static void sx1276_rx_func (osjob_t* job);
void sx1276_rx(osjobcb_t func);

static bool sx1276_receive_complete = false;
bool sx1276_receive_active = false;
static bool sx1276_transmit_complete = false;

static uint8_t sx1276_channel_prev = (uint8_t) -1;

#define SX1276_RegVersion          0x42 // common

static u1_t sx1276_readReg (u1_t addr) {
    hal_pin_nss(0);
    hal_spi(addr & 0x7F);
    u1_t val = hal_spi(0x00);
    hal_pin_nss(1);
    return val;
}

bool sx1276_probe()
{
  u1_t v, v_reset;

  SoC->SPI_begin();

  hal_init();

  // manually reset radio
  hal_pin_rst(0); // drive RST pin low
  hal_waitUntil(os_getTime()+ms2osticks(1)); // wait >100us

  v_reset = sx1276_readReg(SX1276_RegVersion);

  hal_pin_rst(2); // configure RST pin floating!
  hal_waitUntil(os_getTime()+ms2osticks(5)); // wait 5ms
  
  v = sx1276_readReg(SX1276_RegVersion);

  SPI.end();

  if (v == 0x12) {

    if (v_reset == 0x12) {
      RF_SX1276_RST_is_connected = false;
    }

    return true;
  } else {
    return false;  
  }
}

void sx1276_channel(uint8_t channel)
{
  if (channel != sx1276_channel_prev) {
    uint32_t frequency = RF_FreqPlan.getChanFrequency(channel);

    //Serial.print("frequency: "); Serial.println(frequency);

    if (sx1276_receive_active) {
      os_radio(RADIO_RST);
      sx1276_receive_active = false;
    }

    /* Actual RF chip's channel registers will be updated before each Tx or Rx session */
    LMIC.freq = frequency;
    //LMIC.freq = 868200000UL;

    sx1276_channel_prev = channel;
  }
}

void sx1276_setup()
{
  SoC->SPI_begin();

  // initialize runtime env
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  switch (settings->rf_protocol)
  {
  case RF_PROTOCOL_OGNTP:
    LMIC.protocol = &ogntp_proto_desc;
    protocol_encode = &ogntp_encode;
    protocol_decode = &ogntp_decode;
    break;
  case RF_PROTOCOL_P3I:
    LMIC.protocol = &p3i_proto_desc;
    protocol_encode = &p3i_encode;
    protocol_decode = &p3i_decode;
    break;
  case RF_PROTOCOL_FANET:
    LMIC.protocol = &fanet_proto_desc;
    protocol_encode = &fanet_encode;
    protocol_decode = &fanet_decode;
    break;
  case RF_PROTOCOL_LEGACY:
  default:
    LMIC.protocol = &legacy_proto_desc;
    protocol_encode = &legacy_encode;
    protocol_decode = &legacy_decode;
    /*
     * Enforce legacy protocol setting for SX1276
     * if other value (UAT) left in EEPROM from other (CC13XX) radio
     */
    settings->rf_protocol = RF_PROTOCOL_LEGACY;
    break;
  }

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
}

void sx1276_setvars()
{
  if (LMIC.protocol && LMIC.protocol->modulation_type == RF_MODULATION_TYPE_LORA) {
    LMIC.datarate = LMIC.protocol->bitrate;
    LMIC.preamble = LMIC.protocol->syncword[0];
  } else {
    LMIC.datarate = DR_FSK;
  }

  // This sets CR 4/5, BW125 (except for DR_SF7B, which uses BW250)
  LMIC.rps = updr2rps(LMIC.datarate);

  if (LMIC.protocol && LMIC.protocol->type == RF_PROTOCOL_FANET) {
    /* for only a few nodes around, increase the coding rate to ensure a more robust transmission */
    LMIC.rps = setCr(LMIC.rps, CR_4_8);
  }
}

bool sx1276_receive()
{
  bool success = false;

  sx1276_receive_complete = false;

  if (!sx1276_receive_active) {
    sx1276_setvars();
    sx1276_rx(sx1276_rx_func);
    sx1276_receive_active = true;
  }

  if (sx1276_receive_complete == false) {
    // execute scheduled jobs and events
    os_runloop_once();
  };

  if (SoC->Bluetooth) {
    SoC->Bluetooth->loop();
  }

  if (sx1276_receive_complete == true) {

    u1_t size = LMIC.dataLen - LMIC.protocol->payload_offset - LMIC.protocol->crc_size;

    if (size >sizeof(RxBuffer)) {
      size = sizeof(RxBuffer);
    }

    for (u1_t i=0; i < size; i++) {
        RxBuffer[i] = LMIC.frame[i + LMIC.protocol->payload_offset];
    }

    RF_last_rssi = LMIC.rssi;
    rx_packets_counter++;
    success = true;
  }

  return success;
}

void sx1276_transmit()
{
    sx1276_transmit_complete = false;
    sx1276_receive_active = false;

    sx1276_setvars();
    os_setCallback(&sx1276_txjob, sx1276_tx_func);

    while (sx1276_transmit_complete == false) {
      // execute scheduled jobs and events
      os_runloop_once();
      yield();
    };
    // HOP - LED pulse
    digitalWrite(14, HIGH);  // HOP - high only, reset to 0 by LED loop
}

void sx1276_shutdown()
{
  LMIC_shutdown();
  SPI.end();
}

// Enable rx mode and call func when a packet is received
void sx1276_rx(osjobcb_t func) {
  LMIC.osjob.func = func;
  LMIC.rxtime = os_getTime(); // RX _now_
  // Enable "continuous" RX for LoRa only (e.g. without a timeout,
  // still stops after receiving a packet)
  os_radio(LMIC.protocol &&
           LMIC.protocol->modulation_type == RF_MODULATION_TYPE_LORA ?
          RADIO_RXON : RADIO_RX);
  //Serial.println("RX");
}


static void sx1276_rx_func (osjob_t* job) {

  u1_t crc8, pkt_crc8;
  u2_t crc16, pkt_crc16;
  u1_t i;

  // SX1276 is in SLEEP after IRQ handler, Force it to enter RX mode
  sx1276_receive_active = false;

  /* FANET (LoRa) LMIC IRQ handler may deliver empty packets here when CRC is invalid. */
  if (LMIC.dataLen == 0) {
    return;
  }

  switch (LMIC.protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_NONE:
     /* crc16 left not initialized */
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    crc8 = 0x71;     /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_0000:
    crc16 = 0x0000;  /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  default:
    crc16 = 0xffff;  /* seed value */
    break;
  }

  //Serial.print("Got ");
  //Serial.print(LMIC.dataLen);
  //Serial.println(" bytes");

  switch (LMIC.protocol->type)
  {
  case RF_PROTOCOL_LEGACY:
    /* take in account NRF905/FLARM "address" bytes */
    crc16 = update_crc_ccitt(crc16, 0x31);
    crc16 = update_crc_ccitt(crc16, 0xFA);
    crc16 = update_crc_ccitt(crc16, 0xB6);
    break;
  case RF_PROTOCOL_P3I:
  case RF_PROTOCOL_OGNTP:
  default:
    break;
  }

  for (i = LMIC.protocol->payload_offset;
       i < (LMIC.dataLen - LMIC.protocol->crc_size);
       i++)
  {

    switch (LMIC.protocol->crc_type)
    {
    case RF_CHECKSUM_TYPE_GALLAGER:
    case RF_CHECKSUM_TYPE_NONE:
      break;
    case RF_CHECKSUM_TYPE_CRC8_107:
      update_crc8(&crc8, (u1_t)(LMIC.frame[i]));
      break;
    case RF_CHECKSUM_TYPE_CCITT_FFFF:
    case RF_CHECKSUM_TYPE_CCITT_0000:
    default:
      crc16 = update_crc_ccitt(crc16, (u1_t)(LMIC.frame[i]));
      break;
    }

    switch (LMIC.protocol->whitening)
    {
    case RF_WHITENING_NICERF:
      LMIC.frame[i] ^= pgm_read_byte(&whitening_pattern[i - LMIC.protocol->payload_offset]);
      break;
    case RF_WHITENING_MANCHESTER:
    case RF_WHITENING_NONE:
    default:
      break;
    }

#if DEBUG
    Serial.printf("%02x", (u1_t)(LMIC.frame[i]));
#endif
  }

  switch (LMIC.protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_NONE:
    sx1276_receive_complete = true;
    break;
  case RF_CHECKSUM_TYPE_GALLAGER:
    if (LDPC_Check((uint8_t  *) &LMIC.frame[0])) {
#if DEBUG
      Serial.printf(" %02x%02x%02x%02x%02x%02x is wrong FEC",
        LMIC.frame[i], LMIC.frame[i+1], LMIC.frame[i+2],
        LMIC.frame[i+3], LMIC.frame[i+4], LMIC.frame[i+5]);
#endif
      sx1276_receive_complete = false;
    } else {
      sx1276_receive_complete = true;
    }
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    pkt_crc8 = LMIC.frame[i];
#if DEBUG
    if (crc8 == pkt_crc8 ) {
      Serial.printf(" %02x is valid crc", pkt_crc8);
    } else {
      Serial.printf(" %02x is wrong crc", pkt_crc8);
    }
#endif
    if (crc8 == pkt_crc8) {
      sx1276_receive_complete = true;
    } else {
      sx1276_receive_complete = false;
    }
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  case RF_CHECKSUM_TYPE_CCITT_0000:
  default:
    pkt_crc16 = (LMIC.frame[i] << 8 | LMIC.frame[i+1]);
#if DEBUG
    if (crc16 == pkt_crc16 ) {
      Serial.printf(" %04x is valid crc", pkt_crc16);
    } else {
      Serial.printf(" %04x is wrong crc", pkt_crc16);
    }
#endif
    if (crc16 == pkt_crc16) {
      sx1276_receive_complete = true;
    } else {
      sx1276_receive_complete = false;
    }
    break;
  }

#if DEBUG
  Serial.println();
#endif

}

// Transmit the given string and call the given function afterwards
void sx1276_tx(unsigned char *buf, size_t size, osjobcb_t func) {

  u1_t crc8;
  u2_t crc16;

  switch (LMIC.protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_NONE:
     /* crc16 left not initialized */
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    crc8 = 0x71;     /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_0000:
    crc16 = 0x0000;  /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  default:
    crc16 = 0xffff;  /* seed value */
    break;
  }
  
  os_radio(RADIO_RST); // Stop RX first
  delay(1); // Wait a bit, without this os_radio below asserts, apparently because the state hasn't changed yet

  LMIC.dataLen = 0;

  switch (LMIC.protocol->type)
  {
  case RF_PROTOCOL_LEGACY:
    /* take in account NRF905/FLARM "address" bytes */
    crc16 = update_crc_ccitt(crc16, 0x31);
    crc16 = update_crc_ccitt(crc16, 0xFA);
    crc16 = update_crc_ccitt(crc16, 0xB6);
    break;
  case RF_PROTOCOL_P3I:
    /* insert Net ID */
    LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >> 24) & 0x000000FF);
    LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >> 16) & 0x000000FF);
    LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >>  0) & 0x000000FF);
    LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >>  0) & 0x000000FF);
    /* insert byte with payload size */
    LMIC.frame[LMIC.dataLen++] = LMIC.protocol->payload_size;

    /* insert byte with CRC-8 seed value when necessary */
    if (LMIC.protocol->crc_type == RF_CHECKSUM_TYPE_CRC8_107) {
      LMIC.frame[LMIC.dataLen++] = crc8;
    }

    break;
  case RF_PROTOCOL_OGNTP:
  default:
    break;
  }

  for (u1_t i=0; i < size; i++) {

    switch (LMIC.protocol->whitening)
    {
    case RF_WHITENING_NICERF:
      LMIC.frame[LMIC.dataLen] = buf[i] ^ pgm_read_byte(&whitening_pattern[i]);
      break;
    case RF_WHITENING_MANCHESTER:
    case RF_WHITENING_NONE:
    default:
      LMIC.frame[LMIC.dataLen] = buf[i];
      break;
    }

    switch (LMIC.protocol->crc_type)
    {
    case RF_CHECKSUM_TYPE_GALLAGER:
    case RF_CHECKSUM_TYPE_NONE:
      break;
    case RF_CHECKSUM_TYPE_CRC8_107:
      update_crc8(&crc8, (u1_t)(LMIC.frame[LMIC.dataLen]));
      break;
    case RF_CHECKSUM_TYPE_CCITT_FFFF:
    case RF_CHECKSUM_TYPE_CCITT_0000:
    default:
      crc16 = update_crc_ccitt(crc16, (u1_t)(LMIC.frame[LMIC.dataLen]));
      break;
    }

    LMIC.dataLen++;
  }

  switch (LMIC.protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_NONE:
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    LMIC.frame[LMIC.dataLen++] = crc8;
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  case RF_CHECKSUM_TYPE_CCITT_0000:
  default:
    LMIC.frame[LMIC.dataLen++] = (crc16 >>  8) & 0xFF;
    LMIC.frame[LMIC.dataLen++] = (crc16      ) & 0xFF;
    break;
  }

  LMIC.osjob.func = func;
  os_radio(RADIO_TX);
  //Serial.println("TX");
}

static void sx1276_txdone_func (osjob_t* job) {
  sx1276_transmit_complete = true;
}

static void sx1276_tx_func (osjob_t* job) {

  if (RF_tx_size > 0) {
    sx1276_tx((unsigned char *) &TxBuffer[0], RF_tx_size, sx1276_txdone_func);
  }
}

/*
 * CC13XX-specific code
 *
 *
 */

#define UAT_RINGBUF_SIZE  (sizeof(Stratux_frame_t) * 2)

static unsigned char uat_ringbuf[UAT_RINGBUF_SIZE];
static unsigned int uatbuf_head = 0;
Stratux_frame_t uatradio_frame;

const char UAT_ident[] PROGMEM = SOFTRF_UAT_IDENT;

bool cc13xx_probe()
{
  bool success = false;
  unsigned long startTime;
  unsigned int uatbuf_tail;
  u1_t keylen = strlen_P(UAT_ident);
  u1_t i=0;

  /* Do not probe on itself and ESP8266 */
  if (SoC->id == SOC_CC13XX ||
      SoC->id == SOC_ESP8266) {
    return success;
  }

  SoC->UATSerial_begin(2000000);

  SoC->CC13XX_restart();

  startTime = millis();

  // Timeout if no valid response in 1 second
  while (millis() - startTime < 1000) {

    if (UATSerial.available() > 0) {
      unsigned char c = UATSerial.read();
#if DEBUG
      Serial.println(c, HEX);
#endif
      uat_ringbuf[uatbuf_head % UAT_RINGBUF_SIZE] = c;

      uatbuf_tail = uatbuf_head - keylen;
      uatbuf_head++;

      for (i=0; i < keylen; i++) {
        if (pgm_read_byte(&UAT_ident[i]) != uat_ringbuf[(uatbuf_tail + i) % UAT_RINGBUF_SIZE]) {
          break;
        }
      }

      if (i >= keylen) {
        success = true;
        break;
      }
    }
  }

  /* cleanup UAT data buffer */
  uatbuf_head = 0;
  memset(uat_ringbuf, 0, sizeof(uat_ringbuf));

  /* Current ESP32 Core has a bug with Serial2.end()+Serial2.begin() cycle */
  if (SoC->id != SOC_ESP32) {
    UATSerial.end();
  }

  return success;
}

void cc13xx_channel(uint8_t channel)
{
  /* Nothing to do */
}

void cc13xx_setup()
{
  /* Current ESP32 Core has a bug with Serial2.end()+Serial2.begin() cycle */
  if (SoC->id != SOC_ESP32) {
    SoC->UATSerial_begin(2000000);
  }

  init_fec();

  /* Enforce radio settings to follow UAT978 protocol's RF specs */
  settings->rf_protocol = RF_PROTOCOL_ADSB_UAT;

  protocol_encode = &uat978_encode;
  protocol_decode = &uat978_decode;
}

bool cc13xx_receive()
{
  bool success = false;
  unsigned int uatbuf_tail;
  int rs_errors;

  while (UATSerial.available()) {
    unsigned char c = UATSerial.read();

    uat_ringbuf[uatbuf_head % UAT_RINGBUF_SIZE] = c;

    uatbuf_tail = uatbuf_head - sizeof(Stratux_frame_t);
    uatbuf_head++;

    if (uat_ringbuf[ uatbuf_tail      % UAT_RINGBUF_SIZE] == STRATUX_UATRADIO_MAGIC_1 &&
        uat_ringbuf[(uatbuf_tail + 1) % UAT_RINGBUF_SIZE] == STRATUX_UATRADIO_MAGIC_2 &&
        uat_ringbuf[(uatbuf_tail + 2) % UAT_RINGBUF_SIZE] == STRATUX_UATRADIO_MAGIC_3 &&
        uat_ringbuf[(uatbuf_tail + 3) % UAT_RINGBUF_SIZE] == STRATUX_UATRADIO_MAGIC_4) {

      unsigned char *pre_fec_buf = (unsigned char *) &uatradio_frame;
      for (u1_t i=0; i < sizeof(Stratux_frame_t); i++) {
          pre_fec_buf[i] = uat_ringbuf[(uatbuf_tail + i) % UAT_RINGBUF_SIZE];
      }

      int frame_type = correct_adsb_frame(uatradio_frame.data, &rs_errors);

      if (frame_type == -1) {
        continue;
      }

      u1_t size = uatradio_frame.msgLen > sizeof(RxBuffer) ?
                        sizeof(RxBuffer) : uatradio_frame.msgLen;

      for (u1_t i=0; i < size; i++) {
          RxBuffer[i] = uatradio_frame.data[i];
      }

      RF_last_rssi = uatradio_frame.rssi;
      rx_packets_counter++;

      success = true;
      break;
    }
  }
  return success;
}

void cc13xx_transmit()
{
  /* Nothing to do */
}

void cc13xx_shutdown()
{
  /* Nothing to do */
}

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

#if defined(USE_OGN_RF_DRIVER)
/*
 * OGN driver specific code
 *
 *
 */

static RFM_TRX  TRX;

static uint8_t ognrf_channel_prev  = (uint8_t) -1;
static bool ognrf_receive_active   = false;

void RFM_Select  (void)                 { hal_pin_nss(0); }
void RFM_Deselect(void)                 { hal_pin_nss(1); }
uint8_t RFM_TransferByte(uint8_t Byte)  { return hal_spi(Byte); }

bool RFM_IRQ_isOn(void)   { return lmic_pins.dio[0] == LMIC_UNUSED_PIN ? \
                                  false : digitalRead(lmic_pins.dio[0]); }

#ifdef WITH_RFM95                     // RESET is active LOW
void RFM_RESET(uint8_t On)
{ if(On) hal_pin_rst(0);
    else hal_pin_rst(1); }
#endif

#if defined(WITH_RFM69) || defined(WITH_SX1272) // RESET is active HIGH
void RFM_RESET(uint8_t On)
{ if(On) hal_pin_rst(1);
    else hal_pin_rst(0); }
#endif

bool ognrf_probe()
{
  bool success = false;

  TRX.Select       = RFM_Select;
  TRX.Deselect     = RFM_Deselect;
  TRX.TransferByte = RFM_TransferByte;
  TRX.RESET        = RFM_RESET;

  SoC->SPI_begin();
  hal_init();

  TRX.RESET(1);                      // RESET active
  vTaskDelay(10);                    // wait 10ms
  TRX.RESET(0);                      // RESET released
  vTaskDelay(10);                    // wait 10ms

  uint8_t ChipVersion = TRX.ReadVersion();

  SPI.end();

#if defined(WITH_RFM95)
  if (ChipVersion == 0x12) success = true;
#endif /* WITH_RFM95 */
#if defined(WITH_RFM69)
  if (ChipVersion == 0x24) success = true;
#endif /* WITH_RFM69 */
#if defined(WITH_SX1272)
  if (ChipVersion == 0x22) success = true;
#endif /* WITH_SX1272 */
#if defined(WITH_SI4X32)
  if (ChipVersion == 0x06 /* 4032 */ ||
      ChipVersion == 0x08 /* 4432 */ ) success = true;
#endif /* WITH_SI4X32 */

  return success;
}

void ognrf_channel(uint8_t channel)
{
  if (channel != ognrf_channel_prev) {

    if (ognrf_receive_active) {

      TRX.WriteMode(RF_OPMODE_STANDBY);
      vTaskDelay(1);

      /* restart Rx upon a channel switch */
      ognrf_receive_active = false;
    }

    TRX.setChannel(channel & 0x7F);

    ognrf_channel_prev = channel;
  }
}

void ognrf_setup()
{
  uint8_t TxPower = 0;

  /* Enforce radio settings to follow OGNTP protocol's RF specs */
  settings->rf_protocol = RF_PROTOCOL_OGNTP;

  LMIC.protocol = &ogntp_proto_desc;

  protocol_encode = &ogntp_encode;
  protocol_decode = &ogntp_decode;

  TRX.Select       = RFM_Select;
  TRX.Deselect     = RFM_Deselect;
  TRX.TransferByte = RFM_TransferByte;
  TRX.DIO0_isOn    = RFM_IRQ_isOn;
  TRX.RESET        = RFM_RESET;

  SoC->SPI_begin();
  hal_init();

  TRX.RESET(1);                      // RESET active
  vTaskDelay(10);                    // wait 10ms
  TRX.RESET(0);                      // RESET released
  vTaskDelay(10);                    // wait 10ms

  // set TRX base frequency and channel separation
  TRX.setBaseFrequency(RF_FreqPlan.BaseFreq);
  TRX.setChannelSpacing(RF_FreqPlan.ChanSepar);
  TRX.setFrequencyCorrection(0);

  TRX.Configure(0, ogntp_proto_desc.syncword);  // setup RF chip parameters and set to channel #0
  TRX.WriteMode(RF_OPMODE_STANDBY);             // set RF chip mode to STANDBY

  switch(settings->txpower)
  {
  case RF_TX_POWER_FULL:

    /* Load regional max. EIRP at first */
    TxPower = RF_FreqPlan.MaxTxPower;

    if (TxPower > 20)
      TxPower = 20;
#if 1
    if (TxPower > 17)
      TxPower = 17;
#endif

#ifdef WITH_RFM69
    TRX.WriteTxPower(TxPower, RFM69_POWER_RATING == 1 ? true : false);
#else
    TRX.WriteTxPower(TxPower);
#endif /* WITH_RFM69 */
    break;
  case RF_TX_POWER_OFF:
  case RF_TX_POWER_LOW:
  default:
    TRX.WriteTxPowerMin();
    break;
  }

  /* Leave IC in standby mode */
}

bool ognrf_receive()
{
  bool success = false;

#if !defined(WITH_SI4X32)

  uint8_t RxRSSI = 0;
  uint8_t Err [OGNTP_PAYLOAD_SIZE + OGNTP_CRC_SIZE];

  // Put into receive mode
  if (!ognrf_receive_active) {

//    TRX.ClearIrqFlags();

    TRX.WriteSYNC(7, 7, ogntp_proto_desc.syncword); // Shorter SYNC for RX
    TRX.WriteMode(RF_OPMODE_RECEIVER);
    vTaskDelay(1);

    ognrf_receive_active = true;
  }

  if(TRX.DIO0_isOn()) {
    RxRSSI = TRX.ReadRSSI();

    TRX.ReadPacket(RxBuffer, Err);
    if (LDPC_Check((uint8_t  *) RxBuffer) == 0) {
      success = true;
    }
  }

  if (success) {
    RF_last_rssi = RxRSSI;
    rx_packets_counter++;
  }

#endif /* WITH_SI4X32 */

  if (SoC->Bluetooth) {
    SoC->Bluetooth->loop();
  }

  return success;
}

void ognrf_transmit()
{
  ognrf_receive_active = false;

#if defined(WITH_SI4X32)

  TRX.WritePacket((uint8_t *) &TxBuffer[0]);
  TRX.Transmit();
  vTaskDelay(6);

#else

  TRX.WriteMode(RF_OPMODE_STANDBY);
  vTaskDelay(1);

  TRX.WriteSYNC(8, 7, ogntp_proto_desc.syncword);            // Full SYNC for TX

  TRX.ClearIrqFlags();
  TRX.WritePacket((uint8_t *) &TxBuffer[0]);

  TRX.WriteMode(RF_OPMODE_TRANSMITTER);
  vTaskDelay(5);

  uint8_t Break=0;
  for(uint16_t Wait=400; Wait; Wait--)        // wait for transmission to end
  {
    uint16_t Flags=TRX.ReadIrqFlags();
    if(Flags&RF_IRQ_PacketSent) Break++;
    if(Break>=2) break;
  }

  TRX.WriteMode(RF_OPMODE_STANDBY);

#endif /* WITH_SI4X32 */
}

void ognrf_shutdown()
{
  TRX.WriteMode(RF_OPMODE_STANDBY);
  SPI.end();
}

#endif /* USE_OGN_RF_DRIVER */
