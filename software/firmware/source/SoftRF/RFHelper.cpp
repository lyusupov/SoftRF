/*
 * RFHelper.cpp
 * Copyright (C) 2016-2017 Linar Yusupov
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

#include "RFHelper.h"
#include "Protocol_Legacy.h"
#include "Protocol_OGNTP.h"
#include "Protocol_P3I.h"

#include <freqplan.h>

#if LOGGER_IS_ENABLED
#include "LogHelper.h"
#endif /* LOGGER_IS_ENABLED */

byte RxBuffer[PKT_SIZE];
unsigned long TxTimeMarker = 0;

byte TxPkt[MAX_PKT_SIZE];

uint32_t tx_packets_counter = 0;
uint32_t rx_packets_counter = 0;

static FreqPlan RF_FreqPlan;
static bool RF_ready = false;

static size_t RF_tx_size = 0;

rfchip_ops_t *rf_chip = NULL;

size_t (*protocol_encode)(void *, ufo_t *);
bool (*protocol_decode)(void *, ufo_t *, ufo_t *);

rfchip_ops_t nrf905_ops = {
  nrf905_probe,
  nrf905_setup,
  nrf905_channel,
  nrf905_receive,
  nrf905_transmit  
};

rfchip_ops_t sx1276_ops = {
  sx1276_probe,
  sx1276_setup,
  sx1276_channel,
  sx1276_receive,
  sx1276_transmit  
};

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
 
void RF_setup(void)
{

  if (rf_chip == NULL) {
    if (sx1276_ops.probe()) {
      rf_chip = &sx1276_ops;  
      Serial.println(F("SX1276 RFIC is detected."));
    } else {
      rf_chip = &nrf905_ops;
      Serial.println(F("SX1276 RFIC is NOT detected! Fallback to NRF905 operations."));
    }  
  }  

  /* "AUTO" freq. will set the plan upon very first valid GNSS fix */
  if (settings->band == RF_BAND_AUTO) {
    /* Supersede EU plan with UK when PAW is selected */
    if (rf_chip != &nrf905_ops && settings->rf_protocol == RF_PROTOCOL_P3I) {
      RF_FreqPlan.setPlan(RF_BAND_UK);
    }
  } else {
    RF_FreqPlan.setPlan(settings->band);
  }

  rf_chip->setup();
}

void RF_SetChannel(void)
{
  uint32_t Time = (uint32_t) now(); // ThisAircraft.timestamp ;
  /* stick EU freq. on 868.4 MHz for now */
  uint8_t Slot = 1; /* only #1 "400ms" timeslot is currently in use */
  uint8_t OGN = (settings->rf_protocol == RF_PROTOCOL_OGNTP ? 1 : 0);

#if 1  /* Temporarily force both OGN and Legacy to live on the same channel */
  OGN = 0;
#endif

  uint8_t chan = RF_FreqPlan.getChannel(Time, Slot, OGN);

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

size_t RF_Encode(void)
{
  size_t size = 0;
  if (RF_ready && protocol_encode) {
    size = (*protocol_encode)((void *) &TxPkt[0], &ThisAircraft);
  }
  return size;
}

void RF_Transmit(size_t size)
{
  if (RF_ready && rf_chip && (size > 0)) {
    RF_tx_size = size;
    rf_chip->transmit();
  }
}

bool RF_Receive(void)
{
  bool rval = false;

  if (RF_ready && rf_chip) {
    rval = rf_chip->receive();
  }
  
  return rval;
}

/*
 * NRF905-specific code
 *
 *
 */

static uint8_t nrf905_channel_prev = (uint8_t) -1;

bool nrf905_probe()
{
  return true;
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
  }
}

void nrf905_setup()
{
  // Start up
  nRF905_init();

  /* Channel selection is now part of RF_loop() */
//  nrf905_channel(channel);

  //nRF905_setTransmitPower(NRF905_PWR_10);
  //nRF905_setTransmitPower(NRF905_PWR_n10);

  if (settings->txpower == NRF905_TX_PWR_OFF ) {
    nRF905_setTransmitPower((nRF905_pwr_t)NRF905_PWR_n10);
  } else {
    nRF905_setTransmitPower((nRF905_pwr_t)settings->txpower);
  }

  nRF905_setCRC(NRF905_CRC_16);
  //nRF905_setCRC(NRF905_CRC_DISABLE);

  // Set address of this device
  byte addr[] = RXADDR;
  nRF905_setRXAddress(addr);

  // Put into receive mode
  nRF905_receive();

  protocol_encode = &legacy_encode;
  protocol_decode = &legacy_decode;

}

bool nrf905_receive()
{
  bool success = false;

  // Put into receive mode
  nRF905_receive();

  // Wait for reply with timeout
  unsigned long sendStartTime = millis();
  while (1)
  {
    success = nRF905_getData(RxBuffer, sizeof(RxBuffer));
    if (success) { // Got data
      rx_packets_counter++;
      break;        
    }

    // Timeout
    if (millis() - sendStartTime > TIMEOUT) {
#if DEBUG
      Serial.println(F("Timeout"));
#endif
      break;
    }
    delay(0);
  }

  return success;
}

void nrf905_transmit()
{
  long RandomValue;

  if (settings->txpower == NRF905_TX_PWR_OFF ) {
    return;
  }
  
  RandomValue = ESP8266TrueRandom.random(500,1000);

  if ((millis() - TxTimeMarker > (int)RandomValue)) {
#if 0
    Serial.print("Valid: ");
    Serial.println(gnss.location.isValid());
    Serial.print("isUpdated: ");
    Serial.println(gnss.location.isUpdated());
    Serial.print("age: ");
    Serial.println(gnss.location.age());
#endif

    time_t timestamp = now();

    // Set address of device to send to
    byte addr[] = TXADDR;
    nRF905_setTXAddress(addr);

    // Set payload data
    nRF905_setData(&TxPkt[0], NRF905_PAYLOAD_SIZE );

    // Send payload (send fails if other transmissions are going on, keep trying until success)
    while (!nRF905_send()) {
      delay(0);
    } ;
    if (settings->nmea_p) {
      StdOut.print(F("$PSRFO,")); StdOut.print(timestamp); StdOut.print(F(",")); StdOut.println(Bin2Hex((byte *) &TxPkt[0]));
    }
    tx_packets_counter++;
    RF_tx_size = 0;
    TxTimeMarker = millis();
  }
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

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = D8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {D0, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
};

osjob_t sx1276_txjob;
osjob_t sx1276_timeoutjob;

static void sx1276_tx_func (osjob_t* job);
static void sx1276_rx_func (osjob_t* job);
void sx1276_rx(osjobcb_t func);

static bool sx1276_receive_complete = false;
static bool sx1276_transmit_complete = false;

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
  u1_t v;

  hal_init();
  
  v = sx1276_readReg(SX1276_RegVersion);

  if (v == 0x12) {
    return true;
  } else {
    return false;  
  }
}

void sx1276_channel(uint8_t channel)
{
  uint32_t frequency = RF_FreqPlan.getChanFrequency(channel);

  /* Actual RF chip's channel registers will be updated before each Tx or Rx session */
  LMIC.freq = frequency;
//LMIC.freq = 868400000UL;
}

void sx1276_setup()
{
  // initialize runtime env
  os_init();

  /* Channel selection is now part of RF_loop() */
//  sx1276_channel(channel);

  // Maximum TX power
//  LMIC.txpow = 27;
  LMIC.txpow = 15;
  // Use a medium spread factor. This can be increased up to SF12 for
  // better range, but then the interval should be (significantly)
  // lowered to comply with duty cycle limits as well.
  LMIC.datarate =  DR_FSK /*  DR_SF9  */ ;
  // This sets CR 4/5, BW125 (except for DR_SF7B, which uses BW250)
  LMIC.rps = updr2rps(LMIC.datarate);

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
  case RF_PROTOCOL_LEGACY:
  default:
    LMIC.protocol = &legacy_proto_desc;
    protocol_encode = &legacy_encode;
    protocol_decode = &legacy_decode;
    break;
  }
}

bool sx1276_receive()
{
  bool success = false;

  // Wait for reply with timeout
  unsigned long sendStartTime = millis();

  sx1276_receive_complete = false;

  sx1276_rx(sx1276_rx_func);

  while (sx1276_receive_complete == false) {
      // execute scheduled jobs and events
      os_runloop_once();
      // Timeout
      if (millis() - sendStartTime > TIMEOUT) {
#if DEBUG
        Serial.println(F("Timeout"));
#endif
        break;
      }

    delay(0);
  };

  os_radio(RADIO_RST);

  if (sx1276_receive_complete == true) {

    u1_t size = LMIC.dataLen - LMIC.protocol->payload_offset - LMIC.protocol->crc_size;

    for (u1_t i=0; i < size; i++) {
        RxBuffer[i] = LMIC.frame[i + LMIC.protocol->payload_offset];
    }

    rx_packets_counter++;
    success = true;
  }

  return success;
}

void sx1276_transmit()
{
  long RandomValue;

  if (settings->txpower == NRF905_TX_PWR_OFF ) {
    return;
  }
  
  RandomValue = ESP8266TrueRandom.random(500,1000);

  if ((millis() - TxTimeMarker > (int)RandomValue)) {

    time_t timestamp = now();

    sx1276_transmit_complete = false;
    os_setCallback(&sx1276_txjob, sx1276_tx_func);

    while (sx1276_transmit_complete == false) {
      // execute scheduled jobs and events
      os_runloop_once();
      delay(0);
    } ;

    if (settings->nmea_p) {
      StdOut.print(F("$PSRFO,")); StdOut.print(timestamp); StdOut.print(F(",")); StdOut.println(Bin2Hex((byte *) &TxPkt[0]));
    }
    tx_packets_counter++;
    TxTimeMarker = millis();
  } 
  
}

// Enable rx mode and call func when a packet is received
void sx1276_rx(osjobcb_t func) {
  LMIC.osjob.func = func;
  LMIC.rxtime = os_getTime(); // RX _now_
  // Enable "continuous" RX (e.g. without a timeout, still stops after
  // receiving a packet)
  os_radio(RADIO_RX /* RADIO_RXON */);
  //Serial.println("RX");
}


static void sx1276_rx_func (osjob_t* job) {

  u1_t crc8, pkt_crc8;
  u2_t crc16, pkt_crc16;
  u1_t i;

  switch (LMIC.protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
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
  RF_tx_size = 0;
  sx1276_transmit_complete = true;
}

static void sx1276_tx_func (osjob_t* job) {

  if (RF_tx_size > 0) {
    sx1276_tx((unsigned char *) &TxPkt[0], RF_tx_size, sx1276_txdone_func);
  }
}
