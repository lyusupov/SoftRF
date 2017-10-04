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

#if LOGGER_IS_ENABLED
#include "LogHelper.h"
#endif /* LOGGER_IS_ENABLED */

byte TxBuffer[PKT_SIZE], RxBuffer[PKT_SIZE];
unsigned long TxTimeMarker = 0;
legacy_packet TxPkt;

uint32_t tx_packets_counter = 0;
uint32_t rx_packets_counter = 0;

rfchip_ops_t *rf_chip = NULL;

rfchip_ops_t nrf905_ops = {
  nrf905_probe,
  nrf905_setup,
  nrf905_receive,
  nrf905_transmit  
};

rfchip_ops_t sx1276_ops = {
  sx1276_probe,
  sx1276_setup,
  sx1276_receive,
  sx1276_transmit  
};
 
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

  rf_chip->setup();
}

void RF_Transmit(void)
{
  if (rf_chip) {
    rf_chip->transmit();
  }
}


bool RF_Receive(void)
{
  bool rval = false;

  if (rf_chip) {
    rval = rf_chip->receive();
  }
  
  return rval;
}

/*
 * NRF905-specific code
 *
 *
 */

bool nrf905_probe()
{
  return true;
}

void nrf905_setup()
{
  // Start up
  nRF905_init();

  //nRF905_setFrequency(NRF905_BAND_868 , RF_FREQ);
  //nRF905_setFrequency(NRF905_BAND_433 , 433200000UL);
  //nRF905_setFrequency(NRF905_BAND_868 , 868200000UL);
  if (settings->band == RF_BAND_EU) {
    nRF905_setFrequency(NRF905_BAND_868 , 868400000UL); 
  } else if (settings->band == RF_BAND_RU1) {
    nRF905_setFrequency(NRF905_BAND_868 , 868200000UL); 
  } else if (settings->band == RF_BAND_RU2) {
    nRF905_setFrequency(NRF905_BAND_868 , 868800000UL);
  } else if (settings->band == RF_BAND_NZ) {
    nRF905_setFrequency(NRF905_BAND_868 , 869250000UL);  
  } else if (settings->band == RF_BAND_US) {
    nRF905_setFrequency(NRF905_BAND_915 , 915000000UL);
  } else if (settings->band == RF_BAND_AU) {
    nRF905_setFrequency(NRF905_BAND_915 , 921000000UL);   
  } else {  /* RF_BAND_CN */
    nRF905_setFrequency(NRF905_BAND_433 , 433200000UL);  
  }

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

    // Make data
    //char *data = (char *) TxBuffer;

    time_t timestamp = now();
    char *data = (char *) legacy_encode(&TxPkt, &ThisAircraft);
    //Serial.println(Bin2Hex((byte *) data));

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
      StdOut.print(F("$PSRFO,")); StdOut.print(timestamp); StdOut.print(F(",")); StdOut.println(Bin2Hex((byte *) data));
    }
    tx_packets_counter++;
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

void sx1276_setup()
{
  // initialize runtime env
  os_init();

  // Set up these settings once, and use them for both TX and RX

  if (settings->band == RF_BAND_EU) {
    LMIC.freq = 868400000UL; 
  } else if (settings->band == RF_BAND_RU1) {
    LMIC.freq = 868200000UL; 
  } else if (settings->band == RF_BAND_RU2) {
    LMIC.freq = 868800000UL;
  } else if (settings->band == RF_BAND_NZ) {
    LMIC.freq = 869250000UL;  
  } else if (settings->band == RF_BAND_US) {
    LMIC.freq = 915000000UL;
  } else if (settings->band == RF_BAND_AU) {
    LMIC.freq = 921000000UL;   
  } else {  /* RF_BAND_CN */
    LMIC.freq = 433200000UL;  
  }

  // Maximum TX power
//  LMIC.txpow = 27;
  LMIC.txpow = 15;
  // Use a medium spread factor. This can be increased up to SF12 for
  // better range, but then the interval should be (significantly)
  // lowered to comply with duty cycle limits as well.
  LMIC.datarate =  DR_FSK /*  DR_SF9  */ ;
  // This sets CR 4/5, BW125 (except for DR_SF7B, which uses BW250)
  LMIC.rps = updr2rps(LMIC.datarate);

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

    for (u1_t i=0; i<(LMIC.dataLen-2); i++) {
      RxBuffer[i] = LMIC.frame[i];
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
      StdOut.print(F("$PSRFO,")); StdOut.print(timestamp); StdOut.print(F(",")); StdOut.println(Bin2Hex((byte *) &TxPkt));
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

  u2_t crc16 = 0xffff;  /* seed value */
  u1_t i;

  //Serial.print("Got ");
  //Serial.print(LMIC.dataLen);
  //Serial.println(" bytes");

  crc16 = update_crc_ccitt(crc16, 0x31);
  crc16 = update_crc_ccitt(crc16, 0xFA);
  crc16 = update_crc_ccitt(crc16, 0xB6);
  
  for (i=0; i<(LMIC.dataLen-2); i++)
  {
#if DEBUG
    Serial.printf(F("%02x"), (u1_t)(LMIC.frame[i]));
#endif
    crc16 = update_crc_ccitt(crc16, (u1_t)(LMIC.frame[i]));
  }
  u2_t pkt_crc = (LMIC.frame[i] << 8 | LMIC.frame[i+1]);
#if DEBUG
  if (crc16 == pkt_crc ) {
    Serial.printf(F(" %04x is valid crc"), pkt_crc);
  } else {
    Serial.printf(F(" %04x is wrong crc"), pkt_crc);
  }
  Serial.println();
#endif
  if (crc16 == pkt_crc) {
    sx1276_receive_complete = true;  
  } else {
    sx1276_receive_complete = false;
  }
}

// Transmit the given string and call the given function afterwards
void sx1276_tx(unsigned char *buf, size_t size, osjobcb_t func) {

  u2_t crc16 = 0xffff;  /* seed value */
  
  os_radio(RADIO_RST); // Stop RX first
  delay(1); // Wait a bit, without this os_radio below asserts, apparently because the state hasn't changed yet

  LMIC.dataLen = 0;

  crc16 = update_crc_ccitt(crc16, 0x31);
  crc16 = update_crc_ccitt(crc16, 0xFA);
  crc16 = update_crc_ccitt(crc16, 0xB6);

  for (u1_t i=0; i<size; i++) {
    LMIC.frame[LMIC.dataLen] = buf[i];

    crc16 = update_crc_ccitt(crc16, (u1_t)(LMIC.frame[LMIC.dataLen]));
    LMIC.dataLen++;
  }


  LMIC.frame[LMIC.dataLen++] = (crc16 >>  8) & 0xFF;
  LMIC.frame[LMIC.dataLen++] = (crc16      ) & 0xFF;

  LMIC.osjob.func = func;
  os_radio(RADIO_TX);
  //Serial.println("TX");
}

static void sx1276_txdone_func (osjob_t* job) {
  sx1276_transmit_complete = true;
}

static void sx1276_tx_func (osjob_t* job) {

  unsigned char *data = (unsigned char *) legacy_encode(&TxPkt, &ThisAircraft);

  sx1276_tx(data, sizeof(TxPkt), sx1276_txdone_func);
}