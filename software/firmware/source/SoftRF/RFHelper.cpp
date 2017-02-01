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

byte TxBuffer[PKT_SIZE], RxBuffer[PKT_SIZE];
unsigned long TxTimeMarker = 0;
legacy_packet TxPkt;

uint32_t tx_packets_counter = 0;
uint32_t rx_packets_counter = 0;

void RF_setup(void)
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
  nRF905_setTransmitPower((nRF905_pwr_t)settings->txpower);

  nRF905_setCRC(NRF905_CRC_16);
  //nRF905_setCRC(NRF905_CRC_DISABLE);

  // Set address of this device
  byte addr[] = RXADDR;
  nRF905_setRXAddress(addr);

  // Put into receive mode
  nRF905_receive();
}

void RF_Transmit(void)
{
  long RandomValue = ESP8266TrueRandom.random(500,1000);

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
    char *data = (char *) legacy_encode(&TxPkt, ThisAircraft.addr, ThisAircraft.latitude,
          ThisAircraft.longtitude, ThisAircraft.altitude, ThisAircraft.timestamp);
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
      Serial.print(F("$PSRFO,")); Serial.print(timestamp); Serial.print(F(",")); Serial.println(Bin2Hex((byte *) data));
    }
    tx_packets_counter++;
    TxTimeMarker = millis();
  }
}


bool RF_Receive(void)
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
      Serial.println("Timeout");
#endif
      break;
    }
    delay(0);
  }

  return success;
}