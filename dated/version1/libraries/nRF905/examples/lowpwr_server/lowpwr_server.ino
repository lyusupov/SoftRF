/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2013 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

/*
 * Wait for data and reply.
 * Output power is set to the lowest setting, receive sensitivity is lowered.
 *
 * 7 -> CE
 * 8 -> PWR
 * 9 -> TXE
 * 2 -> CD
 * 3 -> DR
 * 10 -> CSN
 * 12 -> SO
 * 11 -> SI
 * 13 -> SCK
 */

#include <nRF905.h>
#include <SPI.h>

#define RXADDR {0x58, 0x6F, 0x2E, 0x10} // Address of this device (4 bytes)
#define TXADDR {0xFE, 0x4C, 0xA6, 0xE5} // Address of device to send to (4 bytes)

void setup()
{
	// Start up
	nRF905_init();
	
	// Set address of this device
	byte addr[] = RXADDR;
	nRF905_setRXAddress(addr);

	// Lowest transmit level -10db
	nRF905_setTransmitPower(NRF905_PWR_n10);

	// Reduce receive sensitivity to save a few mA
	nRF905_setLowRxPower(NRF905_LOW_RX_ENABLE);

	// Put into receive mode
	nRF905_receive();

	Serial.begin(9600);

	Serial.println(F("Server started"));
}

void loop()
{
	Serial.println(F("Waiting for ping..."));

	// Make buffer for data
	byte buffer[NRF905_MAX_PAYLOAD];

	// Wait for data
	while(!nRF905_getData(buffer, sizeof(buffer)));

	Serial.println(F("Got ping"));

	// Set address of device to send to
	byte addr[] = TXADDR;
	nRF905_setTXAddress(addr);

	// Set payload data (reply with data received)
	nRF905_setData(buffer, sizeof(buffer));
	
	Serial.println(F("Sending reply..."));

	// Send payload (send fails if other transmissions are going on, keep trying until success)
	while(!nRF905_send());

	// Put back into receive mode
	nRF905_receive();

	Serial.println(F("Reply sent"));

	// Printout ping contents
	Serial.print(F("Data: "));
	Serial.write(buffer, sizeof(buffer));
	Serial.println();
}