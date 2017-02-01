/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2014 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

/*
 * Wireless serial link
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

#define PACKET_TYPE_DATA	0
#define PACKET_TYPE_ACK		1

#define MAX_PACKET_SIZE (NRF905_MAX_PAYLOAD - 2)
typedef struct {
	byte dstAddress[NRF905_ADDR_SIZE];
	byte type;
	byte len;
	byte data[MAX_PACKET_SIZE];
} packet_s;

void setup()
{
	// Start up
	nRF905_init();

	// Put into receive mode
	nRF905_receive();

	Serial.begin(9600);
	
	Serial.println(F("Ready"));
}

void loop()
{
	packet_s packet;

	// Send serial data
	byte dataSize;
	while((dataSize = Serial.available()))
	{
		// Make sure we don't try to send more than max packet size
		if(dataSize > MAX_PACKET_SIZE)
			dataSize = MAX_PACKET_SIZE;

		packet.type = PACKET_TYPE_DATA;
		packet.len = dataSize;

		// Copy data from serial to packet buffer
		for(byte i=0;i<dataSize;i++)
			packet.data[i] = Serial.read();

		// Send packet
		sendPacket(&packet);

		// Receive mode
		nRF905_receive();

		// Wait for ACK packet
		byte startTime = millis();
		while(1)
		{
			bool timeout = false;
			while(1)
			{
				if(getPacket(&packet)) // Get new packet
					break;
				else if((byte)(millis() - startTime) > 50) // 50ms timeout
				{
					timeout = true;
					break;
				}
			}

			if(timeout) // Timed out
			{
				Serial.println(F("TO"));
				break;
			}
			else if(packet.type == PACKET_TYPE_ACK) // Is packet type ACK?
				break;
		}
	}

	// Put into receive mode
	nRF905_receive();

	// Wait for data
	while(1)
	{
		if(getPacket(&packet) && packet.type == PACKET_TYPE_DATA) // Got a packet and is it a data packet?
		{
			// Print data
			Serial.write(packet.data, packet.len);

			// Reply with ACK
			packet.type = PACKET_TYPE_ACK;
			packet.len = 0;
			sendPacket(&packet);

			// Put into receive mode
			nRF905_receive();
		}
		else if(Serial.available()) // We've got some serial data, need to send it
			break;
	}
}

// Send a packet
static void sendPacket(void* _packet)
{
	// Void pointer to packet_s pointer hack
	// Arduino puts all the function defs at the top of the file before packet_s being declared :/
	packet_s* packet = (packet_s*)_packet;

	// Convert packet data to plain byte array
	byte totalLength = packet->len + 2;
	byte tmpBuff[totalLength];
	tmpBuff[0] = packet->type;
	tmpBuff[1] = packet->len;
	memcpy(&tmpBuff[2], packet->data, packet->len);

	// Set address of device to send to
	//nRF905_setTXAddress(packet->dstAddress);

	// Set payload data
	nRF905_setData(tmpBuff, totalLength);

	// Send payload (send fails if other transmissions are going on, keep trying until success)
	while(!nRF905_send());
}

// Get a packet
static bool getPacket(void* _packet)
{
	// Void pointer to packet_s pointer hack
	// Arduino puts all the function defs at the top of the file before packet_s being declared :/
	packet_s* packet = (packet_s*)_packet;

	byte buffer[NRF905_MAX_PAYLOAD];

	// See if any data available
	if(!nRF905_getData(buffer, sizeof(buffer)))
		return false;

	// Convert byte array to packet
	packet->type = buffer[0];
	packet->len = buffer[1];

	// Sanity check
	if(packet->len > MAX_PACKET_SIZE)
		packet->len = MAX_PACKET_SIZE;

	memcpy(packet->data, &buffer[2], packet->len);
	
	return true;
}