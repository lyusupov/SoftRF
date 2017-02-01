/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2015 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

/*
 * Read configuration registers
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

#define RXADDR {0xFE, 0x4C, 0xA6, 0xE5} // Address of this device (4 bytes)

void setup()
{
	// Start up
	nRF905_init();
	
	// Set address of this device
	byte addr[] = RXADDR;
	nRF905_setRXAddress(addr);

	// Put into receive mode
	nRF905_receive();

	Serial.begin(9600);

	Serial.println(F("Started"));
}

void loop()
{
	byte regs[NRF905_REGISTER_COUNT];
	nRF905_getConfigRegisters(regs);
	Serial.print(F("Raw: "));

	byte dataValid = 0;
		
	for(byte i=0;i<NRF905_REGISTER_COUNT;i++)
	{
		Serial.print(regs[i]);
		Serial.print(F(" "));
		if(regs[i] == 0xFF || regs[i] == 0x00)
			dataValid++;
	}

	Serial.println();
		
	// Registers were all 0xFF or 0x00,  this is probably bad
	if(dataValid >= NRF905_REGISTER_COUNT)
	{
		Serial.println(F("All registers read as 0xFF or 0x00! Is the nRF905 connected correctly?"));
		delay(1000);
		return;
	}

	char* str;
	byte data;

	uint16_t channel = ((uint16_t)(regs[1] & 0x01)<<8) | regs[0];
	uint32_t freq = (422400UL + (channel * 100UL)) * (1 + ((regs[1] & ~NRF905_MASK_BAND) >> 1));

	Serial.print(F("Channel: "));
	Serial.println(channel);
	Serial.print(F("Freq: "));
	Serial.print(freq);
	Serial.println(F("KHz"));
	Serial.print(F("Auto retransmit: "));
	Serial.println(!!(regs[1] & ~NRF905_MASK_AUTO_RETRAN));
	Serial.print(F("Low power RX: "));
	Serial.println(!!(regs[1] & ~NRF905_MASK_LOW_RX));

	// TX power
	data = regs[1] & ~NRF905_MASK_PWR;
	switch(data)
	{
		case NRF905_PWR_n10:
			data = -10;
			break;
		case NRF905_PWR_n2:
			data = -2;
			break;
		case NRF905_PWR_6:
			data = 6;
			break;
		case NRF905_PWR_10:
			data = 10;
			break;
		default:
			data = -127;
			break;
	}
	Serial.print(F("TX Power: "));
	Serial.print((signed char)data);
	Serial.println(F("dBm"));

	// Freq band
	data = regs[1] & ~NRF905_MASK_BAND;
	switch(data)
	{
		case NRF905_BAND_433:
			str = (char*)"433";
			break;
		default:
			str = (char*)"868/915";
			break;
	}
	Serial.print(F("Band: "));
	Serial.print(str);
	Serial.println(F("MHz"));

	Serial.print(F("TX Address width: "));
	Serial.println(regs[2] >> 4);
	Serial.print(F("RX Address width: "));
	Serial.println(regs[2] & 0x07);

	Serial.print(F("RX Payload size: "));
	Serial.println(regs[3]);
	Serial.print(F("TX Payload size: "));
	Serial.println(regs[4]);

	Serial.print(F("RX Address [0]: "));
	Serial.println(regs[5]);
	Serial.print(F("RX Address [1]: "));
	Serial.println(regs[6]);
	Serial.print(F("RX Address [2]: "));
	Serial.println(regs[7]);
	Serial.print(F("RX Address [3]: "));
	Serial.println(regs[8]);
	Serial.print(F("RX Address: "));
	Serial.println(((unsigned long)regs[8]<<24 | (unsigned long)regs[7]<<16 | (unsigned long)regs[6]<<8 | (unsigned long)regs[5]));

	// CRC mode
	data = regs[9] & ~NRF905_MASK_CRC;
	switch(data)
	{
		case NRF905_CRC_16:
			str = "16bit";
			break;
		case NRF905_CRC_8:
			str = "8bit";
			break;
		default:
			str = "Disabled";
			break;
	}
	Serial.print(F("CRC Mode: "));
	Serial.println(str);

	// Xtal freq
	data = regs[9] & ~NRF905_MASK_CLK;
	switch(data)
	{
		case NRF905_CLK_4MHZ:
			data = 4;
			break;
		case NRF905_CLK_8MHZ:
			data = 8;
			break;
		case NRF905_CLK_12MHZ:
			data = 12;
			break;
		case NRF905_CLK_16MHZ:
			data = 16;
			break;
		case NRF905_CLK_20MHZ:
			data = 20;
			break;
		default:
			data = 0;
			break;
	}
	Serial.print(F("Xtal freq: "));
	Serial.print(data);
	Serial.println("MHz");

	// Clock out freq
	data = regs[9] & ~NRF905_MASK_OUTCLK;
	switch(data)
	{
		case NRF905_OUTCLK_4MHZ:
			str = "4MHz";
			break;
		case NRF905_OUTCLK_2MHZ:
			str = "2MHz";
			break;
		case NRF905_OUTCLK_1MHZ:
			str = "1MHz";
			break;
		case NRF905_OUTCLK_500KHZ:
			str = "500KHz";
			break;
		default:
			str = "Disabled";
			break;
	}
	Serial.print(F("Clock out freq: "));
	Serial.println(str);

	delay(1000);
}
