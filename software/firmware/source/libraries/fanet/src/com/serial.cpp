/*
 * serial.cpp
 *
 *  Created on: 1 Oct 2016
 *      Author: sid
 */

#include <Arduino.h>


#include "../fanet.h"
#include "../app.h"
#include "../fanet_stack/fmac.h"
#include "../fanet_stack/sx1272.h"
#ifdef FANET_BLUETOOTH
	#include "bm78.h"
#endif
#include "jump2dfu.h"
#include "serial.h"

#ifdef FANET_NMEA_EXTENTION
#include "../nmea/TinyGPS++.h"
	TinyGPSPlus gps;
#endif

String rxStr;

/* Fanet Commands */

/* State: #FNS lat(deg),lon(deg),alt(m),speed(km/h),climb(m/s),heading(deg)[,turn(deg/s)] */
//note: all values in float (NOT hex)
void Serial_Interface::fanet_cmd_state(char *ch_str)
{
#if defined(SerialDEBUG) && (SERIAL_debug_mode > 0)
	SerialDEBUG.print(F("### State "));
	SerialDEBUG.print(ch_str);
#endif
	/* state */
	char *p = (char *)ch_str;
	float lat = atof(p);
	p = strchr(p, SEPARATOR);
	if(p == NULL)
	{
		println(FN_REPLYE_CMD_TOO_SHORT);
		return;
	}
	float lon = atof(++p);
	p = strchr(p, SEPARATOR);
	if(p == NULL)
	{
		println(FN_REPLYE_CMD_TOO_SHORT);
		return;
	}
	float alt = atof(++p);
	p = strchr(p, SEPARATOR);
	if(p == NULL)
	{
		println(FN_REPLYE_CMD_TOO_SHORT);
		return;
	}
	float speed = atof(++p);
	p = strchr(p, SEPARATOR);
	if(p == NULL)
	{
		println(FN_REPLYE_CMD_TOO_SHORT);
		return;
	}
	float climb = atof(++p);
	p = strchr(p, SEPARATOR);
	if(p == NULL)
	{
		println(FN_REPLYE_CMD_TOO_SHORT);
		return;
	}
	float heading = atof(++p);
	p = strchr(p, SEPARATOR);
	float turn = NAN;
	if(p)
		turn = atof(++p);

	/* ensure heading in [0..360] */
	while(heading > 360.0f)
		heading -= 360.0f;
	while(heading < 0.0f)
		heading += 360.0f;

	app.set(lat, lon, alt, speed, climb, heading, turn);

	/* The state is only of interest if a src addr is set */
	if(fmac.my_addr == MacAddr())
	{
		println(FN_REPLYE_NO_SRC_ADDR);
	}
	else
	{
		println(FN_REPLY_OK);
	}
}

/* Address: #FNA manufacturer(hex),id(hex) */
void Serial_Interface::fanet_cmd_addr(char *ch_str)
{
#if defined(SerialDEBUG) && (SERIAL_debug_mode > 0)
	SerialDEBUG.print(F("### Addr "));
	SerialDEBUG.print(ch_str);
#endif
	/* address */
	char *p = (char *)ch_str;
	int manufacturer = strtol(p, NULL, 16);
	p = strchr(p, SEPARATOR)+1;
	int id = strtol(p, NULL, 16);

	if(manufacturer<=0 || manufacturer >= 0xFE || id <= 0 || id >= 0xFFFF)
	{
		println(FN_REPLYE_INVALID_ADDR);
	}
	else
	{
		fmac.my_addr.manufacturer = manufacturer;
		fmac.my_addr.id = id;
		println(FN_REPLY_OK);
	}
}

/* Config: #FNC type(0..7),onlinelogging(0..1) */
void Serial_Interface::fanet_cmd_config(char *ch_str)
{
#if defined(SerialDEBUG) && (SERIAL_debug_mode > 0)
	SerialDEBUG.print(F("### Config "));
	SerialDEBUG.print(ch_str);
#endif
	/* config */
	char *p = (char *)ch_str;
	int type = strtol(p, NULL, 16);
	p = strchr(p, SEPARATOR)+1;
	int logging = strtol(p, NULL, 16);

	if(type < 0 || type > 7)
	{
		println(FN_REPLYE_INCOMPATIBLE_TYPE);
	}
	else
	{
		app.aircraft_type = type;
		app.do_online_tracking = !!logging;
		println(FN_REPLY_OK);
	}
}

/* Transmit: #FNT type,dest_manufacturer,dest_id,forward,ack_required,length,length*2hex */
//note: all in HEX
void Serial_Interface::fanet_cmd_transmit(char *ch_str)
{
#if defined(SerialDEBUG) && (SERIAL_debug_mode > 0)
	SerialDEBUG.print(F("### Packet "));
	SerialDEBUG.print(ch_str);
#endif

	/* w/o an address we can not tx */
	if(fmac.my_addr == MacAddr())
	{
		println(FN_REPLYE_NO_SRC_ADDR);
		return;
	}

	Frame *frm = new Frame(fmac.my_addr);

	/* header */
	char *p = (char *)ch_str;
	frm->type = strtol(p, NULL, 16);
	p = strchr(p, SEPARATOR)+1;
	frm->dest.manufacturer = strtol(p, NULL, 16);
	p = strchr(p, SEPARATOR)+1;
	frm->dest.id = strtol(p, NULL, 16);
	p = strchr(p, SEPARATOR)+1;
	frm->forward = !!strtol(p, NULL, 16);
	p = strchr(p, SEPARATOR)+1;
	/* ACK required */
	if(strtol(p, NULL, 16))
	{
		frm->ack_requested = frm->forward?MAC_ACK_TWOHOP:MAC_ACK_SINGLEHOP;
		frm->num_tx = MAC_TX_RETRANSMISSION_RETRYS;
	}
	else
	{
		frm->ack_requested = MAC_NOACK;
		frm->num_tx = 0;
	}

	/* payload */
	p = strchr(p, SEPARATOR)+1;
	frm->payload_length = strtol(p, NULL, 16);
	frm->payload = new uint8_t[frm->payload_length];

	p = strchr(p, SEPARATOR)+1;
	for(int i=0; i<frm->payload_length; i++)
	{
		char sstr[3] = {p[i*2], p[i*2+1], '\0'};
		frm->payload[i] = strtol(sstr,  NULL,  16);
	}

	/* pass to mac */
	if(fmac.transmit(frm) == 0)
	{
		println(FANET_CMD_OK, 0, NULL);
#ifdef FANET_NAME_AUTOBRDCAST
		if(frm->type == FRM_TYPE_NAME)
			app.allow_brdcast_name(false);
#endif
	}
	else
	{
		delete frm;
		println(FN_REPLYE_TX_BUFF_FULL);
	}
}

/* mux string */
void Serial_Interface::fanet_eval(String &str)
{
	const char cmd = str.charAt(strlen(FANET_CMD_START));
	switch(cmd)
	{
	case CMD_STATE:
		fanet_cmd_state((char *) str.substring(strlen(FANET_CMD_START) + 1).c_str());
		break;
	case CMD_TRANSMIT:
		fanet_cmd_transmit((char *) str.substring(strlen(FANET_CMD_START) + 1).c_str());
		break;
	case CMD_ADDR:
		fanet_cmd_addr((char *) str.substring(strlen(FANET_CMD_START) + 1).c_str());
		break;
	case CMD_CONFIG:
		fanet_cmd_config((char *) str.substring(strlen(FANET_CMD_START) + 1).c_str());
		break;
	default:
		println(FN_REPLYE_FN_UNKNOWN_CMD);
	}
}

/*
 * Dongle Stuff
 */

void Serial_Interface::dongle_cmd_version(char *ch_str)
{
	if(mySerial == NULL)
		return;

	mySerial->print(DONGLE_CMD_START);
	mySerial->print(CMD_VERSION);
	mySerial->print(' ');
	mySerial->println(FANET_VERSION);
}

void Serial_Interface::dongle_cmd_jump(char *ch_str)
{
	if(!strncmp(ch_str, " BLsw", 5))
		jump2dfu(false);
	else if(!strncmp(ch_str, " BLhw", 5))
		jump2dfu(true);

	println(DN_REPLYE_JUMP);
}

void Serial_Interface::dongle_cmd_power(char *ch_str)
{
	/* remove \r\n and any spaces*/
	char *ptr = strchr(ch_str, '\r');
	if(ptr == NULL)
		ptr = strchr(ch_str, '\n');
	if(ptr != NULL)
		*ptr = '\0';
	while(*ch_str == ' ')
		ch_str++;

	if(strlen(ch_str) == 0)
	{
		/* report armed state */
		mySerial->print(DONGLE_CMD_START);
		mySerial->print(CMD_POWER);
		mySerial->print(' ');
		mySerial->println(sx1272.isArmed());
		return;
	}

	/* set status */
	if(sx1272.setArmed(!!atoi(ch_str)))
		println(DN_REPLY_OK);
	else
		println(DN_REPLYE_POWER);
}

void Serial_Interface::dongle_cmd_region(char *ch_str)
{
#if defined(SerialDEBUG) && (SERIAL_debug_mode > 0)
	SerialDEBUG.print(F("### Region "));
	SerialDEBUG.print(ch_str);
#endif

	/* eval parameter */
	char *p = (char *)ch_str;
	if(p == NULL)
	{
		println(DN_REPLYE_TOOLESSPARAMETER);
		return;
	}
	int freq = strtol(p, NULL, 10);
	p = strchr(p, SEPARATOR)+1;
	if(p == NULL)
	{
		println(DN_REPLYE_TOOLESSPARAMETER);
		return;
	}
	sx_region_t region = {.channel = 0, .dBm = strtol(p, NULL, 10)};

	switch(freq)
	{
	case 868:
		region.channel = CH_868_200;
		break;
	}

	/* configure hardware */
	if(region.channel && sx1272.setRegion(region))
		println(DN_REPLY_OK);
	else
		println(DN_REPLYE_UNKNOWNPARAMETER);
}

/* mux string */
void Serial_Interface::dongle_eval(String &str)
{
	const char cmd = str.charAt(strlen(DONGLE_CMD_START));
	switch(cmd)
	{
	case CMD_VERSION:
		dongle_cmd_version((char *) str.substring(strlen(DONGLE_CMD_START) + 1).c_str());
		break;
	case CMD_POWER:
		dongle_cmd_power((char *) str.substring(strlen(DONGLE_CMD_START) + 1).c_str());
		break;
	case CMD_REGION:
		dongle_cmd_region((char *) str.substring(strlen(DONGLE_CMD_START) + 1).c_str());
		break;
	case CMD_BOOTLOADER:
		dongle_cmd_jump((char *) str.substring(strlen(DONGLE_CMD_START) + 1).c_str());
		break;
	default:
		println(DN_REPLYE_DONGLE_UNKNOWN_CMD);
	}
}

/*
 * Bluetooth Commands
 */
#ifdef FANET_BLUETOOTH

/* Address: #BTA id(hex) */
void Serial_Interface::bt_cmd_addr(char *ch_str)
{
#if defined(SerialDEBUG) && (SERIAL_debug_mode > 0)
	SerialDEBUG.print(F("### BT id "));
	SerialDEBUG.print(ch_str);
#endif
	if(fmac.my_addr.id != 0)
		println(FN_REPLYW_BT_RECONF_ID);

	/* address */
	char *p = (char *)ch_str;
	int id = strtol(p, NULL, 16);

	if(id <= 0 || id >= 0xFFFF)
	{
		println(FN_REPLYE_INVALID_ADDR);
		return;
	}
	/* confirm before set value as we will loose the connection */
	mySerial->println(BT_CMD_OK);
	mySerial->flush();
	delay(500);

	bm78.write_id(id);
}

/* Address: #BTN name */
void Serial_Interface::bt_cmd_name(char *ch_str)
{
#if defined(SerialDEBUG) && (SERIAL_debug_mode > 0)
	SerialDEBUG.print(F("### BT Name "));
	SerialDEBUG.print(ch_str);
#endif

	/* remove trailing spaces */
	while(*ch_str == ' ')
		ch_str++;

	/* remove \r\n */
	char *ptr = strchr(ch_str, '\r');
 	if(ptr != NULL)
 		*ptr = '\0';
 	ptr = strchr(ch_str, '\n');
 	if(ptr != NULL)
 		*ptr = '\0';

	if(strlen(ch_str) == 0)
	{
		println(FN_REPLYE_BT_NAME_TOO_SHORT);
		return;
	}

	/* confirm before set value as we will loose the connection */
	mySerial->println(BT_CMD_OK);
	mySerial->flush();
	delay(500);

	bm78.write_name(ch_str);
}

void Serial_Interface::bt_eval(String &str)
{
	const char cmd = str.charAt(strlen(FANET_CMD_START));
	switch(cmd)
	{
#ifndef FANET_BLUETOOTH_ENDUSER_SAVE
	case CMD_ADDR:
		bt_cmd_addr((char *) str.substring(strlen(BT_CMD_START) + 1).c_str());
		break;
#endif
	case CMD_NAME:
		bt_cmd_name((char *) str.substring(strlen(BT_CMD_START) + 1).c_str());
		break;
	default:
		println(FN_REPYLE_BT_UNKNOWN_CMD);
	}
}

#endif

/* collect string */
void Serial_Interface::handle_rx(void)
{
	if(mySerial == NULL)
		return;

	while(mySerial->available())
	{
		char recieved = mySerial->read();
		rxStr += recieved;

		// Process message when new line character is received
		if (recieved == '\n')
		{
#if defined(SerialDEBUG) && (SERIAL_debug_mode > 1)
			SerialDEBUG.print(F("### rx:"));
			SerialDEBUG.print(rxStr);
#endif
			if(rxStr.startsWith(FANET_CMD_START))
			{
				fanet_eval(rxStr);
			}
#ifdef FANET_BLUETOOTH
			else if(rxStr.startsWith(BT_CMD_START))
			{
				bt_eval(rxStr);
			}
#endif
#ifdef FANET_NMEA_EXTENTION
			else if(rxStr.charAt(0) == '$')
			{
				/* only feeds strings of interest */
				if(rxStr.startsWith(_GPRMCterm, 1) || rxStr.startsWith(_GPGGAterm, 1))
				{
					const char *line = rxStr.c_str();
					while (*line)
					{
						if (gps.encode(*line++))
						{
							/* inform app layer */
							if(gps.location.isValid() && gps.altitude.isValid())
								app.set(gps.location.lat(), gps.location.lng(), gps.altitude.meters(),
									gps.speed.kmph(), gps.altitude.climb_mps(), gps.course.deg(), 0.0f);
							has_nmea = true;
						}
					}
					/* no response on NMEA data */
				}
				//else PFLAC,<QueryType>,<ConfigurationItem>,<Value> ???
				//# configure aircraft id
				//$PFLAC,S,ID,FFFFFF
			}
#endif
			else if(rxStr.startsWith(DONGLE_CMD_START))
			{
				dongle_eval(rxStr);
			}
			else
			{
				println(FN_REPLYE_UNKNOWN_CMD);
			}

			/* clear receiver */
			rxStr = "";

			mySerial->flush();

			last_activity = millis();
		}
	}
}

void Serial_Interface::begin(Stream &port)
{
	mySerial = &port;
}

/*
 * Handle redirected App Stuff
 */

void Serial_Interface::handle_acked(boolean ack, MacAddr &addr)
{
	if(mySerial == NULL)
		return;

	if(ack)
		mySerial->print(FANET_CMD_ACK);
	else
		mySerial->print(FANET_CMD_NACK);
	mySerial->print(F(","));
	mySerial->print(addr.manufacturer, HEX);
	mySerial->print(F(","));
	mySerial->println(addr.id, HEX);
	mySerial->flush();
}

void Serial_Interface::handle_frame(Frame *frm)
{
	if(mySerial == NULL || frm == NULL)
		return;

	/* simply print frame */

	mySerial->print(F(FANET_CMD_START CMD_RX_FRAME " "));

	/* src_manufacturer,src_id,broadcast,signature,type,payloadlength,payload */
	mySerial->print(frm->src.manufacturer, HEX);
	mySerial->print(',');
	mySerial->print(frm->src.id, HEX);
	mySerial->print(',');
	mySerial->print(frm->dest == MacAddr());	//broadcast
	mySerial->print(',');
	mySerial->print(frm->signature, HEX);
	mySerial->print(',');
	mySerial->print(frm->type, HEX);
	mySerial->print(',');
	mySerial->print(frm->payload_length, HEX);
	mySerial->print(',');
	for(int i=0; i<frm->payload_length; i++)
	{
		char buf[8];
		sprintf(buf, "%02X", frm->payload[i]);
		mySerial->print(buf);
	}

	mySerial->println();
	mySerial->flush();
}

void Serial_Interface::println(const __FlashStringHelper *type, int key, const __FlashStringHelper *msg)
{
	if(mySerial == NULL || type == NULL)
		return;

	mySerial->print(type);
	if(key > 0)
	{
		mySerial->print(',');
		mySerial->print(key);
		if(msg != NULL)
		{
			mySerial->print(',');
			mySerial->print(msg);
		}
	}
	mySerial->println();
	mySerial->flush();
}

Serial_Interface serial_int = Serial_Interface();

