/*
 * app.h
 *
 *  Created on: 17 Oct 2016
 *      Author: sid
 */

#ifndef STACK_APP_H_
#define STACK_APP_H_

#include <math.h>

#include "fanet.h"
#include "com/serial.h"
#include "fanet_stack/fmac.h"

#define APP_VALID_STATE_MS			10000

//todo... worst case coding...
#define APP_TYPE1_AIRTIME_MS			40		//actually 20-25ms
#define	APP_TYPE1_MINTAU_MS			250
#define	APP_TYPE1_TAU_MS			5000

#define APP_TYPE1_SIZE				10

class App : public Fapp
{
private:
	/* units are degrees, seconds, and meter */
	float latitude = NAN;
	float longitude = NAN;
	int altitude;
	float speed;
	float climb;
	float heading;
	float turnrate;

	Serial_Interface *mySerialInt = NULL;

	/* ensures the broadcasted information are still valid */
	unsigned long valid_until = 0;

	/* determines the tx rate */
	unsigned long last_tx = 0;
	unsigned long next_tx = 0;

#ifdef FANET_NAME_AUTOBRDCAST
	char name[20] = "\0";
	bool brdcast_name = false;

	int serialize_name(uint8_t*& buffer)
	{
		const int namelength = strlen(name);
		buffer = new uint8_t[namelength];
		memcpy(buffer, name, namelength);
		return namelength;
	}
#endif

	int serialize_tracking(uint8_t*& buffer)
	{
		buffer = new uint8_t[APP_TYPE1_SIZE];

		/* position */
		((uint16_t*)buffer)[0] = Frame::coord2payloadword(latitude);
		((uint16_t*)buffer)[1] = Frame::coord2payloadword(longitude);

		/* altitude set the lower 12bit */
		int alt = constrain(altitude, 0, 8190);
		if(alt > 2047)
			((uint16_t*)buffer)[2] = ((alt+2)/4) | (1<<11);				//set scale factor
		else
			((uint16_t*)buffer)[2] = alt;
		/* online tracking */
		((uint16_t*)buffer)[2] |= !!do_online_tracking<<15;
		/* aircraft type */
		((uint16_t*)buffer)[2] |= (aircraft_type&0x7)<<12;

		/* Speed */
		int speed2 = constrain((int)roundf(speed*2.0f), 0, 635);
		if(speed2 > 127)
			buffer[6] = ((speed2+2)/5) | (1<<7);					//set scale factor
		else
			buffer[6] = speed2;

		/* Climb */
		int climb10 = constrain((int)roundf(climb*10.0f), -315, 315);
		if(abs(climb10) > 63)
			buffer[7] = ((climb10 + (climb10>=0?2:-2))/5) | (1<<7);			//set scale factor
		else
			buffer[7] = climb10 & 0x7F;

		/* Heading */
		buffer[8] = constrain((int)roundf(heading*256.0f)/360.0f, 0, 255);

		/* Turn rate */
		if(!isnan(turnrate))
		{
			int turnr4 = constrain((int)roundf(turnrate*4.0f), 0, 255);
			if(abs(turnr4) > 63)
				buffer[9] = ((turnr4 + (turnr4>=0?2:-2))/4) | (1<<7);			//set scale factor
			else
				buffer[9] = turnr4 & 0x7f;
			return APP_TYPE1_SIZE;
		}
		else
		{
			return APP_TYPE1_SIZE - 1;
		}
	}

public:
	int aircraft_type;
	bool do_online_tracking;

	void set(float lat, float lon, float alt, float speed, float climb, float heading, float turn)
	{
		/* currently only used in linear mode */
		//noInterrupts();

		latitude = lat;
		longitude = lon;
		altitude = roundf(alt);
		this->speed = speed;
		this->climb = climb;
		if(heading < 0.0f)
			heading += 360.0f;
		this->heading = heading;
		turnrate = turn;

		valid_until = millis() + APP_VALID_STATE_MS;

		//interrupts();
	}

	/* device -> air */
	bool is_broadcast_ready(int num_neighbors)
	{
		/* is the state valid? */
		if(millis() > valid_until || isnan(latitude) || isnan(longitude))
			return false;

		/* in case of a busy channel, ensure that frames from the fifo get also a change */
		if(next_tx > millis())
			return false;

		/* determine if its time to send something (again) */
		const int tau_add = (num_neighbors/10 + 1) * APP_TYPE1_TAU_MS;
		if(last_tx + tau_add > millis())
			return false;

		return true;
	}

	void broadcast_successful(int type)
	{
		last_tx = millis();
	}

	Frame *get_frame()
	{
		/* prepare frame */
		Frame *frm = new Frame(fmac.my_addr);
#ifdef FANET_NAME_AUTOBRDCAST
		static uint32_t framecount = 0;
		if(brdcast_name && (framecount & 0x7F) == 0)
		{
			/* broadcast name */
			frm->type = FRM_TYPE_NAME;
			frm->payload_length = serialize_name(frm->payload);
		}
		else
		{
#endif
			/* broadcast tracking information */
			frm->type = FRM_TYPE_TRACKING;
			frm->payload_length = serialize_tracking(frm->payload);
#ifdef FANET_NAME_AUTOBRDCAST
		}
		framecount++;
#endif

		/* in case of a busy channel, ensure that frames from the fifo gets also a change */
		next_tx = millis() + APP_TYPE1_MINTAU_MS;

		return frm;
	}

#ifdef FANET_NAME_AUTOBRDCAST
	/* Name automation in case the host application does not know this... */
	void set_name(char *devname) { snprintf(name, sizeof(name), devname); };
	void allow_brdcast_name(boolean value)
	{
		if(value == false)
			brdcast_name = false;
		else
			brdcast_name = (strlen(name)==0?false:true);
	};
#endif


	void begin(Serial_Interface &si)
	{
		mySerialInt = &si;
	}

	/* air -> device */
	void handle_acked(boolean ack, MacAddr &addr)
	{
		if(mySerialInt == NULL)
			return;

		mySerialInt->handle_acked(ack, addr);
	}

	void handle_frame(Frame *frm)
	{
		if(mySerialInt == NULL)
			return;

		mySerialInt->handle_frame(frm);
	}
};

extern App app;

#endif /* STACK_APP_H_ */
