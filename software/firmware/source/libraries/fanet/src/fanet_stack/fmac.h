/*
 * mac.h
 *
 *  Created on: 30 Sep 2016
 *      Author: sid
 */

#ifndef FANET_STACK_FMAC_H_
#define FANET_STACK_FMAC_H_

/*
 * Timing defines
 * ONLY change if you know what you are doing. Can destroy the hole nearby network!
 */

#define MAC_SLOT_MS				20

#define MAC_TX_MINTIME				50
#define MAC_TX_ACKTIMEOUT			1000
#define MAC_TX_RETRANSMISSION_TIME		1000
#define MAC_TX_RETRANSMISSION_RETRYS		3
#define MAC_TX_BACKOFF_EXP_MIN			7
#define MAC_TX_BACKOFF_EXP_MAX			12

#define MAC_FORWARD_MAX_RSSI_DBM		-90		//todo test
#define MAC_FORWARD_MIN_DB_BOOST		20
#define MAC_FORWARD_DELAY_MIN			100
#define MAC_FORWARD_DELAY_MAX			300

#define NEIGHBOR_MAX_TIMEOUT_MS			240000		//4min


/*
 * Number defines
 */
#define MAC_NEIGHBOR_SIZE			64
#define MAC_MAXNEIGHBORS_4_TRACKING_2HOP	8
#define MAC_CODING48_THRESHOLD			10

#define MAC_FIFO_SIZE				16
#define MAC_FRAME_LENGTH			254

/*
 * Frame
 */
#define MAC_FRM_MIN_HEADER_LENGTH		4
#define MAC_FRM_ADDR_LENGTH			3
#define MAC_FRM_SIGNATURE_LENGTH		4

/* Header Byte */
#define MAC_FRM_HEADER_EXTHEADER_BIT		7
#define MAC_FRM_HEADER_FORWARD_BIT		6
#define MAC_FRM_HEADER_TYPE_MASK		0x3F

/* Extended Header Byte */
#define MAC_FRM_EXTHEADER_ACK_BIT1		7
#define MAC_FRM_EXTHEADER_ACK_BIT0		6
#define MAC_FRM_EXTHEADER_UNICAST_BIT		5
#define MAC_FRM_EXTHEADER_SIGNATURE_BIT		4
//bits 3-0 reserved

#define MAC_NOACK				0
#define MAC_ACK_SINGLEHOP			1
#define MAC_ACK_TWOHOP				2

/* frame Types */
#define FRM_TYPE_ACK				0
#define FRM_TYPE_TRACKING			1
#define FRM_TYPE_NAME				2
#define FRM_TYPE_MESSAGE			3
#define FRM_TYPE_SERVICE			4
#define FRM_TYPE_LANDMARK			5

/* Debug */
//#define SerialDEBUG				Serial1
#define MAC_debug_mode				2

#include <LinkedList.h>
#include <TimerObject.h>
#include <Stream.h>

/* note: zero copy stack might be faster and more memory efficient, but who cares @ 9kBaud and 8Ks of ram... */

/*
 * 0, 0 == Broadcast
 */

class MacAddr
{
public:
	int manufacturer;
	int id;

	MacAddr(int manufacturer_addr, int id_addr): manufacturer(manufacturer_addr), id(id_addr) {};
	MacAddr() : manufacturer(0), id(0) {};									//broadcast address
	MacAddr(const MacAddr &ma) : manufacturer(ma.manufacturer), id(ma.id) {};

	inline bool operator == (const MacAddr& rhs) const { return ((id == rhs.id) && (manufacturer == rhs.manufacturer));};
	inline bool operator != (const MacAddr& rhs) const { return ((id != rhs.id) || (manufacturer != rhs.manufacturer));};
};

class NeighborNode
{
private:
	unsigned long last_seen;
public:
	const MacAddr addr;

	NeighborNode(MacAddr addr) : addr(addr) { last_seen = millis(); }
	void seen(void) { last_seen = millis(); }
	bool isaround(void) { return last_seen + NEIGHBOR_MAX_TIMEOUT_MS > millis();}
};

class Frame
{
public:
	/* general stuff */
	static uint16_t coord2payloadword(float deg)
	{
		float deg_round =  roundf(deg);
		bool deg_odd = ((int)deg_round) & 1;
		const float decimal = deg-deg_round;
		const int dec_int = constrain((int)(decimal*32767), -16383, 16383);

		return ((dec_int&0x7FFF) | (!!deg_odd<<15));
	}

	/* addresses */
	MacAddr src;
	MacAddr dest;

	//ack and forwards (also geo based) will be handled by mac...
	int ack_requested = 0;
	bool forward = false;

	int signature = 0;

	/* payload */
	int type = 0;
	int payload_length = 0;
	uint8_t *payload = NULL;

	/* Transmit stuff */
	int num_tx = 0;
	unsigned long next_tx = 0;		//used for backoff

	/* Received stuff */
	int rssi = 0;

	int serialize(uint8_t*& buffer)
	{
		if(src.id <= 0 || src.id >= 0xFFFF || src.manufacturer <= 0 || src.manufacturer>=0xFE)
			return -2;

		int blength = MAC_FRM_MIN_HEADER_LENGTH + payload_length;

		/* extended header? */
		if(ack_requested || dest.id != 0 || dest.manufacturer != 0 || signature != 0)
			blength++;

		/* none broadcast frame */
		if(dest.id != 0 || dest.manufacturer != 0)
			blength += MAC_FRM_ADDR_LENGTH;

		/* signature */
		if(signature != 0)
			blength += MAC_FRM_SIGNATURE_LENGTH;

		/* frame to long */
		if(blength > 255)
			return -1;

		/* get memory */
		buffer = new uint8_t[blength];
		int idx = 0;

		/* header */
		buffer[idx++] = !!(ack_requested || dest.id != 0 || dest.manufacturer != 0 || signature != 0)<<MAC_FRM_HEADER_EXTHEADER_BIT |
				!!forward<<MAC_FRM_HEADER_FORWARD_BIT | (type & MAC_FRM_HEADER_TYPE_MASK);
		buffer[idx++] = src.manufacturer & 0x000000FF;
		buffer[idx++] = src.id & 0x000000FF;
		buffer[idx++] = (src.id>>8) & 0x000000FF;

		/* extended header */
		if(buffer[0] & 1<<7)
			buffer[idx++] = (ack_requested & 3)<<MAC_FRM_EXTHEADER_ACK_BIT0 |
					!!(dest.id != 0 || dest.manufacturer != 0)<<MAC_FRM_EXTHEADER_UNICAST_BIT |
					!!signature<<MAC_FRM_EXTHEADER_SIGNATURE_BIT;

		/* extheader and unicast -> add destination addr */
		if((buffer[0] & 1<<7) && (buffer[4] & 1<<5))
		{
			buffer[idx++] = dest.manufacturer & 0x000000FF;
			buffer[idx++] = dest.id & 0x000000FF;
			buffer[idx++] = (dest.id>>8) & 0x000000FF;
		}

		/* extheader and signature -> add signature */
		if((buffer[0] & 1<<7) && (buffer[4] & 1<<4))
		{
			buffer[idx++] = signature & 0x000000FF;
			buffer[idx++] = (signature>>8) & 0x000000FF;
			buffer[idx++] = (signature>>16) & 0x000000FF;
			buffer[idx++] = (signature>>24) & 0x000000FF;
		}

		/* fill payload */
		for(int i=0; i<payload_length && idx<blength; i++)
			buffer[idx++] = payload[i];

		return blength;
	}

	Frame(MacAddr addr) : src(addr) {};
	Frame();
	~Frame() {delete [] payload;};

	/* deserialize packet */
	Frame(int length, uint8_t *data)
	{
		int payload_start = MAC_FRM_MIN_HEADER_LENGTH;

		/* header */
		forward = !!(data[0] & (1<<MAC_FRM_HEADER_FORWARD_BIT));
		type = data[0] & MAC_FRM_HEADER_TYPE_MASK;
		src.manufacturer = data[1];
		src.id = data[2] | (data[3]<<8);

		/* extended header */
		if(data[0] & 1<<MAC_FRM_HEADER_EXTHEADER_BIT)
		{
			payload_start++;

			/* ack type */
			ack_requested = (data[4] >> MAC_FRM_EXTHEADER_ACK_BIT0) & 3;

			/* unicast */
			if(data[4] & (1<<MAC_FRM_EXTHEADER_UNICAST_BIT))
			{
				dest.manufacturer = data[5];
				dest.id = data[6] | (data[7]<<8);

				payload_start += MAC_FRM_ADDR_LENGTH;
			}

			/* signature */
			if(data[4] & (1<<MAC_FRM_EXTHEADER_SIGNATURE_BIT))
			{
				signature = data[payload_start] | (data[payload_start+1]<<8) | (data[payload_start+2]<<16) | (data[payload_start+3]<<24);
				payload_start += MAC_FRM_SIGNATURE_LENGTH;
			}
		}

		/* payload */
		payload_length = length - payload_start;
		payload = new uint8_t[payload_length];
		memcpy(payload, &data[payload_start], payload_length);
	}

	inline bool operator == (const Frame& frm) const
	{
		if(src != frm.src)
			return false;

		if(dest != frm.dest)
			return false;

		if(type != frm.type)
			return false;

		if(payload_length != frm.payload_length)
			return false;

		for(int i=0; i<payload_length; i++)
			if(payload[i] != frm.payload[i])
				return false;

		return true;
	};
};

class Fapp
{
public:
	Fapp() {};
	virtual ~Fapp() {};

	/* device -> air */
	virtual bool is_broadcast_ready(int num_neighbors) = 0;
	virtual void broadcast_successful(int type) = 0;
	virtual Frame *get_frame() = 0;

	/* air -> device */
	virtual void handle_frame(Frame *frm) = 0;
	virtual void handle_acked(boolean ack, MacAddr &addr) = 0;
};

class MacFifo
{
private:
	LinkedList<Frame*> fifo;
public:
//todo find a save (resource saving) and non-blocking way... good luck...
	/* not usable in async mode */
	Frame* get_nexttx();
	Frame* frame_in_list(Frame *frm);
	Frame* front();

	/* usable in async mode */
	bool remove_delete_acked_frame(MacAddr dest);
	bool remove_delete(Frame *frm);
	int add(Frame *frm);
	int size() { return fifo.size(); }
};

class FanetMac
{
private:
	TimerObject my_timer;
	MacFifo tx_fifo;
	MacFifo rx_fifo;
	LinkedList<NeighborNode *> neighbors;
	Fapp *myApp = NULL;

	unsigned long csma_next_tx = 0;
	int csma_backoff_exp = MAC_TX_BACKOFF_EXP_MIN;

	/* used for interrupt handler */
	uint8_t rx_frame[MAC_FRAME_LENGTH];
	int num_received = 0;

	static void frame_rx_wrapper(int length);
	void frame_received(int length);

	void ack(Frame* frm);

	static void state_wrapper();
	void handle_tx();
	void handle_rx();

	bool isNeighbor(MacAddr addr);
public:
	MacAddr my_addr;

	FanetMac();
	~FanetMac(){};
	void handle(){my_timer.Update();};

	bool begin(Fapp &app);

	int transmit(Frame *frm) { return tx_fifo.add(frm); };
};

extern FanetMac fmac;

#endif /* FANET_STACK_FMAC_H_ */
