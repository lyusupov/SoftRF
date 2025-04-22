// RHReliableDatagram.cpp
//
// Define addressed datagram
// 
// Part of the Arduino RH library for operating with HopeRF RH compatible transceivers 
// (see http://www.hoperf.com)
// RHDatagram will be received only by the addressed node or all nodes within range if the 
// to address is RH_BROADCAST_ADDRESS
//
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2011 Mike McCauley
// $Id: RHReliableDatagram.cpp,v 1.18 2018/11/08 02:31:43 mikem Exp $

#include <RHReliableDatagram.h>

////////////////////////////////////////////////////////////////////
// Constructors
RHReliableDatagram::RHReliableDatagram(RHGenericDriver& driver, uint8_t thisAddress) 
    : RHDatagram(driver, thisAddress)
{
    _retransmissions = 0;
    _lastSequenceNumber = 0;
    _timeout = RH_DEFAULT_TIMEOUT;
    _retries = RH_DEFAULT_RETRIES;
    memset(_seenIds, 0, sizeof(_seenIds));
}

////////////////////////////////////////////////////////////////////
// Public methods
void RHReliableDatagram::setTimeout(uint16_t timeout)
{
    _timeout = timeout;
}

////////////////////////////////////////////////////////////////////
void RHReliableDatagram::setRetries(uint8_t retries)
{
    _retries = retries;
}

////////////////////////////////////////////////////////////////////
uint8_t RHReliableDatagram::retries()
{
    return _retries;
}

////////////////////////////////////////////////////////////////////
bool RHReliableDatagram::sendtoWait(uint8_t* buf, uint8_t len, uint8_t address)
{
    // Assemble the message
    uint8_t thisSequenceNumber = ++_lastSequenceNumber;
    uint8_t retries = 0;
    while (retries++ <= _retries)
    {
	setHeaderId(thisSequenceNumber);

        // Set and clear header flags depending on if this is an
        // initial send or a retry.
        uint8_t headerFlagsToSet = RH_FLAGS_NONE;
        // Always clear the ACK flag
        uint8_t headerFlagsToClear = RH_FLAGS_ACK;
        if (retries == 1) {
            // On an initial send, clear the RETRY flag in case
            // it was previously set
            headerFlagsToClear |= RH_FLAGS_RETRY;
        } else {
            // Not an initial send, set the RETRY flag
            headerFlagsToSet = RH_FLAGS_RETRY;
        }
        setHeaderFlags(headerFlagsToSet, headerFlagsToClear);

	sendto(buf, len, address);
	waitPacketSent();

	// Never wait for ACKS to broadcasts:
	if (address == RH_BROADCAST_ADDRESS)
	    return true;

	if (retries > 1)
	    _retransmissions++;
	unsigned long thisSendTime = millis(); // Timeout does not include original transmit time

	// Compute a new timeout, random between _timeout and _timeout*2
	// This is to prevent collisions on every retransmit
	// if 2 nodes try to transmit at the same time
#if (RH_PLATFORM == RH_PLATFORM_RASPI) // use standard library random(), bugs in random(min, max)
	uint16_t timeout = _timeout + (_timeout * (random() & 0xFF) / 256);
#else
	uint16_t timeout = _timeout + (_timeout * random(0, 256) / 256);
#endif
	int32_t timeLeft;
        while ((timeLeft = timeout - (millis() - thisSendTime)) > 0)
	{
	    if (waitAvailableTimeout(timeLeft))
	    {
		uint8_t from, to, id, flags;
		if (recvfrom(0, 0, &from, &to, &id, &flags)) // Discards the message
		{
		    // Now have a message: is it our ACK?
		    if (   from == address 
			   && to == _thisAddress 
			   && (flags & RH_FLAGS_ACK) 
			   && (id == thisSequenceNumber))
		    {
			// Its the ACK we are waiting for
			return true;
		    }
		    else if (   !(flags & RH_FLAGS_ACK)
				&& (id == _seenIds[from]))
		    {
			// This is a request we have already received. ACK it again
			acknowledge(id, from);
		    }
		    // Else discard it
		}
	    }
	    // Not the one we are waiting for, maybe keep waiting until timeout exhausted
	    YIELD;
	}
	// Timeout exhausted, maybe retry
	YIELD;
    }
    // Retries exhausted
    return false;
}

////////////////////////////////////////////////////////////////////
bool RHReliableDatagram::recvfromAck(uint8_t* buf, uint8_t* len, uint8_t* from, uint8_t* to, uint8_t* id, uint8_t* flags)
{  
    uint8_t _from;
    uint8_t _to;
    uint8_t _id;
    uint8_t _flags;
    // Get the message before its clobbered by the ACK (shared rx and tx buffer in some drivers
    if (available() && recvfrom(buf, len, &_from, &_to, &_id, &_flags))
    {
	// Never ACK an ACK
	if (!(_flags & RH_FLAGS_ACK))
	{
	    // Its a normal message not an ACK
	    if (_to ==_thisAddress)
	    {
		// In some networks with mixed processor speeds, may need to delay
		// the ack with a define in say platformio.ini:
		#if defined(RH_ACK_DELAY)
		// a fast processor should wait a little before sending the acknowledge message
		unsigned long ts = millis();
		while ((millis() - ts) <= RH_ACK_DELAY)
		    YIELD;
                #endif

	        // Its for this node and
		// Its not a broadcast, so ACK it
		// Acknowledge message with ACK set in flags and ID set to received ID
		acknowledge(_id, _from);
	    }
            // Filter out retried messages that we have seen before. This explicitly
            // only filters out messages that are marked as retries to protect against
            // the scenario where a transmitting device sends just one message and
            // shuts down between transmissions. Devices that do this will report the
            // the same ID each time since their internal sequence number will reset
            // to zero each time the device starts up.
	    if ((RH_ENABLE_EXPLICIT_RETRY_DEDUP && !(_flags & RH_FLAGS_RETRY)) || _id != _seenIds[_from])
	    {
		if (from)  *from =  _from;
		if (to)    *to =    _to;
		if (id)    *id =    _id;
		if (flags) *flags = _flags;
		_seenIds[_from] = _id;
		return true;
	    }
	    // Else just re-ack it and wait for a new one
	}
    }
    // No message for us available
    return false;
}

bool RHReliableDatagram::recvfromAckTimeout(uint8_t* buf, uint8_t* len, uint16_t timeout, uint8_t* from, uint8_t* to, uint8_t* id, uint8_t* flags)
{
    unsigned long starttime = millis();
    int32_t timeLeft;
    while ((timeLeft = timeout - (millis() - starttime)) > 0)
    {
	if (waitAvailableTimeout(timeLeft))
	{
	    if (recvfromAck(buf, len, from, to, id, flags))
		return true;
	}
	YIELD;
    }
    return false;
}

uint32_t RHReliableDatagram::retransmissions()
{
    return _retransmissions;
}

void RHReliableDatagram::resetRetransmissions()
{
    _retransmissions = 0;
}
 
void RHReliableDatagram::acknowledge(uint8_t id, uint8_t from)
{
    setHeaderId(id);
    setHeaderFlags(RH_FLAGS_ACK);
    // We would prefer to send a zero length ACK,
    // but if an RH_RF22 receives a 0 length message with a CRC error, it will never receive
    // a 0 length message again, until its reset, which makes everything hang :-(
    // So we send an ACK of 1 octet
    // REVISIT: should we send the RSSI for the information of the sender?
    uint8_t ack = '!';
    sendto(&ack, sizeof(ack), from); 
    waitPacketSent();
}

