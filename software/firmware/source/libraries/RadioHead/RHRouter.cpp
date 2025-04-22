// RHRouter.cpp
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
// $Id: RHRouter.cpp,v 1.10 2020/08/04 09:02:14 mikem Exp $

#include <RHRouter.h>

RHRouter::RoutedMessage RHRouter::_tmpMessage;

////////////////////////////////////////////////////////////////////
// Constructors
RHRouter::RHRouter(RHGenericDriver& driver, uint8_t thisAddress) 
    : RHReliableDatagram(driver, thisAddress)
{
    _max_hops = RH_DEFAULT_MAX_HOPS;
    _isa_router = true;
    clearRoutingTable();
}

////////////////////////////////////////////////////////////////////
// Public methods
bool RHRouter::init()
{
    bool ret = RHReliableDatagram::init();
    if (ret)
	_max_hops = RH_DEFAULT_MAX_HOPS;
    return ret;
}

////////////////////////////////////////////////////////////////////
void RHRouter::setMaxHops(uint8_t max_hops)
{
    _max_hops = max_hops;
}

////////////////////////////////////////////////////////////////////
void RHRouter::setIsaRouter(bool isa_router)
{
    _isa_router = isa_router;
}
////////////////////////////////////////////////////////////////////
void RHRouter::addRouteTo(uint8_t dest, uint8_t next_hop, uint8_t state)
{
    uint8_t i;

    // First look for an existing entry we can update
    for (i = 0; i < RH_ROUTING_TABLE_SIZE; i++)
    {
	if (_routes[i].dest == dest)
	{
	    _routes[i].dest = dest;
	    _routes[i].next_hop = next_hop;
	    _routes[i].state = state;
	    return;
	}
    }

    // Look for an invalid entry we can use
    for (i = 0; i < RH_ROUTING_TABLE_SIZE; i++)
    {
	if (_routes[i].state == Invalid)
	{
	    _routes[i].dest = dest;
	    _routes[i].next_hop = next_hop;
	    _routes[i].state = state;
	    return;
	}
    }

    // Need to make room for a new one
    retireOldestRoute();
    // Should be an invalid slot now
    for (i = 0; i < RH_ROUTING_TABLE_SIZE; i++)
    {
	if (_routes[i].state == Invalid)
	{
	    _routes[i].dest = dest;
	    _routes[i].next_hop = next_hop;
	    _routes[i].state = state;
	}
    }
}

////////////////////////////////////////////////////////////////////
RHRouter::RoutingTableEntry* RHRouter::getRouteTo(uint8_t dest)
{
    uint8_t i;
    for (i = 0; i < RH_ROUTING_TABLE_SIZE; i++)
	if (_routes[i].dest == dest && _routes[i].state != Invalid)
	    return &_routes[i];
    return NULL;
}

////////////////////////////////////////////////////////////////////
//blase 7/27/20
//allows one to scan through the routing table.
bool RHRouter::getNextValidRoutingTableEntry(RoutingTableEntry *RTE_p, int *lastIndex_p)
{
  bool retval = false; // default
  bool stop = false;
  uint8_t startIndex;
  
  if (*lastIndex_p < 0)
      startIndex = 0;
  else
      startIndex = *lastIndex_p + 1;
  
  if (startIndex >= RH_ROUTING_TABLE_SIZE)
  {
    return true; // finished, safety.
  }
  else
  {
    uint8_t i = startIndex;
    do
    {
      if (_routes[i].state == Valid)
      {
        *RTE_p = _routes[i];
        *lastIndex_p = i;
        retval = true; //found one
        stop = true;
      }
      else
      {
        i++;
        if (i >= RH_ROUTING_TABLE_SIZE)
	    stop = true; // no more entries
      }
    } while (!stop);
  }
  return retval;
}

////////////////////////////////////////////////////////////////////
void RHRouter::deleteRoute(uint8_t index)
{
    // Delete a route by copying following routes on top of it
    memcpy(&_routes[index], &_routes[index+1], 
	   sizeof(RoutingTableEntry) * (RH_ROUTING_TABLE_SIZE - index - 1));
    _routes[RH_ROUTING_TABLE_SIZE - 1].state = Invalid;
}

////////////////////////////////////////////////////////////////////
void RHRouter::printRoutingTable()
{
#ifdef RH_HAVE_SERIAL
    uint8_t i;
    for (i = 0; i < RH_ROUTING_TABLE_SIZE; i++)
    {
	Serial.print(i, DEC);
	Serial.print(" Dest: ");
	Serial.print(_routes[i].dest, DEC);
	Serial.print(" Next Hop: ");
	Serial.print(_routes[i].next_hop, DEC);
	Serial.print(" State: ");
	Serial.println(_routes[i].state, DEC);
    }
#endif
}

////////////////////////////////////////////////////////////////////
bool RHRouter::deleteRouteTo(uint8_t dest)
{
    uint8_t i;
    for (i = 0; i < RH_ROUTING_TABLE_SIZE; i++)
    {
	if (_routes[i].dest == dest)
	{
	    deleteRoute(i);
	    return true;
	}
    }
    return false;
}

////////////////////////////////////////////////////////////////////
void RHRouter::retireOldestRoute()
{
    // We just obliterate the first in the table and clear the last
    deleteRoute(0);
}

////////////////////////////////////////////////////////////////////
void RHRouter::clearRoutingTable()
{
    uint8_t i;
    for (i = 0; i < RH_ROUTING_TABLE_SIZE; i++)
	_routes[i].state = Invalid;
}


uint8_t RHRouter::sendtoWait(uint8_t* buf, uint8_t len, uint8_t dest, uint8_t flags)
{
    return sendtoFromSourceWait(buf, len, dest, _thisAddress, flags);
}

////////////////////////////////////////////////////////////////////
// Waits for delivery to the next hop (but not for delivery to the final destination)
uint8_t RHRouter::sendtoFromSourceWait(uint8_t* buf, uint8_t len, uint8_t dest, uint8_t source, uint8_t flags)
{
    if (((uint16_t)len + sizeof(RoutedMessageHeader)) > _driver.maxMessageLength())
	return RH_ROUTER_ERROR_INVALID_LENGTH;

    // Construct a RH RouterMessage message
    _tmpMessage.header.source = source;
    _tmpMessage.header.dest = dest;
    _tmpMessage.header.hops = 0;
    _tmpMessage.header.id = _lastE2ESequenceNumber++;
    _tmpMessage.header.flags = flags;
    memcpy(_tmpMessage.data, buf, len);

    return route(&_tmpMessage, sizeof(RoutedMessageHeader)+len);
}

////////////////////////////////////////////////////////////////////
uint8_t RHRouter::route(RoutedMessage* message, uint8_t messageLen)
{
    // Reliably deliver it if possible. See if we have a route:
    uint8_t next_hop = RH_BROADCAST_ADDRESS;
    if (message->header.dest != RH_BROADCAST_ADDRESS)
    {
	RoutingTableEntry* route = getRouteTo(message->header.dest);
	if (!route)
	    return RH_ROUTER_ERROR_NO_ROUTE;
	next_hop = route->next_hop;
    }

    if (!RHReliableDatagram::sendtoWait((uint8_t*)message, messageLen, next_hop))
	return RH_ROUTER_ERROR_UNABLE_TO_DELIVER;

    return RH_ROUTER_ERROR_NONE;
}

////////////////////////////////////////////////////////////////////
// Subclasses may want to override this to peek at messages going past
void RHRouter::peekAtMessage(RoutedMessage* message, uint8_t messageLen)
{
  // Default does nothing
  (void)message; // Not used
  (void)messageLen; // Not used
}

////////////////////////////////////////////////////////////////////
bool RHRouter::recvfromAck(uint8_t* buf, uint8_t* len, uint8_t* source, uint8_t* dest, uint8_t* id, uint8_t* flags, uint8_t* hops)
{  
    uint8_t tmpMessageLen = sizeof(_tmpMessage);
    uint8_t _from;
    uint8_t _to;
    uint8_t _id;
    uint8_t _flags;
    if (RHReliableDatagram::recvfromAck((uint8_t*)&_tmpMessage, &tmpMessageLen, &_from, &_to, &_id, &_flags))
    {
	// Here we simulate networks with limited visibility between nodes
	// so we can test routing
#ifdef RH_TEST_NETWORK
	if (
#if RH_TEST_NETWORK==1
	    // This network looks like 1-2-3-4
	       (_thisAddress == 1 && _from == 2)
	    || (_thisAddress == 2 && (_from == 1 || _from == 3))
	    || (_thisAddress == 3 && (_from == 2 || _from == 4))
	    || (_thisAddress == 4 && _from == 3)
	    
#elif RH_TEST_NETWORK==2
	       // This network looks like 1-2-4
	       //                         | | |
	       //                         --3--
	       (_thisAddress == 1 && (_from == 2 || _from == 3))
	    ||  _thisAddress == 2
	    ||  _thisAddress == 3
	    || (_thisAddress == 4 && (_from == 2 || _from == 3))

#elif RH_TEST_NETWORK==3
	       // This network looks like 1-2-4
	       //                         |   |
	       //                         --3--
	       (_thisAddress == 1 && (_from == 2 || _from == 3))
	    || (_thisAddress == 2 && (_from == 1 || _from == 4))
	    || (_thisAddress == 3 && (_from == 1 || _from == 4))
	    || (_thisAddress == 4 && (_from == 2 || _from == 3))

#elif RH_TEST_NETWORK==4
	       // This network looks like 1-2-3
	       //                           |
	       //                           4
	       (_thisAddress == 1 && _from == 2)
	    ||  _thisAddress == 2
	    || (_thisAddress == 3 && _from == 2)
	    || (_thisAddress == 4 && _from == 2)

#endif
	    )
	{
	    // OK
	}
	else
	{
	    return false; // Pretend we got nothing
	}
#endif

	peekAtMessage(&_tmpMessage, tmpMessageLen);
	// See if its for us or has to be routed
	if (_tmpMessage.header.dest == _thisAddress || _tmpMessage.header.dest == RH_BROADCAST_ADDRESS)
	{
	    // Deliver it here
	    if (source) *source  = _tmpMessage.header.source;
	    if (dest)   *dest    = _tmpMessage.header.dest;
	    if (id)     *id      = _tmpMessage.header.id;
	    if (flags)  *flags   = _tmpMessage.header.flags;
	    if (hops)   *hops    = _tmpMessage.header.hops;
	    uint8_t msgLen = tmpMessageLen - sizeof(RoutedMessageHeader);
	    if (*len > msgLen)
		*len = msgLen;
	    memcpy(buf, _tmpMessage.data, *len);
	    return true; // Its for you!
	}
	else if (   _tmpMessage.header.dest != RH_BROADCAST_ADDRESS
		 && _tmpMessage.header.hops++ < _max_hops)
	{
	    // Maybe it has to be routed to the next hop
	    // REVISIT: if it fails due to no route or unable to deliver to the next hop, 
	    // tell the originator. BUT HOW?
	    
	    // If we are forwarding packets, do so. Otherwise, drop.
	    if (_isa_router)
	        route(&_tmpMessage, tmpMessageLen);
	}
	// Discard it and maybe wait for another
    }
    return false;
}

////////////////////////////////////////////////////////////////////
bool RHRouter::recvfromAckTimeout(uint8_t* buf, uint8_t* len, uint16_t timeout, uint8_t* source, uint8_t* dest, uint8_t* id, uint8_t* flags, uint8_t* hops)
{  
    unsigned long starttime = millis();
    int32_t timeLeft;
    while ((timeLeft = timeout - (millis() - starttime)) > 0)
    {
	if (waitAvailableTimeout(timeLeft))
	{
	    if (recvfromAck(buf, len, source, dest, id, flags, hops))
		return true;
	}
	YIELD;
    }
    return false;
}

