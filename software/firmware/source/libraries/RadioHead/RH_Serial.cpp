// RH_Serial.cpp
//
// Copyright (C) 2014 Mike McCauley
// $Id: RH_Serial.cpp,v 1.17 2020/01/07 23:35:02 mikem Exp $

#include <RH_Serial.h>
#include <RHCRC.h>

#ifdef RH_HAVE_SERIAL
RH_Serial::RH_Serial(HardwareSerial& serial)
    :
    _serial(serial),
    _rxState(RxStateInitialising)
{
}

HardwareSerial& RH_Serial::serial()
{
    return _serial;
}

bool RH_Serial::init()
{
    if (!RHGenericDriver::init())
	return false;
    _rxState = RxStateIdle;
    return true;
}

// Call this often
bool RH_Serial::available()
{
    while (!_rxBufValid &&_serial.available())
	handleRx(_serial.read());
    return _rxBufValid;
}

void RH_Serial::waitAvailable(uint16_t polldelay)
{
#if (RH_PLATFORM == RH_PLATFORM_UNIX)
    // Unix version driver in RHutil/HardwareSerial knows how to wait without polling
    while (!available())
	_serial.waitAvailable();
#else
    RHGenericDriver::waitAvailable(polldelay);
#endif
}

bool RH_Serial::waitAvailableTimeout(uint16_t timeout, uint16_t polldelay)
{
#if (RH_PLATFORM == RH_PLATFORM_UNIX)
    // Unix version driver in RHutil/HardwareSerial knows how to wait without polling
    unsigned long starttime = millis();
    while ((millis() - starttime) < timeout)
    {
	_serial.waitAvailableTimeout(timeout - (millis() - starttime));
        if (available())
           return true;
	YIELD;
	if (polldelay)
	    delay(polldelay);
    }
    return false;
#else
    return RHGenericDriver::waitAvailableTimeout(timeout, polldelay);
#endif
}

void  RH_Serial::handleRx(uint8_t ch)
{
    // State machine for receiving chars
    switch(_rxState)
    {
	case RxStateIdle:
	{
	    if (ch == DLE)
		_rxState = RxStateDLE;
	}
	break;
	    
	case RxStateDLE:
	{
	    if (ch == STX)
	    {
		clearRxBuf();
		_rxState = RxStateData;
	    }
	    else
		_rxState = RxStateIdle;
	}
	break;

	case RxStateData:
	{
	    if (ch == DLE)
		_rxState = RxStateEscape;
	    else
		appendRxBuf(ch);
	}
	break;

	case RxStateEscape:
	{
	    if (ch == ETX)
	    {
		// add fcs for DLE, ETX
		_rxFcs = RHcrc_ccitt_update(_rxFcs, DLE);
		_rxFcs = RHcrc_ccitt_update(_rxFcs, ETX);
		_rxState = RxStateWaitFCS1; // End frame
	    }
	    else if (ch == DLE)
	    {
		appendRxBuf(ch);
		_rxState = RxStateData;
	    }
	    else
		_rxState = RxStateIdle; // Unexpected
	}
	break;

	case RxStateWaitFCS1:
	{
	    _rxRecdFcs = ch << 8;
	    _rxState = RxStateWaitFCS2;
	}
	break;

	case RxStateWaitFCS2:
	{
	    _rxRecdFcs |= ch;
	    _rxState = RxStateIdle;
	    validateRxBuf();
	}
	break;

	default: // Else some compilers complain
	    break; 
    }
}

void RH_Serial::clearRxBuf()
{
    _rxBufValid = false;
    _rxFcs = 0xffff;
    _rxBufLen = 0;
}

void RH_Serial::appendRxBuf(uint8_t ch)
{
    if (_rxBufLen < RH_SERIAL_MAX_PAYLOAD_LEN)
    {
	// Normal data, save and add to FCS
	_rxBuf[_rxBufLen++] = ch;
	_rxFcs = RHcrc_ccitt_update(_rxFcs, ch);
    }
    // If the buffer overflows, we dont record the trailing data, and the FCS will be wrong,
    // causing the message to be dropped when the FCS is received
}

// Check whether the latest received message is complete and uncorrupted
void RH_Serial::validateRxBuf()
{
    if (_rxRecdFcs != _rxFcs)
    {
	_rxBad++;
	return;
    }

    // Extract the 4 headers
    _rxHeaderTo    = _rxBuf[0];
    _rxHeaderFrom  = _rxBuf[1];
    _rxHeaderId    = _rxBuf[2];
    _rxHeaderFlags = _rxBuf[3];
    if (_promiscuous ||
	_rxHeaderTo == _thisAddress ||
	_rxHeaderTo == RH_BROADCAST_ADDRESS)
    {
	_rxGood++;
	_rxBufValid = true;
    }
}

bool RH_Serial::recv(uint8_t* buf, uint8_t* len)
{
    if (!available())
	return false;
    if (buf && len)
    {
	// Skip the 4 headers that are at the beginning of the rxBuf
	if (*len > _rxBufLen-RH_SERIAL_HEADER_LEN)
	    *len = _rxBufLen-RH_SERIAL_HEADER_LEN;
	memcpy(buf, _rxBuf+RH_SERIAL_HEADER_LEN, *len);
    }
    clearRxBuf(); // This message accepted and cleared
    return true;
}

// Caution: this may block
bool RH_Serial::send(const uint8_t* data, uint8_t len)
{
    if (len > RH_SERIAL_MAX_MESSAGE_LEN)
	return false;

    if (!waitCAD()) 
	return false;  // Check channel activity

    _txFcs = 0xffff;    // Initial value
    _serial.write(DLE); // Not in FCS
    _serial.write(STX); // Not in FCS
    // First the 4 headers
    txData(_txHeaderTo);
    txData(_txHeaderFrom);
    txData(_txHeaderId);
    txData(_txHeaderFlags);
    // Now the payload
    while (len--)
	txData(*data++);
    // End of message
    _serial.write(DLE);
    _txFcs = RHcrc_ccitt_update(_txFcs, DLE);
    _serial.write(ETX);
    _txFcs = RHcrc_ccitt_update(_txFcs, ETX);

    // Now send the calculated FCS for this message
    _serial.write((_txFcs >> 8) & 0xff);
    _serial.write(_txFcs & 0xff);
    return true;
}

void  RH_Serial::txData(uint8_t ch)
{
    if (ch == DLE)    // DLE stuffing required?
	_serial.write(DLE); // Not in FCS
    _serial.write(ch);
    _txFcs = RHcrc_ccitt_update(_txFcs, ch);
}

uint8_t RH_Serial::maxMessageLength()
{
    return RH_SERIAL_MAX_MESSAGE_LEN;
}

#endif // HAVE_SERIAL
