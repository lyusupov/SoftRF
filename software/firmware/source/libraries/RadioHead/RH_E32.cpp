// RH_E32.cpp
//
// Copyright (C) 2017 Mike McCauley
// $Id: RH_E32.cpp,v 1.6 2020/01/07 23:35:02 mikem Exp $

#include <RadioHead.h>
#ifdef RH_HAVE_SERIAL // No serial

#include <RH_E32.h>

RH_E32::RH_E32(Stream *s, uint8_t m0_pin, uint8_t m1_pin, uint8_t aux_pin)
  :
  _s(s),
  _m0_pin(m0_pin),
  _m1_pin(m1_pin),
  _aux_pin(aux_pin)
{
  // Prevent glitches at startup
  pinMode(_aux_pin, INPUT);
  digitalWrite(_m0_pin, HIGH);
  digitalWrite(_m1_pin, HIGH);
  pinMode(_m0_pin, OUTPUT);
  pinMode(_m1_pin, OUTPUT);
}

bool RH_E32::init()
{
  // When a message is available, Aux will go low 5 msec before the first character is output
  // So if we ever wait more than this period of time after Aux low, can conclude there will be no data
  _s->setTimeout(10);

  // Wait until the module is connected
  waitAuxHigh();

  if (!getVersion())
      return false; // Could not communicate with module or wrong type of module
  
  setMode(RHModeRx);
  clearRxBuf();

  if (!setDataRate(DataRate5kbps))
    return false;

  if (!setPower(Power21dBm))
    return false;

  //  if (!setBaudRate(BaudRate9600, Parity8N1))
  //  return false;

  if (!setFrequency(433))
    return false;
  
  return true;
}

bool RH_E32::reset()
{
  setOperatingMode(ModeSleep);
  uint8_t resetCommand[] = { RH_E32_COMMAND_RESET, RH_E32_COMMAND_RESET, RH_E32_COMMAND_RESET };
  size_t result = _s->write(resetCommand, sizeof(resetCommand));
  setOperatingMode(ModeNormal);
  return (result == sizeof(resetCommand));
}

bool RH_E32::readParameters(Parameters& params)
{
  setOperatingMode(ModeSleep);
  uint8_t readParamsCommand[] = { RH_E32_COMMAND_READ_PARAMS, RH_E32_COMMAND_READ_PARAMS, RH_E32_COMMAND_READ_PARAMS };
  _s->write(readParamsCommand, sizeof(readParamsCommand));
  size_t result = _s->readBytes((char*)&params, sizeof(params)); // default 1 sec timeout
  setOperatingMode(ModeNormal);
  return (result == sizeof(Parameters));
}

bool RH_E32::writeParameters(Parameters& params, bool save)
{
  setOperatingMode(ModeSleep);
  params.head = save ? RH_E32_COMMAND_WRITE_PARAMS_SAVE : RH_E32_COMMAND_WRITE_PARAMS_NOSAVE;
  //  printBuffer("writing now", (uint8_t*)&params, sizeof(params));
  size_t result = _s->write((uint8_t*)&params, sizeof(params));
  if (result != sizeof(params))
    return false;
  
  // Now we expect to get the same data back
  result = _s->readBytes((char*)&params, sizeof(params));
  if (result != sizeof(params))
    return false;
  //    printBuffer("additional read", (uint8_t*)&params, sizeof(params));
  // Without a little delay here, writing params often fails
  delay(20);
  
  setOperatingMode(ModeNormal);
  return result == sizeof(params);
}

void RH_E32::setOperatingMode(OperatingMode mode)
{
  waitAuxHigh();
  switch (mode)
    {
    case ModeNormal:
      digitalWrite(_m0_pin, LOW);
      digitalWrite(_m1_pin, LOW);
      break;
      
    case ModeWakeUp:
      digitalWrite(_m0_pin, HIGH);
      digitalWrite(_m1_pin, LOW);
      break;
      
    case ModePowerSaving:
      digitalWrite(_m0_pin, LOW);
      digitalWrite(_m1_pin, HIGH);
      break;
      
    case ModeSleep:
      digitalWrite(_m0_pin, HIGH);
      digitalWrite(_m1_pin, HIGH);
      break;
      
    }
  delay(10); // Takes a little while to start its response
  waitAuxHigh();
}

bool RH_E32::getVersion()
{
  setOperatingMode(ModeSleep);
  uint8_t readVersionCommand[] = { RH_E32_COMMAND_READ_VERSION, RH_E32_COMMAND_READ_VERSION, RH_E32_COMMAND_READ_VERSION };
  _s->write(readVersionCommand, sizeof(readVersionCommand));
  uint8_t version[4];
  size_t result = _s->readBytes((char *)version, sizeof(version)); // default 1 sec timeout
  setOperatingMode(ModeNormal);
  if (result == 4)
    {
      // Successful read
      //      printBuffer("read version", version, sizeof(version));
      if (version[0] != 0xc3 || version [1] != 0x32)
	{
	  // Not an E32
	  return false;
	}
      else
	{
	  // REVISIT: do something with it?
	}
    }
  else
    {
      // Read failed: no module? Wrong baud?
      return false;
    }
  return true;
}

void RH_E32::waitAuxHigh()
{
  // REVISIT: timeout needed?
  while (digitalRead(_aux_pin) == false)
    ;
}

void RH_E32::waitAuxLow()
{
  while (digitalRead(_aux_pin) == true)
    ;
}

// Check whether the latest received message is complete and uncorrupted
void RH_E32::validateRxBuf()
{
    if (_bufLen < RH_E32_HEADER_LEN)
	return; // Too short to be a real message
    if (_bufLen != _buf[0])
      return; // Do we have all the message?
    
    // Extract the 4 headers
    _rxHeaderTo    = _buf[1];
    _rxHeaderFrom  = _buf[2];
    _rxHeaderId    = _buf[3];
    _rxHeaderFlags = _buf[4];
    if (_promiscuous ||
	_rxHeaderTo == _thisAddress ||
	_rxHeaderTo == RH_BROADCAST_ADDRESS)
    {
	_rxGood++;
	_rxBufValid = true;
    }
}

void RH_E32::clearRxBuf()
{
    _rxBufValid = false;
    _bufLen = 0;
}

bool RH_E32::available()
{
    // Caution: long packets could be sent in several bursts
    if (!_rxBufValid)
    {
	if (_mode == RHModeTx)
	  return false;

	if (!_s->available())
	  return false;

	// Suck up all the characters we can
	uint8_t data;
	while (_s->readBytes((char *)&data, 1) == 1) // Not read timeout
	  {
	    _buf[_bufLen++] = data;
	  }
	// Now assess what we have
	if (_bufLen < RH_E32_HEADER_LEN)
	  {
	    //	    Serial.println("Incomplete header");
	    return false;
	  }
	else if (_bufLen < _buf[0])
	  {
	    //	    Serial.println("Incomplete message");
	    return false;
	  }
	else if (   _bufLen > _buf[0]
		 || _bufLen > RH_E32_MAX_PAYLOAD_LEN)
	  {
	    //	    Serial.println("Overrun");
	    clearRxBuf();
	    _rxBad++;
	    return false;
	  }

	// Else it a partial or complete message, test it
	//	printBuffer("read success", _buf, _bufLen);
	validateRxBuf(); 
    }
    return _rxBufValid;
}

bool RH_E32::recv(uint8_t* buf, uint8_t* len)
{
    if (!available())
	return false;
    if (buf && len)
    {
	// Skip the 4 headers that are at the beginning of the rxBuf
	if (*len > _bufLen - RH_E32_HEADER_LEN)
	    *len = _bufLen - RH_E32_HEADER_LEN;
	memcpy(buf, _buf + RH_E32_HEADER_LEN, *len);
    }
    clearRxBuf(); // This message accepted and cleared
    return true;
}

bool RH_E32::send(const uint8_t* data, uint8_t len)
{
  if (len > RH_E32_MAX_MESSAGE_LEN)
    return false;

  waitPacketSent(); // Make sure we dont collide with previous message

  // Set up the headers
  _buf[0] = len + RH_E32_HEADER_LEN; // Number of octets in teh whole message
  _buf[1] = _txHeaderTo;
  _buf[2] = _txHeaderFrom;
  _buf[3] = _txHeaderId;
  _buf[4] = _txHeaderFlags;

  // REVISIT: do we really have to do this? perhaps just write it after writing the header?
  memcpy(_buf+RH_E32_HEADER_LEN, data, len);
  
  _s->write(_buf, len + RH_E32_HEADER_LEN);
  setMode(RHModeTx);
  _txGood++;
  // Aux will return high when the TX buffer is empty
  
  return true;
}

uint8_t RH_E32::maxMessageLength()
{
  return RH_E32_MAX_MESSAGE_LEN;
}

bool RH_E32::waitPacketSent()
{
  if (_mode == RHModeTx)
    waitAuxHigh();
  setMode(RHModeRx);
  return true;
}

bool RH_E32::setDataRate(DataRate rate)
{
  Parameters params;
  if (!readParameters(params))
    return false;
  // The DataRate enums are the same values as the register bitmasks
  params.sped &= ~RH_E32_PARAM_SPED_DATARATE_MASK;
  params.sped |= (rate & RH_E32_PARAM_SPED_DATARATE_MASK);
  return writeParameters(params);
}

bool RH_E32::setPower(PowerLevel level)
{
  Parameters params;
  if (!readParameters(params))
    return false;
  // The DataRate enums are the same values as the register bitmasks
  params.option &= ~RH_E32_PARAM_OPTION_POWER_MASK;
  params.option |= (level & RH_E32_PARAM_OPTION_POWER_MASK);
  return writeParameters(params);
}

bool RH_E32::setBaudRate(BaudRate rate, Parity parity)
{
  Parameters params;
  if (!readParameters(params))
    return false;
  // The DataRate enums are the same values as the register bitmasks
  params.sped &= ~RH_E32_PARAM_SPED_UART_BAUD_MASK;
  params.sped |= (rate & RH_E32_PARAM_SPED_UART_BAUD_MASK);

  // Also set the parity
  params.sped &= ~RH_E32_PARAM_SPED_UART_MODE_MASK;
  params.sped |= (parity & RH_E32_PARAM_SPED_UART_MODE_MASK);
  
  return writeParameters(params);
}


bool RH_E32::setFrequency(uint16_t frequency)
{
  if (frequency < 410 || frequency > 441)
    return false;
  
  Parameters params;
  if (!readParameters(params))
    return false;
  params.chan = frequency - 410;
  return writeParameters(params);
  
}

#endif // RH_HAVE_SERIAL
