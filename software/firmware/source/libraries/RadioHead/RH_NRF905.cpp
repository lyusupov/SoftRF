// RH_NRF905.cpp
//
// Copyright (C) 2012 Mike McCauley
// $Id: RH_NRF905.cpp,v 1.7 2017/01/12 23:58:00 mikem Exp $

#include <RH_NRF905.h>

RH_NRF905::RH_NRF905(uint8_t chipEnablePin, uint8_t txEnablePin, uint8_t slaveSelectPin, RHGenericSPI& spi)
    :
    RHNRFSPIDriver(slaveSelectPin, spi)
{
    _chipEnablePin = chipEnablePin;
    _txEnablePin   = txEnablePin;
}

bool RH_NRF905::init()
{
#if defined (__MK20DX128__) || defined (__MK20DX256__)
    // Teensy is unreliable at 8MHz:
    _spi.setFrequency(RHGenericSPI::Frequency1MHz);
#else
    _spi.setFrequency(RHGenericSPI::Frequency8MHz);
#endif
    if (!RHNRFSPIDriver::init())
	return false;

    // Initialise the slave select pin and the tx Enable pin
    pinMode(_chipEnablePin, OUTPUT);
    pinMode(_txEnablePin, OUTPUT);
    digitalWrite(_chipEnablePin, LOW);
    digitalWrite(_txEnablePin, LOW);

    // Configure the chip
    // CRC 16 bits enabled. 16MHz crystal freq
    spiWriteRegister(RH_NRF905_CONFIG_9, RH_NRF905_CONFIG_9_CRC_EN | RH_NRF905_CONFIG_9_CRC_MODE_16BIT | RH_NRF905_CONFIG_9_XOF_16MHZ);

    // Make sure we are powered down
    setModeIdle();

    // Some innocuous defaults
    setChannel(108, LOW); // 433.2 MHz
    setRF(RH_NRF905::TransmitPowerm10dBm);

    return true;
}

// Use the register commands to read and write the registers
uint8_t RH_NRF905::spiReadRegister(uint8_t reg)
{
    return spiRead((reg & RH_NRF905_REG_MASK) | RH_NRF905_REG_R_CONFIG);
}

uint8_t RH_NRF905::spiWriteRegister(uint8_t reg, uint8_t val)
{
    return spiWrite((reg & RH_NRF905_REG_MASK) | RH_NRF905_REG_W_CONFIG, val);
}

uint8_t RH_NRF905::spiBurstReadRegister(uint8_t reg, uint8_t* dest, uint8_t len)
{
    return spiBurstRead((reg & RH_NRF905_REG_MASK) | RH_NRF905_REG_R_CONFIG, dest, len);
}

uint8_t RH_NRF905::spiBurstWriteRegister(uint8_t reg, uint8_t* src, uint8_t len)
{
    return spiBurstWrite((reg & RH_NRF905_REG_MASK) | RH_NRF905_REG_W_CONFIG, src, len);
}

uint8_t RH_NRF905::statusRead()
{
    // The status is a byproduct of sending a command
    return spiCommand(0);
}

bool RH_NRF905::setChannel(uint16_t channel, bool hiFrequency)
{
    spiWriteRegister(RH_NRF905_CONFIG_0, channel & RH_NRF905_CONFIG_0_CH_NO);
    // Set or clear the high bit of the channel
    uint8_t bit8 = (channel >> 8) & 0x01;
    uint8_t reg1 = spiReadRegister(RH_NRF905_CONFIG_1);
    reg1 = (reg1 & ~0x01) | bit8;
    // Set or clear the HFREQ_PLL bit
    reg1 &= ~RH_NRF905_CONFIG_1_HFREQ_PLL;
    if (hiFrequency)
	reg1 |= RH_NRF905_CONFIG_1_HFREQ_PLL;
    spiWriteRegister(RH_NRF905_CONFIG_1, reg1);
    return true;
}

bool RH_NRF905::setNetworkAddress(uint8_t* address, uint8_t len)
{
    if (len < 1 || len > 4)
	return false;
    // Set RX_AFW and TX_AFW
    spiWriteRegister(RH_NRF905_CONFIG_2, len | (len << 4));
    spiBurstWrite(RH_NRF905_REG_W_TX_ADDRESS, address, len);
    spiBurstWriteRegister(RH_NRF905_CONFIG_5, address, len);
    return true;
}

bool RH_NRF905::setRF(TransmitPower power)
{
    // Enum definitions of power are the same numerical values as the register
    uint8_t reg1 = spiReadRegister(RH_NRF905_CONFIG_1);
    reg1 &= ~RH_NRF905_CONFIG_1_PA_PWR;
    reg1 |= ((power & 0x3) << 2) & RH_NRF905_CONFIG_1_PA_PWR;
    spiWriteRegister(RH_NRF905_CONFIG_1, reg1);
    return true;
}

void RH_NRF905::setModeIdle()
{
    if (_mode != RHModeIdle)
    {
	digitalWrite(_chipEnablePin, LOW);
	digitalWrite(_txEnablePin, LOW);
	_mode = RHModeIdle;
    }
}

void RH_NRF905::setModeRx()
{
    if (_mode != RHModeRx)
    {
	digitalWrite(_txEnablePin, LOW);
	digitalWrite(_chipEnablePin, HIGH);
	_mode = RHModeRx;
    }
}

void RH_NRF905::setModeTx()
{
    if (_mode != RHModeTx)
    {
	// Its the high transition that puts us into TX mode
	digitalWrite(_txEnablePin, HIGH);
	digitalWrite(_chipEnablePin, HIGH);
	_mode = RHModeTx;
    }
}

bool RH_NRF905::send(const uint8_t* data, uint8_t len)
{
    if (len > RH_NRF905_MAX_MESSAGE_LEN)
	return false;

    if (!waitCAD()) 
	return false;  // Check channel activity

    // Set up the headers
    _buf[0] = _txHeaderTo;
    _buf[1] = _txHeaderFrom;
    _buf[2] = _txHeaderId;
    _buf[3] = _txHeaderFlags;
    _buf[4] = len;
    memcpy(_buf+RH_NRF905_HEADER_LEN, data, len);
    spiBurstWrite(RH_NRF905_REG_W_TX_PAYLOAD, _buf, len + RH_NRF905_HEADER_LEN);
    setModeTx();
    // Radio will return to Standby mode after transmission is complete
    _txGood++;
    return true;
}

bool RH_NRF905::waitPacketSent()
{
    if (_mode != RHModeTx)
	return false;

    while (!(statusRead() & RH_NRF905_STATUS_DR))
	YIELD;
    setModeIdle();
    return true;
}

bool RH_NRF905::isSending()
{
    if (_mode != RHModeTx)
	return false;
    
    return !(statusRead() & RH_NRF905_STATUS_DR);
}

bool RH_NRF905::printRegister(uint8_t reg)
{
#ifdef RH_HAVE_SERIAL
    Serial.print(reg, HEX);
    Serial.print(": ");
    Serial.println(spiReadRegister(reg), HEX);
#endif

    return true;
}

bool RH_NRF905::printRegisters()
{
    uint8_t registers[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};

    uint8_t i;
    for (i = 0; i < sizeof(registers); i++)
	printRegister(registers[i]);
    return true;
}

// Check whether the latest received message is complete and uncorrupted
void RH_NRF905::validateRxBuf()
{
    // Check the length
    uint8_t len = _buf[4];
    if (len > RH_NRF905_MAX_MESSAGE_LEN)
	return; // Silly LEN header

    // Extract the 4 headers
    _rxHeaderTo    = _buf[0];
    _rxHeaderFrom  = _buf[1];
    _rxHeaderId    = _buf[2];
    _rxHeaderFlags = _buf[3];
    if (_promiscuous ||
	_rxHeaderTo == _thisAddress ||
	_rxHeaderTo == RH_BROADCAST_ADDRESS)
    {
	_rxGood++;
	_bufLen = len + RH_NRF905_HEADER_LEN; // _buf still includes the headers
	_rxBufValid = true;
    }
}

bool RH_NRF905::available()
{
    if (!_rxBufValid)
    {
	if (_mode == RHModeTx)
	    return false;
	setModeRx();
	if (!(statusRead() & RH_NRF905_STATUS_DR))
	    return false;
	// Get the message into the RX buffer, so we can inspect the headers
	// we still dont know how long is the user message
	spiBurstRead(RH_NRF905_REG_R_RX_PAYLOAD, _buf, RH_NRF905_MAX_PAYLOAD_LEN);
	validateRxBuf(); 
	if (_rxBufValid)
	    setModeIdle(); // Got one

    }
    return _rxBufValid;
}

void RH_NRF905::clearRxBuf()
{
    _rxBufValid = false;
    _bufLen = 0;
}

bool RH_NRF905::recv(uint8_t* buf, uint8_t* len)
{
    if (!available())
	return false;
    if (buf && len)
    {
	// Skip the 4 headers that are at the beginning of the rxBuf
	if (*len > _bufLen-RH_NRF905_HEADER_LEN)
	    *len = _bufLen-RH_NRF905_HEADER_LEN;
	memcpy(buf, _buf+RH_NRF905_HEADER_LEN, *len);
    }
    clearRxBuf(); // This message accepted and cleared
    return true;
}

uint8_t RH_NRF905::maxMessageLength()
{
    return RH_NRF905_MAX_MESSAGE_LEN;
}
