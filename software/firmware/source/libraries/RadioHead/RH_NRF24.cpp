// NRF24.cpp
//
// Copyright (C) 2012 Mike McCauley
// $Id: RH_NRF24.cpp,v 1.26 2018/01/06 23:50:45 mikem Exp $

#include <RH_NRF24.h>

RH_NRF24::RH_NRF24(uint8_t chipEnablePin, uint8_t slaveSelectPin, RHGenericSPI& spi)
    :
    RHNRFSPIDriver(slaveSelectPin, spi),
    _rxBufValid(0)
{
    _configuration = RH_NRF24_EN_CRC | RH_NRF24_CRCO; // Default: 2 byte CRC enabled
    _chipEnablePin = chipEnablePin;
}

bool RH_NRF24::init()
{
    // Teensy with nRF24 is unreliable at 8MHz:
    // so is Arduino with RF73
    _spi.setFrequency(RHGenericSPI::Frequency1MHz);
    if (!RHNRFSPIDriver::init())
	return false;

    // Initialise the slave select pin
    pinMode(_chipEnablePin, OUTPUT);
    digitalWrite(_chipEnablePin, LOW);
  
    // Clear interrupts
    spiWriteRegister(RH_NRF24_REG_07_STATUS, RH_NRF24_RX_DR | RH_NRF24_TX_DS | RH_NRF24_MAX_RT);
    // Enable dynamic payload length on all pipes
    spiWriteRegister(RH_NRF24_REG_1C_DYNPD, RH_NRF24_DPL_ALL);
    // Enable dynamic payload length, disable payload-with-ack, enable noack
    spiWriteRegister(RH_NRF24_REG_1D_FEATURE, RH_NRF24_EN_DPL | RH_NRF24_EN_DYN_ACK);
    // Test if there is actually a device connected and responding
    // CAUTION: RFM73 and version 2.0 silicon may require ACTIVATE
    if (spiReadRegister(RH_NRF24_REG_1D_FEATURE) != (RH_NRF24_EN_DPL | RH_NRF24_EN_DYN_ACK))
    { 
	spiWrite(RH_NRF24_COMMAND_ACTIVATE, 0x73);
        // Enable dynamic payload length, disable payload-with-ack, enable noack
        spiWriteRegister(RH_NRF24_REG_1D_FEATURE, RH_NRF24_EN_DPL | RH_NRF24_EN_DYN_ACK);
        if (spiReadRegister(RH_NRF24_REG_1D_FEATURE) != (RH_NRF24_EN_DPL | RH_NRF24_EN_DYN_ACK))
            return false;
    }

    clearRxBuf();

    // Make sure we are powered down
    setModeIdle();

    // Flush FIFOs
    flushTx();
    flushRx();

    setChannel(2); // The default, in case it was set by another app without powering down
    setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm);

    return true;
}

// Use the register commands to read and write the registers
uint8_t RH_NRF24::spiReadRegister(uint8_t reg)
{
    return spiRead((reg & RH_NRF24_REGISTER_MASK) | RH_NRF24_COMMAND_R_REGISTER);
}

uint8_t RH_NRF24::spiWriteRegister(uint8_t reg, uint8_t val)
{
    return spiWrite((reg & RH_NRF24_REGISTER_MASK) | RH_NRF24_COMMAND_W_REGISTER, val);
}

uint8_t RH_NRF24::spiBurstReadRegister(uint8_t reg, uint8_t* dest, uint8_t len)
{
    return spiBurstRead((reg & RH_NRF24_REGISTER_MASK) | RH_NRF24_COMMAND_R_REGISTER, dest, len);
}

uint8_t RH_NRF24::spiBurstWriteRegister(uint8_t reg, uint8_t* src, uint8_t len)
{
    return spiBurstWrite((reg & RH_NRF24_REGISTER_MASK) | RH_NRF24_COMMAND_W_REGISTER, src, len);
}

uint8_t RH_NRF24::statusRead()
{
    // status is a side-effect of NOP, faster than reading reg 07
    return spiCommand(RH_NRF24_COMMAND_NOP); 
}

uint8_t RH_NRF24::flushTx()
{
    return spiCommand(RH_NRF24_COMMAND_FLUSH_TX);
}

uint8_t RH_NRF24::flushRx()
{
    return spiCommand(RH_NRF24_COMMAND_FLUSH_RX);
}

bool RH_NRF24::setChannel(uint8_t channel)
{
    spiWriteRegister(RH_NRF24_REG_05_RF_CH, channel & RH_NRF24_RF_CH);
    return true;
}

bool RH_NRF24::setOpMode(uint8_t mode)
{
    _configuration = mode;
    return true;
}

bool RH_NRF24::setNetworkAddress(uint8_t* address, uint8_t len)
{
    if (len < 3 || len > 5)
	return false;

    // Set both TX_ADDR and RX_ADDR_P0 for auto-ack with Enhanced shockwave
    spiWriteRegister(RH_NRF24_REG_03_SETUP_AW, len-2);	// Mapping [3..5] = [1..3]
    spiBurstWriteRegister(RH_NRF24_REG_0A_RX_ADDR_P0, address, len);
    spiBurstWriteRegister(RH_NRF24_REG_10_TX_ADDR, address, len);
    return true;
}

bool RH_NRF24::setRF(DataRate data_rate, TransmitPower power)
{
    uint8_t value = (power << 1) & RH_NRF24_PWR;
    // Ugly mapping of data rates to noncontiguous 2 bits:
    if (data_rate == DataRate250kbps)
	value |= RH_NRF24_RF_DR_LOW;
    else if (data_rate == DataRate2Mbps)
	value |= RH_NRF24_RF_DR_HIGH;
    // else DataRate1Mbps, 00

    // RFM73 needs this:
    value |= RH_NRF24_LNA_HCURR;
    
    spiWriteRegister(RH_NRF24_REG_06_RF_SETUP, value);
    // If we were using auto-ack, we would have to set the appropriate timeout in reg 4 here
    // see NRF24::setRF()
    return true;
}

void RH_NRF24::setModeIdle()
{
    if (_mode != RHModeIdle)
    {
	spiWriteRegister(RH_NRF24_REG_00_CONFIG, _configuration);
	digitalWrite(_chipEnablePin, LOW);
	_mode = RHModeIdle;
    }
}

bool RH_NRF24::sleep()
{
    if (_mode != RHModeSleep)
    {
	spiWriteRegister(RH_NRF24_REG_00_CONFIG, 0); // Power Down mode
	digitalWrite(_chipEnablePin, LOW);
	_mode = RHModeSleep;
	return true;
    }
    return false; // Already there?
}

void RH_NRF24::setModeRx()
{
    if (_mode != RHModeRx)
    {
	spiWriteRegister(RH_NRF24_REG_00_CONFIG, _configuration | RH_NRF24_PWR_UP | RH_NRF24_PRIM_RX);
	digitalWrite(_chipEnablePin, HIGH);
	_mode = RHModeRx;
    }
}

void RH_NRF24::setModeTx()
{
    if (_mode != RHModeTx)
    {
	// Its the CE rising edge that puts us into TX mode
	// CE staying high makes us go to standby-II when the packet is sent
	digitalWrite(_chipEnablePin, LOW);
	// Ensure DS is not set
	spiWriteRegister(RH_NRF24_REG_07_STATUS, RH_NRF24_TX_DS | RH_NRF24_MAX_RT);
	spiWriteRegister(RH_NRF24_REG_00_CONFIG, _configuration | RH_NRF24_PWR_UP);
	digitalWrite(_chipEnablePin, HIGH);
	_mode = RHModeTx;
    }
}

bool RH_NRF24::send(const uint8_t* data, uint8_t len)
{
    if (len > RH_NRF24_MAX_MESSAGE_LEN)
	return false;

    if (!waitCAD()) 
	return false;  // Check channel activity

    // Set up the headers
    _buf[0] = _txHeaderTo;
    _buf[1] = _txHeaderFrom;
    _buf[2] = _txHeaderId;
    _buf[3] = _txHeaderFlags;
    memcpy(_buf+RH_NRF24_HEADER_LEN, data, len);
    spiBurstWrite(RH_NRF24_COMMAND_W_TX_PAYLOAD_NOACK, _buf, len + RH_NRF24_HEADER_LEN);
    setModeTx();
    // Radio will return to Standby II mode after transmission is complete
    _txGood++;
    return true;
}

bool RH_NRF24::waitPacketSent()
{
    // If we are not currently in transmit mode, there is no packet to wait for
    if (_mode != RHModeTx)
	return false;

    // Wait for either the Data Sent or Max ReTries flag, signalling the 
    // end of transmission
    // We dont actually use auto-ack, so prob dont expect to see RH_NRF24_MAX_RT
    uint8_t status;
    uint32_t start = millis();
    while (!((status = statusRead()) & (RH_NRF24_TX_DS | RH_NRF24_MAX_RT)))
    {
	if (((uint32_t)millis() - start) > 100) // Longer than any possible message
	    break;  // Should never happen: TX never completed. Why?
	YIELD;
    }

    // Must clear RH_NRF24_MAX_RT if it is set, else no further comm
    if (status & RH_NRF24_MAX_RT)
	flushTx();
    setModeIdle();
    spiWriteRegister(RH_NRF24_REG_07_STATUS, RH_NRF24_TX_DS | RH_NRF24_MAX_RT);
    // Return true if data sent, false if MAX_RT
    return status & RH_NRF24_TX_DS;
}

bool RH_NRF24::isSending()
{
    return !(spiReadRegister(RH_NRF24_REG_00_CONFIG) & RH_NRF24_PRIM_RX) && 
	   !(statusRead() & (RH_NRF24_TX_DS | RH_NRF24_MAX_RT));
}

bool RH_NRF24::printRegisters()
{
#ifdef RH_HAVE_SERIAL
    // Iterate over register range, but don't process registers not in use.
    for (uint8_t r = RH_NRF24_REG_00_CONFIG; r <= RH_NRF24_REG_1D_FEATURE; r++)
    {
	if ((r <= RH_NRF24_REG_17_FIFO_STATUS) || (r >= RH_NRF24_REG_1C_DYNPD))
	{
	    Serial.print(r, HEX);
	    Serial.print(": ");
	    uint8_t len = 1;
	    // Address registers are 5 bytes in size
	    if (    (RH_NRF24_REG_0A_RX_ADDR_P0 == r)
		    || (RH_NRF24_REG_0B_RX_ADDR_P1 == r)
		    || (RH_NRF24_REG_10_TX_ADDR    == r) )
	    {
		len = 5;
	    }
	    uint8_t buf[5];
	    spiBurstReadRegister(r, buf, len);
	    for (uint8_t j = 0; j < len; ++j)
	    {
		Serial.print(buf[j], HEX);
		Serial.print(" ");
	    }
	    Serial.println("");
	}
    }
#endif

    return true;
}

// Check whether the latest received message is complete and uncorrupted
void RH_NRF24::validateRxBuf()
{
    if (_bufLen < 4)
	return; // Too short to be a real message
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
	_rxBufValid = true;
    }
}

bool RH_NRF24::available()
{
    if (!_rxBufValid)
    {
	if (_mode == RHModeTx)
	    return false;
	setModeRx();
	if (spiReadRegister(RH_NRF24_REG_17_FIFO_STATUS) & RH_NRF24_RX_EMPTY)
	    return false;
	// Manual says that messages > 32 octets should be discarded
	uint8_t len = spiRead(RH_NRF24_COMMAND_R_RX_PL_WID);
	if (len > 32)
	{
	    flushRx();
	    clearRxBuf();
	    setModeIdle();
	    return false;
	}
	// Clear read interrupt
	spiWriteRegister(RH_NRF24_REG_07_STATUS, RH_NRF24_RX_DR);
	// Get the message into the RX buffer, so we can inspect the headers
	spiBurstRead(RH_NRF24_COMMAND_R_RX_PAYLOAD, _buf, len);
	_bufLen = len;
	// 140 microsecs (32 octet payload)
	validateRxBuf(); 
	if (_rxBufValid)
	    setModeIdle(); // Got one
    }
    return _rxBufValid;
}

void RH_NRF24::clearRxBuf()
{
    _rxBufValid = false;
    _bufLen = 0;
}

bool RH_NRF24::recv(uint8_t* buf, uint8_t* len)
{
    if (!available())
	return false;
    if (buf && len)
    {
	// Skip the 4 headers that are at the beginning of the rxBuf
	if (*len > _bufLen-RH_NRF24_HEADER_LEN)
	    *len = _bufLen-RH_NRF24_HEADER_LEN;
	memcpy(buf, _buf+RH_NRF24_HEADER_LEN, *len);
    }
    clearRxBuf(); // This message accepted and cleared
    return true;
}

uint8_t RH_NRF24::maxMessageLength()
{
    return RH_NRF24_MAX_MESSAGE_LEN;
}
