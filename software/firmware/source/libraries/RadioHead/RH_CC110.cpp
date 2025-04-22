// RH_CC110.cpp
//
// Driver for Texas Instruments CC110L transceiver.
//
// Copyright (C) 2016 Mike McCauley
// $Id: RH_CC110.cpp,v 1.11 2020/01/05 07:02:23 mikem Exp $

#include <RH_CC110.h>

// Interrupt vectors for the 3 Arduino interrupt pins
// Each interrupt can be handled by a different instance of RH_CC110, allowing you to have
// 2 or more LORAs per Arduino
RH_CC110* RH_CC110::_deviceForInterrupt[RH_CC110_NUM_INTERRUPTS] = {0, 0, 0};
uint8_t RH_CC110::_interruptCount = 0; // Index into _deviceForInterrupt for next device

// We need 2 tables of modem configuration registers, since some values change depending on the Xtal frequency
// These are indexed by the values of ModemConfigChoice
// Canned modem configurations generated with the TI SmartRF Studio v7 version 2.3.0 on boodgie
// based on the sample 'Typical settings'
// Stored in flash (program) memory to save SRAM
// For 26MHz crystals
PROGMEM static const RH_CC110::ModemConfig MODEM_CONFIG_TABLE_26MHZ[] =
{
    // 0B    0C    10    11    12    15    19    1A    1B    1C    1D    21    22    23    24    25    26    2C    2D    2E
    {0x06, 0x00, 0xf5, 0x83, 0x13, 0x15, 0x16, 0x6c, 0x03, 0x40, 0x91, 0x56, 0x10, 0xe9, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb1_2Fd5_2
    {0x06, 0x00, 0xf6, 0x83, 0x13, 0x15, 0x16, 0x6c, 0x03, 0x40, 0x91, 0x56, 0x10, 0xe9, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb2_4Fd5_2
    {0x06, 0x00, 0xc7, 0x83, 0x13, 0x40, 0x16, 0x6c, 0x43, 0x40, 0x91, 0x56, 0x10, 0xe9, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb4_8Fd25_4
    {0x06, 0x00, 0xc8, 0x93, 0x13, 0x34, 0x16, 0x6c, 0x43, 0x40, 0x91, 0x56, 0x10, 0xe9, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb10Fd19
    {0x06, 0x00, 0xca, 0x83, 0x13, 0x35, 0x16, 0x6c, 0x43, 0x40, 0x91, 0x56, 0x10, 0xe9, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb38_4Fd20
    {0x08, 0x00, 0x7b, 0x83, 0x13, 0x42, 0x1d, 0x1c, 0xc7, 0x00, 0xb2, 0xb6, 0x10, 0xea, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb76_8Fd32
    {0x08, 0x00, 0x5b, 0xf8, 0x13, 0x47, 0x1d, 0x1c, 0xc7, 0x00, 0xb2, 0xb6, 0x10, 0xea, 0x2a, 0x00, 0x1f, 0x81, 0x31, 0x09}, // GFSK_Rb100Fd47
    {0x0c, 0x00, 0x2d, 0x3b, 0x13, 0x62, 0x1d, 0x1c, 0xc7, 0x00, 0xb0, 0xb6, 0x10, 0xea, 0x2a, 0x00, 0x1f, 0x88, 0x31, 0x09}, // GFSK_Rb250Fd127
};

// For 27MHz crystals
PROGMEM static const RH_CC110::ModemConfig MODEM_CONFIG_TABLE_27MHZ[] =
{
    // 0B    0C    10    11    12    15    19    1A    1B    1C    1D    21    22    23    24    25    26    2C    2D    2E
    {0x06, 0x00, 0xf5, 0x75, 0x13, 0x14, 0x16, 0x6c, 0x03, 0x40, 0x91, 0x56, 0x10, 0xe9, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb1_2Fd5_2
    {0x06, 0x00, 0xf6, 0x75, 0x13, 0x14, 0x16, 0x6c, 0x03, 0x40, 0x91, 0x56, 0x10, 0xe9, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb2_4Fd5_2
    {0x06, 0x00, 0xc7, 0x75, 0x13, 0x37, 0x16, 0x6c, 0x43, 0x40, 0x91, 0x56, 0x10, 0xe9, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb4_8Fd25_4
    {0x06, 0x00, 0xc8, 0x84, 0x13, 0x33, 0x16, 0x6c, 0x43, 0x40, 0x91, 0x56, 0x10, 0xe9, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb10Fd19
    {0x06, 0x00, 0xca, 0x75, 0x13, 0x34, 0x16, 0x6c, 0x43, 0x40, 0x91, 0x56, 0x10, 0xe9, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb38_4Fd20
    {0x08, 0x00, 0x7b, 0x75, 0x13, 0x42, 0x1d, 0x1c, 0xc7, 0x00, 0xb2, 0xb6, 0x10, 0xea, 0x2a, 0x00, 0x1f, 0x81, 0x35, 0x09}, // GFSK_Rb76_8Fd32
    {0x08, 0x00, 0x5b, 0xf8, 0x13, 0x47, 0x1d, 0x1c, 0xc7, 0x00, 0xb2, 0xb6, 0x10, 0xea, 0x2a, 0x00, 0x1f, 0x81, 0x31, 0x09}, // GFSK_Rb100Fd47
    {0x0c, 0x00, 0x2d, 0x2f, 0x13, 0x62, 0x1d, 0x1c, 0xc7, 0x00, 0xb0, 0xb6, 0x10, 0xea, 0x2a, 0x00, 0x1f, 0x88, 0x31, 0x09}, // GFSK_Rb250Fd127
};

// These power outputs are based on the suggested optimum values for 
// multilayer inductors in the 915MHz frequency band. Per table 5-15.
// Yes these are not linear.
// Caution: this table is indexed by the values of enum TransmitPower
// Do not change one without changing the other.
// If you do not like these values, use setPaTable() directly.
PROGMEM static const uint8_t paPowerValues[] = 
{
    0x03, // -30dBm
    0x0e, // -20dBm
    0x1e, // -15dBm
    0x27, // -10dBm
    0x8e, // 0dBm
    0xcd, // 5dBm
    0xc7, // 7dBm
    0xc0, // 10dBm
};

RH_CC110::RH_CC110(uint8_t slaveSelectPin, uint8_t interruptPin, bool is27MHz, RHGenericSPI& spi)
    :
    RHNRFSPIDriver(slaveSelectPin, spi),
    _rxBufValid(false),
    _is27MHz(is27MHz)
{
    _interruptPin = interruptPin;
    _myInterruptIndex = 0xff; // Not allocated yet
}

bool RH_CC110::init()
{
    if (!RHNRFSPIDriver::init())
	return false;

    // Determine the interrupt number that corresponds to the interruptPin
    int interruptNumber = digitalPinToInterrupt(_interruptPin);
    if (interruptNumber == NOT_AN_INTERRUPT)
	return false;
#ifdef RH_ATTACHINTERRUPT_TAKES_PIN_NUMBER
    interruptNumber = _interruptPin;
#endif

    // Tell the low level SPI interface we will use SPI within this interrupt
    spiUsingInterrupt(interruptNumber);

    // Reset the chip
    // Strobe the reset
    uint8_t val = spiCommand(RH_CC110_STROBE_30_SRES); // Reset
    delay(100);
    val = spiCommand(RH_CC110_STROBE_36_SIDLE); // IDLE
    if (val != 0x0f)
	return false; // No chip there or reset failed.

    // Add by Adrien van den Bossche <vandenbo@univ-tlse2.fr> for Teensy
    // ARM M4 requires the below. else pin interrupt doesn't work properly.
    // On all other platforms, its innocuous, belt and braces
    pinMode(_interruptPin, INPUT); 

    // Set up interrupt handler
    // Since there are a limited number of interrupt glue functions isr*() available,
    // we can only support a limited number of devices simultaneously
    // ON some devices, notably most Arduinos, the interrupt pin passed in is actuallt the 
    // interrupt number. You have to figure out the interruptnumber-to-interruptpin mapping
    // yourself based on knwledge of what Arduino board you are running on.
    if (_myInterruptIndex == 0xff)
    {
	// First run, no interrupt allocated yet
	if (_interruptCount <= RH_CC110_NUM_INTERRUPTS)
	    _myInterruptIndex = _interruptCount++;
	else
	    return false; // Too many devices, not enough interrupt vectors
    }
    _deviceForInterrupt[_myInterruptIndex] = this;
    if (_myInterruptIndex == 0)
	attachInterrupt(interruptNumber, isr0, RISING);
    else if (_myInterruptIndex == 1)
	attachInterrupt(interruptNumber, isr1, RISING);
    else if (_myInterruptIndex == 2)
	attachInterrupt(interruptNumber, isr2, RISING);
    else
	return false; // Too many devices, not enough interrupt vectors

    spiWriteRegister(RH_CC110_REG_02_IOCFG0, RH_CC110_GDO_CFG_CRC_OK_AUTORESET);  // gdo0 interrupt on CRC_OK
    spiWriteRegister(RH_CC110_REG_06_PKTLEN, RH_CC110_MAX_PAYLOAD_LEN); // max packet length
    spiWriteRegister(RH_CC110_REG_07_PKTCTRL1, RH_CC110_CRC_AUTOFLUSH | RH_CC110_APPEND_STATUS); // append status, crc autoflush, no addr check
    spiWriteRegister(RH_CC110_REG_08_PKTCTRL0, RH_CC110_PKT_FORMAT_NORMAL | RH_CC110_CRC_EN | RH_CC110_LENGTH_CONFIG_VARIABLE);
    spiWriteRegister(RH_CC110_REG_13_MDMCFG1, RH_CC110_NUM_PREAMBLE_4); // 4 preamble bytes, chan spacing not used
    spiWriteRegister(RH_CC110_REG_17_MCSM1, RH_CC110_CCA_MODE_RSSI_PACKET | RH_CC110_RXOFF_MODE_RX | RH_CC110_TXOFF_MODE_IDLE);
    spiWriteRegister(RH_CC110_REG_18_MCSM0, RH_CC110_FS_AUTOCAL_FROM_IDLE | RH_CC110_PO_TIMEOUT_64); // cal when going to tx or rx
    spiWriteRegister(RH_CC110_REG_20_WORCTRL, 0xfb); // from smartrf
    spiWriteRegister(RH_CC110_REG_29_FSTEST, 0x59); // from smartrf
    spiWriteRegister(RH_CC110_REG_2A_PTEST, 0x7f); // from smartrf
    spiWriteRegister(RH_CC110_REG_2B_AGCTEST, 0x3f); // from smartrf

    // Set some reasonable default values
    uint8_t syncWords[] = { 0xd3, 0x91 };
    setSyncWords(syncWords, sizeof(syncWords));
    setTxPower(TransmitPower5dBm);
    setFrequency(915.0);
    setModemConfig(GFSK_Rb1_2Fd5_2);
    return true;
}

void RH_CC110::setIs27MHz(bool is27MHz)
{
    _is27MHz = is27MHz;
}

// C++ level interrupt handler for this instance
// We use this to get RxDone and TxDone interrupts
void RH_CC110::handleInterrupt()
{
//    Serial.println("I");
    if (_mode == RHModeRx)
    {
	// Radio is configured to stay in RX until we move it to IDLE after a CRC_OK message for us
	// We only get interrupts in RX mode, on CRC_OK

#if 0
	// SIGH: reading RH_CC110_REG_34_RSSI here does not yield the RSSI of when the packet was received
	// Must get it from the end of the packet read
	uint8_t raw_rssi = spiBurstReadRegister(RH_CC110_REG_34_RSSI); // Was set when sync word was detected
	// Conversion of RSSI value to received power level in dBm per TI section 5.18.2
	if (raw_rssi >= 128) 
	    _lastRssi = (((int16_t)raw_rssi - 256) / 2) - 74;
	else 
	    _lastRssi = ((int16_t)raw_rssi / 2) - 74;
#endif
	
	_bufLen = spiReadRegister(RH_CC110_REG_3F_FIFO);
	if (_bufLen < 4)
	{
	    // Something wrong there, flush the FIFO
	    spiCommand(RH_CC110_STROBE_3A_SFRX);
	    clearRxBuf();
	    return;
	}
	
	// Because we have RH_CC110_APPEND_STATUS set, we can read another 2 octets, the RSSI and the CRC state
	spiBurstRead(RH_CC110_REG_3F_FIFO | RH_CC110_SPI_BURST_MASK | RH_CC110_SPI_READ_MASK, _buf, _bufLen + 2);
	// And then decode the RSSI
	int  raw_rssi = _buf[_bufLen];
	if (raw_rssi >= 128) 
	    _lastRssi = (((int16_t)raw_rssi - 256) / 2) - 74;
	else 
	    _lastRssi = ((int16_t)raw_rssi / 2) - 74;
	
	// All good so far. See if its for us
	validateRxBuf(); 
	if (_rxBufValid)
	    setModeIdle(); // Done
    }
}

// These are low level functions that call the interrupt handler for the correct
// instance of RH_CC110.
// 3 interrupts allows us to have 3 different devices
void RH_INTERRUPT_ATTR RH_CC110::isr0()
{
    if (_deviceForInterrupt[0])
	_deviceForInterrupt[0]->handleInterrupt();
}
void RH_INTERRUPT_ATTR RH_CC110::isr1()
{
    if (_deviceForInterrupt[1])
	_deviceForInterrupt[1]->handleInterrupt();
}
void RH_INTERRUPT_ATTR RH_CC110::isr2()
{
    if (_deviceForInterrupt[2])
	_deviceForInterrupt[2]->handleInterrupt();
}

uint8_t RH_CC110::spiReadRegister(uint8_t reg)
{
    return spiRead((reg & 0x3f) | RH_CC110_SPI_READ_MASK);
}

uint8_t RH_CC110::spiBurstReadRegister(uint8_t reg)
{
    return spiRead((reg & 0x3f) | RH_CC110_SPI_READ_MASK | RH_CC110_SPI_BURST_MASK);
}

uint8_t RH_CC110::spiWriteRegister(uint8_t reg, uint8_t val)
{
    return spiWrite((reg & 0x3f), val);
}

uint8_t  RH_CC110::spiBurstWriteRegister(uint8_t reg, const uint8_t* src, uint8_t len)
{
    return spiBurstWrite((reg & 0x3f) | RH_CC110_SPI_BURST_MASK, src, len);
}

bool RH_CC110::printRegisters()
{
#ifdef RH_HAVE_SERIAL
    uint8_t i;
    for (i = 0; i <= 0x2f; i++)
    {
	Serial.print(i, HEX);
	Serial.print(": ");
	Serial.println(spiReadRegister(i), HEX);
    }
    // Burst registers
    for (i = 0x30; i <= 0x3e; i++)
    {
	Serial.print(i, HEX);
	Serial.print(": ");
	Serial.println(spiBurstReadRegister(i), HEX);
    }
#endif
    return true;
}

// Check whether the latest received message is complete and uncorrupted
void RH_CC110::validateRxBuf()
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

bool RH_CC110::available()
{
    if (_mode == RHModeTx)
	return false;
    if (_rxBufValid) // Will be set by the interrupt handler when a good message is received
	return true;
    setModeRx(); // Make sure we are receiving
    return false; // Nothing yet
}

void RH_CC110::clearRxBuf()
{
    ATOMIC_BLOCK_START;
    _rxBufValid = false;
    _bufLen = 0;
    ATOMIC_BLOCK_END;
}

bool RH_CC110::recv(uint8_t* buf, uint8_t* len)
{
    if (!available())
	return false;

    if (buf && len)
    {
	ATOMIC_BLOCK_START;
	// Skip the 4 headers that are at the beginning of the rxBuf
	if (*len > _bufLen - RH_CC110_HEADER_LEN)
	    *len = _bufLen - RH_CC110_HEADER_LEN;
	memcpy(buf, _buf + RH_CC110_HEADER_LEN, *len);
	ATOMIC_BLOCK_END;
    }
    clearRxBuf(); // This message accepted and cleared

    return true;
}

bool RH_CC110::send(const uint8_t* data, uint8_t len)
{
    if (len > RH_CC110_MAX_MESSAGE_LEN)
	return false;

    waitPacketSent(); // Make sure we dont interrupt an outgoing message
    setModeIdle();

    if (!waitCAD()) 
	return false;  // Check channel activity

    spiWriteRegister(RH_CC110_REG_3F_FIFO, len + RH_CC110_HEADER_LEN);
    spiWriteRegister(RH_CC110_REG_3F_FIFO,_txHeaderTo);
    spiWriteRegister(RH_CC110_REG_3F_FIFO,_txHeaderFrom);
    spiWriteRegister(RH_CC110_REG_3F_FIFO,_txHeaderId);
    spiWriteRegister(RH_CC110_REG_3F_FIFO,_txHeaderFlags);
    spiBurstWriteRegister(RH_CC110_REG_3F_FIFO, data, len);

    // Radio returns to Idle when TX is finished
    // need waitPacketSent() to detect change of _mode and TX completion
    setModeTx();

    return true;
}

uint8_t RH_CC110::maxMessageLength()
{
    return RH_CC110_MAX_MESSAGE_LEN;
}

void RH_CC110::handleOverFlows(uint8_t status)
{
    spiCommand(RH_CC110_STROBE_3A_SFRX);
    //Handle RX and TX overflows so we don't get stuck in either state
    if( (status&RH_CC110_STATUS_RXFIFO_OVERFLOW) == RH_CC110_STATUS_RXFIFO_OVERFLOW ) {
        spiCommand(RH_CC110_STROBE_3A_SFRX);
        clearRxBuf();
    }
    else if( (status&RH_CC110_STATUS_TXFIFO_UNDERFLOW) == RH_CC110_STATUS_TXFIFO_UNDERFLOW ) {
        spiCommand(RH_CC110_STROBE_3B_SFTX);
    }
}

void RH_CC110::setModeIdle()
{
    if (_mode != RHModeIdle)
    {
        uint8_t status = spiCommand(RH_CC110_STROBE_36_SIDLE);
        _mode = RHModeIdle;
        handleOverFlows(status);
    }
}

bool RH_CC110::sleep()
{
    if (_mode != RHModeSleep)
    {
	spiCommand(RH_CC110_STROBE_36_SIDLE); //preceeding sleep IDLE first
	spiCommand(RH_CC110_STROBE_39_SPWD);
	_mode = RHModeSleep;
    }
    return true;
}

void RH_CC110::setModeRx()
{
    if (_mode != RHModeRx)
    {
	// Radio is configuewd to stay in RX mode
	// only receipt of a CRC_OK wil cause us to return it to IDLE
	spiCommand(RH_CC110_STROBE_34_SRX);
	_mode = RHModeRx;
    }
}

void RH_CC110::setModeTx()
{
    if (_mode != RHModeTx)
    {
	spiCommand(RH_CC110_STROBE_35_STX);
	_mode = RHModeTx;
    }
}

uint8_t RH_CC110::statusRead()
{	
    uint8_t status = spiCommand(RH_CC110_STROBE_3D_SNOP);
    handleOverFlows(status);
    return status;
}

// Sigh, this chip has no TXDONE type interrupt, so we have to poll
bool RH_CC110::waitPacketSent()
{
    // If we are not currently in transmit mode, there is no packet to wait for
    if (_mode != RHModeTx)
	return false;

    // Caution: may transition through CALIBRATE
    while ((statusRead() & RH_CC110_STATUS_STATE) != RH_CC110_STATUS_IDLE)
	YIELD;

    _mode = RHModeIdle;
    return true;
}

bool RH_CC110::setTxPower(TransmitPower power)
{
    if (power > sizeof(paPowerValues))
	return false;

    uint8_t patable[2];
    memcpy_P(&patable[0], (void*)&paPowerValues[power], sizeof(uint8_t));
    patable[1] = 0x00;
    setPaTable(patable, sizeof(patable));
    return true;
}

void RH_CC110::setPaTable(uint8_t* patable, uint8_t patablesize)
{
    spiBurstWriteRegister(RH_CC110_REG_3E_PATABLE, patable, patablesize);
}

bool RH_CC110::setFrequency(float centre)
{
    // From section 5.21: fcarrier = fxosc / 2^16 * FREQ
    uint32_t FREQ;
    float fxosc = _is27MHz ? 27.0 : 26.0;
    FREQ = (uint32_t)(centre * 65536 / fxosc);
    // Some trivial checks
    if (FREQ & 0xff000000)
	return false;
    spiWriteRegister(RH_CC110_REG_0D_FREQ2, (FREQ >> 16) & 0xff);
    spiWriteRegister(RH_CC110_REG_0E_FREQ1, (FREQ >> 8) & 0xff);
    spiWriteRegister(RH_CC110_REG_0F_FREQ0, FREQ & 0xff);

    // Radio is configured to calibrate automatically whenever it enters RX or TX mode
    // so no need to check for PLL lock here
    return true;
}

// Sets registers from a canned modem configuration structure
void RH_CC110::setModemRegisters(const ModemConfig* config)
{
    spiWriteRegister(RH_CC110_REG_0B_FSCTRL1,  config->reg_0b);
    spiWriteRegister(RH_CC110_REG_0C_FSCTRL0,  config->reg_0c);
    spiWriteRegister(RH_CC110_REG_10_MDMCFG4,  config->reg_10);
    spiWriteRegister(RH_CC110_REG_11_MDMCFG3,  config->reg_11);
    spiWriteRegister(RH_CC110_REG_12_MDMCFG2,  config->reg_12);
    spiWriteRegister(RH_CC110_REG_15_DEVIATN,  config->reg_15);
    spiWriteRegister(RH_CC110_REG_19_FOCCFG,   config->reg_19);
    spiWriteRegister(RH_CC110_REG_1A_BSCFG,    config->reg_1a);
    spiWriteRegister(RH_CC110_REG_1B_AGCCTRL2, config->reg_1b);
    spiWriteRegister(RH_CC110_REG_1C_AGCCTRL1, config->reg_1c);
    spiWriteRegister(RH_CC110_REG_1D_AGCCTRL0, config->reg_1d);
    spiWriteRegister(RH_CC110_REG_21_FREND1,   config->reg_21);
    spiWriteRegister(RH_CC110_REG_22_FREND0,   config->reg_22);
    spiWriteRegister(RH_CC110_REG_23_FSCAL3,   config->reg_23);
    spiWriteRegister(RH_CC110_REG_24_FSCAL2,   config->reg_24);
    spiWriteRegister(RH_CC110_REG_25_FSCAL1,   config->reg_25);
    spiWriteRegister(RH_CC110_REG_26_FSCAL0,   config->reg_26);
    spiWriteRegister(RH_CC110_REG_2C_TEST2,    config->reg_2c);
    spiWriteRegister(RH_CC110_REG_2D_TEST1,    config->reg_2d);
    spiWriteRegister(RH_CC110_REG_2E_TEST0,    config->reg_2e);
}

// Set one of the canned Modem configs
// Returns true if its a valid choice
bool RH_CC110::setModemConfig(ModemConfigChoice index)
{
    if (index > (signed int)(sizeof(MODEM_CONFIG_TABLE_27MHZ) / sizeof(ModemConfig)))
        return false;

    const RH_CC110::ModemConfig *p = _is27MHz ? MODEM_CONFIG_TABLE_27MHZ : MODEM_CONFIG_TABLE_26MHZ ;
    RH_CC110::ModemConfig cfg;
    memcpy_P(&cfg, p + index, sizeof(RH_CC110::ModemConfig));
    setModemRegisters(&cfg);

    return true;
}

void RH_CC110::setSyncWords(const uint8_t* syncWords, uint8_t len)
{
    if (!syncWords || len != 2)
	return; // Only 2 byte sync words are supported

    spiWriteRegister(RH_CC110_REG_04_SYNC1, syncWords[0]);
    spiWriteRegister(RH_CC110_REG_05_SYNC0, syncWords[1]);
}
