// RH_SX126x.cpp
//
// Copyright (C) 2023 Mike McCauley
// $Id: RH_SX126x.cpp,v 1.27 2020/07/05 08:52:21 mikem Exp $
//
// UNFINISHED. TODO:
// power setting for different device types
// specific subclasses for 1261, 1262, 1268?

#include <RH_SX126x.h>

// Maybe a mutex for multithreading on Raspberry Pi?
#ifdef RH_USE_MUTEX
RH_DECLARE_MUTEX(lock);
#endif

// Interrupt vectors for the 3 Arduino interrupt pins
// Each interrupt can be handled by a different instance of RH_SX126x, allowing you to have
// 2 or more LORAs per Arduino
RH_SX126x* RH_SX126x::_deviceForInterrupt[RH_SX126x_NUM_INTERRUPTS] = {0, 0, 0};

// These are indexed by the values of ModemConfigChoice
// Stored in flash (program) memory to save SRAM
PROGMEM static const RH_SX126x::ModemConfig MODEM_CONFIG_TABLE[] =
{
    //  packetType, p1, p2, p3, p4, p5, p6, p7, p8
     // LoRa_Bw125Cr45Sf128 Works with RH_RF95
    { RH_SX126x::PacketTypeLoRa, RH_SX126x_LORA_SF_128, RH_SX126x_LORA_BW_125_0, RH_SX126x_LORA_CR_4_5, RH_SX126x_LORA_LOW_DATA_RATE_OPTIMIZE_OFF, 0, 0, 0, 0},
    // LoRa_Bw500Cr45Sf128 Works with RH_RF95
    { RH_SX126x::PacketTypeLoRa, RH_SX126x_LORA_SF_128, RH_SX126x_LORA_BW_500_0, RH_SX126x_LORA_CR_4_5, RH_SX126x_LORA_LOW_DATA_RATE_OPTIMIZE_OFF, 0, 0, 0, 0},

    // LoRa_Bw31_25Cr48Sf512 no interop with RH_RF95 WHY????
    { RH_SX126x::PacketTypeLoRa, RH_SX126x_LORA_SF_512, RH_SX126x_LORA_BW_31_25, RH_SX126x_LORA_CR_4_8, RH_SX126x_LORA_LOW_DATA_RATE_OPTIMIZE_ON, 0, 0, 0, 0},

    // LoRa_Bw125Cr48Sf4096 Works with RH_RF95
    { RH_SX126x::PacketTypeLoRa, RH_SX126x_LORA_SF_4096, RH_SX126x_LORA_BW_125_0, RH_SX126x_LORA_CR_4_8, RH_SX126x_LORA_LOW_DATA_RATE_OPTIMIZE_ON, 0, 0, 0, 0},
    
    // LoRa_Bw125Cr45Sf2048 Works with RH_RF95
    { RH_SX126x::PacketTypeLoRa, RH_SX126x_LORA_SF_2048, RH_SX126x_LORA_BW_125_0, RH_SX126x_LORA_CR_4_5, RH_SX126x_LORA_LOW_DATA_RATE_OPTIMIZE_ON, 0, 0, 0, 0},
};

RH_SX126x::RH_SX126x(uint8_t slaveSelectPin, uint8_t interruptPin,  uint8_t busyPin, uint8_t resetPin,
		     RHGenericSPI& spi, RadioPinConfig* radioPinConfig)
    :
    RHSPIDriver(slaveSelectPin, spi),
    _rxBufValid(0)
{
    _interruptPin = interruptPin;
    _busyPin = busyPin;
    _resetPin = resetPin;
    _myInterruptIndex = 0xff; // Not allocated yet
    _enableCRC = true;
    // There should be (but may not be) a configuration structure toi tell us how to manage the
    // radio RF switch control pins
    setRadioPinConfig(radioPinConfig);
}

bool RH_SX126x::init()
{
    if (!RHSPIDriver::init())
	return false;

#ifdef RH_USE_MUTEX
    if (RH_MUTEX_INIT(lock) != 0)
    { 
    	Serial.println("\n mutex init has failed\n");
    	return false;
    }
#endif

    if (!setupInterruptHandler())
	return false;

    // Reset the radio, if we know the reset pin:
    if (_resetPin != RH_INVALID_PIN)
    {
	pinMode(_resetPin, OUTPUT);
	digitalWrite(_resetPin, HIGH);
	digitalWrite(_resetPin, LOW);
	delay(2);
	digitalWrite(_resetPin, HIGH);
	// Expect to be busy for about 1ms after this
	delay(200); // After reset: else can get runt transmission during startup
    }

    // Configure the BUSY pin if available
    if (_busyPin != RH_INVALID_PIN)
	pinMode(_busyPin, INPUT);

    // No way to check the device type :-(
    // Can read data like 'SX1261 TKF 1A11', at register 0x0320 RH_SX126x_REG_VERSION_STRING but it does not change for SX1261
    
    // Get the current status and guess if we are connected
    uint8_t status = getStatus();
    if (status == 0x00 || status == 0xff) // Should never get this: probably not connected by SPI
        return false;
    setModeIdle();
    
    setRegulatorMode(RH_SX126x_REGULATOR_DC_DC); // == SMPS mode
    clearDeviceErrors();
    setRxFallbackMode(RH_SX126x_RX_TX_FALLBACK_MODE_STDBY_RC);
    calibrate(RH_SX126x_CALIBRATE_ALL); // All blocks get (re)calibrated when setFrequency() is called with calibrate true
    // This LoRa Sync word 0x1424 is compatible with single byte 0x12 default for RH_RF95.
    // https://forum.lora-developers.semtech.com/t/sx1272-and-sx1262-lora-sync-word-compatibility/988/13
    setLoRaSyncWord(0x1424);

    // You may need to change these after init to suit your radio and its wiring:
    setDIO2AsRfSwitchCtrl(true); // Use the radios DIO2 pin control the transmitter. This is common in 3rd party RF modules
    setTCXO(3.3, 100); // Enable the TCXO. Typical values for the control voltage and delay

    // These are the interrupts we are willing to process
    setDioIrqParams(_irqMask, _irqMask, RH_SX126x_IRQ_NONE, RH_SX126x_IRQ_NONE);

    // Set up default configuration
    setModemConfig(LoRa_Bw125Cr45Sf128); // Radio default
//    setModemConfig(LoRa_Bw125Cr48Sf4096); // slow and reliable?
    // Default preamble length is 8, so dont set it here
    // An innocuous ISM frequency, in the HF range for the WiO-E5, compatible with RH_RF95
    setFrequency(915.0);
    // Lowish power
    setTxPower(13);

    return true;
}

void RH_SX126x::enableCrcErrorIrq(bool enable)
{
    if ( enable )
        _irqMask |= RH_SX126x_IRQ_CRC_ERR;        
    else
        _irqMask &= ~RH_SX126x_IRQ_CRC_ERR;
    setDioIrqParams(_irqMask, _irqMask, RH_SX126x_IRQ_NONE, RH_SX126x_IRQ_NONE);    
}

void RH_SX126x::enableRawMode(bool enable)
{
    _raw = enable;   
}

// Some subclasses may need to override
bool RH_SX126x::setupInterruptHandler()
{
    // For some subclasses (eg RH_ABZ)  we dont want to set up interrupt
    int interruptNumber = NOT_AN_INTERRUPT;
    if (_interruptPin != RH_INVALID_PIN)
    {
	// Determine the interrupt number that corresponds to the interruptPin
	interruptNumber = digitalPinToInterrupt(_interruptPin);
	if (interruptNumber == NOT_AN_INTERRUPT)
	    return false;
#ifdef RH_ATTACHINTERRUPT_TAKES_PIN_NUMBER
	interruptNumber = _interruptPin;
#endif

	// Tell the low level SPI interface we will use SPI within this interrupt
	spiUsingInterrupt(interruptNumber);
    }

    if (_interruptPin != RH_INVALID_PIN)
    {
	// Add by Adrien van den Bossche <vandenbo@univ-tlse2.fr> for Teensy
	// ARM M4 requires the below. else pin interrupt doesn't work properly.
	// On all other platforms, its innocuous, belt and braces
	pinMode(_interruptPin, INPUT); 
	
	// Set up interrupt handler
	// Since there are a limited number of interrupt glue functions isr*() available,
	// we can only support a limited number of devices simultaneously
	// ON some devices, notably most Arduinos, the interrupt pin passed in is actually the 
	// interrupt number. You have to figure out the interruptnumber-to-interruptpin mapping
	// yourself based on knwledge of what Arduino board you are running on.
	if (_myInterruptIndex == 0xff)
	{
	    static uint8_t interruptCount = 0; // Index into _deviceForInterrupt for next device
	    // First run, no interrupt allocated yet
	    if (interruptCount <= RH_SX126x_NUM_INTERRUPTS)
		_myInterruptIndex = interruptCount++;
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
    }
    return true;
}

// C++ level interrupt handler for this instance
// LORA is unusual in that it has several interrupt lines, and not a single, combined one.
// On MiniWirelessLoRa, only one of the several interrupt lines (DI0) from the RFM95 is usefuly 
// connnected to the processor.
// We use this to get RxDone and TxDone interrupts
void RH_SX126x::handleInterrupt()
{
    RH_MUTEX_LOCK(lock); // Multithreading support
    uint16_t interrupts = getIrqStatus();
    _lastirq = interrupts;
    _iflag = true; // Debugging
    clearIrqStatus(interrupts);
//    Serial.print("int: ");
//    Serial.println(interrupts, HEX);
    if (_mode == RHModeRx && (interrupts & (RH_SX126x_IRQ_CRC_ERR | RH_SX126x_IRQ_HEADER_ERR)))
    {
	// CrcErr HeaderErr
	_rxBad++;
        clearRxBuf();
	// If there was an error, the SX126x is now in standby mode. Need to force RH state back to idle too
	_mode = RHModeIdle;
//	setModeRx(); // Keep trying?
    }
    else if (_mode == RHModeRx && (interrupts & RH_SX126x_IRQ_RX_DONE))
    {
	// RxDone
	// Should now be in STDBY
	// Get received packet length
	uint8_t rxbufferstatus[2]; // PayloadLengthRx, RxStartBufferPointer
	getCommand(RH_SX126x_CMD_GET_RX_BUFFER_STATUS, rxbufferstatus, sizeof(rxbufferstatus));
	_bufLen        = rxbufferstatus[0];
	uint8_t offset = rxbufferstatus[1];
	// Get the packet
	readBuffer(offset, _buf, _bufLen);

	uint8_t packetstatus[3];
	getCommand(RH_SX126x_CMD_GET_PKT_STATUS, packetstatus, sizeof(packetstatus));
	if (_packetType == PacketTypeLoRa)
	{
	    _lastRssi = -(packetstatus[0] / 2); // dBm
	    _lastSNR = (packetstatus[1] / 4);
	}
	else if (_packetType == PacketTypeGFSK)
	{
	    _lastRssi = -(packetstatus[2] / 2); // dBm
	    _lastSNR = 0; // Unobtainable
	}
	
	validateRxBuf(); 
	if (_rxBufValid)
	    setModeIdle(); // Got one 
    }
    else if (_mode == RHModeTx && (interrupts & RH_SX126x_IRQ_TX_DONE))
    {
	// TxDone
	// Should now be in STDBY
	_txGood++;
	setModeIdle();
    }
    else if (_mode == RHModeCad && (interrupts & RH_SX126x_IRQ_CAD_DONE))
    {
//	Serial.println("caddone");
	// CadDone
	// Should now be in STDBY_RX
	setModeIdle();
    }
    else if (_mode == RHModeCad && (interrupts & RH_SX126x_IRQ_CAD_DETECTED))
    {
	// CadDetected: there was activity
//	Serial.println("caddetected");
	_cad = true;
	setModeIdle();
    }
    RH_MUTEX_UNLOCK(lock); 
}

// These are low level functions that call the interrupt handler for the correct
// instance of RH_SX126x.
// 3 interrupts allows us to have 3 different devices
void RH_INTERRUPT_ATTR RH_SX126x::isr0()
{
    if (_deviceForInterrupt[0])
	_deviceForInterrupt[0]->handleInterrupt();
}
void RH_INTERRUPT_ATTR RH_SX126x::isr1()
{
    if (_deviceForInterrupt[1])
	_deviceForInterrupt[1]->handleInterrupt();
}
void RH_INTERRUPT_ATTR RH_SX126x::isr2()
{
    if (_deviceForInterrupt[2])
	_deviceForInterrupt[2]->handleInterrupt();
}

// Check whether the latest received message is complete and uncorrupted
void RH_SX126x::validateRxBuf()
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

bool RH_SX126x::available()
{
    RH_MUTEX_LOCK(lock); // Multithreading support
    if (_mode == RHModeTx)
    {
    	RH_MUTEX_UNLOCK(lock);
	return false;
    }
    setModeRx();
    RH_MUTEX_UNLOCK(lock);
    return _rxBufValid; // Will be set by the interrupt handler when a good message is received
}

void RH_SX126x::clearRxBuf()
{
    waitUntilNotBusy();
    ATOMIC_BLOCK_START;
    _rxBufValid = false;
    _bufLen = 0;
    ATOMIC_BLOCK_END;
}

bool RH_SX126x::recv(uint8_t* buf, uint8_t* len)
{
    uint8_t hdr_len = RH_SX126x_HEADER_LEN;
    if (!available())
	return false;
    if ( _raw )
	hdr_len = 0;
    if (buf && len)
    {
	ATOMIC_BLOCK_START;
	// Skip the 4 headers that are at the beginning of the rxBuf
	if (*len > _bufLen-hdr_len)
	    *len = _bufLen-hdr_len;
	memcpy(buf, _buf+hdr_len, *len);
	ATOMIC_BLOCK_END;
    }
    clearRxBuf(); // This message accepted and cleared
    RH_MUTEX_UNLOCK(lock);
    return true;
}

bool RH_SX126x::send(const uint8_t* data, uint8_t len)
{
    if (len > RH_SX126x_MAX_MESSAGE_LEN)
	return false;

    waitPacketSent(); // Make sure we dont interrupt an outgoing message
    setModeIdle();
    
    if (!waitCAD()) 
	return false;  // Check channel activity

    setBufferBaseAddress(0, 0);
    // The headers
    if ( _raw )
    {
    // The message data
    writeBuffer(0, data, len);
    setPacketParametersLoRa(len); // Tell modem how much to send
    }
    else
    {
    uint8_t headers[RH_SX126x_HEADER_LEN] = {_txHeaderTo, _txHeaderFrom, _txHeaderId, _txHeaderFlags };
    writeBuffer(0, headers, sizeof(headers));

    // The message data
    writeBuffer(RH_SX126x_HEADER_LEN, data, len);
    setPacketParametersLoRa(len + RH_SX126x_HEADER_LEN); // Tell modem how much to send
    }

    RH_MUTEX_LOCK(lock); // Multithreading support
    setModeTx(); // Start the transmitter
    RH_MUTEX_UNLOCK(lock);

    // Hmmmm, some chips fail to enter TX mode the first time,and need to be tried again
    // Why?
    if (getStatus() == 0x2a)
    {
	// Retry Tx mode directly
	RH_MUTEX_LOCK(lock); // Multithreading support
	setTx(RH_SX126x_RX_TIMEOUT_NONE); // Timeout 0
	RH_MUTEX_UNLOCK(lock);
    }
	
    // when Tx is done, interruptHandler will fire and radio mode will return to STANDBY
    return true;
}


uint8_t RH_SX126x::maxMessageLength()
{
    return RH_SX126x_MAX_MESSAGE_LEN;
}

bool RH_SX126x::setFrequency(float centre, bool calibrate)
{
    if (calibrate)
    {
        if (centre > 900.0)
            calibrateImage(RH_SX126x_CAL_IMG_902_MHZ_1, RH_SX126x_CAL_IMG_902_MHZ_2);
        else if (centre > 850.0)
            calibrateImage(RH_SX126x_CAL_IMG_863_MHZ_1, RH_SX126x_CAL_IMG_863_MHZ_2);
        else if (centre > 770.0)
            calibrateImage(RH_SX126x_CAL_IMG_779_MHZ_1, RH_SX126x_CAL_IMG_779_MHZ_2);
        else if (centre > 460.0)
            calibrateImage(RH_SX126x_CAL_IMG_470_MHZ_1, RH_SX126x_CAL_IMG_470_MHZ_2);
        else
            calibrateImage(RH_SX126x_CAL_IMG_430_MHZ_1, RH_SX126x_CAL_IMG_430_MHZ_2);
    }
    
    // Frf = FRF / FSTEP
    uint32_t frf = (centre * 1000000.0) / RH_SX126x_FSTEP; 
    
    uint8_t setting[] =
        {static_cast<uint8_t>(frf >> 24),
         static_cast<uint8_t>(frf >> 16),
         static_cast<uint8_t>(frf >> 8),
         static_cast<uint8_t>(frf)};
    return sendCommand(RH_SX126x_CMD_SET_RF_FREQUENCY, setting, sizeof(setting));
}

void RH_SX126x::setModeIdle()
{
    if (_mode != RHModeIdle)
    {
	modeWillChange(RHModeIdle);
	setStandby(RH_SX126x_STANDBY_RC);
	_mode = RHModeIdle;
    }
}

bool RH_SX126x::sleep()
{
    if (_mode != RHModeSleep)
    {
	modeWillChange(RHModeSleep);
	setSleep(RH_SX126x_SLEEP_START_WARM);
	_mode = RHModeSleep;
    }
    return true;
}

void RH_SX126x::setModeRx()
{
    if (_mode != RHModeRx)
    {
	modeWillChange(RHModeRx);
	setPacketParametersLoRa(RH_SX126x_MAX_PAYLOAD_LEN);
	setRx(RH_SX126x_RX_TIMEOUT_NONE); // Timeout 0
	_mode = RHModeRx;
    }
}

void RH_SX126x::setModeTx()
{
    if (_mode != RHModeTx)
    {
	modeWillChange(RHModeTx);
	setTx(RH_SX126x_RX_TIMEOUT_NONE); // Timeout 0
	// Expect to be busy for about 0.5ms
	_mode = RHModeTx;
    }
}

// Sets registers from a canned modem configuration structure
bool RH_SX126x::setModemRegisters(const ModemConfig* config)
{
    _packetType = config->packetType;
    switch (_packetType)
    {
    case PacketTypeLoRa:
	sendCommand(RH_SX126x_CMD_SET_PKT_TYPE, RH_SX126x_PACKET_TYPE_LORA);
	break;

    case PacketTypeGFSK:
	sendCommand(RH_SX126x_CMD_SET_PKT_TYPE, RH_SX126x_PACKET_TYPE_GFSK);
	break;
    }
    return setModulationParameters(
	config->p1,
	config->p2,
	config->p3,
	config->p4,
	config->p5,
	config->p6,
	config->p7,
	config->p8);
}

// Set one of the canned FSK Modem configs
// Returns true if its a valid choice
bool RH_SX126x::setModemConfig(ModemConfigChoice index)
{
    if (index > (signed int)(sizeof(MODEM_CONFIG_TABLE) / sizeof(ModemConfig)))
        return false;

    // Get it out of PROG_MEM
    ModemConfig cfg;
    memcpy_P(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(RH_SX126x::ModemConfig));
    return setModemRegisters(&cfg);
}

void RH_SX126x::setPreambleLength(uint16_t bytes)
{
    _preambleLength = bytes;
}

bool RH_SX126x::isChannelActive()
{
    // Set mode RHModeCad
    if (_mode != RHModeCad)
    {
	modeWillChange(RHModeCad);
	setCad();
        _mode = RHModeCad;
    }

    while (_mode == RHModeCad)
        YIELD;

    return _cad;
}

int RH_SX126x::lastSNR()
{
    return _lastSNR;
}

//////////////////////////////////////////////////
// Specialised SPI routines.
// This device has slightly unusual SPI behaviour: including no RH_SPI_WRITE_MASK, so need to rewrite
// most SPI access functions

bool RH_SX126x::waitUntilNotBusy()
{
    uint8_t busy_timeout_cnt = 0;
    if (_busyPin == RH_INVALID_PIN)
	return false;
    
    while (digitalRead(_busyPin))
    {
	delay(1);
	busy_timeout_cnt++;

	// Need to wait up to 150us when in sleep mode, see table 8-1
	if (busy_timeout_cnt > 10) // Should be configurable
	{
	    Serial.println("ERROR: waitUntilNotBusy TIMEOUT");
	    return false;
	}
    }
    return true; // OK
}

uint8_t RH_SX126x::getStatus()
{
    waitUntilNotBusy();
    uint8_t status;
    ATOMIC_BLOCK_START;
    beginTransaction();
    _spi.transfer(RH_SX126x_CMD_GET_STATUS);
    status = _spi.transfer(RH_SX126x_CMD_NOP);
    endTransaction();
    ATOMIC_BLOCK_END;
    return status;
}

bool RH_SX126x::sendCommand(uint8_t command, uint8_t data[], uint8_t len)
{
#if 0
    Serial.print("sendCommand ");
    Serial.print(command, HEX);
    Serial.print(" ");
    for (uint8_t i = 0; i < len; i++)
    {
	Serial.print(data[i], HEX);
	Serial.print(" ");
    }
    Serial.println("");
#endif

    ATOMIC_BLOCK_START;
    beginTransaction();
    waitUntilNotBusy();
    _spi.transfer(command);
    for (uint8_t i = 0; i < len; i++)
        _spi.transfer(data[i]);
    endTransaction();
    ATOMIC_BLOCK_END;

    return true;
}

// Single value command
bool RH_SX126x::sendCommand(uint8_t command, uint8_t value)
{
  uint8_t data = value;
  return sendCommand(command, &data, sizeof(data));
}

bool RH_SX126x::sendCommand(uint8_t command)
{
    ATOMIC_BLOCK_START;
    beginTransaction();
    waitUntilNotBusy();
    _spi.transfer(command);
    endTransaction();
    ATOMIC_BLOCK_END;
    return true;
}

bool RH_SX126x::getCommand(uint8_t command, uint8_t data[], uint8_t len)
{ 
    ATOMIC_BLOCK_START;
    beginTransaction();
    waitUntilNotBusy();
    _spi.transfer(command);
    _spi.transfer(0); // Wait for data
    for (uint8_t i = 0; i < len; i++)
        data[i] = _spi.transfer(0);
    endTransaction();
    ATOMIC_BLOCK_END;
    return true;
}

bool RH_SX126x::readRegisters(uint16_t address, uint8_t data[], uint8_t len)
{
    ATOMIC_BLOCK_START;
    beginTransaction();
    waitUntilNotBusy();
    _spi.transfer(RH_SX126x_CMD_READ_REGISTER);
    _spi.transfer(static_cast<uint8_t>(address >> 8));
    _spi.transfer(static_cast<uint8_t>(address));
    _spi.transfer(RH_SX126x_CMD_NOP); // Wait for data
    
    for (uint8_t i = 0; i < len; i++)
	data[i] = _spi.transfer(0);
    endTransaction();
    ATOMIC_BLOCK_END;
    return true;
}

uint8_t RH_SX126x::readRegister(uint16_t address)
{
  uint8_t data = 0;
  readRegisters(address, &data, 1);
  return data;
}

bool RH_SX126x::writeRegisters(uint16_t address, uint8_t data[], uint8_t len)
{
#if 0
    Serial.print("writeRegisters ");
    Serial.print(address, HEX);
    Serial.print(" ");
    for (uint8_t i = 0; i < len; i++)
    {
	Serial.print(data[i], HEX);
	Serial.print(" ");
    }
    Serial.println("");
#endif
    
    ATOMIC_BLOCK_START;
    beginTransaction();
    waitUntilNotBusy();
    _spi.transfer(RH_SX126x_CMD_WRITE_REGISTER);
    _spi.transfer(static_cast<uint8_t>(address >> 8));
    _spi.transfer(static_cast<uint8_t>(address));
    for (uint8_t i = 0; i < len; i++)
	_spi.transfer(data[i]);
    endTransaction();
    ATOMIC_BLOCK_END;
    return true;
}

bool RH_SX126x::writeRegister(uint16_t address, uint8_t data)
{
  uint8_t d = data; // Single register bye to write
  return writeRegisters(address, &d, 1);
}


bool RH_SX126x::writeBuffer(uint8_t offset, const uint8_t data[], uint8_t len)
{
    // A bit different to sendCommand because of offset
    ATOMIC_BLOCK_START;
    beginTransaction();
    waitUntilNotBusy();
    _spi.transfer(RH_SX126x_CMD_WRITE_BUFFER);
    _spi.transfer(offset);
    for (uint8_t i = 0; i < len; i++)
        _spi.transfer(data[i]);
    endTransaction();
    ATOMIC_BLOCK_END;
    return true;
}
 
bool RH_SX126x::writeBuffer(uint8_t offset, const char* text)
{
    return writeBuffer(offset, (uint8_t*) text, strlen(text));
}

bool RH_SX126x::readBuffer(uint8_t offset, uint8_t data[], uint8_t len)
{
    // This is a bit different from getCommand, because of offset
    ATOMIC_BLOCK_START;
    beginTransaction();
    waitUntilNotBusy();
    _spi.transfer(RH_SX126x_CMD_READ_BUFFER);
    _spi.transfer(offset);
    _spi.transfer(RH_SX126x_CMD_NOP); // Wait for data
    for (uint8_t i = 0; i < len; i++)
	data[i] = _spi.transfer(0);
    endTransaction();
    ATOMIC_BLOCK_END;
    return true;
}

bool RH_SX126x::setPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut)
{
    uint8_t settings[] = {paDutyCycle, hpMax, deviceSel, paLut};
    return sendCommand(RH_SX126x_CMD_SET_PA_CFG, settings, sizeof(settings));
}

bool RH_SX126x::setTxParams(uint8_t power, uint8_t rampTime)
{
    uint8_t settings[] = {power, rampTime};
    return sendCommand(RH_SX126x_CMD_SET_TX_PARAMS, settings, sizeof(settings));
}

bool RH_SX126x::setTxPower(int8_t power)
{
    // The device may have 2 transmitter power amps (PAs), low power and high power
    // However, depending on the specific model and how it is connected, one or the other might not be available
    // so we depend on configuration to tell what PAs are available
    // STM32WLE5JC has 2, but perhaps only 1 is connected to the antenna, depending on the actual circuit
    // SX1261 only has low power
    // SX1262 only has high power
    // SX1268 only has high power
    bool lp_supported = findRadioPinConfigEntry(RadioPinConfigMode_TX_LOW_POWER) != nullptr;
    bool hp_supported = findRadioPinConfigEntry(RadioPinConfigMode_TX_HIGH_POWER) != nullptr;
    bool useHP = false; // Whether we are to use the high or low power amp

    // REVISIT: needs more work for reduced powers
    // Depends on if 1261 or 1262 or 1268
    if (hp_supported && lp_supported)
    {
	// -17 to +22 dBm
	power = constrain(power, -17, 22);
	if (power > 14)
	    useHP = true;
	else
	    useHP = false;
    }
    else if (!hp_supported && lp_supported) 
    {
	// -17 to +15 dBm
	power = constrain(power, -17, 15);
	useHP = false;
    }
    else // Assume only HP is available
    {
	// -9 to +22 dBm
	power = constrain(power, -9, 22);
	useHP = true;
    }
    
    // Adjust for optimal settings per Table 13-1
    // CAUTION: needs to change for SX1268
    if (useHP)
    {
	// CAUTION: device power supply needs to be able to provide 3.3V at currents up to 118 mA with high power settings
	// -9 to 22 dBm SX1262
	_requiredPAMode = RadioPinConfigMode_TX_HIGH_POWER;
	fixPAClamping(true);
	switch (power)
	{
	case 22:
	case 21:
	    setPaConfig(0x04, 0x07, RH_SX126x_PA_CONFIG_DEVICE_SEL_SX1262, RH_SX126x_PA_CONFIG_PA_LUT);
	    break;

	case 20:
	case 19:
	case 18:
	    setPaConfig(0x03, 0x05, RH_SX126x_PA_CONFIG_DEVICE_SEL_SX1262, RH_SX126x_PA_CONFIG_PA_LUT);
	    power = power + 2;
	    break;

	case 17:
	case 16:
	case 15:
	    setPaConfig(0x02, 0x03, RH_SX126x_PA_CONFIG_DEVICE_SEL_SX1262, RH_SX126x_PA_CONFIG_PA_LUT);
	    power = power + 5;
	    break;

	default:
	    // 14 and less
	    setPaConfig(0x02, 0x02, RH_SX126x_PA_CONFIG_DEVICE_SEL_SX1262, RH_SX126x_PA_CONFIG_PA_LUT);
	    power = power + 8;
	    break;
	}
    }
    else
    {
	// CAUTION: device power supply needs to be able to provide 3.3V at currents up to 32.5 mA with high power settings
	// -17 to 14 dBm SX1262
	_requiredPAMode = RadioPinConfigMode_TX_LOW_POWER;
	switch (power)
	{
	case 15:
	    setPaConfig(0x06, 0x00, RH_SX126x_PA_CONFIG_DEVICE_SEL_SX1261, RH_SX126x_PA_CONFIG_PA_LUT);
	    power = 14;
	    break;

	case 14:
	case 13:
	case 12:
	case 11:
	    setPaConfig(0x04, 0x00, RH_SX126x_PA_CONFIG_DEVICE_SEL_SX1261, RH_SX126x_PA_CONFIG_PA_LUT);
	    break;

	default:
	    // 10 and less
	    setPaConfig(0x01, 0x00, RH_SX126x_PA_CONFIG_DEVICE_SEL_SX1261, RH_SX126x_PA_CONFIG_PA_LUT);
	    power = power + 3;
	    break;
	}
    }
    // SetTxParams with the corrected power level
    return setTxParams(power, RH_SX126x_PA_RAMP_200U); // power, RampTime = 200us. REVISIT: should be configurable ramp time?
}

bool RH_SX126x::printRegisters(uint16_t address, uint8_t count)
{
#ifdef RH_HAVE_SERIAL
    uint8_t buf[256];
    readRegisters(address, buf, count);
    Serial.print("registers starting at: ");
    Serial.println(address, HEX);
    for (uint8_t i = 0; i < count; i++)
    {
        Serial.println(buf[i], HEX);
    }
#endif
    return true;
}

bool RH_SX126x::clearDeviceErrors()
{
    uint8_t data[] = {0x00, 0x00};
    return sendCommand(RH_SX126x_CMD_CLR_DEVICE_ERRORS, data, sizeof(data));
}

bool RH_SX126x::setDIO2AsRfSwitchCtrl(bool value)
{
    return sendCommand(RH_SX126x_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, value);
}

bool RH_SX126x::setRxFallbackMode(uint8_t mode)
{
    return sendCommand(RH_SX126x_CMD_SET_RX_TX_FALLBACK_MODE, mode);
}

bool RH_SX126x::setModulationParameters(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4, uint8_t p5, uint8_t p6, uint8_t p7, uint8_t p8)
{
    _lorabw500 = (p2 == RH_SX126x_LORA_BW_500_0); // Need to remember this for modulation quality workaround
    switch(p2)
    {
    case RH_SX126x_LORA_BW_7_8:
	_bandwidth = 7.8;
	break;
    case RH_SX126x_LORA_BW_10_4:
	_bandwidth = 10.4;
	break;
    case RH_SX126x_LORA_BW_15_6:
	_bandwidth = 15.6;
	break;
    case RH_SX126x_LORA_BW_20_8:
	_bandwidth = 20.8;
	break;
    case RH_SX126x_LORA_BW_31_25:
	_bandwidth = 31.25;
	break;
    case RH_SX126x_LORA_BW_41_7:
	_bandwidth = 41.7;
	break;
    case RH_SX126x_LORA_BW_62_5:
	_bandwidth = 62.5;
	break;
    case RH_SX126x_LORA_BW_125_0:
	_bandwidth = 125.0;
	break;
    case RH_SX126x_LORA_BW_250_0:
	_bandwidth = 250.0;
	break;
    case RH_SX126x_LORA_BW_500_0:
	_bandwidth = 500.0;
	break;
    default:
	_bandwidth = 0;
    }
    uint8_t params[] = {p1, p2, p3, p4, p5, p6, p7, p8};
    return sendCommand(RH_SX126x_CMD_SET_MODULATION_PARAMS, params, sizeof(params));
}

bool RH_SX126x::setModulationParametersLoRa(uint8_t sf, float bw, uint8_t cr, bool ldro)
{
    _packetType = PacketTypeLoRa;

    // Set packet type LoRa. CAUTION: Must be done in STDBY_RC mode (ie in RH idle mode)
    sendCommand(RH_SX126x_CMD_SET_PKT_TYPE, RH_SX126x_PACKET_TYPE_LORA);

    // REVISIT: could automatically calculate LDRO based on symbollength = (1 << SF) / BW 
    _bandwidth = bw;    
    // sf must be 5 to 12
    // bw must be 0.0 to 510.0
    // cr must be 5 - 8 inclusive (represents 4/5, 4/6, 4/7, 4/8 respectively)
    uint8_t bw_div2 = bw / 2 + 0.01;
    uint8_t bwsetting = RH_SX126x_LORA_BW_125_0;
    switch (bw_div2) 
    {
    case 3: // 7.8:
        bwsetting = RH_SX126x_LORA_BW_7_8;
        break;
    case 5: // 10.4:
        bwsetting = RH_SX126x_LORA_BW_10_4;
        break;
    case 7: // 15.6:
        bwsetting = RH_SX126x_LORA_BW_15_6;
        break;
    case 10: // 20.8:
        bwsetting = RH_SX126x_LORA_BW_20_8;
        break;
    case 15: // 31.25:
        bwsetting = RH_SX126x_LORA_BW_31_25;
        break;
    case 20: // 41.7:
        bwsetting = RH_SX126x_LORA_BW_41_7;
        break;
    case 31: // 62.5:
        bwsetting = RH_SX126x_LORA_BW_62_5;
        break;
    case 62: // 125.0:
        bwsetting = RH_SX126x_LORA_BW_125_0;
        break;
    case 125: // 250.0
        bwsetting = RH_SX126x_LORA_BW_250_0;
        break;
    case 250: // 500.0
        bwsetting = RH_SX126x_LORA_BW_500_0;
        break;
    }
    return setModulationParameters(sf, bwsetting, cr - 4, ldro, 0, 0, 0, 0);
}


bool RH_SX126x::setModulationParametersGFSK(uint32_t br, uint8_t sh, uint8_t rxBw, uint32_t freqDev)
{
    _packetType = PacketTypeGFSK;
    return false;
}

bool RH_SX126x::calibrate(uint8_t calib_param)
{
    return sendCommand(RH_SX126x_CMD_CALIBRATE, calib_param);
}

bool RH_SX126x::calibrateImage(uint8_t f1, uint8_t f2)
{
    uint8_t frequencies[] = {f1, f2};
    return sendCommand(RH_SX126x_CMD_CALIBRATE_IMAGE, frequencies, sizeof(frequencies));
}

bool RH_SX126x::setLoRaSyncWord(uint16_t sync)
{
    uint8_t sync_word[] = {static_cast<uint8_t>(sync >> 8), static_cast<uint8_t>(sync)};
    return writeRegisters(RH_SX126x_REG_LR_SYNCWORD, sync_word, sizeof(sync_word));
}

bool RH_SX126x::setOCPConfiguration(uint8_t setting)
{
    return writeRegister(RH_SX126x_REG_OCP, setting);
}

bool RH_SX126x::setDIO3AsTcxoCtrl(uint8_t voltage, uint32_t delay)
{
    uint8_t settings[] = {voltage,
        static_cast<uint8_t>(delay >> 16),
        static_cast<uint8_t>(delay >> 8),
        static_cast<uint8_t>(delay)};
    return sendCommand(RH_SX126x_CMD_SET_DIO3_AS_TCXO_CTRL, settings, sizeof(settings));
}

bool RH_SX126x::setTCXO(float voltage, uint32_t delay)
{
    uint8_t voltageValue = RH_SX126x_DIO3_OUTPUT_1_7;

    if (fabs(voltage - 1.6) <= 0.001)
        voltageValue = RH_SX126x_DIO3_OUTPUT_1_6;
    else if (fabs(voltage - 1.7) <= 0.001)
        voltageValue = RH_SX126x_DIO3_OUTPUT_1_7;
    else if (fabs(voltage - 1.8) <= 0.001)
        voltageValue = RH_SX126x_DIO3_OUTPUT_1_8;
    else if (fabs(voltage - 2.2) <= 0.001)
        voltageValue = RH_SX126x_DIO3_OUTPUT_2_2;
    else if (fabs(voltage - 2.4) <= 0.001)
        voltageValue = RH_SX126x_DIO3_OUTPUT_2_4;
    else if (fabs(voltage - 2.7) <= 0.001)
        voltageValue = RH_SX126x_DIO3_OUTPUT_2_7;
    else if (fabs(voltage - 3.0) <= 0.001)
        voltageValue = RH_SX126x_DIO3_OUTPUT_3_0;
    else if (fabs(voltage - 3.3) <= 0.001)
        voltageValue = RH_SX126x_DIO3_OUTPUT_3_3;

    uint32_t delayValue = (float)delay / 15.625;
    return setDIO3AsTcxoCtrl(voltageValue, delayValue);
}

bool RH_SX126x::setPacketParams(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4, uint8_t p5, uint8_t p6, uint8_t p7, uint8_t p8, uint8_t p9)
{
    uint8_t settings[] = {p1, p2, p3, p4, p5, p6, p7, p8, p9};
    return sendCommand(RH_SX126x_CMD_SET_PKT_PARAMS, settings, sizeof(settings));
}

bool RH_SX126x::setPacketParametersLoRa(uint8_t payload_length)
{
    _packetType = PacketTypeLoRa;
    // IQ polarity fix per SX1262_datasheet.pdf section 15.4 
    uint8_t value = readRegister(RH_SX126x_REG_IQ_POLARITY);
    if (_invertIQ)
        value = value & ~0x04;
    else
        value = value | 0x04;
    writeRegister(RH_SX126x_REG_IQ_POLARITY, value);

    // _preambleLength, fixedlength = false, _enableCRC, invertIQ
    return setPacketParams(0, _preambleLength, RH_SX126x_LORA_PACKET_VARIABLE,  payload_length, _enableCRC, _invertIQ, 0, 0, 0);
}

bool RH_SX126x::setBufferBaseAddress(uint8_t txbase, uint8_t rxbase)
{
    uint8_t settings[] = {txbase, rxbase};
    return sendCommand(RH_SX126x_CMD_SET_BUFFER_BASE_ADDRESS, settings, sizeof(settings));
}

bool RH_SX126x::setSleep(uint8_t config)
{
    setRadioPinsForMode(RadioPinConfigMode_IDLE);
    uint8_t sleep_config = config;
    return sendCommand(RH_SX126x_CMD_SET_SLEEP, &sleep_config, sizeof(sleep_config));
}

bool RH_SX126x::setStandby(uint8_t config)
{
    setRadioPinsForMode(RadioPinConfigMode_IDLE);
    uint8_t standby_config = config; // 0 == STDBY_RC, 1 == STDBY_XOSC
    return sendCommand(RH_SX126x_CMD_SET_STANDBY, &standby_config, sizeof(standby_config));
}

bool RH_SX126x::setTx(uint32_t timeout)
{
    // Need workaround for Modulation Qulity if LoRa BW is 500kHz
    // Per SX1262_datasheet.pdf section 15.1
    if (_packetType == PacketTypeLoRa)
    {
	if (_lorabw500)
	{
	    uint8_t value = readRegister(RH_SX126x_REG_TX_MODULATION);
	    value &= 0xfb;
	    writeRegister(RH_SX126x_REG_TX_MODULATION, value);
	}
	else
	{
	    uint8_t value = readRegister(RH_SX126x_REG_TX_MODULATION);
	    value |= 0x04;
	    writeRegister(RH_SX126x_REG_TX_MODULATION, value);
	}
    }
    
    setRadioPinsForMode(_requiredPAMode);
    uint8_t settings[] =
        {static_cast<uint8_t>(timeout >> 16),
         static_cast<uint8_t>(timeout >> 8),
         static_cast<uint8_t>(timeout)};
    return sendCommand(RH_SX126x_CMD_SET_TX, settings, sizeof(settings));
}

bool RH_SX126x::setTxContinuous()
{
    setRadioPinsForMode(_requiredPAMode);
    return sendCommand(RH_SX126x_CMD_SET_TX_CONTINUOUS_WAVE); 
}

bool RH_SX126x::setRx(uint32_t timeout)
{
    setBufferBaseAddress(0, 0);
    setRadioPinsForMode(RadioPinConfigMode_RX);
    uint8_t settings[] =
        {static_cast<uint8_t>(timeout >> 16),
         static_cast<uint8_t>(timeout >> 8),
         static_cast<uint8_t>(timeout)};
    // Sigh, on some chips it doesnt enter RX mode on the first try, so try again.
    // Why?
    bool status = sendCommand(RH_SX126x_CMD_SET_RX, settings, sizeof(settings));
    status = sendCommand(RH_SX126x_CMD_SET_RX, settings, sizeof(settings));
    return status;
}

bool RH_SX126x::setCad()
{
    // REVISIT: CAD is not yet working. See comments in datasheet:

    // Choosing the right value is not easy and the values selected must
    // be carefully tested to ensure a good detection at sensitivity level, and also to limit the number of false detections.
    // Application note AN1200.48 provides guidance for the selection of these parameters.
    // See Semtech AN1200.48, page 41.
    return false;
    if (_packetType == PacketTypeLoRa)
    {
	// REVISIT: detPeak depends on spreading factor
	uint8_t cadparams[] = {RH_SX126x_CAD_ON_2_SYMB, 22, RH_SX126x_CAD_PARAM_DET_MIN, RH_SX126x_CAD_GOTO_STDBY, 0, 0, 0};
	sendCommand(RH_SX126x_CMD_SET_CAD_PARAMS, cadparams, sizeof(cadparams));
	return sendCommand(RH_SX126x_CMD_SET_CAD); // Only available in LoRa mode
    }
    else
	return false;
}

bool RH_SX126x::setRxBoostMode(bool boost, bool retain)
{
    if (boost)
	writeRegister(RH_SX126x_REG_RXGAIN, RH_SX126x_RX_GAIN_BOOSTED);
    else
	writeRegister(RH_SX126x_REG_RXGAIN, RH_SX126x_RX_GAIN_POWER_SAVING);  
    
    if (retain)
    {
	// Include this register in the retention memory
	uint8_t settings[] = {0x01, 0x08, 0xac};
	writeRegisters(RH_SX126x_REG_RETENTION_LIST_BASE_ADDRESS, settings, sizeof(settings));
    }
    return true;
}

bool RH_SX126x::setRegulatorMode(uint8_t mode)
{
    return sendCommand(RH_SX126x_CMD_SET_REGULATOR_MODE, mode);
}

bool RH_SX126x::setDioIrqParams(uint16_t irqmask, uint16_t dio1mask, uint16_t dio2mask, uint16_t dio3mask)
{
    uint8_t settings[] = {static_cast<uint8_t>(irqmask >> 8), static_cast<uint8_t>(irqmask), 
        static_cast<uint8_t>(dio1mask >> 8), static_cast<uint8_t>(dio1mask), 
        static_cast<uint8_t>(dio2mask >> 8), static_cast<uint8_t>(dio2mask), 
        static_cast<uint8_t>(dio3mask >> 8), static_cast<uint8_t>(dio3mask)};
    return sendCommand(RH_SX126x_CMD_SET_DIO_IRQ_PARAMS, settings, sizeof(settings));
}

bool RH_SX126x::clearIrqStatus(uint16_t mask)
{
    uint8_t settings[] = {static_cast<uint8_t>(mask >> 8), static_cast<uint8_t>(mask)};
    return sendCommand(RH_SX126x_CMD_CLR_IRQ_STATUS, settings, sizeof(settings));
}

uint16_t RH_SX126x::getIrqStatus()
{
    uint8_t status[2];
    getCommand(RH_SX126x_CMD_GET_IRQ_STATUS, status, sizeof(status));
    return ((uint16_t)status[0] << 8) | status[1];
}

uint8_t RH_SX126x::getPacketType()
{
    uint8_t ptype[1];
    getCommand(RH_SX126x_CMD_GET_PKT_TYPE, ptype, sizeof(ptype));
    return ptype[0];
}

void RH_SX126x::setInvertIQ(bool invertIQ)
{
    _invertIQ = invertIQ;
}

bool RH_SX126x::fixPAClamping(bool enable)
{
    // Per SX1262_datasheet.pdf Rev 1.2 section 15.2
    uint8_t clamp;
    clamp = readRegister(RH_SX126x_REG_TX_CLAMP_CFG);
    if (enable)
	clamp |= 0x1E;
    else
	clamp = (clamp & ~0x1E) | 0x08;
    return writeRegister(RH_SX126x_REG_TX_CLAMP_CFG, clamp);
}

bool RH_SX126x::setRadioPinsForMode(RadioPinConfigMode mode)
{
    if (!_radioPinConfig)
	return false;
    
    RadioPinConfigEntry* entry = findRadioPinConfigEntry(mode);
    if (!entry)
	return false;

    for (uint8_t i = 0; i < RH_SX126x_MAX_RADIO_CONTROL_PINS ; i++)
    {
#if 0
	Serial.print("set pin ");
	Serial.print(_radioPinConfig->pinNumber[i]);
	Serial.print(" to ");
	Serial.print(entry->pinState[i]);
	Serial.println("");
#endif
	if (_radioPinConfig->pinNumber[i] != RH_INVALID_PIN)
	    digitalWrite(_radioPinConfig->pinNumber[i], entry->pinState[i]);
    }
    return true;
}

void RH_SX126x::setRadioPinConfig(RadioPinConfig* config)
{
    for (uint8_t i = 0; i < RH_SX126x_MAX_RADIO_CONTROL_PINS; i++)
    {
	if (config && config->pinNumber[i] != RH_INVALID_PIN)
	    pinMode(config->pinNumber[i], OUTPUT);
    }
    _radioPinConfig = config;
}

// Can be overridden
RH_SX126x::RadioPinConfigEntry* RH_SX126x::findRadioPinConfigEntry(RadioPinConfigMode mode)
{
    // Sigh, linear search, max of 5 entries
    if (!_radioPinConfig)
	return nullptr; // No configurations, Not found
    
    for (uint8_t i = 0; i < RH_SX126x_MAX_RADIO_PIN_CONFIG_MODES; i++)
    {
	if (_radioPinConfig->configState[i].mode == RadioPinConfigMode_EOT)
	    return nullptr; // End of table, Not found
	else if (_radioPinConfig->configState[i].mode == mode)
	    return &_radioPinConfig->configState[i]; // Found the one we want
    }
    return nullptr; // Table too big, Not found
}

float RH_SX126x::getFrequencyError()
{
  // check packetType
    
    if( _packetType != PacketTypeLoRa)
        return(0.0);
    
    // read the raw frequency error register values
    uint8_t regBytes[3] = {0};
    regBytes[0] = readRegister(RH_SX126x_REG_FREQ_ERROR);
    regBytes[1] = readRegister(RH_SX126x_REG_FREQ_ERROR + 1);
    regBytes[2] = readRegister(RH_SX126x_REG_FREQ_ERROR + 2);
    uint32_t tmp = ((uint32_t) regBytes[0] << 16) | ((uint32_t) regBytes[1] << 8) | regBytes[2];
    tmp &= 0x0FFFFF;
    
    float error = 0;
    
    // check the first bit
    if (tmp & 0x80000) {
	// frequency error is negative
        tmp |= (uint32_t) 0xFFF00000;
        tmp = ~tmp + 1;
        error = 1.55 * (float) tmp / (1600.0 / _bandwidth) * -1.0;
    }
    else
    {
        error = 1.55 * (float) tmp / (1600.0 / _bandwidth);
    }
    return(error);
}
