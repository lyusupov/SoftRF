// RH_RF69.cpp
//
// Copyright (C) 2011 Mike McCauley
// $Id: RH_RF69.cpp,v 1.31 2019/09/02 05:21:52 mikem Exp $

#include <RH_RF69.h>

// Interrupt vectors for the 3 Arduino interrupt pins
// Each interrupt can be handled by a different instance of RH_RF69, allowing you to have
// 2 or more RF69s per Arduino
RH_RF69* RH_RF69::_deviceForInterrupt[RH_RF69_NUM_INTERRUPTS] = {0, 0, 0};
uint8_t RH_RF69::_interruptCount = 0; // Index into _deviceForInterrupt for next device

// These are indexed by the values of ModemConfigChoice
// Stored in flash (program) memory to save SRAM
// It is important to keep the modulation index for FSK between 0.5 and 10
// modulation index = 2 * Fdev / BR
// Note that I have not had much success with FSK with Fd > ~5
// You have to construct these by hand, using the data from the RF69 Datasheet :-(
// or use the SX1231 starter kit software (Ctl-Alt-N to use that without a connected radio)
#define CONFIG_FSK (RH_RF69_DATAMODUL_DATAMODE_PACKET | RH_RF69_DATAMODUL_MODULATIONTYPE_FSK | RH_RF69_DATAMODUL_MODULATIONSHAPING_FSK_NONE)
#define CONFIG_GFSK (RH_RF69_DATAMODUL_DATAMODE_PACKET | RH_RF69_DATAMODUL_MODULATIONTYPE_FSK | RH_RF69_DATAMODUL_MODULATIONSHAPING_FSK_BT1_0)
#define CONFIG_OOK (RH_RF69_DATAMODUL_DATAMODE_PACKET | RH_RF69_DATAMODUL_MODULATIONTYPE_OOK | RH_RF69_DATAMODUL_MODULATIONSHAPING_OOK_NONE)

// Choices for RH_RF69_REG_37_PACKETCONFIG1:
#define CONFIG_NOWHITE (RH_RF69_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RH_RF69_PACKETCONFIG1_DCFREE_NONE | RH_RF69_PACKETCONFIG1_CRC_ON | RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NONE)
#define CONFIG_WHITE (RH_RF69_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RH_RF69_PACKETCONFIG1_DCFREE_WHITENING | RH_RF69_PACKETCONFIG1_CRC_ON | RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NONE)
#define CONFIG_MANCHESTER (RH_RF69_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RH_RF69_PACKETCONFIG1_DCFREE_MANCHESTER | RH_RF69_PACKETCONFIG1_CRC_ON | RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NONE)
PROGMEM static const RH_RF69::ModemConfig MODEM_CONFIG_TABLE[] =
{
    //  02,        03,   04,   05,   06,   19,   1a,  37
    // FSK, No Manchester, no shaping, whitening, CRC, no address filtering
    // AFC BW == RX BW == 2 x bit rate
    // Low modulation indexes of ~ 1 at slow speeds do not seem to work very well. Choose MI of 2.
    { CONFIG_FSK,  0x3e, 0x80, 0x00, 0x52, 0xf4, 0xf4, CONFIG_WHITE}, // FSK_Rb2Fd5      
    { CONFIG_FSK,  0x34, 0x15, 0x00, 0x4f, 0xf4, 0xf4, CONFIG_WHITE}, // FSK_Rb2_4Fd4_8
    { CONFIG_FSK,  0x1a, 0x0b, 0x00, 0x9d, 0xf4, 0xf4, CONFIG_WHITE}, // FSK_Rb4_8Fd9_6

    { CONFIG_FSK,  0x0d, 0x05, 0x01, 0x3b, 0xf4, 0xf4, CONFIG_WHITE}, // FSK_Rb9_6Fd19_2
    { CONFIG_FSK,  0x06, 0x83, 0x02, 0x75, 0xf3, 0xf3, CONFIG_WHITE}, // FSK_Rb19_2Fd38_4
    { CONFIG_FSK,  0x03, 0x41, 0x04, 0xea, 0xf2, 0xf2, CONFIG_WHITE}, // FSK_Rb38_4Fd76_8

    { CONFIG_FSK,  0x02, 0x2c, 0x07, 0xae, 0xe2, 0xe2, CONFIG_WHITE}, // FSK_Rb57_6Fd120
    { CONFIG_FSK,  0x01, 0x00, 0x08, 0x00, 0xe1, 0xe1, CONFIG_WHITE}, // FSK_Rb125Fd125
    { CONFIG_FSK,  0x00, 0x80, 0x10, 0x00, 0xe0, 0xe0, CONFIG_WHITE}, // FSK_Rb250Fd250
    { CONFIG_FSK,  0x02, 0x40, 0x03, 0x33, 0x42, 0x42, CONFIG_WHITE}, // FSK_Rb55555Fd50 

    //  02,        03,   04,   05,   06,   19,   1a,  37
    // GFSK (BT=1.0), No Manchester, whitening, CRC, no address filtering
    // AFC BW == RX BW == 2 x bit rate
    { CONFIG_GFSK, 0x3e, 0x80, 0x00, 0x52, 0xf4, 0xf5, CONFIG_WHITE}, // GFSK_Rb2Fd5
    { CONFIG_GFSK, 0x34, 0x15, 0x00, 0x4f, 0xf4, 0xf4, CONFIG_WHITE}, // GFSK_Rb2_4Fd4_8
    { CONFIG_GFSK, 0x1a, 0x0b, 0x00, 0x9d, 0xf4, 0xf4, CONFIG_WHITE}, // GFSK_Rb4_8Fd9_6

    { CONFIG_GFSK, 0x0d, 0x05, 0x01, 0x3b, 0xf4, 0xf4, CONFIG_WHITE}, // GFSK_Rb9_6Fd19_2
    { CONFIG_GFSK, 0x06, 0x83, 0x02, 0x75, 0xf3, 0xf3, CONFIG_WHITE}, // GFSK_Rb19_2Fd38_4
    { CONFIG_GFSK, 0x03, 0x41, 0x04, 0xea, 0xf2, 0xf2, CONFIG_WHITE}, // GFSK_Rb38_4Fd76_8

    { CONFIG_GFSK, 0x02, 0x2c, 0x07, 0xae, 0xe2, 0xe2, CONFIG_WHITE}, // GFSK_Rb57_6Fd120
    { CONFIG_GFSK, 0x01, 0x00, 0x08, 0x00, 0xe1, 0xe1, CONFIG_WHITE}, // GFSK_Rb125Fd125
    { CONFIG_GFSK, 0x00, 0x80, 0x10, 0x00, 0xe0, 0xe0, CONFIG_WHITE}, // GFSK_Rb250Fd250
    { CONFIG_GFSK, 0x02, 0x40, 0x03, 0x33, 0x42, 0x42, CONFIG_WHITE}, // GFSK_Rb55555Fd50 

    //  02,        03,   04,   05,   06,   19,   1a,  37
    // OOK, No Manchester, no shaping, whitening, CRC, no address filtering
    // with the help of the SX1231 configuration program
    // AFC BW == RX BW
    // All OOK configs have the default:
    // Threshold Type: Peak
    // Peak Threshold Step: 0.5dB
    // Peak threshiold dec: ONce per chip
    // Fixed threshold: 6dB
    { CONFIG_OOK,  0x7d, 0x00, 0x00, 0x10, 0x88, 0x88, CONFIG_WHITE}, // OOK_Rb1Bw1
    { CONFIG_OOK,  0x68, 0x2b, 0x00, 0x10, 0xf1, 0xf1, CONFIG_WHITE}, // OOK_Rb1_2Bw75
    { CONFIG_OOK,  0x34, 0x15, 0x00, 0x10, 0xf5, 0xf5, CONFIG_WHITE}, // OOK_Rb2_4Bw4_8
    { CONFIG_OOK,  0x1a, 0x0b, 0x00, 0x10, 0xf4, 0xf4, CONFIG_WHITE}, // OOK_Rb4_8Bw9_6
    { CONFIG_OOK,  0x0d, 0x05, 0x00, 0x10, 0xf3, 0xf3, CONFIG_WHITE}, // OOK_Rb9_6Bw19_2
    { CONFIG_OOK,  0x06, 0x83, 0x00, 0x10, 0xf2, 0xf2, CONFIG_WHITE}, // OOK_Rb19_2Bw38_4
    { CONFIG_OOK,  0x03, 0xe8, 0x00, 0x10, 0xe2, 0xe2, CONFIG_WHITE}, // OOK_Rb32Bw64

//    { CONFIG_FSK,  0x68, 0x2b, 0x00, 0x52, 0x55, 0x55, CONFIG_WHITE}, // works: Rb1200 Fd 5000 bw10000, DCC 400
//    { CONFIG_FSK,  0x0c, 0x80, 0x02, 0x8f, 0x52, 0x52, CONFIG_WHITE}, // works 10/40/80
//    { CONFIG_FSK,  0x0c, 0x80, 0x02, 0x8f, 0x53, 0x53, CONFIG_WHITE}, // works 10/40/40

};
RH_RF69::RH_RF69(uint8_t slaveSelectPin, uint8_t interruptPin, RHGenericSPI& spi)
    :
    RHSPIDriver(slaveSelectPin, spi)
{
    _interruptPin = interruptPin;
    _idleMode = RH_RF69_OPMODE_MODE_STDBY;
    _myInterruptIndex = 0xff; // Not allocated yet
}

void RH_RF69::setIdleMode(uint8_t idleMode)
{
    _idleMode = idleMode;
}

bool RH_RF69::init()
{
    if (!RHSPIDriver::init())
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

    // Get the device type and check it
    // This also tests whether we are really connected to a device
    // My test devices return 0x24
    _deviceType = spiRead(RH_RF69_REG_10_VERSION);
    Serial.println( _deviceType);
    if (_deviceType == 00 ||
	_deviceType == 0xff)
	return false;

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
	if (_interruptCount <= RH_RF69_NUM_INTERRUPTS)
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

    setModeIdle();

    // Configure important RH_RF69 registers
    // Here we set up the standard packet format for use by the RH_RF69 library:
    // 4 bytes preamble
    // 2 SYNC words 2d, d4
    // 2 CRC CCITT octets computed on the header, length and data (this in the modem config data)
    // 0 to 60 bytes data
    // RSSI Threshold -114dBm
    // We dont use the RH_RF69s address filtering: instead we prepend our own headers to the beginning
    // of the RH_RF69 payload
    spiWrite(RH_RF69_REG_3C_FIFOTHRESH, RH_RF69_FIFOTHRESH_TXSTARTCONDITION_NOTEMPTY | 0x0f); // thresh 15 is default
    // RSSITHRESH is default
//    spiWrite(RH_RF69_REG_29_RSSITHRESH, 220); // -110 dbM
    // SYNCCONFIG is default. SyncSize is set later by setSyncWords()
//    spiWrite(RH_RF69_REG_2E_SYNCCONFIG, RH_RF69_SYNCCONFIG_SYNCON); // auto, tolerance 0
    // PAYLOADLENGTH is default
//    spiWrite(RH_RF69_REG_38_PAYLOADLENGTH, RH_RF69_FIFO_SIZE); // max size only for RX
    // PACKETCONFIG 2 is default 
    spiWrite(RH_RF69_REG_6F_TESTDAGC, RH_RF69_TESTDAGC_CONTINUOUSDAGC_IMPROVED_LOWBETAOFF);
    // If high power boost set previously, disable it
    spiWrite(RH_RF69_REG_5A_TESTPA1, RH_RF69_TESTPA1_NORMAL);
    spiWrite(RH_RF69_REG_5C_TESTPA2, RH_RF69_TESTPA2_NORMAL);

    // The following can be changed later by the user if necessary.
    // Set up default configuration
    uint8_t syncwords[] = { 0x2d, 0xd4 };
    setSyncWords(syncwords, sizeof(syncwords)); // Same as RF22's
    // Reasonably fast and reliable default speed and modulation
    setModemConfig(GFSK_Rb250Fd250);

    // 3 would be sufficient, but this is the same as RF22's
    setPreambleLength(4);
    // An innocuous ISM frequency, same as RF22's
    setFrequency(434.0);
    // No encryption
    setEncryptionKey(NULL);
    // +13dBm, same as power-on default
    setTxPower(13); 

    return true;
}

// C++ level interrupt handler for this instance
// RH_RF69 is unusual in that it has several interrupt lines, and not a single, combined one.
// On Moteino, only one of the several interrupt lines (DI0) from the RH_RF69 is connnected to the processor.
// We use the single interrupt line to get PACKETSENT and PAYLOADREADY interrupts.
void RH_RF69::handleInterrupt()
{
    // Get the interrupt cause
    uint8_t irqflags2 = spiRead(RH_RF69_REG_28_IRQFLAGS2);
    if (_mode == RHModeTx && (irqflags2 & RH_RF69_IRQFLAGS2_PACKETSENT))
    {
	// A transmitter message has been fully sent
	setModeIdle(); // Clears FIFO
	_txGood++;
//	Serial.println("PACKETSENT");
    }

    // Must look for PAYLOADREADY, not CRCOK, since only PAYLOADREADY occurs _after_ AES decryption
    // has been done
    if (_mode == RHModeRx && (irqflags2 & RH_RF69_IRQFLAGS2_PAYLOADREADY))
    {
	// A complete message has been received with good CRC
	_lastRssi = -((int8_t)(spiRead(RH_RF69_REG_24_RSSIVALUE) >> 1));
	_lastPreambleTime = millis();

	setModeIdle();
	// Save it in our buffer
	readFifo();
//	Serial.println("PAYLOADREADY");
    }
}

// Low level function reads the FIFO and checks the address
// Caution: since we put our headers in what the RH_RF69 considers to be the payload, if encryption is enabled
// we have to suffer the cost of decryption before we can determine whether the address is acceptable. 
// Performance issue?
void RH_RF69::readFifo()
{
    ATOMIC_BLOCK_START;
    _spi.beginTransaction();
    digitalWrite(_slaveSelectPin, LOW);
    _spi.transfer(RH_RF69_REG_00_FIFO); // Send the start address with the write mask off
    uint8_t payloadlen = _spi.transfer(0); // First byte is payload len (counting the headers)
    if (payloadlen <= RH_RF69_MAX_ENCRYPTABLE_PAYLOAD_LEN &&
	payloadlen >= RH_RF69_HEADER_LEN)
    {
	_rxHeaderTo = _spi.transfer(0);
	// Check addressing
	if (_promiscuous ||
	    _rxHeaderTo == _thisAddress ||
	    _rxHeaderTo == RH_BROADCAST_ADDRESS)
	{
	    // Get the rest of the headers
	    _rxHeaderFrom  = _spi.transfer(0);
	    _rxHeaderId    = _spi.transfer(0);
	    _rxHeaderFlags = _spi.transfer(0);
	    // And now the real payload
	    for (_bufLen = 0; _bufLen < (payloadlen - RH_RF69_HEADER_LEN); _bufLen++)
		_buf[_bufLen] = _spi.transfer(0);
	    _rxGood++;
	    _rxBufValid = true;
	}
    }
    digitalWrite(_slaveSelectPin, HIGH);
    _spi.endTransaction();
    ATOMIC_BLOCK_END;
    // Any junk remaining in the FIFO will be cleared next time we go to receive mode.
}

// These are low level functions that call the interrupt handler for the correct
// instance of RH_RF69.
// 3 interrupts allows us to have 3 different devices
void RH_INTERRUPT_ATTR RH_RF69::isr0()
{
    if (_deviceForInterrupt[0])
	_deviceForInterrupt[0]->handleInterrupt();
}
void RH_INTERRUPT_ATTR RH_RF69::isr1()
{
    if (_deviceForInterrupt[1])
	_deviceForInterrupt[1]->handleInterrupt();
}
void RH_INTERRUPT_ATTR RH_RF69::isr2()
{
    if (_deviceForInterrupt[2])
	_deviceForInterrupt[2]->handleInterrupt();
}

int8_t RH_RF69::temperatureRead()
{
    // Caution: must be ins standby.
//    setModeIdle();
    spiWrite(RH_RF69_REG_4E_TEMP1, RH_RF69_TEMP1_TEMPMEASSTART); // Start the measurement
    while (spiRead(RH_RF69_REG_4E_TEMP1) & RH_RF69_TEMP1_TEMPMEASRUNNING)
	; // Wait for the measurement to complete
    return 166 - spiRead(RH_RF69_REG_4F_TEMP2); // Very approximate, based on observation
}

bool RH_RF69::setFrequency(float centre, float afcPullInRange)
{
    // Frf = FRF / FSTEP
    uint32_t frf = (uint32_t)((centre * 1000000.0) / RH_RF69_FSTEP);
    spiWrite(RH_RF69_REG_07_FRFMSB, (frf >> 16) & 0xff);
    spiWrite(RH_RF69_REG_08_FRFMID, (frf >> 8) & 0xff);
    spiWrite(RH_RF69_REG_09_FRFLSB, frf & 0xff);

    // afcPullInRange is not used
    (void)afcPullInRange;
    return true;
}

int8_t RH_RF69::rssiRead()
{
    // Force a new value to be measured
    // Hmmm, this hangs forever!
#if 0
    spiWrite(RH_RF69_REG_23_RSSICONFIG, RH_RF69_RSSICONFIG_RSSISTART);
    while (!(spiRead(RH_RF69_REG_23_RSSICONFIG) & RH_RF69_RSSICONFIG_RSSIDONE))
	;
#endif
    return -((int8_t)(spiRead(RH_RF69_REG_24_RSSIVALUE) >> 1));
}

void RH_RF69::setOpMode(uint8_t mode)
{
    uint8_t opmode = spiRead(RH_RF69_REG_01_OPMODE);
    opmode &= ~RH_RF69_OPMODE_MODE;
    opmode |= (mode & RH_RF69_OPMODE_MODE);
    spiWrite(RH_RF69_REG_01_OPMODE, opmode);

    // Wait for mode to change.
    while (!(spiRead(RH_RF69_REG_27_IRQFLAGS1) & RH_RF69_IRQFLAGS1_MODEREADY))
	;
}

void RH_RF69::setModeIdle()
{
    if (_mode != RHModeIdle)
    {
	if (_power >= 18)
	{
	    // If high power boost, return power amp to receive mode
	    spiWrite(RH_RF69_REG_5A_TESTPA1, RH_RF69_TESTPA1_NORMAL);
	    spiWrite(RH_RF69_REG_5C_TESTPA2, RH_RF69_TESTPA2_NORMAL);
	}
	setOpMode(_idleMode);
	_mode = RHModeIdle;
    }
}

bool RH_RF69::sleep()
{
    if (_mode != RHModeSleep)
    {
	spiWrite(RH_RF69_REG_01_OPMODE, RH_RF69_OPMODE_MODE_SLEEP);
	_mode = RHModeSleep;
    }
    return true;
}

void RH_RF69::setModeRx()
{
    if (_mode != RHModeRx)
    {
	if (_power >= 18)
	{
	    // If high power boost, return power amp to receive mode
	    spiWrite(RH_RF69_REG_5A_TESTPA1, RH_RF69_TESTPA1_NORMAL);
	    spiWrite(RH_RF69_REG_5C_TESTPA2, RH_RF69_TESTPA2_NORMAL);
	}
	spiWrite(RH_RF69_REG_25_DIOMAPPING1, RH_RF69_DIOMAPPING1_DIO0MAPPING_01); // Set interrupt line 0 PayloadReady
	setOpMode(RH_RF69_OPMODE_MODE_RX); // Clears FIFO
	_mode = RHModeRx;
    }
}

void RH_RF69::setModeTx()
{
    if (_mode != RHModeTx)
    {
	if (_power >= 18)
	{
	    // Set high power boost mode
	    // Note that OCP defaults to ON so no need to change that.
	    spiWrite(RH_RF69_REG_5A_TESTPA1, RH_RF69_TESTPA1_BOOST);
	    spiWrite(RH_RF69_REG_5C_TESTPA2, RH_RF69_TESTPA2_BOOST);
	}
	spiWrite(RH_RF69_REG_25_DIOMAPPING1, RH_RF69_DIOMAPPING1_DIO0MAPPING_00); // Set interrupt line 0 PacketSent
	setOpMode(RH_RF69_OPMODE_MODE_TX); // Clears FIFO
	_mode = RHModeTx;
    }
}

void RH_RF69::setTxPower(int8_t power, bool ishighpowermodule)
{
  _power = power;
  uint8_t palevel;
  
  if (ishighpowermodule)
  {
    if (_power < -2)
      _power = -2; //RFM69HW only works down to -2. 
    if (_power <= 13)
    {
      // -2dBm to +13dBm
      //Need PA1 exclusivelly on RFM69HW
      palevel = RH_RF69_PALEVEL_PA1ON | ((_power + 18) & 
      RH_RF69_PALEVEL_OUTPUTPOWER);
    }
    else if (_power >= 18)
    {
      // +18dBm to +20dBm
      // Need PA1+PA2
      // Also need PA boost settings change when tx is turned on and off, see setModeTx()
      palevel = RH_RF69_PALEVEL_PA1ON
	| RH_RF69_PALEVEL_PA2ON
	| ((_power + 11) & RH_RF69_PALEVEL_OUTPUTPOWER);
    }
    else
    {
      // +14dBm to +17dBm
      // Need PA1+PA2
      palevel = RH_RF69_PALEVEL_PA1ON
	| RH_RF69_PALEVEL_PA2ON
	| ((_power + 14) & RH_RF69_PALEVEL_OUTPUTPOWER);
    }
  }
  else
  {
    if (_power < -18) _power = -18;
    if (_power > 13) _power = 13; //limit for RFM69W
    palevel = RH_RF69_PALEVEL_PA0ON
      | ((_power + 18) & RH_RF69_PALEVEL_OUTPUTPOWER);
  }
  spiWrite(RH_RF69_REG_11_PALEVEL, palevel);
}

// Sets registers from a canned modem configuration structure
void RH_RF69::setModemRegisters(const ModemConfig* config)
{
    spiBurstWrite(RH_RF69_REG_02_DATAMODUL,     &config->reg_02, 5);
    spiBurstWrite(RH_RF69_REG_19_RXBW,          &config->reg_19, 2);
    spiWrite(RH_RF69_REG_37_PACKETCONFIG1,       config->reg_37);
}

// Set one of the canned FSK Modem configs
// Returns true if its a valid choice
bool RH_RF69::setModemConfig(ModemConfigChoice index)
{
    if (index > (signed int)(sizeof(MODEM_CONFIG_TABLE) / sizeof(ModemConfig)))
        return false;

    ModemConfig cfg;
    memcpy_P(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(RH_RF69::ModemConfig));
    setModemRegisters(&cfg);

    return true;
}

void RH_RF69::setPreambleLength(uint16_t bytes)
{
    spiWrite(RH_RF69_REG_2C_PREAMBLEMSB, bytes >> 8);
    spiWrite(RH_RF69_REG_2D_PREAMBLELSB, bytes & 0xff);
}

void RH_RF69::setSyncWords(const uint8_t* syncWords, uint8_t len)
{
    uint8_t syncconfig = spiRead(RH_RF69_REG_2E_SYNCCONFIG);
    if (syncWords && len && len <= 4)
    {
	spiBurstWrite(RH_RF69_REG_2F_SYNCVALUE1, syncWords, len);
	syncconfig |= RH_RF69_SYNCCONFIG_SYNCON;
    }
    else
	syncconfig &= ~RH_RF69_SYNCCONFIG_SYNCON;
    syncconfig &= ~RH_RF69_SYNCCONFIG_SYNCSIZE;
    syncconfig |= (len-1) << 3;
    spiWrite(RH_RF69_REG_2E_SYNCCONFIG, syncconfig);
}

void RH_RF69::setEncryptionKey(uint8_t* key)
{
    if (key)
    {
	spiBurstWrite(RH_RF69_REG_3E_AESKEY1, key, 16);
	spiWrite(RH_RF69_REG_3D_PACKETCONFIG2, spiRead(RH_RF69_REG_3D_PACKETCONFIG2) | RH_RF69_PACKETCONFIG2_AESON);
    }
    else
    {
	spiWrite(RH_RF69_REG_3D_PACKETCONFIG2, spiRead(RH_RF69_REG_3D_PACKETCONFIG2) & ~RH_RF69_PACKETCONFIG2_AESON);
    }
}

bool RH_RF69::available()
{
    if (_mode == RHModeTx)
	return false;
    setModeRx(); // Make sure we are receiving
    return _rxBufValid;
}

bool RH_RF69::recv(uint8_t* buf, uint8_t* len)
{
    if (!available())
	return false;

    if (buf && len)
    {
	ATOMIC_BLOCK_START;
	if (*len > _bufLen)
	    *len = _bufLen;
	memcpy(buf, _buf, *len);
	ATOMIC_BLOCK_END;
    }
    _rxBufValid = false; // Got the most recent message
//    printBuffer("recv:", buf, *len);
    return true;
}

bool RH_RF69::send(const uint8_t* data, uint8_t len)
{
    if (len > RH_RF69_MAX_MESSAGE_LEN)
	return false;

    waitPacketSent(); // Make sure we dont interrupt an outgoing message
    setModeIdle(); // Prevent RX while filling the fifo

    if (!waitCAD()) 
	return false;  // Check channel activity

    ATOMIC_BLOCK_START;
     _spi.beginTransaction();
    digitalWrite(_slaveSelectPin, LOW);
    _spi.transfer(RH_RF69_REG_00_FIFO | RH_RF69_SPI_WRITE_MASK); // Send the start address with the write mask on
    _spi.transfer(len + RH_RF69_HEADER_LEN); // Include length of headers
    // First the 4 headers
    _spi.transfer(_txHeaderTo);
    _spi.transfer(_txHeaderFrom);
    _spi.transfer(_txHeaderId);
    _spi.transfer(_txHeaderFlags);
    // Now the payload
    while (len--)
	_spi.transfer(*data++);
    digitalWrite(_slaveSelectPin, HIGH);
    _spi.endTransaction();
    ATOMIC_BLOCK_END;

    setModeTx(); // Start the transmitter
    return true;
}

uint8_t RH_RF69::maxMessageLength()
{
    return RH_RF69_MAX_MESSAGE_LEN;
}

bool RH_RF69::printRegister(uint8_t reg)
{  
#ifdef RH_HAVE_SERIAL
    Serial.print(reg, HEX);
    Serial.print(" ");
    Serial.println(spiRead(reg), HEX);
#endif
    return true;
}

bool RH_RF69::printRegisters()
{  
    uint8_t i;
    for (i = 0; i < 0x50; i++)
	printRegister(i);
    // Non-contiguous registers
    printRegister(RH_RF69_REG_58_TESTLNA);
    printRegister(RH_RF69_REG_6F_TESTDAGC);
    printRegister(RH_RF69_REG_71_TESTAFC);
    
    return true;
}
