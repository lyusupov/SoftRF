// RH_MRF89.cpp
//
// Copyright (C) 2015 Mike McCauley
// $Id: RH_MRF89.cpp,v 1.10 2019/09/02 05:21:52 mikem Exp $

#include <RH_MRF89.h>
#define BAND_915
#define DATA_RATE_200
#define LNA_GAIN LNA_GAIN_0_DB
#define TX_POWER TX_POWER_13_DB

// Interrupt vectors for the 3 Arduino interrupt pins
// Each interrupt can be handled by a different instance of RH_MRF89, allowing you to have
// 2 or more LORAs per Arduino
RH_MRF89* RH_MRF89::_deviceForInterrupt[RH_MRF89_NUM_INTERRUPTS] = {0, 0, 0};
uint8_t RH_MRF89::_interruptCount = 0; // Index into _deviceForInterrupt for next device

// These are indexed by the values of ModemConfigChoice
// Values based on sample modulation values from MRF89XA.h
// TXIPOLFV set to be more than Fd
PROGMEM static const RH_MRF89::ModemConfig MODEM_CONFIG_TABLE[] =
{
    // MODSEL, FDVAL, BRVAL, FILCREG=(PASFILV|BUTFILV), TXIPOLFV
    // FSK, No Manchester, Whitening
    { RH_MRF89_MODSEL_FSK, 0x0B, 0x63, 0x40 | 0x01, 0x20 }, // FSK_Rb2Fd33
    { RH_MRF89_MODSEL_FSK, 0x0B, 0x27, 0x40 | 0x01, 0x20 }, // FSK_Rb5Fd33
    { RH_MRF89_MODSEL_FSK, 0x0B, 0x13, 0x40 | 0x01, 0x20 }, // FSK_Rb10Fd33
    { RH_MRF89_MODSEL_FSK, 0x09, 0x09, 0x70 | 0x02, 0x20 }, // FSK_Rb20Fd40
    { RH_MRF89_MODSEL_FSK, 0x04, 0x04, 0xB0 | 0x05, 0x40 }, // FSK_Rb40Fd80
    { RH_MRF89_MODSEL_FSK, 0x03, 0x03, 0xD0 | 0x06, 0x40 }, // FSK_Rb50Fd100
    { RH_MRF89_MODSEL_FSK, 0x02, 0x02, 0xE0 | 0x09, 0x60 }, // FSK_Rb66Fd133
    { RH_MRF89_MODSEL_FSK, 0x01, 0x01, 0xF0 | 0x0F, 0x80 }, // FSK_Rb100Fd200
    { RH_MRF89_MODSEL_FSK, 0x01, 0x00, 0xF0 | 0x0F, 0x80 }  // FSK_Rb200Fd200

};


RH_MRF89::RH_MRF89(uint8_t csconPin, uint8_t csdatPin, uint8_t interruptPin, RHGenericSPI& spi)
    :
    RHNRFSPIDriver(csconPin, spi),
    _csconPin(csconPin),
    _csdatPin(csdatPin),
    _interruptPin(interruptPin)
{
    _myInterruptIndex = 0xff; // Not allocated yet
}

bool RH_MRF89::init()
{
    // MRF89 data cant handle SPI greater than 1MHz.  
    // Sigh on teensy at 1MHz, need special delay after writes, see RHNRFSPIDriver::spiWrite
    _spi.setFrequency(RHGenericSPI::Frequency1MHz);
    if (!RHNRFSPIDriver::init())
	return false;

    // Initialise the chip select pins
    pinMode(_csconPin, OUTPUT);
    digitalWrite(_csconPin, HIGH);
    pinMode(_csdatPin, OUTPUT);
    digitalWrite(_csdatPin, HIGH);

    // Determine the interrupt number that corresponds to the interruptPin
    int interruptNumber = digitalPinToInterrupt(_interruptPin);
    if (interruptNumber == NOT_AN_INTERRUPT)
	return false;
#ifdef RH_ATTACHINTERRUPT_TAKES_PIN_NUMBER
    interruptNumber = _interruptPin;
#endif

    // Tell the low level SPI interface we will use SPI within this interrupt
    spiUsingInterrupt(interruptNumber);

    // Make sure we are not in some unexpected mode from a previous run    
    setOpMode(RH_MRF89_CMOD_STANDBY); 

    // No way to check the device type but lets trivially check there is something there
    // by trying to change a register:
    spiWriteRegister(RH_MRF89_REG_02_FDEVREG, 0xaa);
    if (spiReadRegister(RH_MRF89_REG_02_FDEVREG) != 0xaa)
	return false;
    spiWriteRegister(RH_MRF89_REG_02_FDEVREG, 0x3); // Back to the default for FDEV
    if (spiReadRegister(RH_MRF89_REG_02_FDEVREG) != 0x3)
	return false;

    // Add by Adrien van den Bossche <vandenbo@univ-tlse2.fr> for Teensy
    // ARM M4 requires the below. else pin interrupt doesn't work properly.
    // On all other platforms, its innocuous, belt and braces
    pinMode(_interruptPin, INPUT); 

    // Set up interrupt handler
    // Since there are a limited number of interrupt glue functions isr*() available,
    // we can only support a limited number of devices simultaneously
    // On some devices, notably most Arduinos, the interrupt pin passed in is actually the 
    // interrupt number. You have to figure out the interruptnumber-to-interruptpin mapping
    // yourself based on knowledge of what Arduino board you are running on.
    if (_myInterruptIndex == 0xff)
    {
	// First run, no interrupt allocated yet
	if (_interruptCount <= RH_MRF89_NUM_INTERRUPTS)
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

    // When used with the MRF89XAM9A module, per 75017B.pdf section 1.3, need:
    // crystal freq = 12.8MHz
    // clock output disabled
    // frequency bands 902-915 or 915-928
    // VCOT 60mV
    // OOK max 28kbps
    // Based on 70622C.pdf, section 3.12:

    spiWriteRegister(RH_MRF89_REG_00_GCONREG, RH_MRF89_CMOD_STANDBY | RH_MRF89_FBS_902_915 | RH_MRF89_VCOT_60MV);
    spiWriteRegister(RH_MRF89_REG_01_DMODREG, RH_MRF89_MODSEL_FSK | RH_MRF89_OPMODE_PACKET); // FSK, Packet mode, LNA 0dB
    spiWriteRegister(RH_MRF89_REG_02_FDEVREG, 0); // Set by setModemConfig
    spiWriteRegister(RH_MRF89_REG_03_BRSREG,  0); // Set by setModemConfig
    spiWriteRegister(RH_MRF89_REG_04_FLTHREG, 0); // Set by setModemConfig (OOK only)
    spiWriteRegister(RH_MRF89_REG_05_FIFOCREG, RH_MRF89_FSIZE_64);
    spiWriteRegister(RH_MRF89_REG_06_R1CREG, 0); // Set by setFrequency
    spiWriteRegister(RH_MRF89_REG_07_P1CREG, 0); // Set by setFrequency
    spiWriteRegister(RH_MRF89_REG_08_S1CREG, 0); // Set by setFrequency
    spiWriteRegister(RH_MRF89_REG_09_R2CREG, 0); // Frequency set 2 not used
    spiWriteRegister(RH_MRF89_REG_0A_P2CREG, 0); // Frequency set 2 not used
    spiWriteRegister(RH_MRF89_REG_0B_S2CREG, 0); // Frequency set 2 not used
    spiWriteRegister(RH_MRF89_REG_0C_PACREG, RH_MRF89_PARC_23);
    // IRQ0 rx mode: SYNC (not used)
    // IRQ1 rx mode: CRCOK
    // IRQ1 tx mode: TXDONE
    spiWriteRegister(RH_MRF89_REG_0D_FTXRXIREG, RH_MRF89_IRQ0RXS_PACKET_SYNC | RH_MRF89_IRQ1RXS_PACKET_CRCOK | RH_MRF89_IRQ1TX);
    spiWriteRegister(RH_MRF89_REG_0E_FTPRIREG, RH_MRF89_LENPLL);
    spiWriteRegister(RH_MRF89_REG_0F_RSTHIREG, 0x00); // default not used if no RSSI interrupts
    spiWriteRegister(RH_MRF89_REG_10_FILCREG, 0); // Set by setModemConfig

    spiWriteRegister(RH_MRF89_REG_11_PFCREG, 0x38);// 100kHz, recommended, but not used, see RH_MRF89_REG_12_SYNCREG OOK only?
    spiWriteRegister(RH_MRF89_REG_12_SYNCREG, RH_MRF89_SYNCREN | RH_MRF89_SYNCWSZ_32); // No polyphase, no bsync, sync, 0 errors
    spiWriteRegister(RH_MRF89_REG_13_RSVREG, 0x07);//default
//    spiWriteRegister(RH_MRF89_REG_14_RSTSREG, 0x00); // NO, read only
    spiWriteRegister(RH_MRF89_REG_15_OOKCREG, 0x00); // Set by setModemConfig OOK only
    spiWriteRegister(RH_MRF89_REG_16_SYNCV31REG, 0x69); // Set by setSyncWords
    spiWriteRegister(RH_MRF89_REG_17_SYNCV23REG, 0x81); // Set by setSyncWords
    spiWriteRegister(RH_MRF89_REG_18_SYNCV15REG, 0x7E); // Set by setSyncWords
    spiWriteRegister(RH_MRF89_REG_19_SYNCV07REG, 0x96); // Set by setSyncWords
    // TXIPOLFV set by setModemConfig. power set by setTxPower
    spiWriteRegister(RH_MRF89_REG_1A_TXCONREG, 0xf0 | RH_MRF89_TXOPVAL_13DBM); // TX cutoff freq=375kHz,
    spiWriteRegister(RH_MRF89_REG_1B_CLKOREG, 0x00); // Disable clock output to save power
    spiWriteRegister(RH_MRF89_REG_1C_PLOADREG, 0x40); // payload=64bytes (no RX-filtering on packet length)
    spiWriteRegister(RH_MRF89_REG_1D_NADDSREG, 0x00); // Node Address (0=default) Not used
    spiWriteRegister(RH_MRF89_REG_1E_PKTCREG, RH_MRF89_PKTLENF | RH_MRF89_PRESIZE_4 | RH_MRF89_WHITEON | RH_MRF89_CHKCRCEN | RH_MRF89_ADDFIL_OFF);
    spiWriteRegister(RH_MRF89_REG_1F_FCRCREG, 0x00); // default (FIFO access in standby=write, clear FIFO on CRC mismatch)

    // Looking OK now
    // Set some suitable defaults:
    setPreambleLength(3); // The default
    uint8_t syncwords[] = { 0x69, 0x81, 0x7e, 0x96 }; // Same as RH_MRF89XA
    setSyncWords(syncwords, sizeof(syncwords));
    setTxPower(RH_MRF89_TXOPVAL_1DBM);
    // try first MRF89XAM9A then MRF89XAM8A
    if (!setFrequency(915.4) && !setFrequency(865.0))
	return false;
    // Some slow, reliable default speed and modulation
    if (!setModemConfig(FSK_Rb20Fd40))
	return false;

    return true;
}

bool RH_MRF89::printRegisters()
{
#ifdef RH_HAVE_SERIAL
    uint8_t i;
    for (i = 0; i <= 0x1f; i++)
    {
	Serial.print(i, HEX);
	Serial.print(": ");
	Serial.println(spiReadRegister(i), HEX);
    }
#endif
    return true;
}

// C++ level interrupt handler for this instance
// MRF89XA is unusual in that it has 2 interrupt lines, and not a single, combined one.
// Only one of the several interrupt lines (IRQ1) from the RFM95 needs to be
// connnected to the processor.
// We use this to get CRCOK and TXDONE  interrupts
void RH_MRF89::handleInterrupt()
{
//    Serial.println("I");
    if (_mode == RHModeTx)
    {
//    Serial.println("T");
	// TXDONE
	// Transmit is complete
	_txGood++;
	setModeIdle();
    }
    else if (_mode == RHModeRx)
    {
//	Serial.println("R");
	// CRCOK
	// We have received a packet.
        // First byte in FIFO is packet length

	// REVISIT: Capture last rssi from RSTSREG
	// based roughly on Figure 3-9
	_lastRssi = (spiReadRegister(RH_MRF89_REG_14_RSTSREG) >> 1) - 120;

	_bufLen = spiReadData();
	if (_bufLen < 4)
	{
	    // Drain the FIFO
	    uint8_t i;
	    for (i = 0; spiReadRegister(RH_MRF89_REG_0D_FTXRXIREG) & RH_MRF89_FIFOEMPTY; i++)
		spiReadData();
	    clearRxBuf();
	    return;
	}
	
	// Now drain all the data from the FIFO into _buf
	uint8_t i;
	for (i = 0; spiReadRegister(RH_MRF89_REG_0D_FTXRXIREG) & RH_MRF89_FIFOEMPTY; i++)
	    _buf[i] = spiReadData();

	// All good. See if its for us
	validateRxBuf(); 
	if (_rxBufValid)
	    setModeIdle(); // Got one 
    }
}

// These are low level functions that call the interrupt handler for the correct
// instance of RH_MRF89.
// 3 interrupts allows us to have 3 different devices
void RH_INTERRUPT_ATTR RH_MRF89::isr0()
{
    if (_deviceForInterrupt[0])
	_deviceForInterrupt[0]->handleInterrupt();
}
void RH_INTERRUPT_ATTR RH_MRF89::isr1()
{
    if (_deviceForInterrupt[1])
	_deviceForInterrupt[1]->handleInterrupt();
}
void RH_INTERRUPT_ATTR RH_MRF89::isr2()
{
    if (_deviceForInterrupt[2])
	_deviceForInterrupt[2]->handleInterrupt();
}

uint8_t RH_MRF89::spiReadRegister(uint8_t reg)
{
    // Tell the chip we want to talk to the configuration registers
    setSlaveSelectPin(_csconPin);
    digitalWrite(_csdatPin, HIGH);
    return spiRead(((reg & 0x1f) << 1) | RH_MRF89_SPI_READ_MASK);
}

uint8_t RH_MRF89::spiWriteRegister(uint8_t reg, uint8_t val)
{
    // Tell the chip we want to talk to the configuration registers
    setSlaveSelectPin(_csconPin);
    digitalWrite(_csdatPin, HIGH);
    // Hmmm, on teensy 3.1, needed some special behaviour in RHNRFSPIDriver::spiWrite
    // because otherwise, CSCON returns high before the final clock goes low,
    // which prevents the MRF89XA spi write succeeding. Clock must be low when CSCON goes high.
    return spiWrite(((reg & 0x1f) << 1), val);
}

uint8_t RH_MRF89::spiWriteData(uint8_t data)
{
    spiWriteRegister(RH_MRF89_REG_1F_FCRCREG, RH_MRF89_ACFCRC); // Write to FIFO
    setSlaveSelectPin(_csdatPin);
    digitalWrite(_csconPin, HIGH);
    return spiCommand(data);
}

uint8_t RH_MRF89::spiWriteData(const uint8_t* data, uint8_t len)
{
    spiWriteRegister(RH_MRF89_REG_1F_FCRCREG, RH_MRF89_ACFCRC); // Write to FIFO
    setSlaveSelectPin(_csdatPin);
    digitalWrite(_csconPin, HIGH);

    uint8_t status = 0;
    ATOMIC_BLOCK_START;
    _spi.beginTransaction();
    digitalWrite(_slaveSelectPin, LOW);
    while (len--)
	_spi.transfer(*data++);
    digitalWrite(_slaveSelectPin, HIGH);
    _spi.endTransaction();
    ATOMIC_BLOCK_END;
    return status;

}

uint8_t RH_MRF89::spiReadData()
{
    spiWriteRegister(RH_MRF89_REG_1F_FCRCREG, RH_MRF89_ACFCRC | RH_MRF89_FRWAXS); // Read from FIFO
    setSlaveSelectPin(_csdatPin);
    digitalWrite(_csconPin, HIGH);
    return spiCommand(0);
}

void RH_MRF89::setOpMode(uint8_t mode)
{
    // REVISIT: do we need to have time delays when switching between modes?
    uint8_t val = spiReadRegister(RH_MRF89_REG_00_GCONREG);
    val = (val & ~RH_MRF89_CMOD) | (mode & RH_MRF89_CMOD);
    spiWriteRegister(RH_MRF89_REG_00_GCONREG, val);
}

void RH_MRF89::setModeIdle()
{
    if (_mode != RHModeIdle)
    {
	setOpMode(RH_MRF89_CMOD_STANDBY);
	_mode = RHModeIdle;
    }
}

bool RH_MRF89::sleep()
{
    if (_mode != RHModeSleep)
    {
	setOpMode(RH_MRF89_CMOD_SLEEP);
	_mode = RHModeSleep;
    }
    return true;
}

void RH_MRF89::setModeRx()
{
    if (_mode != RHModeRx)
    {
	setOpMode(RH_MRF89_CMOD_RECEIVE);
	_mode = RHModeRx;
    }
}

void RH_MRF89::setModeTx()
{
    if (_mode != RHModeTx)
    {
	setOpMode(RH_MRF89_CMOD_TRANSMIT);
	_mode = RHModeTx;
    }
}

void RH_MRF89::setTxPower(uint8_t power)
{
    uint8_t txconreg = spiReadRegister(RH_MRF89_REG_1A_TXCONREG);
    txconreg |= (power & RH_MRF89_TXOPVAL);
    spiWriteRegister(RH_MRF89_REG_1A_TXCONREG, txconreg);
}

bool RH_MRF89::available()
{
    if (_mode == RHModeTx)
	return false;
    setModeRx();

    return _rxBufValid; // Will be set by the interrupt handler when a good message is received
}

bool RH_MRF89::recv(uint8_t* buf, uint8_t* len)
{
    if (!available())
	return false;

    if (buf && len)
    {
	ATOMIC_BLOCK_START;
	// Skip the 4 headers that are at the beginning of the rxBuf
	if (*len > _bufLen - RH_MRF89_HEADER_LEN)
	    *len = _bufLen - RH_MRF89_HEADER_LEN;
	memcpy(buf, _buf + RH_MRF89_HEADER_LEN, *len);
	ATOMIC_BLOCK_END;
    }
    clearRxBuf(); // This message accepted and cleared

    return true;
}

bool RH_MRF89::send(const uint8_t* data, uint8_t len)
{
    if (len > RH_MRF89_MAX_MESSAGE_LEN)
	return false;
    
    waitPacketSent(); // Make sure we dont interrupt an outgoing message
    setModeIdle();
    
    if (!waitCAD()) 
	return false;  // Check channel activity

    // First octet is the length of the chip payload
    // 0 length messages are transmitted but never trigger a receive!
    spiWriteData(len + RH_MRF89_HEADER_LEN);
    spiWriteData(_txHeaderTo);
    spiWriteData(_txHeaderFrom);
    spiWriteData(_txHeaderId);
    spiWriteData(_txHeaderFlags);
    spiWriteData(data, len);
    setModeTx(); // Start transmitting

    return true;
}

uint8_t RH_MRF89::maxMessageLength()
{
    return RH_MRF89_MAX_MESSAGE_LEN;
}

// Check whether the latest received message is complete and uncorrupted
void RH_MRF89::validateRxBuf()
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

void RH_MRF89::clearRxBuf()
{
    ATOMIC_BLOCK_START;
    _rxBufValid = false;
    _bufLen = 0;
    ATOMIC_BLOCK_END;
}

bool RH_MRF89::verifyPLLLock()
{
    // Verify PLL-lock per instructions in Note 1 section 3.12
    // Need to do this after changing frequency.
    uint8_t ftpriVal = spiReadRegister(RH_MRF89_REG_0E_FTPRIREG);
    spiWriteRegister(RH_MRF89_REG_0E_FTPRIREG, ftpriVal | RH_MRF89_LSTSPLL); // Clear PLL lock bit
    setOpMode(RH_MRF89_CMOD_FS);
    unsigned long ulStartTime = millis();
    while ((millis() - ulStartTime < 1000))
    {
        ftpriVal = spiReadRegister(RH_MRF89_REG_0E_FTPRIREG);
        if ((ftpriVal & RH_MRF89_LSTSPLL) != 0)
            break;
    }
    setOpMode(RH_MRF89_CMOD_STANDBY);
    return ((ftpriVal & RH_MRF89_LSTSPLL) != 0);
}

bool RH_MRF89::setFrequency(float centre)
{
    // REVISIT: FSK only: its different for OOK :-(

    uint8_t FBS;
    if (centre >= 902.0 && centre < 915.0)
    {
	// The MRF89XAM9A does support this frequency band
	FBS = RH_MRF89_FBS_902_915;
    }
    else if (centre >= 915.0 && centre <= 928.0)
    {
	// The MRF89XAM9A does support this frequency band
	FBS = RH_MRF89_FBS_915_928;
    }
    else if (centre >= 950.0 && centre <= 960.0)
    {
	// Not all modules support this frequency band:
	// The MRF98XAM9A does not
        // The MRF89XA does support this frequency band
        FBS = RH_MRF89_FBS_950_960_or_863_870;
    }
    else if (centre >= 863.0 && centre <= 870.0)
    {
	// Not all modules support this frequency band:
	// The MRF98XAM9A does not
        // The MRF89XAM8A does support this frequency band
        FBS = RH_MRF89_FBS_950_960_or_863_870;
    }
    else
    {
	// Cant do this freq
	return false;
    }

    // Based on frequency calcs done in MRF89XA.h
//    uint8_t R = 100; // Recommended
    uint8_t R = 119; // Also recommended :-(
    uint32_t centre_kHz = centre * 1000;
    uint32_t xtal_kHz = (RH_MRF89_XTAL_FREQ * 1000);
    uint32_t compare = (centre_kHz * 8 * (R + 1)) / (9 * xtal_kHz);
    uint8_t P = ((compare - 75) / 76) + 1;
    uint8_t S = compare - (75 * (P + 1));

    // Now set the new register values:
    uint8_t val = spiReadRegister(RH_MRF89_REG_00_GCONREG);
    val = (val & ~RH_MRF89_FBS) | (FBS & RH_MRF89_FBS);
    spiWriteRegister(RH_MRF89_REG_00_GCONREG, val);

    spiWriteRegister(RH_MRF89_REG_06_R1CREG, R); 
    spiWriteRegister(RH_MRF89_REG_07_P1CREG, P); 
    spiWriteRegister(RH_MRF89_REG_08_S1CREG, S); 

    return verifyPLLLock();
}

// Set one of the canned FSK Modem configs
// Returns true if its a valid choice
bool RH_MRF89::setModemConfig(ModemConfigChoice index)
{
    if (index > (signed int)(sizeof(MODEM_CONFIG_TABLE) / sizeof(ModemConfig)))
        return false;

    RH_MRF89::ModemConfig cfg;
    memcpy_P(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(cfg));

    // Now update the registers
    uint8_t val = spiReadRegister(RH_MRF89_REG_01_DMODREG);
    val = (val & ~RH_MRF89_MODSEL) | cfg.MODSEL;
    spiWriteRegister(RH_MRF89_REG_01_DMODREG, val);

    spiWriteRegister(RH_MRF89_REG_02_FDEVREG, cfg.FDVAL);
    spiWriteRegister(RH_MRF89_REG_03_BRSREG,  cfg.BRVAL);
    spiWriteRegister(RH_MRF89_REG_10_FILCREG, cfg.FILCREG);

    // The sample configs in MRF89XA.h all use TXIPOLFV = 0xf0 => 375kHz, which is too wide for most modulations
    val = spiReadRegister(RH_MRF89_REG_1A_TXCONREG);
    val = (val & ~RH_MRF89_TXIPOLFV) | (cfg.TXIPOLFV & RH_MRF89_TXIPOLFV);
    spiWriteRegister(RH_MRF89_REG_1A_TXCONREG, val);

    return true;
}

void RH_MRF89::setPreambleLength(uint8_t bytes)
{
    if (bytes >= 1 && bytes <= 4)
    {
	bytes--;
	uint8_t pktcreg = spiReadRegister(RH_MRF89_REG_1E_PKTCREG);
	pktcreg = (pktcreg & ~RH_MRF89_PRESIZE) | ((bytes << 5) & RH_MRF89_PRESIZE);
	spiWriteRegister(RH_MRF89_REG_1E_PKTCREG, pktcreg);
    }
}

void RH_MRF89::setSyncWords(const uint8_t* syncWords, uint8_t len)
{
    if (syncWords && (len > 0 and len <= 4))
    {
	uint8_t syncreg = spiReadRegister(RH_MRF89_REG_12_SYNCREG);
	syncreg = (syncreg & ~RH_MRF89_SYNCWSZ) | (((len - 1) << 3) & RH_MRF89_SYNCWSZ);
	spiWriteRegister(RH_MRF89_REG_12_SYNCREG, syncreg);
	uint8_t i;
	for (i = 0; i < 4; i++)
	{
	    if (len > i)
		spiWriteRegister(RH_MRF89_REG_16_SYNCV31REG + i, syncWords[i]);
	}
    }
}

