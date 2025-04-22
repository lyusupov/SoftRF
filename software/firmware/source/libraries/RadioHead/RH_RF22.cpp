// RH_RF22.cpp
//
// Copyright (C) 2011 Mike McCauley
// $Id: RH_RF22.cpp,v 1.33 2020/07/05 08:52:21 mikem Exp $

#include <RH_RF22.h>

#if RH_PLATFORM == RH_PLATFORM_ESP8266
// This voltatile array is used in the ESP8266 platform to manage the interrupt
// service routines in a main loop, avoiding SPI calls inside the isr functions.
volatile bool flagIsr[3] = {false, false, false};
#endif

// Interrupt vectors for the 2 Arduino interrupt pins
// Each interrupt can be handled by a different instance of RH_RF22, allowing you to have
// 2 RH_RF22s per Arduino
RH_RF22* RH_RF22::_deviceForInterrupt[RH_RF22_NUM_INTERRUPTS] = {0, 0, 0};
uint8_t RH_RF22::_interruptCount = 0; // Index into _deviceForInterrupt for next device

// These are indexed by the values of ModemConfigChoice
// Canned modem configurations generated with 
// http://www.hoperf.com/upload/rf/RH_RF22B%2023B%2031B%2042B%2043B%20Register%20Settings_RevB1-v5.xls
// Stored in flash (program) memory to save SRAM
PROGMEM static const RH_RF22::ModemConfig MODEM_CONFIG_TABLE[] =
{
    { 0x2b, 0x03, 0xf4, 0x20, 0x41, 0x89, 0x00, 0x36, 0x40, 0x0a, 0x1d, 0x80, 0x60, 0x10, 0x62, 0x2c, 0x00, 0x08 }, // Unmodulated carrier
    { 0x2b, 0x03, 0xf4, 0x20, 0x41, 0x89, 0x00, 0x36, 0x40, 0x0a, 0x1d, 0x80, 0x60, 0x10, 0x62, 0x2c, 0x33, 0x08 }, // FSK, PN9 random modulation, 2, 5

    // All the following enable FIFO with reg 71
    //  1c,   1f,   20,   21,   22,   23,   24,   25,   2c,   2d,   2e,   58,   69,   6e,   6f,   70,   71,   72
    // FSK, No Manchester, Max Rb err <1%, Xtal Tol 20ppm
    { 0x2b, 0x03, 0xf4, 0x20, 0x41, 0x89, 0x00, 0x36, 0x40, 0x0a, 0x1d, 0x80, 0x60, 0x10, 0x62, 0x2c, 0x22, 0x08 }, // 2, 5
    { 0x1b, 0x03, 0x41, 0x60, 0x27, 0x52, 0x00, 0x07, 0x40, 0x0a, 0x1e, 0x80, 0x60, 0x13, 0xa9, 0x2c, 0x22, 0x3a }, // 2.4, 36
    { 0x1d, 0x03, 0xa1, 0x20, 0x4e, 0xa5, 0x00, 0x13, 0x40, 0x0a, 0x1e, 0x80, 0x60, 0x27, 0x52, 0x2c, 0x22, 0x48 }, // 4.8, 45
    { 0x1e, 0x03, 0xd0, 0x00, 0x9d, 0x49, 0x00, 0x45, 0x40, 0x0a, 0x20, 0x80, 0x60, 0x4e, 0xa5, 0x2c, 0x22, 0x48 }, // 9.6, 45
    { 0x2b, 0x03, 0x34, 0x02, 0x75, 0x25, 0x07, 0xff, 0x40, 0x0a, 0x1b, 0x80, 0x60, 0x9d, 0x49, 0x2c, 0x22, 0x0f }, // 19.2, 9.6
    { 0x02, 0x03, 0x68, 0x01, 0x3a, 0x93, 0x04, 0xd5, 0x40, 0x0a, 0x1e, 0x80, 0x60, 0x09, 0xd5, 0x0c, 0x22, 0x1f }, // 38.4, 19.6
    { 0x06, 0x03, 0x45, 0x01, 0xd7, 0xdc, 0x07, 0x6e, 0x40, 0x0a, 0x2d, 0x80, 0x60, 0x0e, 0xbf, 0x0c, 0x22, 0x2e }, // 57.6. 28.8
    { 0x8a, 0x03, 0x60, 0x01, 0x55, 0x55, 0x02, 0xad, 0x40, 0x0a, 0x50, 0x80, 0x60, 0x20, 0x00, 0x0c, 0x22, 0xc8 }, // 125, 125

    { 0x2b, 0x03, 0xa1, 0xe0, 0x10, 0xc7, 0x00, 0x09, 0x40, 0x0a, 0x1d,  0x80, 0x60, 0x04, 0x32, 0x2c, 0x22, 0x04 }, // 512 baud, FSK, 2.5 Khz fd for POCSAG compatibility
    { 0x27, 0x03, 0xa1, 0xe0, 0x10, 0xc7, 0x00, 0x06, 0x40, 0x0a, 0x1d,  0x80, 0x60, 0x04, 0x32, 0x2c, 0x22, 0x07 }, // 512 baud, FSK, 4.5 Khz fd for POCSAG compatibility

    // GFSK, No Manchester, Max Rb err <1%, Xtal Tol 20ppm
    // These differ from FSK only in register 71, for the modulation type
    { 0x2b, 0x03, 0xf4, 0x20, 0x41, 0x89, 0x00, 0x36, 0x40, 0x0a, 0x1d, 0x80, 0x60, 0x10, 0x62, 0x2c, 0x23, 0x08 }, // 2, 5
    { 0x1b, 0x03, 0x41, 0x60, 0x27, 0x52, 0x00, 0x07, 0x40, 0x0a, 0x1e, 0x80, 0x60, 0x13, 0xa9, 0x2c, 0x23, 0x3a }, // 2.4, 36
    { 0x1d, 0x03, 0xa1, 0x20, 0x4e, 0xa5, 0x00, 0x13, 0x40, 0x0a, 0x1e, 0x80, 0x60, 0x27, 0x52, 0x2c, 0x23, 0x48 }, // 4.8, 45
    { 0x1e, 0x03, 0xd0, 0x00, 0x9d, 0x49, 0x00, 0x45, 0x40, 0x0a, 0x20, 0x80, 0x60, 0x4e, 0xa5, 0x2c, 0x23, 0x48 }, // 9.6, 45
    { 0x2b, 0x03, 0x34, 0x02, 0x75, 0x25, 0x07, 0xff, 0x40, 0x0a, 0x1b, 0x80, 0x60, 0x9d, 0x49, 0x2c, 0x23, 0x0f }, // 19.2, 9.6
    { 0x02, 0x03, 0x68, 0x01, 0x3a, 0x93, 0x04, 0xd5, 0x40, 0x0a, 0x1e, 0x80, 0x60, 0x09, 0xd5, 0x0c, 0x23, 0x1f }, // 38.4, 19.6
    { 0x06, 0x03, 0x45, 0x01, 0xd7, 0xdc, 0x07, 0x6e, 0x40, 0x0a, 0x2d, 0x80, 0x60, 0x0e, 0xbf, 0x0c, 0x23, 0x2e }, // 57.6. 28.8
    { 0x8a, 0x03, 0x60, 0x01, 0x55, 0x55, 0x02, 0xad, 0x40, 0x0a, 0x50, 0x80, 0x60, 0x20, 0x00, 0x0c, 0x23, 0xc8 }, // 125, 125

    // OOK, No Manchester, Max Rb err <1%, Xtal Tol 20ppm
    { 0x51, 0x03, 0x68, 0x00, 0x3a, 0x93, 0x01, 0x3d, 0x2c, 0x11, 0x28, 0x80, 0x60, 0x09, 0xd5, 0x2c, 0x21, 0x08 }, // 1.2, 75
    { 0xc8, 0x03, 0x39, 0x20, 0x68, 0xdc, 0x00, 0x6b, 0x2a, 0x08, 0x2a, 0x80, 0x60, 0x13, 0xa9, 0x2c, 0x21, 0x08 }, // 2.4, 335
    { 0xc8, 0x03, 0x9c, 0x00, 0xd1, 0xb7, 0x00, 0xd4, 0x29, 0x04, 0x29, 0x80, 0x60, 0x27, 0x52, 0x2c, 0x21, 0x08 }, // 4.8, 335
    { 0xb8, 0x03, 0x9c, 0x00, 0xd1, 0xb7, 0x00, 0xd4, 0x28, 0x82, 0x29, 0x80, 0x60, 0x4e, 0xa5, 0x2c, 0x21, 0x08 }, // 9.6, 335
    { 0xa8, 0x03, 0x9c, 0x00, 0xd1, 0xb7, 0x00, 0xd4, 0x28, 0x41, 0x29, 0x80, 0x60, 0x9d, 0x49, 0x2c, 0x21, 0x08 }, // 19.2, 335
    { 0x98, 0x03, 0x9c, 0x00, 0xd1, 0xb7, 0x00, 0xd4, 0x28, 0x20, 0x29, 0x80, 0x60, 0x09, 0xd5, 0x0c, 0x21, 0x08 }, // 38.4, 335
    { 0x98, 0x03, 0x96, 0x00, 0xda, 0x74, 0x00, 0xdc, 0x28, 0x1f, 0x29, 0x80, 0x60, 0x0a, 0x3d, 0x0c, 0x21, 0x08 }, // 40, 335
};

RH_RF22::RH_RF22(uint8_t slaveSelectPin, uint8_t interruptPin, RHGenericSPI& spi)
    :
    RHSPIDriver(slaveSelectPin, spi)
{
    _interruptPin = interruptPin;
    _idleMode = RH_RF22_XTON; // Default idle state is READY mode
    _polynomial = CRC_16_IBM; // Historical
    _myInterruptIndex = 0xff; // Not allocated yet
}

void RH_RF22::setIdleMode(uint8_t idleMode)
{
    _idleMode = idleMode;
}

bool RH_RF22::init()
{
#if RH_PLATFORM == RH_PLATFORM_ESP8266
    flagIsr[0] = false;
    flagIsr[1] = false;
    flagIsr[2] = false;
#endif

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

    // Software reset the device
    reset();

    // Get the device type and check it
    // This also tests whether we are really connected to a device
    _deviceType = spiRead(RH_RF22_REG_00_DEVICE_TYPE);
    if (   _deviceType != RH_RF22_DEVICE_TYPE_RX_TRX
        && _deviceType != RH_RF22_DEVICE_TYPE_TX)
    {
//	Serial.print("unknown device type: ");
//	Serial.println(_deviceType);
	return false;
    }

    // Issue software reset to get all registers to default state
    spiWrite(RH_RF22_REG_07_OPERATING_MODE1, RH_RF22_SWRES);
    // Wait for chip ready
    while (!(spiRead(RH_RF22_REG_04_INTERRUPT_STATUS2) & RH_RF22_ICHIPRDY))
	;
    
    // Add by Adrien van den Bossche <vandenbo@univ-tlse2.fr> for Teensy
    // ARM M4 requires the below. else pin interrupt doesn't work properly.
    // On all other platforms, its innocuous, belt and braces
    pinMode(_interruptPin, INPUT); 
    
    // Enable interrupt output on the radio. Interrupt line will now go high until
    // an interrupt occurs
    spiWrite(RH_RF22_REG_05_INTERRUPT_ENABLE1, RH_RF22_ENTXFFAEM | RH_RF22_ENRXFFAFULL | RH_RF22_ENPKSENT | RH_RF22_ENPKVALID | RH_RF22_ENCRCERROR | RH_RF22_ENFFERR);
    spiWrite(RH_RF22_REG_06_INTERRUPT_ENABLE2, RH_RF22_ENPREAVAL);

    // Set up interrupt handler
    // Since there are a limited number of interrupt glue functions isr*() available,
    // we can only support a limited number of devices simultaneously
    // On some devices, notably most Arduinos, the interrupt pin passed in is actually the 
    // interrupt number. You have to figure out the interruptnumber-to-interruptpin mapping
    // yourself based on knowledge of what Arduino board you are running on.
    if (_myInterruptIndex == 0xff)
    {
	// First run, no interrupt allocated yet
	if (_interruptCount <= RH_RF22_NUM_INTERRUPTS)
	    _myInterruptIndex = _interruptCount++;
	else
	    return false; // Too many devices, not enough interrupt vectors
    }
    _deviceForInterrupt[_myInterruptIndex] = this;
    if (_myInterruptIndex == 0)
	attachInterrupt(interruptNumber, isr0, FALLING);
    else if (_myInterruptIndex == 1)
	attachInterrupt(interruptNumber, isr1, FALLING);
    else if (_myInterruptIndex == 2)
	attachInterrupt(interruptNumber, isr2, FALLING);
    else
	return false; // Too many devices, not enough interrupt vectors

    setModeIdle();

    clearTxBuf();
    clearRxBuf();

    // Most of these are the POR default
    spiWrite(RH_RF22_REG_7D_TX_FIFO_CONTROL2, RH_RF22_TXFFAEM_THRESHOLD);
    spiWrite(RH_RF22_REG_7E_RX_FIFO_CONTROL,  RH_RF22_RXFFAFULL_THRESHOLD);
    spiWrite(RH_RF22_REG_30_DATA_ACCESS_CONTROL, RH_RF22_ENPACRX | RH_RF22_ENPACTX | RH_RF22_ENCRC | (_polynomial & RH_RF22_CRC));

    // Configure the message headers
    // Here we set up the standard packet format for use by the RH_RF22 library
    // 8 nibbles preamble
    // 2 SYNC words 2d, d4
    // Header length 4 (to, from, id, flags)
    // 1 octet of data length (0 to 255)
    // 0 to 255 octets data
    // 2 CRC octets as CRC16(IBM), computed on the header, length and data
    // On reception the to address is check for validity against RH_RF22_REG_3F_CHECK_HEADER3
    // or the broadcast address of 0xff
    // If no changes are made after this, the transmitted
    // to address will be 0xff, the from address will be 0xff
    // and all such messages will be accepted. This permits the out-of the box
    // RH_RF22 config to act as an unaddresed, unreliable datagram service
    spiWrite(RH_RF22_REG_32_HEADER_CONTROL1, RH_RF22_BCEN_HEADER3 | RH_RF22_HDCH_HEADER3);
    spiWrite(RH_RF22_REG_33_HEADER_CONTROL2, RH_RF22_HDLEN_4 | RH_RF22_SYNCLEN_2);

    setPreambleLength(8);
    uint8_t syncwords[] = { 0x2d, 0xd4 };
    setSyncWords(syncwords, sizeof(syncwords));
    setPromiscuous(false); 

    // Set some defaults. An innocuous ISM frequency, and reasonable pull-in
    setFrequency(434.0, 0.05);
//    setFrequency(900.0);
    // Some slow, reliable default speed and modulation
    setModemConfig(FSK_Rb2_4Fd36);
//    setModemConfig(FSK_Rb125Fd125);
    setGpioReversed(false);
    // Lowish power
    setTxPower(RH_RF22_TXPOW_8DBM);

    return true;
}

// C++ level interrupt handler for this instance
void RH_RF22::handleInterrupt()
{
    uint8_t _lastInterruptFlags[2];
    // Read the interrupt flags which clears the interrupt
    spiBurstRead(RH_RF22_REG_03_INTERRUPT_STATUS1, _lastInterruptFlags, 2);

#if 0
    // DEVELOPER TESTING ONLY
    // Caution: Serial printing in this interrupt routine can cause mysterious crashes
    Serial.print("interrupt ");
    Serial.print(_lastInterruptFlags[0], HEX);
    Serial.print(" ");
    Serial.println(_lastInterruptFlags[1], HEX);
    if (_lastInterruptFlags[0] == 0 && _lastInterruptFlags[1] == 0)
	Serial.println("FUNNY: no interrupt!");
#endif

#if 0
    // DEVELOPER TESTING ONLY
    // TESTING: fake an RH_RF22_IFFERROR
    static int counter = 0;
    if (_lastInterruptFlags[0] & RH_RF22_IPKSENT && counter++ == 10)
    {
	_lastInterruptFlags[0] = RH_RF22_IFFERROR;
	counter = 0;
    }
#endif

    if (_lastInterruptFlags[0] & RH_RF22_IFFERROR)
    {
	resetFifos(); // Clears the interrupt
	if (_mode == RHModeTx)
	    restartTransmit();
	else if (_mode == RHModeRx)
	    clearRxBuf();
//	Serial.println("IFFERROR");  
    }
    // Caution, any delay here may cause a FF underflow or overflow
    if (_lastInterruptFlags[0] & RH_RF22_ITXFFAEM)
    {
	// See if more data has to be loaded into the Tx FIFO 
  	sendNextFragment();
//	Serial.println("ITXFFAEM");  
    }
    if (_lastInterruptFlags[0] & RH_RF22_IRXFFAFULL)
    {
	// Caution, any delay here may cause a FF overflow
	// Read some data from the Rx FIFO
	readNextFragment();
//	Serial.println("IRXFFAFULL"); 
    }
    if (_lastInterruptFlags[0] & RH_RF22_IEXT)
    {
	// This is not enabled by the base code, but users may want to enable it
	handleExternalInterrupt();
//	Serial.println("IEXT"); 
    }
    if (_lastInterruptFlags[1] & RH_RF22_IWUT)
    {
	// This is not enabled by the base code, but users may want to enable it
	handleWakeupTimerInterrupt();
//	Serial.println("IWUT"); 
    }
    if (_lastInterruptFlags[0] & RH_RF22_IPKSENT)
    {
//	Serial.println("IPKSENT");   
	_txGood++; 
	// Transmission does not automatically clear the tx buffer.
	// Could retransmit if we wanted
	// RH_RF22 transitions automatically to Idle
	_mode = RHModeIdle;
    }
    if (_lastInterruptFlags[0] & RH_RF22_IPKVALID)
    {
	uint8_t len = spiRead(RH_RF22_REG_4B_RECEIVED_PACKET_LENGTH);
//	Serial.println("IPKVALID");   

	// May have already read one or more fragments
	// Get any remaining unread octets, based on the expected length
	// First make sure we dont overflow the buffer in the case of a stupid length
	// or partial bad receives
	if (   len >  RH_RF22_MAX_MESSAGE_LEN
	    || len < _bufLen)
	{
	    _rxBad++;
	    _mode = RHModeIdle;
	    clearRxBuf();
	    return; // Hmmm receiver buffer overflow. 
	}

	spiBurstRead(RH_RF22_REG_7F_FIFO_ACCESS, _buf + _bufLen, len - _bufLen);
	_rxHeaderTo = spiRead(RH_RF22_REG_47_RECEIVED_HEADER3);
	_rxHeaderFrom = spiRead(RH_RF22_REG_48_RECEIVED_HEADER2);
	_rxHeaderId = spiRead(RH_RF22_REG_49_RECEIVED_HEADER1);
	_rxHeaderFlags = spiRead(RH_RF22_REG_4A_RECEIVED_HEADER0);
	_rxGood++;
	_bufLen = len;
	_mode = RHModeIdle;
	_rxBufValid = true;
    }
    if (_lastInterruptFlags[0] & RH_RF22_ICRCERROR)
    {
//	Serial.println("ICRCERR");  
	_rxBad++;
	clearRxBuf();
	resetRxFifo();
	_mode = RHModeIdle;
	setModeRx(); // Keep trying
    }
    if (_lastInterruptFlags[1] & RH_RF22_IPREAVAL)
    {
//	Serial.println("IPREAVAL");  
	_lastRssi = (int8_t)(-120 + ((spiRead(RH_RF22_REG_26_RSSI) / 2)));
	_lastPreambleTime = millis();
	resetRxFifo();
	clearRxBuf();
    }
}

#if RH_PLATFORM == RH_PLATFORM_ESP8266
void RH_RF22::loopIsr()
{
    if (flagIsr[0])
    {
	if (_deviceForInterrupt[0])
	    _deviceForInterrupt[0]->handleInterrupt();
	flagIsr[0] = false;
    }
    if (flagIsr[1])
    {
	if (_deviceForInterrupt[1])
	    _deviceForInterrupt[1]->handleInterrupt();
	flagIsr[1] = false;
    }
    if (flagIsr[2])
    {
	if (_deviceForInterrupt[2])
	    _deviceForInterrupt[2]->handleInterrupt();
	flagIsr[2] = false;
    }
}
#endif

// These are low level functions that call the interrupt handler for the correct
// instance of RH_RF22.
// 3 interrupts allows us to have 3 different devices
void RH_INTERRUPT_ATTR RH_RF22::isr0()
{
#if RH_PLATFORM == RH_PLATFORM_ESP8266
	flagIsr[0] = true;
#else
    if (_deviceForInterrupt[0])
	_deviceForInterrupt[0]->handleInterrupt();
#endif
}
void RH_INTERRUPT_ATTR RH_RF22::isr1()
{
#if RH_PLATFORM == RH_PLATFORM_ESP8266
	flagIsr[1] = true;
#else
    if (_deviceForInterrupt[1])
	_deviceForInterrupt[1]->handleInterrupt();
#endif
}
void RH_INTERRUPT_ATTR RH_RF22::isr2()
{
#if RH_PLATFORM == RH_PLATFORM_ESP8266
	flagIsr[2] = true;
#else
    if (_deviceForInterrupt[2])
	_deviceForInterrupt[2]->handleInterrupt();
#endif
}

void RH_RF22::reset()
{
    spiWrite(RH_RF22_REG_07_OPERATING_MODE1, RH_RF22_SWRES);
    // Wait for it to settle
    delay(1); // SWReset time is nominally 100usec
}

uint8_t RH_RF22::statusRead()
{
    return spiRead(RH_RF22_REG_02_DEVICE_STATUS);
}

uint8_t RH_RF22::adcRead(uint8_t adcsel,
                      uint8_t adcref ,
                      uint8_t adcgain, 
                      uint8_t adcoffs)
{
    uint8_t configuration = adcsel | adcref | (adcgain & RH_RF22_ADCGAIN);
    spiWrite(RH_RF22_REG_0F_ADC_CONFIGURATION, configuration | RH_RF22_ADCSTART);
    spiWrite(RH_RF22_REG_10_ADC_SENSOR_AMP_OFFSET, adcoffs);

    // Conversion time is nominally 305usec
    // Wait for the DONE bit
    while (!(spiRead(RH_RF22_REG_0F_ADC_CONFIGURATION) & RH_RF22_ADCDONE))
	;
    // Return the value  
    return spiRead(RH_RF22_REG_11_ADC_VALUE);
}

uint8_t RH_RF22::temperatureRead(uint8_t tsrange, uint8_t tvoffs)
{
    spiWrite(RH_RF22_REG_12_TEMPERATURE_SENSOR_CALIBRATION, tsrange | RH_RF22_ENTSOFFS);
    spiWrite(RH_RF22_REG_13_TEMPERATURE_VALUE_OFFSET, tvoffs);
    return adcRead(RH_RF22_ADCSEL_INTERNAL_TEMPERATURE_SENSOR | RH_RF22_ADCREF_BANDGAP_VOLTAGE); 
}

uint16_t RH_RF22::wutRead()
{
    uint8_t buf[2];
    spiBurstRead(RH_RF22_REG_17_WAKEUP_TIMER_VALUE1, buf, 2);
    return ((uint16_t)buf[0] << 8) | buf[1]; // Dont rely on byte order
}

// RFM-22 doc appears to be wrong: WUT for wtm = 10000, r, = 0, d = 0 is about 1 sec
void RH_RF22::setWutPeriod(uint16_t wtm, uint8_t wtr, uint8_t wtd)
{
    uint8_t period[3];

    period[0] = ((wtr & 0xf) << 2) | (wtd & 0x3);
    period[1] = wtm >> 8;
    period[2] = wtm & 0xff;
    spiBurstWrite(RH_RF22_REG_14_WAKEUP_TIMER_PERIOD1, period, sizeof(period));
}

// Returns true if centre + (fhch * fhs) is within limits
// Caution, different versions of the RH_RF22 support different max freq
// so YMMV
bool RH_RF22::setFrequency(float centre, float afcPullInRange)
{
    uint8_t fbsel = RH_RF22_SBSEL;
    uint8_t afclimiter;
    if (centre < 240.0 || centre > 960.0) // 930.0 for early silicon
	return false;
    if (centre >= 480.0)
    {
	if (afcPullInRange < 0.0 || afcPullInRange > 0.318750)
	    return false;
	centre /= 2;
	fbsel |= RH_RF22_HBSEL;
	afclimiter = afcPullInRange * 1000000.0 / 1250.0;
    }
    else
    {
	if (afcPullInRange < 0.0 || afcPullInRange > 0.159375)
	    return false;
	afclimiter = afcPullInRange * 1000000.0 / 625.0;
    }
    centre /= 10.0;
    float integerPart = floor(centre);
    float fractionalPart = centre - integerPart;

    uint8_t fb = (uint8_t)integerPart - 24; // Range 0 to 23
    fbsel |= fb;
    uint16_t fc = fractionalPart * 64000;
    spiWrite(RH_RF22_REG_73_FREQUENCY_OFFSET1, 0);  // REVISIT
    spiWrite(RH_RF22_REG_74_FREQUENCY_OFFSET2, 0);
    spiWrite(RH_RF22_REG_75_FREQUENCY_BAND_SELECT, fbsel);
    spiWrite(RH_RF22_REG_76_NOMINAL_CARRIER_FREQUENCY1, fc >> 8);
    spiWrite(RH_RF22_REG_77_NOMINAL_CARRIER_FREQUENCY0, fc & 0xff);
    spiWrite(RH_RF22_REG_2A_AFC_LIMITER, afclimiter);
    return !(statusRead() & RH_RF22_FREQERR);
}

// Step size in 10kHz increments
// Returns true if centre + (fhch * fhs) is within limits
bool RH_RF22::setFHStepSize(uint8_t fhs)
{
    spiWrite(RH_RF22_REG_7A_FREQUENCY_HOPPING_STEP_SIZE, fhs);
    return !(statusRead() & RH_RF22_FREQERR);
}

// Adds fhch * fhs to centre frequency
// Returns true if centre + (fhch * fhs) is within limits
bool RH_RF22::setFHChannel(uint8_t fhch)
{
    spiWrite(RH_RF22_REG_79_FREQUENCY_HOPPING_CHANNEL_SELECT, fhch);
    return !(statusRead() & RH_RF22_FREQERR);
}

uint8_t RH_RF22::rssiRead()
{
    return spiRead(RH_RF22_REG_26_RSSI);
}

uint8_t RH_RF22::ezmacStatusRead()
{
    return spiRead(RH_RF22_REG_31_EZMAC_STATUS);
}

void RH_RF22::setOpMode(uint8_t mode)
{
    spiWrite(RH_RF22_REG_07_OPERATING_MODE1, mode);
}

void RH_RF22::setModeIdle()
{
    if (_mode != RHModeIdle)
    {
	setOpMode(_idleMode);
	_mode = RHModeIdle;
    }
}

bool RH_RF22::sleep()
{
    if (_mode != RHModeSleep)
    {
	setOpMode(0);
	_mode = RHModeSleep;
    }
    return true;
}

void RH_RF22::setModeRx()
{
    if (_mode != RHModeRx)
    {
	setOpMode(_idleMode | RH_RF22_RXON);
	_mode = RHModeRx;
    }
}

void RH_RF22::setModeTx()
{
    if (_mode != RHModeTx)
    {
	setOpMode(_idleMode | RH_RF22_TXON);
	// Hmmm, if you dont clear the RX FIFO here, then it appears that going
	// to transmit mode in the middle of a receive can corrupt the
	// RX FIFO
	resetRxFifo();
	_mode = RHModeTx;
    }
}

void RH_RF22::setTxPower(uint8_t power)
{
    spiWrite(RH_RF22_REG_6D_TX_POWER, power | RH_RF22_LNA_SW); // On RF23, LNA_SW must be set.
}

// Sets registers from a canned modem configuration structure
void RH_RF22::setModemRegisters(const ModemConfig* config)
{
    spiWrite(RH_RF22_REG_1C_IF_FILTER_BANDWIDTH,                    config->reg_1c);
    spiWrite(RH_RF22_REG_1F_CLOCK_RECOVERY_GEARSHIFT_OVERRIDE,      config->reg_1f);
    spiBurstWrite(RH_RF22_REG_20_CLOCK_RECOVERY_OVERSAMPLING_RATE, &config->reg_20, 6);
    spiBurstWrite(RH_RF22_REG_2C_OOK_COUNTER_VALUE_1,              &config->reg_2c, 3);
    spiWrite(RH_RF22_REG_58_CHARGE_PUMP_CURRENT_TRIMMING,           config->reg_58);
    spiWrite(RH_RF22_REG_69_AGC_OVERRIDE1,                          config->reg_69);
    spiBurstWrite(RH_RF22_REG_6E_TX_DATA_RATE1,                    &config->reg_6e, 5);
}

// Set one of the canned FSK Modem configs
// Returns true if its a valid choice
bool RH_RF22::setModemConfig(ModemConfigChoice index)
{
    if (index > (signed int)(sizeof(MODEM_CONFIG_TABLE) / sizeof(ModemConfig)))
        return false;

    RH_RF22::ModemConfig cfg;
    memcpy_P(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(RH_RF22::ModemConfig));
    setModemRegisters(&cfg);

    return true;
}

// REVISIT: top bit is in Header Control 2 0x33
void RH_RF22::setPreambleLength(uint8_t nibbles)
{
    spiWrite(RH_RF22_REG_34_PREAMBLE_LENGTH, nibbles);
}

// Caution doesnt set sync word len in Header Control 2 0x33
void RH_RF22::setSyncWords(const uint8_t* syncWords, uint8_t len)
{
    spiBurstWrite(RH_RF22_REG_36_SYNC_WORD3, syncWords, len);
}

void RH_RF22::clearRxBuf()
{
    ATOMIC_BLOCK_START;
    _bufLen = 0;
    _rxBufValid = false;
    ATOMIC_BLOCK_END;
}

bool RH_RF22::available()
{
    if (!_rxBufValid)
    {
#if RH_PLATFORM == RH_PLATFORM_ESP8266
	loopIsr();
#endif
	if (_mode == RHModeTx)
	    return false;
	setModeRx(); // Make sure we are receiving
	YIELD; // Wait for any previous transmit to finish
    }
    return _rxBufValid;
}

#if RH_PLATFORM == RH_PLATFORM_ESP8266
bool RH_RF22::waitPacketSent()
{
    while (_mode == RHModeTx)
    {
	loopIsr();
	YIELD; // Make sure the watchdog is fed
    }
    return true;
}
#endif

bool RH_RF22::recv(uint8_t* buf, uint8_t* len)
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
    clearRxBuf();
//    printBuffer("recv:", buf, *len);
    return true;
}

void RH_RF22::clearTxBuf()
{
    ATOMIC_BLOCK_START;
    _bufLen = 0;
    _txBufSentIndex = 0;
    ATOMIC_BLOCK_END;
}

void RH_RF22::startTransmit()
{
    sendNextFragment(); // Actually the first fragment
    spiWrite(RH_RF22_REG_3E_PACKET_LENGTH, _bufLen); // Total length that will be sent
    setModeTx(); // Start the transmitter, turns off the receiver
}

// Restart the transmission of a packet that had a problem
void RH_RF22::restartTransmit()
{
    _mode = RHModeIdle;
    _txBufSentIndex = 0;
//	    Serial.println("Restart");
    startTransmit();
}

bool RH_RF22::send(const uint8_t* data, uint8_t len)
{
    bool ret = true;
    waitPacketSent();

    if (!waitCAD()) 
	return false;  // Check channel activity

    ATOMIC_BLOCK_START;
    spiWrite(RH_RF22_REG_3A_TRANSMIT_HEADER3, _txHeaderTo);
    spiWrite(RH_RF22_REG_3B_TRANSMIT_HEADER2, _txHeaderFrom);
    spiWrite(RH_RF22_REG_3C_TRANSMIT_HEADER1, _txHeaderId);
    spiWrite(RH_RF22_REG_3D_TRANSMIT_HEADER0, _txHeaderFlags);
    if (!fillTxBuf(data, len))
	ret = false;
    else
	startTransmit();
    ATOMIC_BLOCK_END;
//    printBuffer("send:", data, len);
    return ret;
}

bool RH_RF22::fillTxBuf(const uint8_t* data, uint8_t len)
{
    clearTxBuf();
    if (!len)
	return false; 
    return appendTxBuf(data, len);
}

bool RH_RF22::appendTxBuf(const uint8_t* data, uint8_t len)
{
    if (((uint16_t)_bufLen + len) > RH_RF22_MAX_MESSAGE_LEN)
	return false;
    ATOMIC_BLOCK_START;
    memcpy(_buf + _bufLen, data, len);
    _bufLen += len;
    ATOMIC_BLOCK_END;
//    printBuffer("txbuf:", _buf, _bufLen);
    return true;
}

// Assumption: there is currently <= RH_RF22_TXFFAEM_THRESHOLD bytes in the Tx FIFO
void RH_RF22::sendNextFragment()
{
    if (_txBufSentIndex < _bufLen)
    {
	// Some left to send?
	uint8_t len = _bufLen - _txBufSentIndex;
	// But dont send too much
	if (len > (RH_RF22_FIFO_SIZE - RH_RF22_TXFFAEM_THRESHOLD - 1))
	    len = (RH_RF22_FIFO_SIZE - RH_RF22_TXFFAEM_THRESHOLD - 1);
	spiBurstWrite(RH_RF22_REG_7F_FIFO_ACCESS, _buf + _txBufSentIndex, len);
//	printBuffer("frag:", _buf  + _txBufSentIndex, len);
	_txBufSentIndex += len;
    }
}

// Assumption: there are at least RH_RF22_RXFFAFULL_THRESHOLD in the RX FIFO
// That means it should only be called after a RXFFAFULL interrupt
void RH_RF22::readNextFragment()
{
    if (((uint16_t)_bufLen + RH_RF22_RXFFAFULL_THRESHOLD) > RH_RF22_MAX_MESSAGE_LEN)
	return; // Hmmm receiver overflow. Should never occur

    // Read the RH_RF22_RXFFAFULL_THRESHOLD octets that should be there
    spiBurstRead(RH_RF22_REG_7F_FIFO_ACCESS, _buf + _bufLen, RH_RF22_RXFFAFULL_THRESHOLD);
    _bufLen += RH_RF22_RXFFAFULL_THRESHOLD;
}

// Clear the FIFOs
void RH_RF22::resetFifos()
{
    spiWrite(RH_RF22_REG_08_OPERATING_MODE2, RH_RF22_FFCLRRX | RH_RF22_FFCLRTX);
    spiWrite(RH_RF22_REG_08_OPERATING_MODE2, 0);
}

// Clear the Rx FIFO
void RH_RF22::resetRxFifo()
{
    spiWrite(RH_RF22_REG_08_OPERATING_MODE2, RH_RF22_FFCLRRX);
    spiWrite(RH_RF22_REG_08_OPERATING_MODE2, 0);
    _rxBufValid = false;
}

// CLear the TX FIFO
void RH_RF22::resetTxFifo()
{
    spiWrite(RH_RF22_REG_08_OPERATING_MODE2, RH_RF22_FFCLRTX);
    spiWrite(RH_RF22_REG_08_OPERATING_MODE2, 0);
}

// Default implmentation does nothing. Override if you wish
void RH_RF22::handleExternalInterrupt()
{
}

// Default implmentation does nothing. Override if you wish
void RH_RF22::handleWakeupTimerInterrupt()
{
}

void RH_RF22::setPromiscuous(bool promiscuous)
{
    RHSPIDriver::setPromiscuous(promiscuous);
    spiWrite(RH_RF22_REG_43_HEADER_ENABLE3, promiscuous ? 0x00 : 0xff);
}

bool RH_RF22::setCRCPolynomial(CRCPolynomial polynomial)
{
    if (polynomial >= CRC_CCITT &&
	polynomial <= CRC_Biacheva)
    {
	_polynomial = polynomial;
	return true;
    }
    else
	return false;
}

uint8_t RH_RF22::maxMessageLength()
{
    return RH_RF22_MAX_MESSAGE_LEN;
}

void RH_RF22::setThisAddress(uint8_t thisAddress)
{
    RHSPIDriver::setThisAddress(thisAddress);
    spiWrite(RH_RF22_REG_3F_CHECK_HEADER3, thisAddress);
}

uint32_t RH_RF22::getLastPreambleTime()
{
    return _lastPreambleTime;
}

void RH_RF22::setGpioReversed(bool gpioReversed)
{
    // Ensure the antenna can be switched automatically according to transmit and receive
    // This assumes GPIO0(out) is connected to TX_ANT(in) to enable tx antenna during transmit
    // This assumes GPIO1(out) is connected to RX_ANT(in) to enable rx antenna during receive
    if (gpioReversed)
    {
	// Reversed for HAB-RFM22B-BOA HAB-RFM22B-BO, also Si4432 sold by Dorji.com via Tindie.com.
	spiWrite(RH_RF22_REG_0B_GPIO_CONFIGURATION0, 0x15) ; // RX state
	spiWrite(RH_RF22_REG_0C_GPIO_CONFIGURATION1, 0x12) ; // TX state
    }
    else
    {
	spiWrite(RH_RF22_REG_0B_GPIO_CONFIGURATION0, 0x12) ; // TX state
	spiWrite(RH_RF22_REG_0C_GPIO_CONFIGURATION1, 0x15) ; // RX state
    }
}

