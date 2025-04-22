// NRF51.cpp
//
// Per: nRF51_Series_Reference_manual v3.0.pdf
// Copyright (C) 2012 Mike McCauley
// $Id: RH_NRF51.cpp,v 1.4 2017/02/01 21:46:02 mikem Exp $

// Set by Arduino IDE and RadioHead.h when compiling for nRF51 or nRF52 chips:

#include <RH_NRF51.h>

#if RH_PLATFORM==RH_PLATFORM_NRF51


RH_NRF51::RH_NRF51()
    : _rxBufValid(false)
#if RH_NRF51_HAVE_ENCRYPTION
      , _encrypting(false)
#endif
{
}

bool RH_NRF51::init()
{
    // Enable the High Frequency clock to the system as a whole
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
	;
    
    // Disable and reset the radio
    NRF_RADIO->POWER = RADIO_POWER_POWER_Disabled;
    NRF_RADIO->POWER = RADIO_POWER_POWER_Enabled;
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE   = 1;
    // Wait until we are in DISABLE state
    while (NRF_RADIO->EVENTS_DISABLED == 0) {}

    // Physical on-air address is set in PREFIX0 + BASE0 by setNetworkAddress
    NRF_RADIO->TXADDRESS    = 0x00;	// Use logical address 0 (PREFIX0 + BASE0)
    NRF_RADIO->RXADDRESSES  = 0x01;	// Enable reception on logical address 0 (PREFIX0 + BASE0)

    // Configure the CRC
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos); // Number of checksum bits
    NRF_RADIO->CRCINIT = 0xFFFFUL;      // Initial value      
    NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1

    // These shorts will make the radio transition from Ready to Start to Disable automatically
    // for both TX and RX, which makes for much shorter on-air times
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos)
	              | (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);

    NRF_RADIO->PCNF0 =   (8 << RADIO_PCNF0_LFLEN_Pos) // Payload size length in bits
	               | (1 << RADIO_PCNF0_S0LEN_Pos) // S0 is 1 octet
	               | (8 << RADIO_PCNF0_S1LEN_Pos); // S1 is 1 octet

    // Make sure we are powered down
    setModeIdle();

    // Set a default network address
    uint8_t default_network_address[] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    setNetworkAddress(default_network_address, sizeof(default_network_address));

    setChannel(2); // The default, in case it was set by another app without powering down
    setRF(RH_NRF51::DataRate2Mbps, RH_NRF51::TransmitPower0dBm);
    setEncryptionKey(NULL);
    return true;
}

bool RH_NRF51::setChannel(uint8_t channel)
{
    NRF_RADIO->FREQUENCY = ((channel << RADIO_FREQUENCY_FREQUENCY_Pos) & RADIO_FREQUENCY_FREQUENCY_Msk);
    return true;
}

bool RH_NRF51::setNetworkAddress(uint8_t* address, uint8_t len)
{
    if (len < 3 || len > 5)
	return false;

    // First byte is the prefix, remainder are base
    NRF_RADIO->PREFIX0	  = ((address[0] << RADIO_PREFIX0_AP0_Pos) & RADIO_PREFIX0_AP0_Msk);
    uint32_t base;
    memcpy(&base, address+1, len-1);
    NRF_RADIO->BASE0 = base;

    NRF_RADIO->PCNF1 =  (
	(((sizeof(_buf)) << RADIO_PCNF1_MAXLEN_Pos)  & RADIO_PCNF1_MAXLEN_Msk)  // maximum length of payload
	| (((0UL)        << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk)	// expand the payload with 0 bytes
	| (((len-1)      << RADIO_PCNF1_BALEN_Pos)   & RADIO_PCNF1_BALEN_Msk)); // base address length in number of bytes.

    return true;
}

bool RH_NRF51::setRF(DataRate data_rate, TransmitPower power)
{
    uint8_t mode;
    uint8_t p;

    if (data_rate == DataRate2Mbps)
	mode = RADIO_MODE_MODE_Nrf_2Mbit;
    else if (data_rate == DataRate1Mbps)
	mode = RADIO_MODE_MODE_Nrf_1Mbit;
    else if (data_rate == DataRate250kbps)
	mode = RADIO_MODE_MODE_Nrf_250Kbit;
    else
	return false;// Invalid

    if      (power == TransmitPower4dBm)
	p = RADIO_TXPOWER_TXPOWER_Pos4dBm;
    else if (power == TransmitPower0dBm)
	p = RADIO_TXPOWER_TXPOWER_0dBm;
    else if (power == TransmitPowerm4dBm)
	p = RADIO_TXPOWER_TXPOWER_Neg4dBm;
    else if (power == TransmitPowerm8dBm)
	p = RADIO_TXPOWER_TXPOWER_Neg8dBm;
    else if (power == TransmitPowerm12dBm)
	p = RADIO_TXPOWER_TXPOWER_Neg12dBm;
    else if (power == TransmitPowerm16dBm)
	p = RADIO_TXPOWER_TXPOWER_Neg16dBm;
    else if (power == TransmitPowerm20dBm)
	p = RADIO_TXPOWER_TXPOWER_Neg20dBm;
    else if (power == TransmitPowerm30dBm)
	p = RADIO_TXPOWER_TXPOWER_Neg30dBm;
    else
	return false; // Invalid


    NRF_RADIO->TXPOWER = ((p << RADIO_TXPOWER_TXPOWER_Pos) & RADIO_TXPOWER_TXPOWER_Msk);
    NRF_RADIO->MODE    = ((mode << RADIO_MODE_MODE_Pos) & RADIO_MODE_MODE_Msk);

    return true;
}

void RH_NRF51::setModeIdle()
{
    if (_mode != RHModeIdle)
    {
	NRF_RADIO->EVENTS_DISABLED = 0U;
	NRF_RADIO->TASKS_DISABLE = 1;
	while (NRF_RADIO->EVENTS_DISABLED == 0U)
	    ; // wait for the radio to be disabled
	NRF_RADIO->EVENTS_END = 0U;
	_mode = RHModeIdle;
    }
}

void RH_NRF51::setModeRx()
{
    if (_mode != RHModeRx)
    {
	setModeIdle(); // Can only start RX from DISABLE state

#if RH_NRF51_HAVE_ENCRYPTION
	// Maybe set the AES CCA module for the correct encryption mode
	if (_encrypting)
	    NRF_CCM->MODE = (CCM_MODE_MODE_Decryption << CCM_MODE_MODE_Pos); // Decrypt
	NRF_CCM->MICSTATUS = 0;	
#endif

	// Radio will transition automatically to Disable state when a message is received
	NRF_RADIO->PACKETPTR = (uint32_t)_buf;
	NRF_RADIO->EVENTS_READY = 0U;
	NRF_RADIO->TASKS_RXEN = 1;
	NRF_RADIO->EVENTS_END = 0U; // So we can detect end of reception
	_mode = RHModeRx;
    }
}

void RH_NRF51::setModeTx()
{
    if (_mode != RHModeTx)
    {
	setModeIdle(); // Can only start RX from DISABLE state

	// Sigh: it seems that it takes longer to start the receiver than the transmitter for this type
	// of radio, so if a message is received and an ACK or reply is sent to soon, the original transmitter
	// may not see the reply. So we delay here to make sure the receiver is ready.
	// Yes, I know this is very ugly
	delay(1);

#if RH_NRF51_HAVE_ENCRYPTION
	// Maybe set the AES CCA module for the correct encryption mode
	if (_encrypting)
	    NRF_CCM->MODE = (CCM_MODE_MODE_Encryption << CCM_MODE_MODE_Pos); // Encrypt
#endif	    
	// Radio will transition automatically to Disable state at the end of transmission
	NRF_RADIO->PACKETPTR = (uint32_t)_buf;
	NRF_RADIO->EVENTS_READY = 0U;
	NRF_RADIO->TASKS_TXEN = 1;
	NRF_RADIO->EVENTS_END = 0U; // So we can detect end of transmission
	_mode = RHModeTx;
    }
}

bool RH_NRF51::send(const uint8_t* data, uint8_t len)
{
    if (len > RH_NRF51_MAX_MESSAGE_LEN)
	return false;

#if RH_NRF51_HAVE_ENCRYPTION
    if (_encrypting && len > RH_NRF51_MAX_ENCRYPTED_MESSAGE_LEN)
	return false;
#endif

    if (!waitCAD()) 
	return false;  // Check channel activity

    // Set up the headers
    _buf[0] = 0; // S0
    _buf[1] = len + RH_NRF51_HEADER_LEN;
    _buf[2] = 0; // S1
    // The following octets are subject to encryption
    _buf[3] = _txHeaderTo;
    _buf[4] = _txHeaderFrom;
    _buf[5] = _txHeaderId;
    _buf[6] = _txHeaderFlags;
    memcpy(_buf+RH_NRF51_HEADER_LEN, data, len);
    _rxBufValid = false;
    setModeTx();

    // Radio will return to Disabled state after transmission is complete
    _txGood++;
    return true;
}

bool RH_NRF51::waitPacketSent()
{
    // If we are not currently in transmit mode, there is no packet to wait for
    if (_mode != RHModeTx)
	return false;

    // When the Disabled event occurs we know the transmission has completed
    while (!NRF_RADIO->EVENTS_END)
    {
	YIELD;
    }
    setModeIdle();

    return true;
}

bool RH_NRF51::isSending()
{
    return (NRF_RADIO->STATE == RADIO_STATE_STATE_Tx) ? true : false;
}

bool RH_NRF51::printRegisters()
{
#ifdef RH_HAVE_SERIAL
    uint16_t i;
    uint32_t* p = (uint32_t*)NRF_RADIO;
    for (i = 0; (p + i) < (uint32_t*) (((NRF_RADIO_Type*)NRF_RADIO) + 1); i++)
    {
	Serial.print("Offset: ");
	Serial.print(i, DEC);
	Serial.print(" ");
	Serial.println(*(p+i), HEX);
    }
#endif

    return true;
}

// Check whether the latest received message is complete and uncorrupted
void RH_NRF51::validateRxBuf()
{
    if (_buf[1] < RH_NRF51_HEADER_LEN)
	return; // Too short to be a real message
    // Extract the 4 headers following S0, LEN and S1
    _rxHeaderTo    = _buf[3];
    _rxHeaderFrom  = _buf[4];
    _rxHeaderId    = _buf[5];
    _rxHeaderFlags = _buf[6];

    if (_promiscuous ||
	_rxHeaderTo == _thisAddress ||
	_rxHeaderTo == RH_BROADCAST_ADDRESS)
    {
	_rxGood++;
	_rxBufValid = true;
    }
}

void RH_NRF51::setEncryptionKey(uint8_t* key)
{
#if RH_NRF51_HAVE_ENCRYPTION
    if (key)
    {
	// Configure for on-the-fly encryption
	// Set the key
	memset(_encryption_cnf, 0, sizeof(_encryption_cnf));
	memcpy(_encryption_cnf, key, RH_NRF51_ENCRYPTION_KEY_LENGTH);
	// AES configuration data area
	// Note that the IV (Nonce) is not set, defaults to 0s
	NRF_CCM->CNFPTR  = (uint32_t)_encryption_cnf;

	// Set AES CCM input and putput buffers
	// Make sure the _buf is encrypted and put back into _buf
	NRF_CCM->INPTR  = (uint32_t)_buf;
	NRF_CCM->OUTPTR = (uint32_t)_buf;
	// Also need to set SCRATCHPTR temp buffer os size 16+MAXPACKETSIZE in RAM
	// FIXME: shared buffers if several radios
	NRF_CCM->SCRATCHPTR = (uint32_t)_scratch;

	// SHORT from RADIO READY to AESCCM KSGEN using PPI predefined channel 24
	// Also RADIO ADDRESS to AESCCM CRYPT using PPI predefined channel 25
	NRF_PPI->CHENSET =   (PPI_CHENSET_CH24_Enabled << PPI_CHENSET_CH24_Pos) 
	                   | (PPI_CHENSET_CH25_Enabled << PPI_CHENSET_CH25_Pos)
	    ;

	// SHORT from AESCCM ENDKSGEN to AESCCM CRYPT
	NRF_CCM->SHORTS = (CCM_SHORTS_ENDKSGEN_CRYPT_Enabled << CCM_SHORTS_ENDKSGEN_CRYPT_Pos);

	// Enable the CCM module
	NRF_CCM->ENABLE = (CCM_ENABLE_ENABLE_Enabled << CCM_ENABLE_ENABLE_Pos);

	_encrypting = true;
    }
    else
    {
	// Disable the CCM module
	NRF_CCM->ENABLE = (CCM_ENABLE_ENABLE_Disabled << CCM_ENABLE_ENABLE_Pos);
	_encrypting = false;
    }
#endif
}

bool RH_NRF51::available()
{
    if (!_rxBufValid)
    {
	if (_mode == RHModeTx)
	    return false;
	setModeRx();
	if (!NRF_RADIO->EVENTS_END)
	    return false; // No message yet
	setModeIdle();
#if RH_NRF51_HAVE_ENCRYPTION
	// If encryption is enabled, the decrypted message is not available yet, and there seems
	// to be no way to be sure when its ready, but a delay of 2ms is enough
	if (_encrypting)
	    delay(2);
#endif
        if (!NRF_RADIO->CRCSTATUS)
	{
	    // Bad CRC, restart the radio	    
	    _rxBad++;
	    setModeRx();
	    return false;
	}
	validateRxBuf(); 
	if (!_rxBufValid)
	    setModeRx(); // Try for another
    }
    return _rxBufValid;
}

void RH_NRF51::clearRxBuf()
{
    _rxBufValid = false;
    _buf[1] = 0;
}

bool RH_NRF51::recv(uint8_t* buf, uint8_t* len)
{
    if (!available())
	return false;
    if (buf && len)
    {
	// Skip the 4 headers that are at the beginning of the rxBuf
	// the payload length is the first octet in _buf
	if (*len > _buf[1]-RH_NRF51_HEADER_LEN)
	    *len = _buf[1]-RH_NRF51_HEADER_LEN;
	memcpy(buf, _buf+RH_NRF51_HEADER_LEN, *len);
    }
    clearRxBuf(); // This message accepted and cleared
    return true;
}

uint8_t RH_NRF51::maxMessageLength()
{
#if RH_NRF51_HAVE_ENCRYPTION
    if (_encrypting)
	return RH_NRF51_MAX_ENCRYPTED_MESSAGE_LEN;
#endif
    return RH_NRF51_MAX_MESSAGE_LEN;
}

float RH_NRF51::get_temperature()
{
    NRF_TEMP->EVENTS_DATARDY = 0;
    NRF_TEMP->TASKS_START = 1;
    
    while (!NRF_TEMP->EVENTS_DATARDY)
	;
    return NRF_TEMP->TEMP * 0.25;
}

#endif // NRF51
