/*
 * Project: nRF905 AVR/Arduino Library/Driver
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2013 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/
 */

#include <string.h>
#if !defined(ESP8266) && !defined(ESP32) && !defined(RASPBERRY_PI) && \
    !defined(ENERGIA_ARCH_CC13XX) && !defined(ENERGIA_ARCH_CC13X2) && \
    !defined(ARDUINO_ARCH_STM32)  && !defined(__ASR6501__)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#else
#define _delay_ms delay
#define _delay_us delayMicroseconds
#endif
#ifdef ARDUINO
#include <Arduino.h>
#include <SPI.h>
#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2)
#define _BV(bit) (1 << (bit))
#endif /* ENERGIA_ARCH_CC13XX || ENERGIA_ARCH_CC13X2 */
#else
#if !defined(RASPBERRY_PI)
#include "nRF905_spi.h"
#else
#define SPI SPI0
#include <raspi/raspi.h>
#endif /* RASPBERRY_PI */
#endif
#include "nRF905.h"
#include "nRF905_config.h"
#include "nRF905_defs.h"
#include "nRF905_types.h"

#if defined(RASPBERRY_PI)
#define ARDUINO
#endif /* RASPBERRY_PI */

#define noinline __attribute__ ((__noinline__))

#define AM_IS_USED_SW (NRF905_AM_SW)
#define NEED_SW_STATUS_SUPPORT (AM_IS_USED_SW || NRF905_DR_SW)

//#define disableStandbyMode()	(nRF905_leaveStandBy())
//#define enableStandbyMode()		(nRF905_enterStandBy())

#ifdef ARDUINO
#define AM_IS_USED_HW			(!NRF905_AM_SW && defined(AM))
#define powerUp()				(digitalWrite(PWR_MODE, HIGH))
#define powerDown()				(digitalWrite(PWR_MODE, LOW))
#define disableStandbyMode()	(digitalWrite(TRX_EN, HIGH))
#define enableStandbyMode()		(digitalWrite(TRX_EN, LOW))
#define spiSelect()				(digitalWrite(CSN, LOW))
#define spiDeselect()			(digitalWrite(CSN, HIGH))
#define receiveMode()			(digitalWrite(TX_EN, LOW))
#define transmitMode()			(digitalWrite(TX_EN, HIGH))
#define spi_transfer(data)		(SPI.transfer(data))
#define spi_transfer_nr(data)	(SPI.transfer(data))
#else
#define AM_IS_USED_HW			(!NRF905_AM_SW && defined(CFG_AM_PORT) && defined(CFG_AM_BIT))
#define powerUp()				(PWR_MODE_PORT |= _BV(PWR_MODE_BIT))
#define powerDown()				(PWR_MODE_PORT &= ~_BV(PWR_MODE_BIT))
#define disableStandbyMode()	(TRX_EN_PORT |= _BV(TRX_EN_BIT))
#define enableStandbyMode()		(TRX_EN_PORT &= ~_BV(TRX_EN_BIT))
#define spiSelect()				(CSN_PORT &= ~_BV(CSN_BIT))
#define spiDeselect()			(CSN_PORT |= _BV(CSN_BIT))
#define receiveMode()			(TX_EN_PORT &= ~_BV(TX_EN_BIT))
#define transmitMode()			(TX_EN_PORT |= _BV(TX_EN_BIT))
#endif

static inline bool cselect(void)
{
	nRF905_interrupt_off();
//	spi_enable();
	spiSelect();
	return true;
}

static inline bool cdeselect(void)
{
	spiDeselect();
//	spi_disable();
	nRF905_interrupt_on();
	return false;
}

// Need to be in standby mode to write registers?
#define STANDBY (enableStandbyMode())

#define CHIPSELECT(standby) standby; \
							for(bool cs = cselect(); cs; cs = cdeselect())

static inline bool interrupt_off(void)
{
	nRF905_interrupt_off();
	return true;
}

static inline bool interrupt_on(void)
{
	nRF905_interrupt_on();
	return false;
}

#define NRF905_ATOMIC() for(bool cs = interrupt_off(); cs; cs = interrupt_on())

typedef struct
{
	uint8_t reg1; // Change to array
	uint8_t reg2;
	uint8_t payloadSize;
} config_s;

typedef struct{
	nRF905_radio_state_t state;
	bool goToRxMode;
} state_s;

typedef struct{
	uint8_t buffer[NRF905_MAX_PAYLOAD];
	bool ready;
} data_s;

static void setConfigReg1(uint8_t, uint8_t, uint8_t);
static void setConfigReg2(uint8_t, uint8_t, uint8_t);
static void setConfigRegister(uint8_t, uint8_t);
static noinline void defaultConfig(void);
static void setAddress(void*, uint8_t);
#if !NRF905_INTERRUPTS
static bool dataReady(void);
#endif
static inline void stateTx(void);
#if NEED_SW_STATUS_SUPPORT
static uint8_t readStatus(void);
#endif

// We could instead read the registers over SPI, but that would be a bit slower
static config_s config;

#if NRF905_INTERRUPTS
static volatile data_s rxData;
static volatile state_s radio;
#else
static state_s radio;
#endif

void nRF905_init()
{
#ifdef ARDUINO
	pinMode(TRX_EN, OUTPUT);
	pinMode(PWR_MODE, OUTPUT);
	pinMode(TX_EN, OUTPUT);

#if NRF905_COLLISION_AVOID
	pinMode(CD, INPUT);
#endif

#if AM_IS_USED_HW
	pinMode(AM, INPUT);
#endif

#if !NRF905_DR_SW
	pinMode(DREADY, INPUT);
#endif

	digitalWrite(CSN, HIGH);
	pinMode(CSN, OUTPUT);

	SPI.begin();

#if !defined(RASPBERRY_PI) && !defined(__ASR6501__)
	SPI.setClockDivider(SPI_CLOCK_DIV2);
#endif /* RASPBERRY_PI */
#else
	TRX_EN_DDR |= _BV(TRX_EN_BIT);
	PWR_MODE_DDR |= _BV(PWR_MODE_BIT);
	TX_EN_DDR |= _BV(TX_EN_BIT);

#if NRF905_COLLISION_AVOID
	CD_DDR &= ~_BV(CD_BIT);
#endif

#if AM_IS_USED_HW
	AM_DDR &= ~_BV(AM_BIT);
#endif

#if !NRF905_DR_SW
	DR_DDR &= ~_BV(DR_BIT);
#endif

	spiDeselect();
	CSN_DDR |= _BV(CSN_BIT);

	spi_init();
#endif

	radio.state = NRF905_RADIO_STATE_IDLE;

	// Startup
	enableStandbyMode();
	receiveMode();
	nRF905_powerDown();
	_delay_ms(3);
	defaultConfig();

#if NRF905_INTERRUPTS
	// Set interrupts
	REG_EXTERNAL_INT_CTL |= BIT_EXTERNAL_INT_CTL;
	nRF905_interrupt_on();
#endif

	nRF905_powerUp();
}

// Set frequency, workout the channel from the frequency
void nRF905_setFrequency(nRF905_band_t band, uint32_t freq)
{
	nRF905_setChannel(band, NRF905_CALC_CHANNEL(freq, band));
}

// Set channel
void nRF905_setChannel(nRF905_band_t band, uint16_t channel)
{
	config.reg1 = (config.reg1 & NRF905_MASK_CHANNEL) | band | ((channel>>8) & 0x01);

	CHIPSELECT(STANDBY)
	{
		spi_transfer_nr(NRF905_CMD_W_CONFIG | NRF905_REG_CHANNEL);
		spi_transfer_nr(channel);
		spi_transfer_nr(config.reg1);
	}
}

// Set auto retransmit
void nRF905_setAutoRetransmit(nRF905_auto_retran_t val)
{
	setConfigReg1(val, NRF905_MASK_AUTO_RETRAN, NRF905_REG_AUTO_RETRAN);
}

// Set low power receive
void nRF905_setLowRxPower(nRF905_low_rx_t val)
{
	setConfigReg1(val, NRF905_MASK_LOW_RX, NRF905_REG_LOW_RX);
}

// Set output power
void nRF905_setTransmitPower(nRF905_pwr_t val)
{
	setConfigReg1(val, NRF905_MASK_PWR, NRF905_REG_PWR);
}

// Set CRC
void nRF905_setCRC(nRF905_crc_t val)
{
	setConfigReg2(val, NRF905_MASK_CRC, NRF905_REG_CRC);
}

// Set clock output
void nRF905_setClockOut(nRF905_outclk_t val)
{
	setConfigReg2(val, NRF905_MASK_OUTCLK, NRF905_REG_OUTCLK);
}

static void setConfigReg1(uint8_t val, uint8_t mask, uint8_t reg)
{
	config.reg1 = (config.reg1 & mask) | val;
	setConfigRegister(NRF905_CMD_W_CONFIG | reg, config.reg1);
}

static void setConfigReg2(uint8_t val, uint8_t mask, uint8_t reg)
{
	config.reg2 = (config.reg2 & mask) | val;
	setConfigRegister(NRF905_CMD_W_CONFIG | reg, config.reg2);
}

static void setConfigRegister(uint8_t cmd, uint8_t val)
{
	CHIPSELECT(STANDBY)
	{
		spi_transfer_nr(cmd);
		spi_transfer_nr(val);
	}
}

// Set configuration
// Radio should be in standby mode and interrupts disabled
static noinline void defaultConfig()
{
	uint16_t channel = NRF905_CALC_CHANNEL(NRF905_FREQ, NRF905_BAND);
	uint8_t reg1 = NRF905_AUTO_RETRAN | NRF905_LOW_RX | NRF905_PWR | NRF905_BAND | ((channel>>8) & 0x01);
	uint8_t reg2 = NRF905_CRC | NRF905_CLK_FREQ | NRF905_OUTCLK;

	config.reg1 = reg1;
	config.reg2 = reg2;
	config.payloadSize = NRF905_PAYLOAD_SIZE;
	
	// Set control registers
	spiSelect();
	spi_transfer_nr(NRF905_CMD_W_CONFIG);
	spi_transfer_nr(channel);
	spi_transfer_nr(reg1);
	spi_transfer_nr((NRF905_ADDR_SIZE<<4) | NRF905_ADDR_SIZE);
	spi_transfer_nr(NRF905_PAYLOAD_SIZE); // RX payload size
	spi_transfer_nr(NRF905_PAYLOAD_SIZE); // TX payload size
	for(uint8_t i=4;i--;)
		spi_transfer_nr(0xE7); // Default receive address
	spi_transfer_nr(reg2);
	spiDeselect();

	// Default transmit address
	spiSelect();
	spi_transfer_nr(NRF905_CMD_W_TX_ADDRESS);
	for(uint8_t i=4;i--;)
		spi_transfer_nr(0xE7);
	spiDeselect();
	
	// Clear transmit payload
	spiSelect();
	spi_transfer_nr(NRF905_CMD_W_TX_PAYLOAD);
	for(uint8_t i=NRF905_MAX_PAYLOAD;i--;)
		spi_transfer_nr(0x00);
	spiDeselect();

	// Clear DR by reading receive payload
	spiSelect();
	spi_transfer_nr(NRF905_CMD_R_RX_PAYLOAD);
	for(uint8_t i=NRF905_MAX_PAYLOAD;i--;)
		spi_transfer_nr(NRF905_CMD_NOP);
	spiDeselect();
}

// Payload size
void nRF905_setPayloadSize(uint8_t size)
{
	CHIPSELECT(STANDBY)
	{
		if(size > NRF905_MAX_PAYLOAD)
			size = NRF905_MAX_PAYLOAD;
		config.payloadSize = size;

		spi_transfer_nr(NRF905_CMD_W_CONFIG | NRF905_REG_RX_PAYLOAD_SIZE);
		spi_transfer_nr(size);
		spi_transfer_nr(size);
	}
}

void nRF905_setAddressSize(uint8_t size)
{
	CHIPSELECT(STANDBY)
	{
		spi_transfer_nr(NRF905_CMD_W_CONFIG | NRF905_REG_ADDR_WIDTH);
		spi_transfer_nr((size<<4) | size);
	}
}

// Set address of device to send to
void nRF905_setTXAddress(void* address)
{
	setAddress(address, NRF905_CMD_W_TX_ADDRESS);
}

// Set address for this device
void nRF905_setRXAddress(void* address)
{
	setAddress(address, NRF905_CMD_W_CONFIG | NRF905_REG_RX_ADDRESS);
}

// Set address
static void setAddress(void* address, uint8_t cmd)
{
	CHIPSELECT(STANDBY)
	{
		spi_transfer_nr(cmd);
		// Address bytes are sent in reverse order, which is fine as long as both ends do the same thing.
		// Reverse loops usually create slightly smaller code.
		for(uint8_t i=NRF905_ADDR_SIZE;i--;)
			spi_transfer_nr(((uint8_t*)address)[i]);
	}
}

// Set the payload data
bool nRF905_setData(void* data, uint8_t len)
{
	// Busy transmitting something else
	if(radio.state == NRF905_RADIO_STATE_TX)
		return false;

	uint8_t maxPayload = config.payloadSize;
	if(len > maxPayload)
		len = maxPayload;

	CHIPSELECT(STANDBY)
	{
		spi_transfer_nr(NRF905_CMD_W_TX_PAYLOAD);

		// Load data
		for(uint8_t i=0;i<len;i++)
			spi_transfer_nr(((uint8_t*)data)[i]);
	}
	
	return true;
}

// See if device is receiving something
// Hardware: Address match pin high
// Software: Address match status bit set
bool nRF905_receiveBusy()
{
#if (!AM_IS_USED_HW && !AM_IS_USED_SW)
	return false;
#elif AM_IS_USED_SW
	return (readStatus() & _BV(NRF905_STATUS_AM));
#elif defined(ARDUINO)
	return digitalRead(AM);
#else
	return (AM_PORT & _BV(AM_BIT));
#endif
}

// See if data ready, true if received new data/finished transmitting
// Hardware: Data ready pin high
// Software: Data ready status bit set
#if !NRF905_INTERRUPTS
static bool dataReady()
{
#if NRF905_DR_SW
	return (readStatus() & _BV(NRF905_STATUS_DR));
#elif defined(ARDUINO)
	return digitalRead(DREADY);
#else
	return (DR_PORT & _BV(DR_BIT));
#endif
}
#endif

// Transmit payload
bool nRF905_send()
{
	// Already transmitting
	if(radio.state == NRF905_RADIO_STATE_TX)
		return false;
#if NRF905_COLLISION_AVOID
	// Don't transmit if busy
	else if(nRF905_airwayBusy())
		return false;
#endif

	// Put into transmit mode
	transmitMode();

	// Enable chip
	disableStandbyMode();

	radio.state = NRF905_RADIO_STATE_TX;

	// Pulse standby pin to start transmission
	_delay_us(14);

#if NRF905_AUTO_RETRAN == NRF905_AUTO_RETRAN_DISABLE
	// Radio will go back into standby mode after transmission, unless nRF905_receive()
	// is called in which case it will go straight to receive mode after transmission.
	// If auto-retransmission is disabled and if the radio isn't set to go into standby or receive mode then it will
	// transmit a carrier signal with no data.
	enableStandbyMode();
#endif

	return true;
}

// Return radio state
nRF905_radio_state_t nRF905_getState()
{
	return radio.state;
}

// Put into receive mode
void nRF905_receive()
{
	NRF905_ATOMIC()
	{
		if(radio.state == NRF905_RADIO_STATE_TX) // Currently transmitting, so wait until finished then go into receive mode
			radio.goToRxMode = true;
		else
		{
			receiveMode();
			disableStandbyMode();
			radio.state = NRF905_RADIO_STATE_RX;
		}
	}	
}

// Get received data if available
bool nRF905_getData(void* data, uint8_t len)
{
	if(len > config.payloadSize)
		len = config.payloadSize;

#if NRF905_INTERRUPTS
	// No data received
	if(!rxData.ready)
		return false;

	NRF905_ATOMIC()
	{
		// Copy and clear data buffer
		memcpy(data, (uint8_t*)rxData.buffer, len);
		memset((uint8_t*)rxData.buffer, 0, sizeof(rxData.buffer));
		rxData.ready = false;
	}
	return true;
#else
//  if (nRF905_receiveBusy())
//    Serial.println("receiveBusy");

	// No data ready
	if(!dataReady())
		return false;
//        Serial.println("dataReady");
	switch(radio.state)
	{
		case NRF905_RADIO_STATE_TX:
			// Finished transmitting payload
			stateTx();
			break;
		case NRF905_RADIO_STATE_RX:
			// New payload received
			CHIPSELECT()
			{
				spi_transfer_nr(NRF905_CMD_R_RX_PAYLOAD);

				// Get received payload
				for(uint8_t i=0;i<len;i++)
					((uint8_t*)data)[i] = spi_transfer(NRF905_CMD_NOP);

				// Must make sure all of the payload has been read, otherwise DR never goes low
				uint8_t remaining = config.payloadSize - len;
				while(remaining--)
					spi_transfer_nr(NRF905_CMD_NOP);
			}
			// We're still in receive mode
			return true;
		default:
			break;
	}
	return false;
#endif
}

static inline void stateTx()
{
	if(radio.goToRxMode)
	{
		// We want to go into receive mode
		radio.goToRxMode = false;
		receiveMode();
		disableStandbyMode();
		radio.state = NRF905_RADIO_STATE_RX;
	}
	else
	{
		// If we didn't want to go into receive mode then we're now in standby mode
		radio.state = NRF905_RADIO_STATE_IDLE;
	}
}

// Power up
void nRF905_powerUp()
{
	radio.state = NRF905_RADIO_STATE_IDLE;
	enableStandbyMode();
	powerUp();

	// Give it time to turn on
	_delay_ms(3);
}

void nRF905_powerDown()
{
	powerDown();
	radio.state = NRF905_RADIO_STATE_IDLE;
}

void nRF905_enterStandBy()
{
	enableStandbyMode();
	radio.state = NRF905_RADIO_STATE_IDLE;
}
/*
void nRF905_leaveStandBy()
{
	nRF905_receive();
	//radio.state = RADIO_STATE_RX;
	//receiveMode();
	//disableStandbyMode();
}
*/

void nRF905_getConfigRegisters(void* regs)
{
	CHIPSELECT()
	{
		spi_transfer_nr(NRF905_CMD_R_CONFIG);
		for(uint8_t i=0;i<NRF905_REGISTER_COUNT;i++)
			((uint8_t*)regs)[i] = spi_transfer(NRF905_CMD_NOP);
	}
}

#if NEED_SW_STATUS_SUPPORT
// Read status register
static uint8_t readStatus()
{
	uint8_t status;
	CHIPSELECT()
		status = spi_transfer(NRF905_CMD_NOP);
	return status;
}
#endif

#if NRF905_INTERRUPTS
// Data ready pin interrupt
ISR(INT_VECTOR)
{
	switch(radio.state)
	{
		case NRF905_RADIO_STATE_TX:
			// Finished transmitting payload
			stateTx();
			break;
		case NRF905_RADIO_STATE_RX:
			// New payload received
			CHIPSELECT()
			{
				spi_transfer_nr(NRF905_CMD_R_RX_PAYLOAD);

				// Get the received payload
				uint8_t len = config.payloadSize;
				for(uint8_t i=0;i<len;i++)
					rxData.buffer[i] = spi_transfer(NRF905_CMD_NOP);
				rxData.ready = true;
			}
			// We're still in receive mode
			break;
		default:
			break;
	}
}
#endif
