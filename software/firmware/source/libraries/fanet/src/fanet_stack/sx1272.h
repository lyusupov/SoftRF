#ifndef SX1272_h
#define SX1272_h

#include <stdint.h>
#include <inttypes.h>
#include <Arduino.h>

//todo uart api
#define SX1272_debug_mode 				0
#if(SX1272_debug_mode > 0)
#include "../com/serial.h"
#endif

/* Default pin configurations */
#define SX1272_RESET_PIN 				9
#define SX1272_NSEL_PIN					10
#define SX1272_TX_SWITCH_PIN				15
#define SX1272_RX_SWITCH_PIN				16
#define SX1272_IRQ_PIN		    			14
#define SX1272_SPI					SPI

//! MACROS //
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)  // read a bit
#define bitSet(value, bit) ((value) |= (1UL << (bit)))    // set bit to '1'
#define bitClear(value, bit) ((value) &= ~(1UL << (bit))) // set bit to '0'

//! REGISTERS //

#define        REG_FIFO        	                        0x00
#define        REG_OP_MODE                              0x01
#define        REG_BITRATE_MSB    			0x02
#define        REG_BITRATE_LSB    			0x03
#define        REG_FDEV_MSB   				0x04
#define        REG_FDEV_LSB    				0x05
#define        REG_FRF_MSB    				0x06
#define        REG_FRF_MID    				0x07
#define        REG_FRF_LSB    				0x08
#define        REG_PA_CONFIG    			0x09
#define        REG_PA_RAMP    				0x0A
#define        REG_OCP    				0x0B
#define        REG_LNA    				0x0C
#define        REG_RX_CONFIG    			0x0D
#define        REG_FIFO_ADDR_PTR  			0x0D
#define        REG_RSSI_CONFIG   			0x0E
#define        REG_FIFO_TX_BASE_ADDR 			0x0E
#define        REG_RSSI_COLLISION    			0x0F
#define        REG_FIFO_RX_BASE_ADDR                    0x0F
#define        REG_RSSI_THRESH    			0x10
#define        REG_FIFO_RX_CURRENT_ADDR                 0x10
#define        REG_RSSI_VALUE_FSK                       0x11
#define        REG_IRQ_FLAGS_MASK			0x11
#define        REG_RX_BW		    		0x12
#define        REG_IRQ_FLAGS	    			0x12
#define        REG_AFC_BW		    		0x13
#define        REG_RX_NB_BYTES	    			0x13
#define        REG_OOK_PEAK	    			0x14
#define        REG_RX_HEADER_CNT_VALUE_MSB 		0x14
#define        REG_OOK_FIX	    			0x15
#define        REG_RX_HEADER_CNT_VALUE_LSB  		0x15
#define        REG_OOK_AVG	 			0x16
#define        REG_RX_PACKET_CNT_VALUE_MSB  		0x16
#define        REG_RX_PACKET_CNT_VALUE_LSB  		0x17
#define        REG_MODEM_STAT	  			0x18
#define        REG_PKT_SNR_VALUE	  		0x19
#define        REG_AFC_FEI	  			0x1A
#define        REG_PKT_RSSI_VALUE	  		0x1A
#define        REG_AFC_MSB	  			0x1B
#define        REG_RSSI_VALUE_LORA	  		0x1B
#define        REG_AFC_LSB	  			0x1C
#define        REG_HOP_CHANNEL	  			0x1C
#define        REG_FEI_MSB	  			0x1D
#define        REG_MODEM_CONFIG1	 		0x1D
#define        REG_FEI_LSB	  			0x1E
#define        REG_MODEM_CONFIG2	  		0x1E
#define        REG_PREAMBLE_DETECT  			0x1F
#define        REG_SYMB_TIMEOUT_LSB  			0x1F
#define        REG_RX_TIMEOUT1	  			0x20
#define        REG_PREAMBLE_MSB_LORA  			0x20
#define        REG_RX_TIMEOUT2	  			0x21
#define        REG_PREAMBLE_LSB_LORA  			0x21
#define        REG_RX_TIMEOUT3	 			0x22
#define        REG_PAYLOAD_LENGTH_LORA			0x22
#define        REG_RX_DELAY	 			0x23
#define        REG_MAX_PAYLOAD_LENGTH 			0x23
#define        REG_OSC		 			0x24
#define        REG_HOP_PERIOD	  			0x24
#define        REG_PREAMBLE_MSB_FSK 			0x25
#define        REG_FIFO_RX_BYTE_ADDR 			0x25
#define        REG_PREAMBLE_LSB_FSK 			0x26
#define        REG_SYNC_CONFIG	  			0x27
#define        REG_SYNC_VALUE1	 			0x28
#define        REG_SYNC_VALUE2	  			0x29
#define        REG_SYNC_VALUE3	  			0x2A
#define        REG_SYNC_VALUE4	  			0x2B
#define        REG_SYNC_VALUE5	  			0x2C
#define        REG_SYNC_VALUE6	  			0x2D
#define        REG_SYNC_VALUE7	  			0x2E
#define        REG_SYNC_VALUE8	  			0x2F
#define        REG_PACKET_CONFIG1	  		0x30
#define        REG_PACKET_CONFIG2	  		0x31
#define        REG_DETECT_OPTIMIZE			0x31
#define        REG_PAYLOAD_LENGTH_FSK			0x32
#define        REG_NODE_ADRS	  			0x33
#define        REG_BROADCAST_ADRS	 		0x34
#define        REG_FIFO_THRESH	  			0x35
#define        REG_SEQ_CONFIG1	  			0x36
#define        REG_SEQ_CONFIG2	  			0x37
#define        REG_DETECTION_THRESHOLD      		0x37
#define        REG_TIMER_RESOL	  			0x38
#define        REG_TIMER1_COEF	  			0x39
#define        REG_TIMER2_COEF	  			0x3A
#define        REG_IMAGE_CAL	  			0x3B
#define        REG_TEMP		  			0x3C
#define        REG_LOW_BAT	  			0x3D
#define        REG_IRQ_FLAGS1	  			0x3E
#define        REG_IRQ_FLAGS2	  			0x3F
#define        REG_DIO_MAPPING1	  			0x40
#define        REG_DIO_MAPPING2	  			0x41
#define        REG_VERSION	  			0x42
#define        REG_AGC_REF	  			0x43
#define        REG_AGC_THRESH1	  			0x44
#define        REG_AGC_THRESH2	  			0x45
#define        REG_AGC_THRESH3	  			0x46
#define        REG_PLL_HOP	  			0x4B
#define        REG_TCXO		  			0x58
#define        REG_PA_DAC		  		0x5A
#define        REG_PLL		  			0x5C
#define        REG_PLL_LOW_PN	  			0x5E
#define        REG_FORMER_TEMP	  			0x6C
#define        REG_BIT_RATE_FRAC	  		0x70

//LORA MODES:
const uint8_t LORA_SLEEP_MODE = 0x80;
const uint8_t LORA_STANDBY_MODE = 0x81;
const uint8_t LORA_TX_MODE = 0x83;
const uint8_t LORA_RXCONT_MODE = 0x85;
const uint8_t LORA_RXSINGLE_MODE = 0x86;
const uint8_t LORA_CAD_MODE = 0x87;
const uint8_t LORA_MODE_MASK = 0x87;
const uint8_t LORA_STANDBY_FSK_REGS_MODE = 0xC1;

//LORA BANDWIDTH:
const uint8_t BW_125 = 0x00;
const uint8_t BW_250 = 0x40;
const uint8_t BW_500 = 0x80;
const uint8_t BW_MASK = 0xC0;

//LORA SPREADING FACTOR:
const uint8_t SF_6 = 0x60;
const uint8_t SF_7 = 0x70;
const uint8_t SF_8 = 0x80;
const uint8_t SF_9 = 0x90;
const uint8_t SF_10 = 0xA0;
const uint8_t SF_11 = 0xB0;
const uint8_t SF_12 = 0xC0;
const uint8_t SF_MASK = 0xF0;

//LORA CODING RATE:
const uint8_t CR_5 = 0x08;
const uint8_t CR_6 = 0x10;
const uint8_t CR_7 = 0x18;
const uint8_t CR_8 = 0x20;
const uint8_t CR_MASK = 0x38;

//CHANNELS
const uint32_t CH_868_200 = 0xD90CCD;		//868.200Mhz
const uint32_t CH_869_525 = 0xD9619A;		//869.525Mhz

//TRANSMITT POWER
const uint8_t PA_DAC_HIGH_PWR = 0x87;
const uint8_t PA_DAC_DEFAULT_PWR = 0x84;

//IRQ FLAGS
const uint8_t IRQ_RX_TIMEOUT = 0x80;
const uint8_t IRQ_RX_DONE = 0x40;
const uint8_t IRQ_PAYLOAD_CRC_ERROR = 0x20;
const uint8_t IRQ_VALID_HEADER = 0x10;
const uint8_t IRQ_TX_DONE = 0x08;
const uint8_t IRQ_CAD_DONE = 0x04;
const uint8_t IRQ_FHSS_CHANGE_CHANNEL = 0x02;
const uint8_t IRQ_CAD_DETECTED = 0x01;

//DIO0 MODES
const uint8_t DIO0_RX_DONE = 0x00;
const uint8_t DIO0_TX_DONE = 0x40;
const uint8_t DIO0_CAD_DONE = 0x10;
const uint8_t DIO0_NONE = 0xC0;

const uint8_t LNAGAIN_G1_MAX = 0x01;
const uint8_t LNAGAIN_G2 = 0x02;
const uint8_t LNAGAIN_G3 = 0x03;
const uint8_t LNAGAIN_G4 = 0x04;
const uint8_t LNAGAIN_G5 = 0x05;
const uint8_t LNAGAIN_G6_MIN = 0x06;
const uint8_t LNAGAIN_OFFSET = 5;

// SEND FRAME RETURN VALUES
const int TX_OK	= 0;
const int TX_TX_ONGOING = -1;
const int TX_RX_ONGOING	= -2;

const int CRC_ON_PAYLOAD = 0x40;

typedef void (*irq_callback)(int length);

typedef struct
{
	uint32_t channel;
	int dBm;
} sx_region_t;

class Sx1272
{
private:
	int res_pin;
	int nsel_pin;
#ifdef SX1272_TX_SWITCH_PIN
	int txsw_pin;
#endif
#ifdef SX1272_RX_SWITCH_PIN
	int rxsw_pin;
#endif
	int irq_pin;
	bool armed = false;
	sx_region_t region = {.channel = 0, .dBm = 0};

	irq_callback irq_cb = NULL;

	byte readRegister(byte address);
	void writeRegister(byte address, byte data);

	void writeFifo(uint8_t addr, uint8_t *data, int length);
	void readFifo(uint8_t addr, uint8_t *data, int length);

	void select()
	{
		noInterrupts();
  		digitalWrite(nsel_pin, LOW);
	}

	void unselect()
	{
  		digitalWrite(nsel_pin, HIGH);
		interrupts();
	}

	bool setOpMode(uint8_t mode);
	uint8_t getOpMode(void)
	{
		return readRegister(REG_OP_MODE);
	}

	void setDio0Irq(int mode);

	static void sx_irq_wrapper(void);
	void sx1272_irq(void);

	uint32_t getChannel(void);
	bool setChannel(uint32_t ch);

	bool setPower(int pwr);

	bool receiveStart(void);

public:
	/*
	 * Configuration
	 */

	Sx1272();

	bool begin(void);
	bool begin(int res_pin, int nsel_pin, int txsw_pin, int rxsw_pin);

	void setPayloadCrc(bool crc);
	void setExplicitHeader(bool exhdr);
	void setLowDataRateOptimize(bool ldro);
	void setLnaGain(uint8_t gain, bool lnaboost);

	uint8_t getBandwidth(void);
	void setBandwidth(uint8_t bw);

	uint8_t getSpreadingFactor(void);
	void setSpreadingFactor(uint8_t spr);

	uint8_t getCodingRate(void);
	void setCodingRate(uint8_t cr);

	bool setArmed(bool rxmode);
	bool isArmed(void);

	bool setRegion(sx_region_t &region);

	/*
	 * Transmission
	 * Note: length must not be longer than 256 bytes!
	 */

	//interrupt enabled
	int sendFrame(uint8_t *data, int length);

	//linear only
	int receiveFrame(uint8_t *data, int max_length);

	void setIrqReceiver(irq_callback cb);
	void setIrqReceiver(int irq_p, irq_callback cb);
	int getFrame(uint8_t *data, int max_length);
	int getRssi(void);
	void clrIrqReceiver(void);
};

extern Sx1272 sx1272;

#endif
