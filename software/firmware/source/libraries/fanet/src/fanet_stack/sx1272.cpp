#include <SPI.h>

#include "../fanet_stack/sx1272.h"

Sx1272::Sx1272()
{
	res_pin = SX1272_RESET_PIN;
	nsel_pin = SX1272_NSEL_PIN;
#ifdef SX1272_TX_SWITCH_PIN
	txsw_pin = SX1272_TX_SWITCH_PIN;
#endif
#ifdef SX1272_RX_SWITCH_PIN
	rxsw_pin = SX1272_RX_SWITCH_PIN;
#endif
	irq_pin = SX1272_IRQ_PIN;
}

bool Sx1272::begin(void)
{
#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.println(F("## SX1272 begin"));
#endif

	/* Pins */
#ifdef SX1272_TX_SWITCH_PIN
	pinMode(txsw_pin, OUTPUT);
        digitalWrite(txsw_pin, LOW);
#endif
#ifdef SX1272_RX_SWITCH_PIN
        pinMode(rxsw_pin, OUTPUT);
        digitalWrite(rxsw_pin, LOW);
#endif
	pinMode(nsel_pin, OUTPUT);
	unselect();

	/* Configure SPI */
	SX1272_SPI.begin();
	SX1272_SPI.setBitOrder(MSBFIRST);
	SX1272_SPI.setClockDivider(SPI_CLOCK_DIV2);
	SX1272_SPI.setDataMode(SPI_MODE0);

	/* Reset pulse for LoRa module initialization */
	pinMode(res_pin, OUTPUT);
	digitalWrite(res_pin, HIGH);
	delay(1);
	digitalWrite(res_pin, LOW);
	delay(6);

	/* set Lora mode, this is only possible during sleep -> do it twice */
	setOpMode(LORA_SLEEP_MODE);
	setOpMode(LORA_SLEEP_MODE);

	/* set standby mode */
	bool mode = setOpMode(LORA_STANDBY_MODE);

	/* some more tweaks here */
	/* disable LowPnTxPllOff -> low phase noise PLL in transmitt mode, however more current */
	writeRegister(REG_PA_RAMP, 0x09);
	/* disable over current detection */
	writeRegister(REG_OCP, 0x1B);

	return mode;
}

bool Sx1272::begin(int _res_pin, int _nsel_pin, int _txsw_pin, int _rxsw_pin)
{
	/* set default pins */
	res_pin = _res_pin;
	nsel_pin = _nsel_pin;
#ifdef SX1272_TX_SWITCH_PIN
	txsw_pin = _txsw_pin;
#endif
#ifdef SX1272_RX_SWITCH_PIN
	rxsw_pin = _rxsw_pin;
#endif

	return begin();
}

byte Sx1272::readRegister(byte address)
{
	select();
	/* bit 7 cleared to read in registers */
	SX1272_SPI.transfer(address & 0x7F);
	byte value = SX1272_SPI.transfer(0x00);
	unselect();

#if (SX1272_debug_mode > 1)
	Serial.print(F("## SX1272 read reg("));
	Serial.print(address, HEX);
	Serial.print(F(")="));
	Serial.print(value, HEX);
	Serial.println();
#endif

	return value;
}

void Sx1272::writeRegister(byte address, byte data)
{
	select();
	/* bit 7 set to read from registers */
	SX1272_SPI.transfer(address | 0x80);
	SX1272_SPI.transfer(data);
	unselect();

#if (SX1272_debug_mode > 1)
	Serial.print(F("## SX1272 write reg("));
	Serial.print(address, HEX);
	Serial.print(F(")="));
	Serial.print(data, HEX);
	Serial.println();
#endif
}

void Sx1272::writeFifo(uint8_t addr, uint8_t *data, int length)
{
#if (SX1272_debug_mode > 1)
	Serial.println();
	Serial.print(F("## SX1272 write fifo, length="));
	Serial.print(length, DEC);

	Serial.print(" [");
	for (int i = 0; i < length; i++)
	{
		Serial.print(data[i], HEX);
		if(i < length-1)
			Serial.print(", ");
	}
	Serial.println("]");
#endif

	/* select location */
	writeRegister(REG_FIFO_ADDR_PTR, addr);

	/* upload data */
	select();
	SX1272_SPI.transfer(REG_FIFO | 0x80);
        for (int i = 0; i < length; i++)
		SX1272_SPI.transfer(data[i]);
	unselect();
}

void Sx1272::readFifo(uint8_t addr, uint8_t *data, int length)
{
	/* select location */
	writeRegister(REG_FIFO_ADDR_PTR, addr);

	/* upload data */
	select();
	SX1272_SPI.transfer(REG_FIFO);
        for (int i = 0; i < length; i++)
		data[i] = SX1272_SPI.transfer(0);
	unselect();

#if (SX1272_debug_mode > 1)
	Serial.print(F("## SX1272 read fifo, length="));
	Serial.print(length, DEC);

	Serial.print(" [");
	for (int i = 0; i < length; i++)
	{
		Serial.print(data[i], HEX);
		if(i < length-1)
			Serial.print(", ");
	}
	Serial.println("]");
#endif
}

bool Sx1272::setOpMode(uint8_t mode)
{
#if (SX1272_debug_mode > 0)
	switch (mode)
	{
	case LORA_RXCONT_MODE:
		Serial.println(F("## SX1272 opmode: rx continous"));
	break;
	case LORA_TX_MODE:
		Serial.println(F("## SX1272 opmode: tx"));
	break;
	case LORA_SLEEP_MODE:
		Serial.println(F("## SX1272 opmode: sleep"));
	break;
	case LORA_STANDBY_MODE:
		Serial.println(F("## SX1272 opmode: standby"));
	break;
	case LORA_CAD_MODE:
		Serial.println(F("## SX1272 opmode: cad"));
	break;
	default:
		Serial.println(F("## SX1272 opmode: unknown"));
	}
#endif

	/* set mode */
	writeRegister(REG_OP_MODE, mode);

	/* set rf analog end */
	switch (mode)
	{
	case LORA_RXSINGLE_MODE:
	case LORA_RXCONT_MODE:
	case LORA_CAD_MODE:
#ifdef SX1272_TX_SWITCH_PIN
		digitalWrite(txsw_pin, LOW);
#endif
#ifdef SX1272_RX_SWITCH_PIN
		digitalWrite(rxsw_pin, HIGH);
#endif
	break;
	case LORA_TX_MODE:
#ifdef SX1272_RX_SWITCH_PIN
		digitalWrite(rxsw_pin, LOW);
#endif
#ifdef SX1272_TX_SWITCH_PIN
		digitalWrite(txsw_pin, HIGH);
#endif
	break;
	default:
#ifdef SX1272_TX_SWITCH_PIN
		digitalWrite(txsw_pin, LOW);
#endif
#ifdef SX1272_RX_SWITCH_PIN
		digitalWrite(rxsw_pin, LOW);
#endif
		break;
	}

	/* wait for frequency synthesis, 5ms timeout */
	uint8_t opmode = 0;
	for(int i=0; i<50 && opmode != mode; i++)
	{
		delayMicroseconds(100);
		opmode = readRegister(REG_OP_MODE);
	}

#if (SX1272_debug_mode > 1)
	Serial.print(F("## SX1272 opmode: "));
	Serial.println(opmode, HEX);
#endif

	return mode == opmode;
}

void Sx1272::setBandwidth(uint8_t bw)
{
#if (SX1272_debug_mode > 0)
	Serial.print(F("## SX1272 BW: "));
	Serial.println(bw, HEX);
#endif

	/* store state */
	uint8_t opmode = getOpMode();

	/* set appropriate mode */
	if(opmode != LORA_STANDBY_MODE)
		setOpMode(LORA_STANDBY_MODE);

	uint8_t config1 = readRegister(REG_MODEM_CONFIG1);
	config1 = (bw&BW_MASK) | (config1&~BW_MASK);
	if(bw == BW_125)
	{
		uint8_t sf = getSpreadingFactor();
		if(sf == SF_11 || sf == SF_12)
			setLowDataRateOptimize(true);
	}
	writeRegister(REG_MODEM_CONFIG1, config1);

	/* restore state */
	if(opmode != LORA_STANDBY_MODE)
		setOpMode(opmode);
}

uint8_t Sx1272::getBandwidth(void)
{
	uint8_t bw = readRegister(REG_MODEM_CONFIG1) & BW_MASK;

#if (SX1272_debug_mode > 0)
	Serial.print(F("## SX1272 BW="));
	Serial.println(bw, HEX);
#endif

	return bw;
}

void Sx1272::setLowDataRateOptimize(bool ldro)
{
#if (SX1272_debug_mode > 0)
	Serial.print(F("## SX1272 LDRO: "));
	Serial.println(!!ldro, DEC);
#endif

	/* store state */
	uint8_t opmode = getOpMode();

	/* set appropriate mode */
	if(opmode != LORA_STANDBY_MODE)
		setOpMode(LORA_STANDBY_MODE);

	uint8_t config1 = readRegister(REG_MODEM_CONFIG1);
	config1 = (config1&0xFE) | !!ldro;
	writeRegister(REG_MODEM_CONFIG1, config1);

	/* restore state */
	if(opmode != LORA_STANDBY_MODE)
		setOpMode(opmode);
}

void Sx1272::setPayloadCrc(bool crc)
{
#if (SX1272_debug_mode > 0)
	Serial.print(F("## SX1272 CRC: "));
	Serial.println(!!crc, DEC);
#endif

	/* store state */
	uint8_t opmode = getOpMode();

	/* set appropriate mode */
	if(opmode != LORA_STANDBY_MODE)
		setOpMode(LORA_STANDBY_MODE);

	uint8_t config1 = readRegister(REG_MODEM_CONFIG1);
	config1 = (config1&0xFD) | !!crc<<1;
	writeRegister(REG_MODEM_CONFIG1, config1);

	/* restore state */
	if(opmode != LORA_STANDBY_MODE)
		setOpMode(opmode);
}

void Sx1272::setSpreadingFactor(uint8_t spr)
{
#if (SX1272_debug_mode > 0)
	Serial.print(F("## SX1272 SF: "));
	Serial.println(spr, HEX);
#endif

	/* store state */
	uint8_t opmode = getOpMode();

	/* set appropriate mode */
	if(opmode != LORA_STANDBY_MODE)
		setOpMode(LORA_STANDBY_MODE);

	uint8_t config2 = readRegister(REG_MODEM_CONFIG2);
	config2 = (spr&SF_MASK) | (config2&~SF_MASK);

	/* LowDataRateOptimize (Mandatory with SF_11 or SF_12 if BW_125) */
	if((spr == SF_11 || spr == SF_12) && getBandwidth() == BW_125)
		setLowDataRateOptimize(true);

	writeRegister(REG_MODEM_CONFIG2, config2);

	/* Check if it is neccesary to set special settings for SF=6 */
	if (spr == SF_6)
	{
		// Mandatory headerOFF with SF = 6 (Implicit mode)
		setExplicitHeader(false);
		writeRegister(REG_DETECT_OPTIMIZE, 0x05);
		writeRegister(REG_DETECTION_THRESHOLD, 0x0C);
	}
	else
	{
		writeRegister(REG_DETECT_OPTIMIZE, 0x03);
		writeRegister(REG_DETECTION_THRESHOLD, 0x0A);
	}

	/* restore state */
	if(opmode != LORA_STANDBY_MODE)
		setOpMode(opmode);
}

uint8_t Sx1272::getSpreadingFactor(void)
{
	uint8_t sf = readRegister(REG_MODEM_CONFIG2) & SF_MASK;

#if (SX1272_debug_mode > 0)
	Serial.print(F("## SX1272 SF="));
	Serial.println(sf, HEX);
#endif

	return sf;
}

void Sx1272::setCodingRate(uint8_t cr)
{
#if (SX1272_debug_mode > 0)
	Serial.print(F("## SX1272 CR: "));
	Serial.println(cr, HEX);
#endif

	/* store state */
	uint8_t opmode = getOpMode();

	/* set appropriate mode */
	if(opmode != LORA_STANDBY_MODE)
		setOpMode(LORA_STANDBY_MODE);

	uint8_t config1 = readRegister(REG_MODEM_CONFIG1);
	config1 = (cr&CR_MASK) | (config1&~CR_MASK);
	writeRegister(REG_MODEM_CONFIG1, config1);

	/* restore state */
	if(opmode != LORA_STANDBY_MODE)
		setOpMode(opmode);
}

uint8_t Sx1272::getCodingRate(void)
{
	uint8_t cr = readRegister(REG_MODEM_CONFIG1) & CR_MASK;

#if (SX1272_debug_mode > 0)
	Serial.print(F("## SX1272 CR="));
	Serial.println(cr, HEX);
#endif

	return cr;
}

bool Sx1272::setChannel(uint32_t ch)
{
#if (SX1272_debug_mode > 0)
	Serial.print(F("## SX1272 CH: "));
	Serial.println(ch, HEX);
#endif

	/* store state */
	uint8_t opmode = getOpMode();

	/* set appropriate mode */
	if(opmode != LORA_STANDBY_MODE)
		setOpMode(LORA_STANDBY_MODE);

	uint8_t freq3 = (ch >> 16) & 0x0FF;		// frequency channel MSB
	uint8_t freq2 = (ch >> 8) & 0xFF;		// frequency channel MID
	uint8_t freq1 = ch & 0xFF;			// frequency channel LSB

	writeRegister(REG_FRF_MSB, freq3);
	writeRegister(REG_FRF_MID, freq2);
	writeRegister(REG_FRF_LSB, freq1);

	/* restore state */
	bool ret = true;
	if(opmode != LORA_STANDBY_MODE)
		ret = setOpMode(opmode);

	#if (SX1272_debug_mode > 1)
		Serial.println();
		Serial.print(F("## SX1272 CH="));
		Serial.println(ch, HEX);
	#endif

	return ret;
}

uint32_t Sx1272::getChannel(void)
{
	uint8_t freq3 = readRegister(REG_FRF_MSB);	// frequency channel MSB
	uint8_t freq2 = readRegister(REG_FRF_MID);	// frequency channel MID
	uint8_t freq1 = readRegister(REG_FRF_LSB);	// frequency channel LSB
	uint32_t ch = ((uint32_t) freq3 << 16) + ((uint32_t) freq2 << 8) + (uint32_t) freq1;

#if (SX1272_debug_mode > 0)
	Serial.println();
	Serial.print(F("## SX1272 CH="));
	Serial.println(ch, HEX);
#endif

	return ch;
}

void Sx1272::setExplicitHeader(bool exhdr)
{
#if (SX1272_debug_mode > 0)
	Serial.print(F("## SX1272 explicit header: "));
	Serial.println(exhdr, DEC);
#endif

	/* store state */
	uint8_t opmode = getOpMode();

	/* set appropriate mode */
	if(opmode != LORA_STANDBY_MODE)
		setOpMode(LORA_STANDBY_MODE);

	/* bit2: 0->explicit 1->implicit */
	uint8_t config1 = readRegister(REG_MODEM_CONFIG1);
	config1 = (config1&0xFB) | !exhdr<<2;
	writeRegister(REG_MODEM_CONFIG1, config1);

	/* restore state */
	if(opmode != LORA_STANDBY_MODE)
		setOpMode(opmode);
}

void Sx1272::setLnaGain(uint8_t gain, bool lnaboost)
{
	/* store state */
	uint8_t opmode = getOpMode();

	/* set appropriate mode */
	if(opmode != LORA_STANDBY_MODE)
		setOpMode(LORA_STANDBY_MODE);

#if (SX1272_debug_mode > 0)
	Serial.print(F("## SX1272 lna gain: "));
	Serial.println((gain<<LNAGAIN_OFFSET) | (lnaboost?0x3:0x0), HEX);
#endif

	writeRegister(REG_LNA, (gain<<LNAGAIN_OFFSET) | (lnaboost?0x3:0x0));

	/* restore state */
	if(opmode != LORA_STANDBY_MODE)
		setOpMode(opmode);
}

//note: we currently only support PA_BOOST
bool Sx1272::setPower(int pwr)
{
	/* clamp pwr requirement */
	if(pwr < 2)
		pwr = 2;
	if(pwr > 20)
		pwr = 20;

#if (SX1272_debug_mode > 0)
	Serial.print(F("## SX1272 dBm: "));
	Serial.println(pwr, DEC);
#endif

	/* store state */
	uint8_t opmode = getOpMode();

	/* set appropriate mode */
	if(opmode != LORA_STANDBY_MODE)
		setOpMode(LORA_STANDBY_MODE);

	if(pwr >= 18)
	{
		/* high power mode */
		writeRegister(REG_PA_DAC, PA_DAC_HIGH_PWR);
		pwr -= 5;
	}
	else
	{
		/* normal mode */
		writeRegister(REG_PA_DAC, PA_DAC_DEFAULT_PWR);
		pwr -= 2;
	}

	/* PA output pin to PA_BOOST */
	writeRegister(REG_PA_CONFIG, 0x80 | pwr);

	/* restore state */
	if(opmode != LORA_STANDBY_MODE)
		return setOpMode(opmode);
	else
		return true;
}

bool Sx1272::setArmed(bool rxmode)
{
	/* store state */
	uint8_t opmode = getOpMode();

	if(rxmode && (opmode == LORA_SLEEP_MODE || opmode == LORA_STANDBY_MODE))
	{
		/* enable rx */
		bool ret = receiveStart();
		if(ret)
			armed = true;
		return ret;
	}
	else if(!rxmode)
	{
		/* enter power save */
		bool ret = setOpMode(LORA_SLEEP_MODE);
		if(ret)
			armed = false;
		return ret;
	}
	return true;
}

bool Sx1272::isArmed(void)
{
	return armed;
}

//note: do not use hardware based interrupts here
void Sx1272::sx1272_irq(void)
{
	#if (SX1272_debug_mode > 0)
		Serial.print(F("## SX1272 irq: "));
	#endif

	setOpMode(LORA_STANDBY_MODE);

	if(irq_cb == NULL)
	{
#if (SX1272_debug_mode > 0)
		Serial.println();
		Serial.println(F("## No callback. Error"));
#endif

		writeRegister(REG_IRQ_FLAGS, 0xFF);
		return;
	}

	uint8_t what = readRegister(REG_IRQ_FLAGS);
#if (SX1272_debug_mode > 0)
	Serial.print(what, HEX);
	Serial.print("...");
	//Serial.println(readRegister(REG_HOP_CHANNEL), HEX);
#endif
	/* Tx done */
	//if(what & IRQ_TX_DONE)
	//{
		//nothing to do...
	//}

	/* Rx done, CRC is valid and CRC was transmitted */
	if((what & IRQ_RX_DONE) && !(what & IRQ_PAYLOAD_CRC_ERROR) && (readRegister(REG_HOP_CHANNEL) & CRC_ON_PAYLOAD))
	{
		/* retrieve data */
		const int received = readRegister(REG_RX_NB_BYTES);

		/* callback function which is supposed to read the fifo */
		if(received > 0)
			irq_cb(received);
	}

	/* clear all flag */
        writeRegister(REG_IRQ_FLAGS, 0xFF);

	/* switch irq behavior back and re-enter rx mode */
	receiveStart();
}

//static
void Sx1272::sx_irq_wrapper(void)
{
	sx1272.sx1272_irq();
}

void Sx1272::setIrqReceiver(irq_callback cb)
{
	pinMode(irq_pin, INPUT);

	setOpMode(LORA_STANDBY_MODE);
	setDio0Irq(DIO0_RX_DONE);

	/* clear all flag */
	writeRegister(REG_IRQ_FLAGS, 0xFF);

	irq_cb = cb;
	attachInterrupt(irq_pin, sx_irq_wrapper, RISING);

	/* enter rx mode */
	//receiveStart();
}

void Sx1272::setIrqReceiver(int irq_p, irq_callback cb)
{
	irq_pin = irq_p;
	setIrqReceiver(cb);
}

void Sx1272::clrIrqReceiver(void)
{
	setOpMode(LORA_STANDBY_MODE);

	detachInterrupt(irq_pin);
	irq_cb = NULL;
}

void Sx1272::setDio0Irq(int mode)
{
	uint8_t map1 = readRegister(REG_DIO_MAPPING1) & 0x3F;
	writeRegister(REG_DIO_MAPPING1, map1 | mode);
}

bool Sx1272::setRegion(sx_region_t &region)
{
	bool success = setChannel(region.channel) && setPower(region.dBm);

	if(success)
		this->region = region;

	Serial1.println(this->region.channel);
	Serial1.println(this->region.dBm);

	return success;
}

/***********************************/
int Sx1272::sendFrame(uint8_t *data, int length)
{
#if (SX1272_debug_mode > 0)
	Serial.print(F("## SX1272 send frame..."));
#endif

	uint8_t mode = getOpMode() & LORA_MODE_MASK;

	/* are we transmitting anyway? */
	if(mode == LORA_TX_MODE)
		return TX_TX_ONGOING;

	/* in case of receiving, is it ongoing? */
	for(int i=0; i<400;i++)
	{
		if((mode == LORA_RXCONT_MODE || mode == LORA_RXSINGLE_MODE) && (readRegister(REG_MODEM_STAT) & 0x0B))
			return TX_RX_ONGOING;
		delayMicroseconds(10);
	}

	/*
	 * CAD
	 */

	setOpMode(LORA_STANDBY_MODE);
	writeRegister(REG_IRQ_FLAGS, IRQ_CAD_DONE | IRQ_CAD_DETECTED);	/* clearing flags */
	setOpMode(LORA_CAD_MODE);

	/* wait for CAD completion */
	byte iflags;
	while(((iflags=readRegister(REG_IRQ_FLAGS)) & IRQ_CAD_DONE) == 0)
		delayMicroseconds(1);

	if(iflags & IRQ_CAD_DETECTED)
	{
		/* re-establish old mode */
		if(mode == LORA_RXCONT_MODE || mode == LORA_RXSINGLE_MODE)
			setOpMode(mode);

		return TX_RX_ONGOING;
	}

	setOpMode(LORA_STANDBY_MODE);

	//todo: check fifo is empty, no rx data..

	/* upload frame */
	writeFifo(0x00, data, length);
	writeRegister(REG_FIFO_TX_BASE_ADDR, 0x00);
	writeRegister(REG_PAYLOAD_LENGTH_LORA, length);

	/* prepare irq */
	if(irq_cb)
	{
		/* clear flag */
		writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE);
		setDio0Irq(DIO0_TX_DONE);
	}

	setOpMode(LORA_TX_MODE);

	/* bypass waiting */
	if(irq_cb)
	{
#if (SX1272_debug_mode > 1)
		Serial.println("INT");
#endif
		return TX_OK;
	}

	for(int i=0; i<1000 && getOpMode() == LORA_TX_MODE; i++)
	{
#if (SX1272_debug_mode > 0)
		Serial.print(".");
#endif
		delay(5);
	}
#if (SX1272_debug_mode > 0)
	Serial.println("done");
#endif
	return TX_OK;
}

int Sx1272::receiveFrame(uint8_t *data, int max_length)
{
#if (SX1272_debug_mode > 0)
	Serial.print(F("## SX1272 receive frame..."));
#endif

	setOpMode(LORA_STANDBY_MODE);

	/* prepare receiving */
	writeRegister(REG_FIFO_RX_BASE_ADDR, 0x00);
	writeRegister(REG_PAYLOAD_LENGTH_LORA, max_length);
	//writeRegister(REG_SYMB_TIMEOUT_LSB, 0xFF);	//test max payload

	setOpMode(LORA_RXCONT_MODE);

	/* wait for rx or timeout */
	for(int i=0; i<400 && !(readRegister(REG_IRQ_FLAGS) & 0x40); i++)
	{
#if (SX1272_debug_mode > 0)
		Serial.print(".");
#endif
		delay(5);
	}

	/* stop rx mode and retrieve data */
	setOpMode(LORA_STANDBY_MODE);
	const int received = readRegister(REG_RX_NB_BYTES);
	const int rxstartaddr = readRegister(REG_RX_NB_BYTES);
	readFifo(rxstartaddr, data, min(received, max_length));

	/* clear the rxDone flag */
        writeRegister(REG_IRQ_FLAGS, IRQ_RX_DONE);

#if (SX1272_debug_mode > 0)
	Serial.print("done, ");
	Serial.println(received, DEC);
#endif
	return received;
}

bool Sx1272::receiveStart(void)
{
#if (SX1272_debug_mode > 0)
	Serial.println(F("## SX1272 receive start"));
#endif

	setOpMode(LORA_STANDBY_MODE);

	/* prepare receiving */
	writeRegister(REG_FIFO_RX_BASE_ADDR, 0x00);
	writeRegister(REG_PAYLOAD_LENGTH_LORA, 0xFF);		//is this evaluated in explicit mode?

	/* clear any rx done flag */
	writeRegister(REG_IRQ_FLAGS, IRQ_RX_DONE);

	/* switch irq behaviour to rx_done and enter rx cont mode */
	setDio0Irq(DIO0_RX_DONE);
	return setOpMode(LORA_RXCONT_MODE);
}

int Sx1272::getFrame(uint8_t *data, int max_length)
{
#if (SX1272_debug_mode > 0)
	Serial.println(F("## SX1272 get frame"));
#endif

	const int received = readRegister(REG_RX_NB_BYTES);
	const int rxstartaddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
	readFifo(rxstartaddr, data, min(received, max_length));

	return min(received, max_length);
}

int Sx1272::getRssi(void)
{
	/* get values */
	const int pktsnr = (int8_t) readRegister(REG_PKT_SNR_VALUE);
	int rssi = -139 + readRegister(REG_PKT_RSSI_VALUE);
	if(pktsnr < 0)
		rssi += ((pktsnr-2)/4);			//note: correct rounding for negative numbers

	return rssi;
}

Sx1272 sx1272 = Sx1272();
