/*
 Arduino Library for Analog Devices ADXL362 - Micropower 3-axis accelerometer
 go to http://www.analog.com/ADXL362 for datasheet
 
 
 License: CC BY-SA 3.0: Creative Commons Share-alike 3.0. Feel free 
 to use and abuse this code however you'd like. If you find it useful
 please attribute, and SHARE-ALIKE!
 
 Created June 2012
 by Anne Mahaffey - hosted on http://annem.github.com/ADXL362

 Modified May 2013
 by Jonathan Ruiz de Garibay
 
 */ 

#include <Arduino.h>
#include <ADXL362.h>
#include <SPI.h>

//#define ADXL362_DEBUG

ADXL362::ADXL362() {
}


//
//  begin()
//  Initial SPI setup, soft reset of device
//
void ADXL362::begin(int16_t chipSelectPin) {
	slaveSelectPin = chipSelectPin;
	pinMode(slaveSelectPin, OUTPUT);
	SPI.begin();
	SPI.setDataMode(SPI_MODE0);	//CPHA = CPOL = 0    MODE = 0
	delay(1000);
    
	// soft reset
	SPIwriteOneRegister(0x1F, 0x52);  // Write to SOFT RESET, "R"
	delay(10);
#ifdef ADXL362_DEBUG
	Serial.println("Soft Reset\n");
#endif
 }
 
//
//  beginMeasure()
//  turn on Measurement mode - required after reset
// 
void ADXL362::beginMeasure() {
	byte temp = SPIreadOneRegister(0x2D);	// read Reg 2D before modifying for measure mode

#ifdef ADXL362_DEBUG
	Serial.print(  "Setting Measeurement Mode - Reg 2D before = "); 
	Serial.print(temp); 
#endif

	// turn on measurement mode
	byte tempwrite = temp | 0x02;			// turn on measurement bit in Reg 2D
	SPIwriteOneRegister(0x2D, tempwrite); // Write to POWER_CTL_REG, Measurement Mode
	delay(10);	
  
#ifdef ADXL362_DEBUG
	temp = SPIreadOneRegister(0x2D);
	Serial.print(  ", Reg 2D after = "); 
	Serial.println(temp); 
	Serial.println();
#endif
}

//
//  readXData(), readYData(), readZData(), readTemp()
//  Read X, Y, Z, and Temp registers
//
int16_t ADXL362::readXData(){
	int16_t XDATA = SPIreadTwoRegisters(0x0E);
	
#ifdef ADXL362_DEBUG
	Serial.print("XDATA = ");
	Serial.println(XDATA);
#endif
	
	return XDATA;
}

int16_t ADXL362::readYData(){
	int16_t YDATA = SPIreadTwoRegisters(0x10);

#ifdef ADXL362_DEBUG
	Serial.print("\tYDATA = "); 
	Serial.println(YDATA);
#endif
	
	return YDATA;
}

int16_t ADXL362::readZData(){
	int16_t ZDATA = SPIreadTwoRegisters(0x12);

#ifdef ADXL362_DEBUG
	Serial.print("\tZDATA = "); 
	Serial.println(ZDATA);
#endif

	return ZDATA;
}

int16_t ADXL362::readTemp(){
	int16_t TEMP = SPIreadTwoRegisters(0x14);

#ifdef ADXL362_DEBUG
	Serial.print("\tTEMP = "); 
	Serial.println(TEMP);
#endif

	return TEMP;
}

void ADXL362::readXYZTData(int16_t &XData, int16_t &YData, int16_t &ZData, int16_t &Temperature){
	  // burst SPI read
	  // A burst read of all three axis is required to guarantee all measurements correspond to same sample time
	  digitalWrite(slaveSelectPin, LOW);
	  SPI.transfer(0x0B);  // read instruction
	  SPI.transfer(0x0E);  // Start at XData Reg
	  XData = SPI.transfer(0x00);
	  XData = XData + (SPI.transfer(0x00) << 8);
	  YData = SPI.transfer(0x00);
	  YData = YData + (SPI.transfer(0x00) << 8);
	  ZData = SPI.transfer(0x00);
	  ZData = ZData + (SPI.transfer(0x00) << 8);
	  Temperature = SPI.transfer(0x00);
	  Temperature = Temperature + (SPI.transfer(0x00) << 8);
	  digitalWrite(slaveSelectPin, HIGH);
  
#ifdef ADXL362_DEBUG
	Serial.print("XDATA = "); Serial.print(XData); 
	Serial.print("\tYDATA = "); Serial.print(YData); 
	Serial.print("\tZDATA = "); Serial.print(ZData); 
	Serial.print("\tTemperature = "); Serial.println(Temperature);
#endif
}

void ADXL362::setupDCActivityInterrupt(int16_t threshold, byte time){
	//  Setup motion and time thresholds
	SPIwriteTwoRegisters(0x20, threshold);
	SPIwriteOneRegister(0x22, time);

	// turn on activity interrupt
	byte ACT_INACT_CTL_Reg = SPIreadOneRegister(0x27);  // Read current reg value
	ACT_INACT_CTL_Reg = ACT_INACT_CTL_Reg | (0x01);     // turn on bit 1, ACT_EN  
	SPIwriteOneRegister(0x27, ACT_INACT_CTL_Reg);       // Write new reg value 
	ACT_INACT_CTL_Reg = SPIreadOneRegister(0x27);       // Verify properly written

#ifdef ADXL362_DEBUG
	Serial.print("DC Activity Threshold set to ");  	Serial.print(SPIreadTwoRegisters(0x20));
	Serial.print(", Time threshold set to ");  		Serial.print(SPIreadOneRegister(0x22)); 
	Serial.print(", ACT_INACT_CTL Register is ");  	Serial.println(ACT_INACT_CTL_Reg, HEX);
#endif
}

void ADXL362::setupACActivityInterrupt(int16_t threshold, byte time){
	//  Setup motion and time thresholds
	SPIwriteTwoRegisters(0x20, threshold);
	SPIwriteOneRegister(0x22, time);
  
	// turn on activity interrupt
	byte ACT_INACT_CTL_Reg = SPIreadOneRegister(0x27);  // Read current reg value
	ACT_INACT_CTL_Reg = ACT_INACT_CTL_Reg | (0x03);     // turn on bit 2 and 1, ACT_AC_DCB, ACT_EN  
	SPIwriteOneRegister(0x27, ACT_INACT_CTL_Reg);       // Write new reg value 
	ACT_INACT_CTL_Reg = SPIreadOneRegister(0x27);       // Verify properly written

#ifdef ADXL362_DEBUG
	Serial.print("AC Activity Threshold set to ");  	Serial.print(SPIreadTwoRegisters(0x20));
	Serial.print(", Time Activity set to ");  		Serial.print(SPIreadOneRegister(0x22));  
	Serial.print(", ACT_INACT_CTL Register is ");  Serial.println(ACT_INACT_CTL_Reg, HEX);
#endif
}

void ADXL362::setupDCInactivityInterrupt(int16_t threshold, int16_t time){
	// Setup motion and time thresholds
	SPIwriteTwoRegisters(0x23, threshold);
	SPIwriteTwoRegisters(0x25, time);

	// turn on inactivity interrupt
	byte ACT_INACT_CTL_Reg = SPIreadOneRegister(0x27);   // Read current reg value 
	ACT_INACT_CTL_Reg = ACT_INACT_CTL_Reg | (0x04);      // turn on bit 3, INACT_EN  
	SPIwriteOneRegister(0x27, ACT_INACT_CTL_Reg);        // Write new reg value 
	ACT_INACT_CTL_Reg = SPIreadOneRegister(0x27);        // Verify properly written

#ifdef ADXL362_DEBUG
	Serial.print("DC Inactivity Threshold set to ");  Serial.print(SPIreadTwoRegisters(0x23));
	Serial.print(", Time Inactivity set to ");  Serial.print(SPIreadTwoRegisters(0x25));
	Serial.print(", ACT_INACT_CTL Register is ");  Serial.println(ACT_INACT_CTL_Reg, HEX);
#endif
}

void ADXL362::setupACInactivityInterrupt(int16_t threshold, int16_t time){
	//  Setup motion and time thresholds
	SPIwriteTwoRegisters(0x23, threshold);
	SPIwriteTwoRegisters(0x25, time);
 
	// turn on inactivity interrupt
	byte ACT_INACT_CTL_Reg = SPIreadOneRegister(0x27);   // Read current reg value
	ACT_INACT_CTL_Reg = ACT_INACT_CTL_Reg | (0x0C);      // turn on bit 3 and 4, INACT_AC_DCB, INACT_EN  
	SPIwriteOneRegister(0x27, ACT_INACT_CTL_Reg);        // Write new reg value 
	ACT_INACT_CTL_Reg = SPIreadOneRegister(0x27);        // Verify properly written

#ifdef ADXL362_DEBUG
	Serial.print("AC Inactivity Threshold set to ");  Serial.print(SPIreadTwoRegisters(0x23));
	Serial.print(", Time Inactivity set to ");  Serial.print(SPIreadTwoRegisters(0x25)); 
	Serial.print(", ACT_INACT_CTL Register is ");  Serial.println(ACT_INACT_CTL_Reg, HEX);
#endif
}

void ADXL362::checkAllControlRegs(){
	//byte filterCntlReg = SPIreadOneRegister(0x2C);
	//byte ODR = filterCntlReg & 0x07;  Serial.print("ODR = ");  Serial.println(ODR, HEX);
	//byte ACT_INACT_CTL_Reg = SPIreadOneRegister(0x27);      Serial.print("ACT_INACT_CTL_Reg = "); Serial.println(ACT_INACT_CTL_Reg, HEX);
	digitalWrite(slaveSelectPin, LOW);
	SPI.transfer(0x0B);  // read instruction
	SPI.transfer(0x20);  // Start burst read at Reg 20
#ifdef ADXL362_DEBUG
	Serial.println("Start Burst Read of all Control Regs - Library version 6-5-2014:");
	Serial.print("Reg 20 = "); 	Serial.println(SPI.transfer(0x00), HEX);
	Serial.print("Reg 21 = "); 	Serial.println(SPI.transfer(0x00), HEX);
	Serial.print("Reg 22 = "); 	Serial.println(SPI.transfer(0x00), HEX);
	Serial.print("Reg 23 = "); 	Serial.println(SPI.transfer(0x00), HEX);
	Serial.print("Reg 24 = "); 	Serial.println(SPI.transfer(0x00), HEX);
	Serial.print("Reg 25 = "); 	Serial.println(SPI.transfer(0x00), HEX);
	Serial.print("Reg 26 = "); 	Serial.println(SPI.transfer(0x00), HEX);
	Serial.print("Reg 27 = "); 	Serial.println(SPI.transfer(0x00), HEX);
	Serial.print("Reg 28 = "); 	Serial.println(SPI.transfer(0x00), HEX);
	Serial.print("Reg 29 = "); 	Serial.println(SPI.transfer(0x00), HEX);
	Serial.print("Reg 2A = "); 	Serial.println(SPI.transfer(0x00), HEX);
	Serial.print("Reg 2B = "); 	Serial.println(SPI.transfer(0x00), HEX);
	Serial.print("Reg 2C = "); 	Serial.println(SPI.transfer(0x00), HEX);
	Serial.print("Reg 2D = "); 	Serial.println(SPI.transfer(0x00), HEX);
	Serial.print("Reg 2E = "); 	Serial.println(SPI.transfer(0x00), HEX);
#endif
	digitalWrite(slaveSelectPin, HIGH);
}

// Basic SPI routines to simplify code
// read and write one register
byte ADXL362::SPIreadOneRegister(byte regAddress){
	byte regValue = 0;
  
	digitalWrite(slaveSelectPin, LOW);
	SPI.transfer(0x0B);  // read instruction
	SPI.transfer(regAddress);
	regValue = SPI.transfer(0x00);
	digitalWrite(slaveSelectPin, HIGH);

	return regValue;
}

void ADXL362::SPIwriteOneRegister(byte regAddress, byte regValue){
  
	digitalWrite(slaveSelectPin, LOW);
	SPI.transfer(0x0A);  // write instruction
	SPI.transfer(regAddress);
	SPI.transfer(regValue);
	digitalWrite(slaveSelectPin, HIGH);
}

int16_t ADXL362::SPIreadTwoRegisters(byte regAddress){
	int16_t twoRegValue = 0;
  
	digitalWrite(slaveSelectPin, LOW);
	SPI.transfer(0x0B);  // read instruction
	SPI.transfer(regAddress);  
	twoRegValue = SPI.transfer(0x00);
	twoRegValue = twoRegValue + (SPI.transfer(0x00) << 8);
	digitalWrite(slaveSelectPin, HIGH);

	return twoRegValue;
}  

void ADXL362::SPIwriteTwoRegisters(byte regAddress, int16_t twoRegValue){
	byte twoRegValueH = twoRegValue >> 8;
	byte twoRegValueL = twoRegValue;
  
	digitalWrite(slaveSelectPin, LOW);
	SPI.transfer(0x0A);  // write instruction
	SPI.transfer(regAddress);  
	SPI.transfer(twoRegValueL);
	SPI.transfer(twoRegValueH);
	digitalWrite(slaveSelectPin, HIGH);
}
