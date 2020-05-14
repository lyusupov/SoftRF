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

#include "Arduino.h"

#ifndef ADXL362_h
#define ADXL362_h

class ADXL362
{
public:

	ADXL362();
	
	//
	// Basic Device control and readback functions
	//
	void begin(int16_t chipSelectPin = 10); 		
	void beginMeasure(); 
	int16_t readXData();
	int16_t readYData();
	int16_t readZData();
	void readXYZTData(int16_t &XData, int16_t &YData, int16_t &ZData, int16_t &Temperature);
	int16_t readTemp();
	
	//
	// Activity/Inactivity interrupt functions
	//
	void setupDCActivityInterrupt(int16_t threshold, byte time);	
	void setupDCInactivityInterrupt(int16_t threshold, int16_t time);
    void setupACActivityInterrupt(int16_t threshold, byte time);
	void setupACInactivityInterrupt(int16_t threshold, int16_t time);
	
	// need to add the following functions
	// void mapINT1(
	// void mapINT2
	// void autoSleep
	// void activityInterruptControl
	//		-Activity, Inactivity, Both
	//		- Referenced, Absolute
	//		- Free Fall, Linked Mode, Loop Mode
	
	void checkAllControlRegs();
	

	
	//  Low-level SPI control, to simplify overall coding
	byte SPIreadOneRegister(byte regAddress);
	void SPIwriteOneRegister(byte regAddress, byte regValue);
	int16_t  SPIreadTwoRegisters(byte regAddress);
	void SPIwriteTwoRegisters(byte regAddress, int16_t twoRegValue);

private:
	int16_t slaveSelectPin;

	
};

#endif
