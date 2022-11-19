/*

Winkel ICT ADXL362

Library for ADXL362 accelerometer: ultralow power (0,270ua) with autonomous motion switch functionality.

Copyright (C) 2022 Klaas-Jan Winkel / Winkel ICT

All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*/

#ifndef __ADXL362_H__
#define __ADXL362_H__


//comment out to remove serial monitor prints, and function pre-condition checks
//#define ADXL362_DEBUG

//needed for printregisters debugging function
//download here: https://playground.arduino.cc/Code/HashMap/
//#define ADXL362_HASHMAPLIBINSTALLED

#define ADXL362_VERIFY_REG_READBACK
//#define ADXL362_VERIFY_REG_WRITES_HAMMING

//untested, if never using values that high, can skip writing it (but then not sure if MSB is 0)
//#define ADXL362_POWERSAVE_SKIPWRITE_MSB

//#define ADXL362_USE_SPI_STUB


//said to be 6hz in datasheet,use executeSelfTest to adjust, use executeSelfTest(true) to print chip specific/new #define's for these values
#define ADXL362_WAKEUPMODE_ACTUALODR 6.0f
#define ADXL362_TIMECORRECTION_INPERCENT 0.0f


#include "ADXL362Reg.h"

#ifdef ADXL362_USE_SPI_STUB
#include <SPIStub.h>
#else
#include <SPI.h>
#endif

#include "Arduino.h"
#include "stdint.h"


//#define new_min(x,y) (((x) <= (y)) ? (x) : (y))


#define SEC_TO_MS(x) x*1000
#define MIN_TO_MS(x) x*60*1000

//threshold active time
#define MAX_8BIT 256-1
//threshold inactive time
#define MAX_16BIT 65536-1
//# FIFO entries
#define MAX_10BIT 1024-1
//treshold G's
#define MAX_11BIT 2048-1
//temperature and xyz (LH)
#define MAX_12BIT 4096-1

//datasheet page 4
#define ADXL362_2G_THRESHOLD_LSBPERG 1000
#define ADXL362_4G_THRESHOLD_LSBPERG 500
//datasheet freefall example uses scale factor 250, while in the specifications it says 235!
//THRESH_INACT = Threshold Value [g] × Scale Factor [LSB per g]
//page 27:
//THRESH_INACT [g] ... =  Threshold Value [g] = THRESH_INACT [codes]/Sensitivity [codes per g]
//sensitive is actually 4, so the specifications dont match!!
//1000/4 = 250 .. but its not 4. Probably its rounded
//using scale factor as this is more accurate
#define ADXL362_8G_THRESHOLD_LSBPERG 235

#define ADXL362_TEMP_SENSITIVY_AVG_CPERLSB 0.065
//averaging bias+standard deviation seems to be more accurate
//TODO: seems to differ per chip
#define ADXL362_TEMP_BIAS_AVG (350+290)/2

#define ADXL362_FIFO_MAX_SAMPLES 512

#define SPI_MIN_WRITEABLE_REG ADXL362_REG_SOFT_RESET
#define SPI_MAX_WRITEABLE_REG ADXL362_REG_SELF_TEST

//officialy should be minimum 4/100*1000 = 40ms
#define ADXL362_SELF_TEST_SETTLETIME_INMS 1000
#define ADXL362_SELF_TEST_NROFSAMPLES 16

#define ADXL362_CALIBRATE_INTERVAL 3000

typedef enum
{
	ad_range_2G 	= ADXL362_FILTER_CTL_RANGE(ADXL362_RANGE_2G), //default, most sensitive
	ad_range_4G 	= ADXL362_FILTER_CTL_RANGE(ADXL362_RANGE_4G),
	ad_range_8G 	= ADXL362_FILTER_CTL_RANGE(ADXL362_RANGE_8G)
} measurementRange;
#define AD_RANGE_OFF ADXL362_FILTER_CTL_RANGE(0xFF)

/* Datasheet (wakeup mode):
 * Switch into full bandwidth measurement mode
• Signal an interrupt to a microcontroller
• Wake up downstream circuitry, depending on the configuration
• In wake-up mode, all accelerometer features are available with the exception of the activity timer.
All registers can be accessed, and real-time data can be read and/or stored in the FIFO.
 */
//Bandwidth is the bandwidth of the analog digital converter, the higher the less aliasing (more data points) / the more accuracy
//the higher the ODR, the more power consumption
//confusing part: ODR is always halved, can be halved one more time with HALF_BW = 1/4
typedef enum
{
	ad_bandwith_externalODR							= -2,
	ad_bandwidth_hz_6_wakeup_ultralowpower			= -1, //wakeup mode  //0,27ua
	ad_bandwidth_hz_3_125 			= ADXL362_FILTER_CTL_ODR(ADXL362_ODR_12_5_HZ)	| ADXL362_FILTER_CTL_HALF_BW,
	ad_bandwidth_hz_6_25_lowpower	= ADXL362_FILTER_CTL_ODR(ADXL362_ODR_12_5_HZ),
	ad_bandwidth_hz_6_25			= ADXL362_FILTER_CTL_ODR(ADXL362_ODR_25_HZ) 	| ADXL362_FILTER_CTL_HALF_BW,
	ad_bandwidth_hz_12_5_lowpower	= ADXL362_FILTER_CTL_ODR(ADXL362_ODR_25_HZ),
	ad_bandwidth_hz_12_5			= ADXL362_FILTER_CTL_ODR(ADXL362_ODR_50_HZ) 	| ADXL362_FILTER_CTL_HALF_BW,
	ad_bandwidth_hz_25_lowpower 	= ADXL362_FILTER_CTL_ODR(ADXL362_ODR_50_HZ),
	ad_bandwidth_hz_25				= ADXL362_FILTER_CTL_ODR(ADXL362_ODR_100_HZ)	| ADXL362_FILTER_CTL_HALF_BW, //DEFAULT (more conversative acc. to datasheet)
	ad_bandwidth_hz_50_lowpower 	= ADXL362_FILTER_CTL_ODR(ADXL362_ODR_100_HZ),   //1.8uA  //ratings based on this
	ad_bandwidth_hz_50				= ADXL362_FILTER_CTL_ODR(ADXL362_ODR_200_HZ)	| ADXL362_FILTER_CTL_HALF_BW,
	ad_bandwidth_hz_100_lowpower 	= ADXL362_FILTER_CTL_ODR(ADXL362_ODR_200_HZ),
	ad_bandwidth_hz_100				= ADXL362_FILTER_CTL_ODR(ADXL362_ODR_400_HZ)	| ADXL362_FILTER_CTL_HALF_BW,
	ad_bandwidth_hz_200 			= ADXL362_FILTER_CTL_ODR(ADXL362_ODR_400_HZ)    //3.0uA
} bandwidth;
#define AD_BANDWIDTH_OFF ADXL362_FILTER_CTL_ODR(0xFF) | ADXL362_FILTER_CTL_HALF_BW //=12.5hz all thats 1 here is going to be set to 0


//when changed from standby to any other mode, settings in write registers will become active
//measurement mode needs to be on also for wakeup and autosleep mode, as otherwise it could be in standby mode
//For internal use only!
typedef enum
{
	ad_power_standby 		= ADXL362_POWER_CTL_MEASURE(ADXL362_MEASURE_STANDBY), //10na, suspend measurement //DEFAULT mode after reset
	ad_power_on		 		= ADXL362_POWER_CTL_MEASURE(ADXL362_MEASURE_ON), //3ua //called default mode in datasheet
	ad_power_wakeup 		= ADXL362_POWER_CTL_WAKEUP 		| ADXL362_POWER_CTL_MEASURE(ADXL362_MEASURE_ON), //270na 6 samples per second //activity time treshold ignored (not inactivity, so interrupt probably will be cleared after inactive?)
	ad_power_autowakeup		= ADXL362_POWER_CTL_AUTOSLEEP 	| ADXL362_POWER_CTL_MEASURE(ADXL362_MEASURE_ON) //have to only set it once, can only be used (optional) with link or loop
} powerMode;

//wakeup, on standby, and autosleep are mutually exclusive
#define AD_POWER_OFF ADXL362_POWER_CTL_MEASURE(0xFF) | ADXL362_POWER_CTL_WAKEUP | ADXL362_POWER_CTL_AUTOSLEEP

typedef enum
{
	ad_seq_none = 0x00,
	ad_seq_link = ADXL362_ACT_INACT_CTL_LINKLOOP(ADXL362_MODE_LINK), //act/inact/awake interrupts need to be cleared by reading status
	ad_seq_loop = ADXL362_ACT_INACT_CTL_LINKLOOP(ADXL362_MODE_LOOP)  //on interrupts need to be cleared
} sequentialMode;
#define AD_LINKLOOP_OFF ADXL362_ACT_INACT_CTL_LINKLOOP(0xFF)


typedef enum {
	ad_noise_normal 	= ADXL362_POWER_CTL_LOW_NOISE(ADXL362_NOISE_MODE_NORMAL), //1.8ua @50hz  DEFAULT
	ad_noise_low 		= ADXL362_POWER_CTL_LOW_NOISE(ADXL362_NOISE_MODE_LOW), //3.3ua
	ad_noise_ultralow 	= ADXL362_POWER_CTL_LOW_NOISE(ADXL362_NOISE_MODE_ULTRALOW)//13ua
} noiseMode; //the more noise reduction the more power is used
#define AD_NOISE_OFF ADXL362_POWER_CTL_LOW_NOISE(0xFF)

//bitmask's are the same for status,intmap1 and intmap2 registers
typedef enum {
	ad_status_awake 		= ADXL362_STATUS_AWAKE, //example : motion switch, bit 7 is for other purposes
	ad_status_inactive 		= ADXL362_STATUS_INACT, //example: freefall
	ad_status_active 		= ADXL362_STATUS_ACT,
	ad_status_fifo_overrun 	= ADXL362_STATUS_FIFO_OVERRUN, //FIFO buffer full
	ad_status_fifo_watermark = ADXL362_STATUS_FIFO_WATERMARK, //more samples in FIFO then in set in samples register
	ad_status_fifo_ready 	= ADXL362_STATUS_FIFO_RDY, //at least 1 sample in FIFO buffer
	ad_status_data_ready 	=ADXL362_STATUS_DATA_RDY //set with 10us delay after read-regs are set
} status;
#define AD_STATUS_OFF B0111111 //not really needed here as none has value 0

typedef enum {
	ad_fifo_disable 	= ADXL362_FIFO_CTL_FIFO_MODE(ADXL362_FIFO_DISABLE),
	ad_fifo_oldestsaved = ADXL362_FIFO_CTL_FIFO_MODE(ADXL362_FIFO_OLDEST_SAVED),
	ad_fifo_stream 		= ADXL362_FIFO_CTL_FIFO_MODE(ADXL362_FIFO_STREAM),
	ad_fifo_trigger 	= ADXL362_FIFO_CTL_FIFO_MODE(ADXL362_FIFO_TRIGGERED)
} fifoMode;
#define AD_FIFO_OFF ADXL362_FIFO_CTL_FIFO_MODE(0xFF)

//ATTENTION: do not set values in this struct yourself but use the functions, some combinations are not allowed
struct ADXL362Config {
	short status; //used for communicating errors in configuration (lot of function wont execute if <= 0)
	measurementRange rangeInG; //used by act/inact to calculate g treshold
	bandwidth activeBandwithInHz; //when accelerometer had active interrupt an is possibly measuring for an inactive interrupt
	bandwidth InActiveBandwithInHz;
	//assuming that wakeup/autosleep not possible with external ODR (active=inactive bandwith=odr same for both)
	uint16_t externalODRInHz; //used by act/inact functions to calculate time treshold
	noiseMode noisemode; //used for debugging only
};

struct MeasurementInMg {
	int16_t x;
	int16_t y;
	int16_t z;
};

//FIFO
typedef enum {
	ad_fifo_X_Mg = ADXL362_FIFOENTRY_X,
	ad_fifo_Y_Mg = ADXL362_FIFOENTRY_Y,
	ad_fifo_Z_Mg = ADXL362_FIFOENTRY_Z,
	ad_fifo_TEMP_C = ADXL362_FIFOENTRY_TEMP,
	ad_fifo_UNKNOWN = -1
} fifoEntryType;

struct FifoEntry {
	int16_t value;
	float floatValue;
	fifoEntryType type;
};

struct FifoMeasurement {
	MeasurementInMg forceInMg;
	float tempInC;
	bool complete;
};

class ADXL362
{
public:
	ADXL362(byte slaveSelectPin, SPIClass *spi=&SPI);

public:
    short init();

    //most simple configuration for getting xyz, temp and possibly testing/using FIFO
    short activateMeasure(bandwidth bandwidthInHz = ad_bandwidth_hz_25, measurementRange measurementRangeInG = ad_range_2G, noiseMode noiseMode = ad_noise_normal);


    //implementation of autonomous motion switch example page 36 of datasheet (defaults are settings from example)
    //page 17: Used in conjunction with loop mode, this configuration implements a trivial, autonomous motion activated switch, as shown in Figure 43.
    short activateAutonomousMotionSwitch(uint16_t minForceInMg = 250, uint16_t maxForceInMg = 150
    									, uint32_t inActminTimeInMs = SEC_TO_MS(5), bool linkMode=false, bool autoSleep = false, bandwidth bandwidthInHz = ad_bandwidth_hz_6_wakeup_ultralowpower);

    //Datasheet page 37:
    //To use inactivity to implement free fall detection, set the value in THRESH_INACT to the desired free fall threshold. Values between 300 mg and 600 mg are recommended
    //Set the value in TIME_INACT to implement the minimum amount of time that the acceleration on all axes must be less than the free fall threshold to generate a free fall condition.
    //Values between 100 ms and 350 ms are recommended;
    //defaults's according to datasheet implementaton on page 36 (attention: if changing treshold/min time configuration could not function as free fall detection anymore)
	short activateFreeFallDetection(uint16_t maxForceInMg = 600, uint32_t minTimeInMs = 30, bandwidth bandwidthInHz = ad_bandwidth_hz_50_lowpower);


	//some bandwith's wont work with all parameters (6hz & actmintimeinms for example)
    short activateCustomDetection(bandwidth bandwidthInHz, sequentialMode smode, bool autoSleep
    						, uint16_t minForceInMg, uint32_t actminTimeInMs
							, uint16_t maxForceInMg, uint32_t inActminTimeInMs
							, status int1Status = ad_status_active, status int2Status = ad_status_inactive, bool activeLow = false, bool absoluteMode = false
							, measurementRange measurementRangeInG = ad_range_2G, noiseMode noiseMode = ad_noise_normal);

    MeasurementInMg executeSelfTest();

    //executes time calibration for 100hz low power and wakeup mode (lowest power mode) as each chip differs
    //will adjust the default values
    //ATTENTION: printresults=true will have no effect when not in debug mode!
    short executeTimeCalibration(bool printResultsWhenDebugMode = false);


    /**************** SEQUENTIAL MODE: adds functionality to default mode  ***********************/
    //Supports awake bit
    //as this interrupt/bit stays on an can act like a on/off switch in addition to an interrupt unlike an interrupt
    ADXL362Config configureSequentialMode(bool linkMode = true, bandwidth bandwidthInHz = ad_bandwidth_hz_25 , bool autoSleep = false
    											, measurementRange measurementRangeInG = ad_range_2G
												,noiseMode noiseMode = ad_noise_normal, uint16_t externalODRInHz = 0);

	//LINK/LOOP Their interrupts (if mapped) must be acknowledged by the host processor by reading the STATUS register.
	//servicing routines for wakeup mode / this is kind of a dummy routine but check proper usage of functions
	//clears/acknowledges interrupt
	short isAwake(sequentialMode sequantialMode); //awake bit/ acknowledgement not necessary in loop mode


	/*************** NON-SEQUANTIAL/DEFAULT MODE *************************/

    //The antialiasing filter of the ADXL362 defaults to the more conservative setting, where bandwidth is set to one-fourth the output data rate.
    ADXL362Config configure(bandwidth bandwidthInHz = ad_bandwidth_hz_25, measurementRange measurementRangeInG = ad_range_2G
    												,noiseMode noiseMode = ad_noise_normal, uint16_t externalODRInHz = 0); //implies power measure, still can use interrupts but not lower power/wakeup mode


	/*************** BOTH MODES *************************/

    short configureInterrupt1(ADXL362Config config, status intstatus, bool activeLow = false, bool externalClock = false);
    short configureInterrupt2(ADXL362Config config, status intstatus, bool activeLow = false, bool externalSampleTrigger = false);

	//absolute mode is to exclude earths gravity, only include when perceived gravity plays a significant role (example: free fall)
	//if mintimeinssec=0 a minimum sample treshold will be set to minimize false positives
    //maximum 2g range = 2047mg, max 8g range = 8700mg, max time treshold 20.4 seconds @3.125hz
	short configureActivity(ADXL362Config config, uint16_t minForceInMg, uint16_t minTimeInMs = 0, bool absoluteMode = false);

	short configureActivityWakeupMode(ADXL362Config config, uint16_t minForceInMg, bool absoluteMode = false);


	//for inactivity detection minimum inactivity time plays a more significant role, so no default
	//maximum 2g range = 2047mg, max 8g range = 8700mg, max time treshold 87,38 minutes @3.125hz
	short configureInActivity(ADXL362Config config, uint16_t maxForceInMg, uint32_t minTimeInMs, bool absoluteMode = false);

	//datasheet: Reading the STATUS register (Address 0x0B) clears activity and inactivity interrupts.
	//servicing routines measurement mode
	//clears/acknowledges interrupt
	short isActInterrupt(); //without awake bit, use active / ianctive bit
	short isInactInterrupt();


	//configuration will only be active when this is called
	//can also be used to re-enable mode after standby
	//cant be done in the configurewake/measure functions as first act/inact has to be configured before enabling the mode
	//can also be used to re-activate
	short activateMode(ADXL362Config config);



	/*************** OTHER *************************/

    //misc functions
	void printRegisters(bool avoidIntteruptAcknowledgement = false, bool printBinary = false);
    short checkDevice();
    byte getRevisionId();
    short softReset(bool verify = true);
    bool hasStatus(status status); //call to clear activity/inactivity interrupts
    short activateStandbyMode(); //to temp disable all measurements and go to 10na power usage

    //Measurement functions
    MeasurementInMg getXYZLowPower(measurementRange range);
    MeasurementInMg getXYZ(measurementRange range);
    float getTemperature();

    //FIFO functions
    short configureFIFO(fifoMode mode, uint16_t maxSamplesEVEN = ADXL362_FIFO_MAX_SAMPLES, bool storeTemp = true);

    short configureFIFOInterrupt1(status int1FIFOstatus);
    short configureFIFOInterrupt2(status int2FIFOstatus);
    uint16_t getNrOf16bitFIFOEntries();
    uint16_t readFIFO(uint16_t* dst, uint16_t lenwanted = ADXL362_FIFO_MAX_SAMPLES);
    FifoEntry parseFIFOEntry(measurementRange range, uint16_t rawentry);
    FifoMeasurement parseFIFOMeasurement(measurementRange range, uint16_t** bufferptr, uint16_t* bufferlen, bool tempEnabled = true);

protected:
    //ATTENTION: these functions do not enforce the right combination of modes and settings, use at your own risk
	short configureINT1(status onStatus, bool activeLow = false);
	short configureINT2(status onStatus, bool activeLow = false);

	short setPowerMode(ADXL362Config config);
	short setPowerMode(powerMode mode);

	//misc features, untested
	short configureAccelerometerInt2SampleTrigger();
	short configureAccelerometerInt1Clock();


private:
    int32_t calculateTimeThreshold(uint32_t timeInMS, bandwidth bandwidthInHz, bool forInactive, uint16_t externalODRInHz);
    short calculateGThreshold(uint16_t mg, measurementRange measurementRangeInG);
    byte statusToBitmask(status status);
    MeasurementInMg rawToMeasurement(measurementRange range, uint16_t xraw, uint16_t yraw, uint16_t zraw);
    int16_t rawToMeasurement(measurementRange range, uint16_t raw);
    float rawToTemp(uint16_t rawTemp);
    uint16_t rangeToCodesPerG(measurementRange range);
    short measureActualTimeInterval(bandwidth bandwidthInHz);

    bool isOn(byte reg, byte bitmask);

    short on2(byte reg, byte onbitmask1, byte resetbitmask1,  byte onbitmask2, byte resetbitmask2);
    short on(byte reg, byte onbitmask); //single bit version
    short on(byte reg, byte onbitmask, byte resetbitmask); //multi bit
    short off(byte reg, byte offbitmask);

    byte getReg(byte reg);
    void getRegBurstBIGENDIAN(byte reg, byte* dst, byte len, byte modeByte = ADXL362_READ_REG);

    uint16_t getReg16(byte reg);
    void getRegBurst16(byte reg, uint16_t* dst, byte len16, byte modeByte = ADXL362_READ_REG);

    short setReg16(byte reg, uint16_t val);
    short setReg(byte reg, uint16_t val, bool twoBytes = false);

    void spiBeginTransaction();
    void spiEndTransaction();

private:
    bool _powerModeInProgress = false;
    uint8_t  _ss;
    SPIClass *_spi;

    float wakeupModeActualOdr = ADXL362_WAKEUPMODE_ACTUALODR;
    float timeTresholdCorrectInPerc = ADXL362_TIMECORRECTION_INPERCENT;
};

#endif /* __ADXL362_H__ */
