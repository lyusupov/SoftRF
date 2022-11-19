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

#include "ADXL362.h"

#ifdef ADXL362_USE_SPI_STUB
#include <SPIStub.h>
#else
#include <SPI.h>
#endif


/************************ Functions Definitions *******************************/


/*
 * Default accelerometer configuration (after: power on, init and softReset)
 *
 * - range : 2g
 * - bandwidth : 1/4
 * - int2 trigger: off
 * - output data rate: 100Hz
 *
 * - power mode: standby
 * - noise mode: normal
 *
 */
ADXL362::ADXL362(byte slaveSelectPin, SPIClass *spi) //, RHGenericSPI& spi
    :
		_ss(slaveSelectPin),
		_spi(spi)
{
}


short ADXL362::init()
{
    volatile short status   = true;

    //setup spi
    pinMode(_ss, OUTPUT);
    digitalWrite(_ss, HIGH);
    _spi->begin();


    status = softReset();
    if (status <= 0) return status;

    status = checkDevice();
    if (status <= 0) return status;

    _powerModeInProgress = false;

    return status;
}


/************************ Preconfigured Modes *********************************/


short ADXL362::activateMeasure(bandwidth bandwidthInHz, measurementRange measurementRangeInG, noiseMode noiseMode) {
	short status=1;
	//ATTENTION: when bandwith = ad_bandwidth_hz_extODR_sampleAndClock, not INT can be set or used
	ADXL362Config config = configure(bandwidthInHz, measurementRangeInG, noiseMode);

	if (config.status <= 0) return config.status;

	status = activateMode(config);

	return status;
}

//default example, motion switch, freefall etc.
//starts with inactivity!
//range does not really matter here, noise reduction cant be used here
short ADXL362::activateAutonomousMotionSwitch(uint16_t minForceInMg, uint16_t maxForceInMg, uint32_t inActminTimeInMs
												,bool linkMode, bool autoSleep, bandwidth bandwidthInHz) {
	short status=1;

	ADXL362Config config;

	config = configureSequentialMode(linkMode, bandwidthInHz,  autoSleep);
	if (config.status <= 0) return config.status;

	//wake up mode ignores min time, set to 1 sample automatically
	status = configureActivityWakeupMode(config, minForceInMg);
	if (status <= 0) return status;

	status = configureInterrupt2(config, ad_status_awake);
	if (status <= 0) return status;

	//this just keeps the accelerometer awake until inactivity detected for inactmintimeinms
	status = configureInActivity(config, maxForceInMg, inActminTimeInMs);
	if (status <= 0) return status;


	//when woken up, use isact/isinact to acknowledge and check what happened and call activateMode(config) again to re-enter sleep
	//in linked mode act/inact will follow each in order
	//in default mode act/inact can happen in any order
	//in loop mode no acknowledgement/interrupt clearing is necessary
	status = activateMode(config);

	return status;
}

short ADXL362::activateFreeFallDetection(uint16_t maxForceInMg, uint32_t minTimeInMs, bandwidth bandwidthInHz) {
	short status=1;

	//unsure: make 8g configurable or not, necessary for free fall detetct? bandwidth should maybe be not to low but can be configurable for power savings
	ADXL362Config config = configure(bandwidthInHz, ad_range_8G);
	if (config.status <= 0) return config.status;

	//absolute mode
	status = configureInActivity(config, maxForceInMg, minTimeInMs, true);
	if (status <= 0) return status;

	status = configureInterrupt1(config, ad_status_inactive);
	if (status <= 0) return status;

	//use isact/isinact to clear interrupt, interrupts will keep occurring in any order
	status = activateMode(config);

	return status;
}


short ADXL362::activateCustomDetection(bandwidth bandwidthInHz, sequentialMode smode, bool autoSleep
									, uint16_t minForceInMg, uint32_t actminTimeInMs
									, uint16_t maxForceInMg, uint32_t inActminTimeInMs
									, status int1Status, status int2Status, bool activeLow, bool absoluteMode
									, measurementRange measurementRangeInG, noiseMode noiseMode) {
	short status=1;
	ADXL362Config config;

	if (smode != ad_seq_none)
		config =  configureSequentialMode((smode == ad_seq_link), bandwidthInHz, autoSleep, measurementRangeInG, noiseMode);
	else
		config =  configure(bandwidthInHz, measurementRangeInG, noiseMode);

	if (config.status <= 0) return config.status;

	status = configureActivity(config, minForceInMg, actminTimeInMs, absoluteMode);
	if (status <= 0) return status;

	status = configureInActivity(config, maxForceInMg, inActminTimeInMs, absoluteMode);
	if (status <= 0) return status;

	status = configureInterrupt1(config, int1Status, activeLow);
	if (status <= 0) return status;

	status = configureInterrupt2(config, int2Status, activeLow);
	if (status <= 0) return status;

	status = activateMode(config);

	return status;
}

MeasurementInMg ADXL362::executeSelfTest() {
	activateMeasure(ad_bandwidth_hz_50_lowpower, ad_range_8G);

	MeasurementInMg xyzbefore;
	uint32_t x = 0;
	uint32_t y = 0;
	uint32_t z = 0;

	for (int i = 0; i<ADXL362_SELF_TEST_NROFSAMPLES; i++) {
		MeasurementInMg xyz = getXYZ(ad_range_8G);
		x = x + xyz.x;
		y = y + xyz.y;
		z = z + xyz.z;
	}

	xyzbefore.x = x/ADXL362_SELF_TEST_NROFSAMPLES;
	xyzbefore.y = y/ADXL362_SELF_TEST_NROFSAMPLES;
	xyzbefore.z = z/ADXL362_SELF_TEST_NROFSAMPLES;

	//self test on
	on(ADXL362_REG_SELF_TEST,ADXL362_SELF_TEST_ST,ADXL362_SELF_TEST_ST);

	x = 0;
	y = 0;
	z = 0;

	//=40ms
	delay(ADXL362_SELF_TEST_SETTLETIME_INMS);

	for (int i = 0; i<ADXL362_SELF_TEST_NROFSAMPLES; i++) {
		MeasurementInMg xyz = getXYZ(ad_range_8G);
		x = x + xyz.x;
		y = y + xyz.y;
		z = z + xyz.z;
	}

	//self test off
	on(ADXL362_REG_SELF_TEST,0x00,ADXL362_SELF_TEST_ST);

	MeasurementInMg xyzafter;
	xyzafter.x = x/ADXL362_SELF_TEST_NROFSAMPLES;
	xyzafter.y = y/ADXL362_SELF_TEST_NROFSAMPLES;
	xyzafter.z = z/ADXL362_SELF_TEST_NROFSAMPLES;


	xyzafter.x = xyzafter.x-xyzbefore.x;
	xyzafter.y = xyzafter.y-xyzbefore.y;
	xyzafter.z = xyzafter.z-xyzbefore.z;

	return xyzafter;
}

short ADXL362::executeTimeCalibration(bool printResultsWhenDebugMode) {
	short status=1;

	short timeinms;

	//reset current values
	timeTresholdCorrectInPerc = 0;
	wakeupModeActualOdr = 10;

	//TODO: low power most important, so calibrate on low hz, chip Revision 3 seems to result in to low times in higher 25hz+ bandwidths
	timeinms = measureActualTimeInterval(ad_bandwidth_hz_3_125);
	if (timeinms < 0) return -180;

	timeTresholdCorrectInPerc = (ADXL362_CALIBRATE_INTERVAL/(float)timeinms*100.0) - 100.0;

	timeinms = measureActualTimeInterval(ad_bandwidth_hz_6_wakeup_ultralowpower);
	if (timeinms < 0) return -181;

	wakeupModeActualOdr = wakeupModeActualOdr * (ADXL362_CALIBRATE_INTERVAL/(float)timeinms);  //factor

	#ifdef ADXL362_DEBUG
	if (printResultsWhenDebugMode) {
		Serial.println(F("Replace in .h to make permanent: "));
		Serial.print(F("#define ADXL362_WAKEUPMODE_ACTUALODR "));
		Serial.print(wakeupModeActualOdr);
		Serial.println(F("f"));
		Serial.print(F("#define ADXL362_TIMECORRECTION_INPERCENT "));
		Serial.print(timeTresholdCorrectInPerc);
		Serial.println(F("f"));
	}
	#endif

	return status;
}


/************************ Configuration Functions (best executed in this order)******/


ADXL362Config ADXL362::configureSequentialMode(bool linkMode, bandwidth bandwidthInHz, bool autoSleep,
		measurementRange measurementRangeInG, noiseMode noiseMode, uint16_t externalODRInHz)
{
	struct ADXL362Config config;
	short statuscode = 1;

	if (_powerModeInProgress) {
		statuscode = softReset(); //includes going to standby mode
	}

	#ifdef ADXL362_DEBUG
	if (statuscode > 0)
		if (bandwidthInHz == ad_bandwidth_hz_6_wakeup_ultralowpower && autoSleep)
			statuscode = -11; //autosleep wont work, accelerometer will always be in lowest power mode/wakeup mode
	#endif

	_powerModeInProgress = true;

	if (statuscode > 0) {
		config = configure(bandwidthInHz, measurementRangeInG, noiseMode, externalODRInHz);
		statuscode = config.status;
	}

	if (statuscode > 0) {
		statuscode = on(ADXL362_REG_ACT_INACT_CTL,linkMode ? ad_seq_link : ad_seq_loop, AD_LINKLOOP_OFF);
	}

	config.status = statuscode;

	if (statuscode > 0) { //return codes make status never be 0 even if added
		config.rangeInG = measurementRangeInG;
		config.activeBandwithInHz = bandwidthInHz;
		//activateMode will check this an use autosleep mode (wakeupauto)
		config.InActiveBandwithInHz = autoSleep ? ad_bandwidth_hz_6_wakeup_ultralowpower : bandwidthInHz;
		config.noisemode = noiseMode;
		config.status = 1;
		config.externalODRInHz =externalODRInHz;
	}

	return config;
}


//clears/acknowledges interrupt, asking for sequantial mode to ensure user knows this will always return true in non sequantial/link loop mode
short ADXL362::isAwake(sequentialMode sequentialmode) {
	if (!sequentialmode)
		return -54;

	return hasStatus(ad_status_awake);
}


//The antialiasing filter of the ADXL362 defaults to the more conservative setting, where bandwidth is set to one-fourth the output data rate.
//ATTENTION: when bandwith = ad_bandwidth_hz_extODR_sampleAndClock, not INT can be set or used
ADXL362Config ADXL362::configure(bandwidth bandwidthInHz, measurementRange measurementRangeInG, noiseMode noiseMode, uint16_t externalODRInHz) {
	struct ADXL362Config config;
	short status = 1;

	if (_powerModeInProgress) {
		status = softReset(); //includes going to standby mode
	}

	#ifdef ADXL362_DEBUG
	//need to specify hz when external clock (not sure about sample/synchronized mode) (see datasheet for how to calculate)
	if (status > 0)
		if (bandwidthInHz == ad_bandwith_externalODR && externalODRInHz == 0)
			status = -40;
	#endif

	_powerModeInProgress = true;

	if (status > 0) {
		if (bandwidthInHz != ad_bandwith_externalODR && bandwidthInHz !=ad_bandwidth_hz_6_wakeup_ultralowpower) {
			status = on2(ADXL362_REG_FILTER_CTL,measurementRangeInG,AD_RANGE_OFF,bandwidthInHz,AD_BANDWIDTH_OFF);
		} else
			status = on(ADXL362_REG_FILTER_CTL,measurementRangeInG,AD_RANGE_OFF);
	}

	if (status > 0)
		status = on(ADXL362_REG_POWER_CTL,noiseMode,AD_NOISE_OFF);

	config.status = status;

	if (status > 0) { //return codes make status never be 0 even if added
		config.rangeInG = measurementRangeInG;
		config.activeBandwithInHz = bandwidthInHz;
		config.InActiveBandwithInHz = bandwidthInHz;
		config.noisemode = noiseMode;
		config.status = 1;
		config.externalODRInHz =externalODRInHz;
	}

	return config;
}


short ADXL362::configureInterrupt1(ADXL362Config config, status intstatus, bool activeLow, bool externalClock) {
	short status = 1;

	#ifdef ADXL362_DEBUG
	if (status != NULL && externalClock)
		return -61; //cant have both on int1

	if (status == NULL && !externalClock)
		return -62; //why has this function been called

	//assuming inactive bandwith cant be set different with external clock (e.g. wakeup/autosleep disabled)
	if (externalClock) {
		if (config.activeBandwithInHz != ad_bandwith_externalODR)
			return -63;
		else if (config.externalODRInHz == 0)
			return -64;
	}
	#endif

	if (status)
		return configureINT1(intstatus,activeLow);
	else {
		return configureAccelerometerInt1Clock();
	}
}

short ADXL362::configureInterrupt2(ADXL362Config config, status intstatus, bool activeLow, bool externalSampleTrigger) {
	short status = 1;

	#ifdef ADXL362_DEBUG
	//if (config.powermode != ad_power_on)
	//	return -70;

	if (status != NULL && externalSampleTrigger)
		return -71; //cant have both on int2

	if (status == NULL && !externalSampleTrigger)
		return -72; //why has this function been called

	/* for sampling still the internal bandwith / odr is used
	if (externalSampleTrigger) {
		if (config.activeBandwithInHz != ad_bandwith_externalODR)
			return -73;
		else if (config.externalODRInHz == 0)
			return -74;
	}
	*/
	#endif

	if (status)
		return configureINT2(intstatus,activeLow);
	else {
		return configureAccelerometerInt2SampleTrigger();
	}
}


//function just to make clear that wakeup mode does not use time
//no time threshold in wakeup mode, based automatically on 1 sample
//config settings are only needed when woken up, in wakeup mode 6 samples per second are made
short ADXL362::configureActivityWakeupMode(ADXL362Config config, uint16_t minForceInMg, bool absoluteMode) {
	return configureActivity(config,minForceInMg,0,absoluteMode);
}

/*
 * Datasheet:
 * In the referenced configuration, activity is detected when acceleration samples are at
 * least a user set amount above an internally defined reference for the user defined amount of time
 * ,as described in Equation:
 *
 * ABS(Acceleration - Reference) > Threshold
 *
 * References basically means: moved from the current position excluding any gravity influences
 * Almost always use this except for special cases like freefall where perceived gravity changes!
 *
 */
//time max 20.4 seconds @3.125hz
//returns negative status when invalid g or time
//for freefall using absolute because static 1g disappears?
short ADXL362::configureActivity(ADXL362Config config, uint16_t minForceInMg, uint16_t minTimeInMS, bool absoluteMode) {
	#ifdef ADXL362_DEBUG
	//cant configure threshold #samples = time for activity in wake up mode, just 1 sample is used(6 hz only possible in wakeup mode)
	//activity is measured in inactive state, time can then not be set in wake and autosleep mode
	//This can happen when wakemode or autosleep
	if (config.InActiveBandwithInHz == ad_bandwidth_hz_6_wakeup_ultralowpower && minTimeInMS != 0)
		return -22;
	#endif

	if (config.status <= 0)
		return -20;

	short status = 1;

	short gThreshold = calculateGThreshold(minForceInMg, config.rangeInG); //11bit
	if (gThreshold < 0) return (short)gThreshold;

	int32_t timeTreshold = calculateTimeThreshold(minTimeInMS, config.InActiveBandwithInHz, false, config.externalODRInHz); //8bit
	if (timeTreshold < 0) return (short)timeTreshold;

	//these functions will convert big (C variables) to little endian (accelerometer)
	status = setReg16(ADXL362_REG_THRESH_ACT_L,(uint16_t)gThreshold);
	if (status <= 0) return status;
	status = setReg(ADXL362_REG_TIME_ACT,(byte)timeTreshold); //convertible to uint16_t as function guarantees the max value
	if (status <= 0) return status;

	status = on(ADXL362_REG_ACT_INACT_CTL,ADXL362_ACT_INACT_CTL_ACT_EN | (!absoluteMode ? ADXL362_ACT_INACT_CTL_ACT_REF : 0x00), ADXL362_ACT_INACT_CTL_ACT_EN | ADXL362_ACT_INACT_CTL_ACT_REF);

	return status;
}


/*
 * Datesheet:
 * Referenced inactivity, like referenced activity, is particularly useful for eliminating the effects of the static acceleration due to
 * gravity (1g). With absolute inactivity, if the inactivity threshold is set lower than 1 g, a device resting motionless may never detect inactivity.
 * With referenced inactivity, the same device under the same configuration detects inactivity.
 */

//time max 87,38 minutes = 5242,8 seconds @3.125hz
//minimum ~2.5ms
//returns negative status when invalid g or time
short ADXL362::configureInActivity(ADXL362Config config, uint16_t maxForceInMg, uint32_t minTimeInMS, bool absoluteMode) {
	if (config.status <= 0)
		return -21;

	short status = 1;

	short gThreshold = calculateGThreshold(maxForceInMg, config.rangeInG); //11bit
	if (gThreshold < 0) return (short)gThreshold;

	//inactivity is measured in active mode, so time is based on active bandwith
	int32_t timeTreshold = calculateTimeThreshold(minTimeInMS, config.activeBandwithInHz, true, config.externalODRInHz); //8bit
	if (timeTreshold < 0) return (short)timeTreshold;


	//these functions will convert big (C variables) to little endian (accelerometer)
	status = setReg16(ADXL362_REG_THRESH_INACT_L,(uint16_t)gThreshold);
	if (status <= 0) return status;
	status = setReg16(ADXL362_REG_TIME_INACT_L,(uint16_t)timeTreshold);
	if (status <= 0) return status;

	status = on(ADXL362_REG_ACT_INACT_CTL,ADXL362_ACT_INACT_CTL_INACT_EN | (!absoluteMode ? ADXL362_ACT_INACT_CTL_INACT_REF : 0x00), ADXL362_ACT_INACT_CTL_INACT_EN | ADXL362_ACT_INACT_CTL_INACT_REF);

	return status;
}

short ADXL362::activateMode(ADXL362Config config) {
	if (config.status <= 0)
		return -30;

	#ifdef ADXL362_DEBUG
	//no noise mode in wake up mode's (and most likely it is there in autosleep+active)
	if (config.noisemode != ad_noise_normal && config.activeBandwithInHz == ad_bandwidth_hz_6_wakeup_ultralowpower)
		return -32;

	//if non equal bandwith's (autosleep) the inactive one can only be wakeup mode (for external ODR also both the same required)
	if (config.InActiveBandwithInHz != config.activeBandwithInHz && config.InActiveBandwithInHz != ad_bandwidth_hz_6_wakeup_ultralowpower)
		return -33;
	#endif

	powerMode pmode;

	if (config.activeBandwithInHz == ad_bandwidth_hz_6_wakeup_ultralowpower)
		pmode = ad_power_wakeup;
	else if (config.InActiveBandwithInHz != config.activeBandwithInHz)
		pmode = ad_power_autowakeup;
	else
		pmode = ad_power_on;

	short status = setPowerMode(pmode); //does not check validity of config

	return status;
}

//clears/acknowledges interrupt
short ADXL362::isActInterrupt() {
	return hasStatus(ad_status_active);
}

//clears/acknowledges interrupt
short ADXL362::isInactInterrupt() {
	return hasStatus(ad_status_inactive);
}



/************************ STATUS Functions *********************************/


short ADXL362::checkDevice()
{
	if (getReg(ADXL362_REG_DEVID_AD) != ADXL362_DEVICE_AD) {
		return -101;
	}
	if (getReg(ADXL362_REG_DEVID_MST) != ADXL362_DEVICE_MST)
		return -102;
	if (getReg(ADXL362_REG_PARTID) != ADXL362_PARTID)
		return -103;

	return true;
}

byte ADXL362::getRevisionId() {
	return getReg(ADXL362_REG_REVID);
}


short ADXL362::softReset(bool verify)
{
    //ignore if error, just to help softreset to work (which it does)
    //only readback can fail here
    activateStandbyMode();

	short status = setReg(ADXL362_REG_SOFT_RESET,ADXL362_RESET_KEY);
	if (status <= 0) return status;

	//Datatsheet: SOFT_RESET: A latency of approximately 0.5 ms is required after soft reset.
	delay(20);

	//according to the datasheet these registers should have these values after reset
	if (verify) {
		if (getReg(ADXL362_REG_FIFO_SAMPLES)!=ADXL362_REG_FIFO_SAMPLES_RESETVALUE)
			return -110;
		if (getReg(ADXL362_REG_FILTER_CTL)!=ADXL362_REG_FILTER_CTL_RESETVALUE)
			return -111;
		byte statusval = getReg(ADXL362_REG_STATUS);
		if (statusval!=ADXL362_REG_STATUS_RESETVALUE && statusval!=ADXL362_REG_STATUS_RESETVALUE_POWERON) {
			#ifdef ADXL362_DEBUG
			Serial.print("\r\nUnexpected status after reset: 0x");
			Serial.println(statusval, HEX);
			#endif
			return -112;
		}
	}

	return true;
}

/*
 * Clear interrupts in one of several ways, as follows:
• Reading the STATUS register (Address 0x0B) clears activity and inactivity interrupts.
• Reading from the data registers. Address 0x08 to Address 0x0A or Address 0x0E to Address 0x15 clears the data ready interrupt.
• Reading enough data from the FIFO buffer so that interrupt conditions are no longer met clears the FIFO ready, FIFO watermark, and FIFO overrun interrupts.
 */
//clears the activity and inactivity interrupts
//use for FIFO interrupts (the other interrupts have special functions above
bool ADXL362::hasStatus(status status) {
	return isOn(ADXL362_REG_STATUS, status);
}

short ADXL362::activateStandbyMode() {
	//readback verification will return -206 or -207
	return setPowerMode(ad_power_standby);
}


/************************ Measurement Functions *********************************/


//clears data ready interrupt
MeasurementInMg ADXL362::getXYZLowPower(measurementRange range) {
	//A burst read of all three axis is required to guarantee all measurements correspond to same sample time?
	byte values[3];
	getRegBurstBIGENDIAN(ADXL362_REG_XDATA, values, 3);

	//when highest bit in reg is 1 its a negative value, then add sign extension to the 12 bit value
	//F000 adds sign extension for 12 bit value
	//values are the 8 bit MSB of a 12 bit value, so shift 4 to left
	return rawToMeasurement(range,
			(values[0] & 0x80) >> 7 ? (values[0] << 4) | 0xF000 : values[0] << 4
				   ,(values[1] & 0x80) >> 7 ? (values[1] << 4) | 0xF000 : values[1] << 4
						   ,(values[2] & 0x80) >> 7 ? (values[2] << 4) | 0xF000 : values[2] << 4
								   );
}

//clears data ready interrupt
MeasurementInMg ADXL362::getXYZ(measurementRange range) {
	uint16_t values[3]; //values = uint16_t*
	getRegBurst16(ADXL362_REG_XDATA_L, values, 3);
	return rawToMeasurement(range,values[0],values[1],values[2]);
}

//clears data ready interrupt
float ADXL362::getTemperature() {
	return rawToTemp(getReg16(ADXL362_REG_TEMP_L));
}


/************************ FIFO Functions *********************************/


short ADXL362::configureFIFO(fifoMode mode, uint16_t maxSamplesEVEN, bool storeTemp) {
	#ifdef ADXL362_DEBUG
	if (maxSamplesEVEN == 0 | maxSamplesEVEN > ADXL362_FIFO_MAX_SAMPLES)
		return -240;

	if(maxSamplesEVEN % 2 != 0)
		return -241;
	#endif

	short status;

	//always use MSB (_AH)
	status = on(ADXL362_REG_FIFO_CTL,ADXL362_FIFO_CTL_AH | (storeTemp ? ADXL362_FIFO_CTL_FIFO_TEMP : 0x00), ADXL362_FIFO_CTL_AH | ADXL362_FIFO_CTL_FIFO_TEMP);


	//TODO: might be possible to set the previous settings within this function
	status = on(ADXL362_REG_FIFO_CTL,mode,AD_FIFO_OFF);

	if (status <= 0) return status;

	//TODO: not sure if this works,AH bit of control (which i set) doubles the range of the samples bit, so i need to divide the max samples with 2 so it will fit?
	//Datasheet: The following bit map (bit3) is duplicated from the FIFO Control Register section to indicate the AH bit.
	status = setReg(ADXL362_REG_FIFO_SAMPLES, (byte)((maxSamplesEVEN-1)/2),false); //-1 as byte starts at 0

	return status;
}

short ADXL362::configureFIFOInterrupt1(status int1FIFOstatus) {
	#ifdef ADXL362_DEBUG
	if (int1FIFOstatus != ad_status_fifo_overrun && int1FIFOstatus != ad_status_fifo_watermark && int1FIFOstatus != ad_status_fifo_ready) {
		return -242;
	}
	#endif
	return configureINT1(int1FIFOstatus);
}

short ADXL362::configureFIFOInterrupt2(status int2FIFOstatus) {
	#ifdef ADXL362_DEBUG
	if (int2FIFOstatus != ad_status_fifo_overrun && int2FIFOstatus != ad_status_fifo_watermark && int2FIFOstatus != ad_status_fifo_ready) {
		return -243;
	}
	#endif
	return configureINT2(int2FIFOstatus);
}

//max 511 / default ox80 to avoid triggering watermark interrupt
uint16_t ADXL362::getNrOf16bitFIFOEntries() {
	return getReg16(ADXL362_REG_FIFO_L);
}


/*
 * Read FIFO Command
Reading from the FIFO buffer is a command structure that does
not have an address.
</CS down> <command byte (0x0D)> <data byte> <data
byte> … </CS up>
It is recommended that an even number of bytes be read (using
a multibyte transaction) because each sample consists of two
bytes: 2 bits of axis information and 14 bits of data. If an odd
number of bytes is read, it is assumed that the desired data was
read; therefore, the second half of the last sample is discarded so a read from the FIFO always starts on a properly aligned even-
byte boundary. Data is presented least significant byte first, followed by the most significant byte.
 */
/*
Data is formatted as a 16-bit value as represented in Table 20.
When reading data, the least significant byte (Bits[B7:B0]) is read first,
followed by the most significant byte (Bits[B15:B8]).
Bits[B11:B0] represent the 12-bit, twos complement acceleration or temperature data.
 Bits[B13:B12] are sign extension bits, and Bits[B15:B14] indicate the type of data, as listed in Table 20.
*/

//Reading enough data from the FIFO buffer so that interrupt conditions are no longer met clears the FIFO ready, FIFO watermark, and FIFO overrun interrupts.
//clears FIFO watermark, and FIFO overrun interrupts. (reads all)
uint16_t ADXL362::readFIFO(uint16_t* dst, uint16_t lenwanted) {
    uint16_t lenmax = getNrOf16bitFIFOEntries();
    uint16_t len;

    if (lenwanted > 0)
    	len = (lenwanted > lenmax ? lenmax : lenwanted);
    else
    	len = lenmax;

    //Datasheet: When a multibyte read is performed, the number of bytes read must always be an even number.
    //also converts all 16 bit values to Big endian
    getRegBurst16(0, dst, len, ADXL362_READ_FIFO);
    return len; //return nr of entries read
}

/*
When reading data, the least significant byte (Bits[B7:B0]) is read first,
		followed by the most significant byte (Bits[B15:B8]).
		Bits[B11:B0] represent the 12-bit, twos complement acceleration or temperature data.
		Bits[B13:B12] are sign extension bits,
		and Bits[B15:B14] indicate the type of data, as listed in Table 20.
*/
FifoEntry ADXL362::parseFIFOEntry(measurementRange range, uint16_t rawentry) {
	FifoEntry fifoEntry;

	//Datasheet: see page 38
	byte rawtype = (byte)(rawentry >> 14);
	fifoEntry.type=(fifoEntryType)rawtype; //00,01,10 or 11, so any value maps to a type

	//remove the type bits
	uint16_t rawvalue = rawentry & 0x3FFF;
	//add extension bits if last extension bit is 1 (negative)
	//higher then 1 if extension bit exists, otherwise zero
	if (rawvalue & 0x2000)
		rawvalue = rawvalue | 0xC000; //add 2 highest extension bits

	if (fifoEntry.type != ad_fifo_TEMP_C)
		fifoEntry.value = rawToMeasurement(range, rawvalue);
	else
		fifoEntry.floatValue = rawToTemp(rawvalue);

	return fifoEntry;
}

//Intented usage: call in a loop, adding +4 to buffer pointer (buffer param) each time and setting remaining length (16bit = 1) in bufferlen until bufferen <= 0
//TODO: can be done faster by skipping parsing to fifoentry + assuming x,y,z, temp order (guaranteed, see datasheet)
FifoMeasurement ADXL362::parseFIFOMeasurement(measurementRange range, uint16_t** bufferptr, uint16_t* bufferlen, bool tempEnabled) {
	FifoMeasurement measurement;
	//byte entrylength = tempEnabled ? 4 : 3;
	byte nrOfEntries = tempEnabled ? 4 : 3;
	byte entriesParsed=0;

	measurement.forceInMg.x = 0;
	measurement.forceInMg.y = 0;
	measurement.forceInMg.z = 0;
	measurement.tempInC = 0;

	if (*bufferlen < nrOfEntries)
		nrOfEntries = *bufferlen;

	uint16_t* buffer = *bufferptr;

	//written in order, but depends on start of buffer
	for (byte i = 0; i < nrOfEntries; i++) {
		FifoEntry fifoentry = parseFIFOEntry(range, *(buffer+i));

		switch (fifoentry.type) {
		case ad_fifo_X_Mg:
			measurement.forceInMg.x = fifoentry.value;
			entriesParsed++;
			break;
		case ad_fifo_Y_Mg:
			measurement.forceInMg.y = fifoentry.value;
			entriesParsed++;
			break;
		case ad_fifo_Z_Mg:
			measurement.forceInMg.z = fifoentry.value;
			entriesParsed++;
			break;
		case ad_fifo_TEMP_C:
			measurement.tempInC = fifoentry.floatValue;
			entriesParsed++;
			break;
		}
	}

	*bufferlen = *bufferlen - nrOfEntries;
	*bufferptr = *bufferptr + nrOfEntries;

	if (entriesParsed == nrOfEntries)
		measurement.complete= true;
	else
		measurement.complete= false;

	return measurement;
}


/************************ Helper/Protected Functions **************************/


//other ways to change sampling rate, configure activity/inactivity times will be calculated wrong when used
short ADXL362::configureAccelerometerInt2SampleTrigger() {
	return on(ADXL362_REG_FILTER_CTL,ADXL362_FILTER_CTL_EXT_SAMPLE);
}
short ADXL362::configureAccelerometerInt1Clock() {
	return on(ADXL362_REG_POWER_CTL,ADXL362_POWER_CTL_EXT_CLK);
}



short ADXL362::configureINT1(status onStatus, bool activeLow) {
	return on(ADXL362_REG_INTMAP1, onStatus | (activeLow ? ADXL362_INTMAP1_INT_LOW : 0x00), ADXL362_INTMAP1_INT_LOW);
}

short ADXL362::configureINT2(status onStatus, bool activeLow) {
	return on(ADXL362_REG_INTMAP2, onStatus | (activeLow ? ADXL362_INTMAP2_INT_LOW : 0x00), ADXL362_INTMAP2_INT_LOW);
}


/*
 * The reference for activity detection is calculated when activity detection is engaged in the following scenarios:
• When the activity function is turned on and measurement mode is engaged;
• If link mode is enabled: when inactivity is detected and activity detection begins; or
• If link mode is not enabled: when activity is detected and activity detection repeats.
 */
short ADXL362::setPowerMode(powerMode mode) {
	short status = 1;

	status = on(ADXL362_REG_POWER_CTL,mode,AD_POWER_OFF);

	return status;
}



/* The time (in seconds) is given by the following equation:
 *
 * Time = TIME_ACT/ODR
 * TIME_ACT = Time(s) * ODR(hz)
 * ODR =
 * max 90sec
 *
 * Inactive supports 16 bit, active just 8 bit
*/
int32_t ADXL362::calculateTimeThreshold(uint32_t timeInMS, bandwidth bandwidthInHz, bool forInactive, uint16_t externalODRInHz) {
	float odr;

	//assuming that wakeup/autosleep not possible with external ODR (active=inactive bandwith=odr same for both)
	if (bandwidthInHz == ad_bandwith_externalODR)
		odr = (float)externalODRInHz;
	else {
		switch (bandwidthInHz) {
		case ad_bandwidth_hz_6_wakeup_ultralowpower:
			odr = wakeupModeActualOdr;
			break;
		case ad_bandwidth_hz_3_125:
		case ad_bandwidth_hz_6_25_lowpower:
			odr = 12.5;
			break;
		case ad_bandwidth_hz_6_25:
		case ad_bandwidth_hz_12_5_lowpower:
			odr = 25;
			break;
		case ad_bandwidth_hz_12_5:
		case ad_bandwidth_hz_25_lowpower:
			odr = 50;
			break;
		case ad_bandwidth_hz_25:
		case ad_bandwidth_hz_50_lowpower:
			odr = 100;
			break;
		case ad_bandwidth_hz_50:
		case ad_bandwidth_hz_100_lowpower:
			odr = 200;
			break;
		case ad_bandwidth_hz_100:
		case ad_bandwidth_hz_200:
			odr = 400;
		}
	}

	//uint32 is to make sure the multiplication result of odr*timeInMs is not clipped, not sure where its needed and not
	int32_t threshold = (int32_t)(((float)timeInMS * odr)/1000.0)*(1.0+timeTresholdCorrectInPerc/100.0); //max 5242.000


	//datasheet: To minimize false positive motion triggers, set the TIME_ACT register greater than 1.
	if (threshold == 0)
		return 2;

	if (forInactive) {
		if (threshold > MAX_16BIT)
			return -150; //65535 / 12,5 = 5242,8 seconds = max 87,38 minutes @3.125hz
	} else {
		if (threshold > MAX_8BIT)
			return -151; //255 / 12,5 = max 20.4 seconds @3.125hz
	}

	return threshold; //cant be higher then 16 bit
}

/*
 * THRESH_ACT is set in codes; the value in g depends on the measurement range setting that is selected.
 *
 * THRESH_ACT [g] = THRESH_ACT [codes]/Sensitivity [codes per g]
 * THRESH_ACT [g] * Sensitivity [codes per g] = THRESH_ACT [codes]
 *
 * (150 codes) to Register 0x23: sets free fall threshold to 600 mg.
 *
 * sensitivity 0,25 codes per mg
 *
 *
 */
short ADXL362::calculateGThreshold(uint16_t mg, measurementRange measurementRangeInG) {
	int codesG = (mg * rangeToCodesPerG(measurementRangeInG))/1000;

	if (codesG > MAX_11BIT)
		return -152; //2047 / 235 = max 8,7g = max 8700mg @4G range
	else return codesG;  //ADXL362_8G_THRESHOLD_CODESPERG
}


MeasurementInMg ADXL362::rawToMeasurement(measurementRange range, uint16_t xraw, uint16_t yraw, uint16_t zraw) {
	MeasurementInMg m;

	uint16_t codesperg = rangeToCodesPerG(range);

	m.x = (int16_t)((float)(int16_t)xraw / (codesperg / 1000.0));
	m.y = (int16_t)((float)(int16_t)yraw / (codesperg / 1000.0));
	m.z = (int16_t)((float)(int16_t)zraw / (codesperg / 1000.0));

	return m;
}

int16_t ADXL362::rawToMeasurement(measurementRange range, uint16_t raw) {
	uint16_t codesperg = rangeToCodesPerG(range);

	return (int16_t)((float)(int16_t)raw / (codesperg / 1000.0));
}

float ADXL362::rawToTemp(uint16_t rawTemp) {
	return (float)(int16_t)(rawTemp+ADXL362_TEMP_BIAS_AVG) * ADXL362_TEMP_SENSITIVY_AVG_CPERLSB;
}

uint16_t ADXL362::rangeToCodesPerG(measurementRange range) {
	switch (range) {
	case ad_range_2G:
		return ADXL362_2G_THRESHOLD_LSBPERG; //max 2g
		break;
	case ad_range_4G:
		return ADXL362_4G_THRESHOLD_LSBPERG; //max 4g
		break;
	case ad_range_8G:
		return ADXL362_8G_THRESHOLD_LSBPERG; //max 8,7
		break;
	default:
		return ADXL362_2G_THRESHOLD_LSBPERG; //should not get here
	}
}

short ADXL362::measureActualTimeInterval(bandwidth bandwidthInHz) {
	short status=1;
	//ATTENTION: when bandwith = ad_bandwidth_hz_extODR_sampleAndClock, not INT can be set or used
	ADXL362Config config = configure(bandwidthInHz);

	if (config.status <= 0) return config.status;

	//TODO: maximum force macro definition
	//TODO: measure time macro def
	status = configureInActivity(config, 2000, ADXL362_CALIBRATE_INTERVAL);
	if (status <=0) return status;

	status = activateMode(config);
	if (status <=0) return status;

	unsigned long start = millis();
	unsigned long lastintervalmeasureinms = 0;
	unsigned long actualintervalinms = 0;
	bool inactoccured = false;
	//measure
	while ((millis()-start) < 30000) {
		bool inactoccured = isInactInterrupt();
		if (inactoccured) {
			delay(10); //small delay, arduino can be to fast and acc cant reset the status in time, this causes measurement to be 0
			if (lastintervalmeasureinms > 0) {
				actualintervalinms = (millis()-lastintervalmeasureinms);
				break;
			}
			lastintervalmeasureinms = millis();
		}
	}

	return actualintervalinms;
}


/************************ Communication Functions *********************************/


/**********************   multi bit ********************************/

short ADXL362::on2(byte reg, byte onbitmask1, byte resetbitmask1,  byte onbitmask2, byte resetbitmask2) {
	return setReg(reg,
		(getReg(reg) & ~resetbitmask1 & ~resetbitmask2) | onbitmask1 | onbitmask2
	);
}

/* not used
short ADXL362::off(byte reg, byte offbitmask) {
	on(reg, 0x00, offbitmask);
}
*/

//sets all the bits in offbitmask to 0, then turns the onbitmask back on (exclusiveOn)
short ADXL362::on(byte reg, byte onbitmask, byte resetbitmask) {
	return setReg(reg, (getReg(reg) & ~resetbitmask) | onbitmask);
}

/***************************** 1 bit  ****************************/

//sets all the bits in offbitmask to 0, then turns the onbitmask back on (exclusiveOn)
short ADXL362::on(byte reg, byte onbitmask) {
	//check if bitmask just has 1 value
	#ifdef ADXL362_DEBUG
	if (!onbitmask || (onbitmask & (onbitmask - 1))) {
		return -240;
	} else
	#endif
		return on(reg, onbitmask, 0x00);
}

bool ADXL362::isOn(byte reg, byte bitmask) {
	//only the bit that was one in value will stay one

	//check if bitmask just has 1 value
	if (!bitmask || (bitmask & (bitmask - 1))) {
		#ifdef ADXL362_DEBUG
		Serial.println(F("ADXL362: bitmask contains more then 1 bit or 0"));
		#endif
		return false;
	} else
		return (getReg(reg) & bitmask);

	return false;
}

byte ADXL362::getReg(byte reg) {
	byte val;

	getRegBurstBIGENDIAN(reg, &val, 1);

	return val;
}


//the rest is little endian (Lsb->msb)
void ADXL362::getRegBurstBIGENDIAN(byte reg, byte* dst, byte len, byte modeByte)  //dont return status, most callers can't pass it back to its caller
{
	#ifdef ADXL362_DEBUG
	if (reg > SPI_MAX_WRITEABLE_REG) {
	  Serial.println(F("ADXL362: Register exceeds min/max readable registers"));
	}
	#endif
    spiBeginTransaction();

    //select mode
    _spi->transfer(modeByte);

    if (modeByte == ADXL362_READ_REG) //for fifo not register needs to be selected
		// send the device the register you want to read:
		_spi->transfer(reg);

    while (len--)
    	*dst++ = _spi->transfer(0x00);

    spiEndTransaction();
}


uint16_t ADXL362::getReg16(byte reg) {
	uint16_t val;

	getRegBurst16(reg, &val, 1);

	return val;
}

void ADXL362::getRegBurst16(byte reg, uint16_t* dst, byte len16, byte modeByte) //dont return status, most callers can't pass it back to its caller
{
    #ifdef ADXL362_DEBUG
    if (reg > SPI_MAX_WRITEABLE_REG) {
      Serial.println(F("ADXL362: Register exceeds min/max readable registers"));
      return;
    }
    #endif

    spiBeginTransaction();

    //select mode
    _spi->transfer(modeByte);

    if (modeByte == ADXL362_READ_REG) //for fifo not register needs to be selected
		// send the device the register you want to read:
		_spi->transfer(reg);

    uint16_t* dstsave = dst;
    //while have another byte to read:
    while (len16 > 0) { //compare first, then decrement
#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2)
        uint16_t b2 = _spi->transfer(0x00);
        b2 = (b2 << 8) | _spi->transfer(0x00);
#else
        uint16_t b2 = _spi->transfer16(0x00);
#endif
    	*dst++ = b2 << 8 | b2 >> 8;
        len16=len16-1;
    }

    spiEndTransaction();
}


short ADXL362::setReg16(byte reg, uint16_t val) {
	return setReg(reg, val, true);
}

/*
 * Latencies not handled by this function as the only mentionable one is the one from SOFT_RESET
 *
 * Datasheet:
 *
 * Reading any of the data registers (0x08 to 0x0A or 0x0E to 0x15 / X,Y,ZDATA / X,Y,ZDATA_L,H / TEMP_L/H)
 * clears the data ready interrupt (INTMAP1/2 bit 0)
 * There can be as much as an 80 us delay from reading (from reading to clearing the data ready interrupt?)
 *
 * SOFT_RESET: A latency of approximately 0.5 ms is required after soft reset.
 */

short ADXL362::setReg(byte reg, uint16_t val, bool twoBytes) {
    short status = 1;

    if (reg < SPI_MIN_WRITEABLE_REG || reg > SPI_MAX_WRITEABLE_REG)
    	return -200; //not a writeable register (write nor readwrite)

    spiBeginTransaction();

    //select mode
    _spi->transfer(ADXL362_WRITE_REG);

    // send the device the register you want to read:
    _spi->transfer(reg);

	#if ADXL362_POWERSAVE_SKIPWRITE_MSB
    twoBytes = false;
    val = (byte)val;
	#endif
    if (!twoBytes)
    	_spi->transfer((byte)val); //C takes the LS byte from the uint16_t when converting to byte (does not round at max etc.)
    else {
#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2)
    	//swap LSB / MSB (spi is on MSBFIRST, but accelerometer uses LSB first)
    	_spi->transfer((byte) val);
    	_spi->transfer((byte) (val>>8));
#else
    	//swap LSB / MSB (spi is on MSBFIRST, but accelerometer uses LSB first)
    	_spi->transfer16((val>>8) | (val<<8)); //msbfirst (most significant byte)
#endif
    }

    spiEndTransaction();

    //only for debugging
	#ifdef ADXL362_DEBUG
	#ifdef ADXL362_VERIFY_REG_READBACK
    //read it back to verify if set
    if (reg != ADXL362_REG_SOFT_RESET) { //datasheet: soft_reset always 0x00 when read / write-only
    	delay(10); //just in case: wait a little so value is set
    	if (!twoBytes) {
    		uint16_t newval = getReg(reg);
			if (val != newval) {
				/*
				#ifdef ADXL362_DEBUG
				Serial.print("REG 0x");
				Serial.print(reg, HEX);
				Serial.print(" readback failed, value written: 0x");
				Serial.print(val, HEX);
				Serial.print(" value read: 0x");
				Serial.print(newval,HEX);
				Serial.print(" ");
				#endif*/
				return -201;
			}
    	} else {
    		uint16_t newval = getReg16(reg);
    		if (val !=  getReg16(reg)) {
    			/*#ifdef ADXL362_DEBUG
				Serial.print("REG16 0x");
				Serial.print(reg, HEX);
				Serial.print(" readback failed, value written: 0x");
				Serial.print(val, HEX);
				Serial.print(" value read: 0x");
				Serial.print(newval,HEX);
				Serial.print(" ");
				#endif*/
    			return -202;
    		}
		}
    }
	#endif
	#endif

	#ifdef ADXL362_VERIFY_REG_WRITES_HAMMING
	//built in hamming error correcting code detected error
	if (isOn(ADXL362_REG_STATUS,ADXL362_STATUS_ERR_USER_REGS))
		return -203; //SEU Error Detect
	#endif

    return status;
}

void ADXL362::spiBeginTransaction() {
	//Note: Best if all 3 settings are constants
	//Attention: MSBFIRST: to keep compatible with other libraries (its the default) keep this setting to msb first and translate in this function if necessary!
	//This library will not work with LSBFIRST
#if defined(SPI_HAS_TRANSACTION)
    _spi->beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
#endif /* SPI_HAS_TRANSACTION */

    // take the chip select low to select the device:
    digitalWrite(_ss, LOW);
}

void ADXL362::spiEndTransaction() {
    // take the chip select high to de-select:
    digitalWrite(_ss, HIGH);

#if defined(SPI_HAS_TRANSACTION)
    _spi->endTransaction();
#endif /* SPI_HAS_TRANSACTION */
}


#ifdef ADXL362_DEBUG
#ifdef ADXL362_HASHMAPLIBINSTALLED
#include <HashMap.h>
void ADXL362::printRegisters(bool avoidIntteruptAcknowledgement, bool printBinary) {
	#define NR_OF_REGISTERS 34

	HashType<int,char*> regnameraw[NR_OF_REGISTERS];
	HashMap<int,char*> registryNames = HashMap<int,char*>( regnameraw , NR_OF_REGISTERS );

	registryNames[0](0x00,"ADXL362_REG_DEVID_AD          ");
	registryNames[1](0x01,"ADXL362_REG_DEVID_MST         ");
	registryNames[2](0x02,"ADXL362_REG_PARTID            ");
	registryNames[3](0x03,"ADXL362_REG_REVID             ");
	registryNames[4](0x08,"ADXL362_REG_XDATA             ");
	registryNames[5](0x09,"ADXL362_REG_YDATA             ");
	registryNames[6](0x0A,"ADXL362_REG_ZDATA             ");
	registryNames[7](0x0B,"ADXL362_REG_STATUS            ");
	registryNames[8](0x0C,"ADXL362_REG_FIFO_L            ");
	registryNames[9](0x0D,"ADXL362_REG_FIFO_H            ");
	registryNames[10](0x0E,"ADXL362_REG_XDATA_L           ");
	registryNames[11](0x0F,"ADXL362_REG_XDATA_H           ");
	registryNames[12](0x10,"ADXL362_REG_YDATA_L           ");
	registryNames[13](0x11,"ADXL362_REG_YDATA_H           ");
	registryNames[14](0x12,"ADXL362_REG_ZDATA_L           ");
	registryNames[15](0x13,"ADXL362_REG_ZDATA_H           ");
	registryNames[16](0x14,"ADXL362_REG_TEMP_L            ");
	registryNames[17](0x15,"ADXL362_REG_TEMP_H            ");
	registryNames[18](0x1F,"ADXL362_REG_SOFT_RESET        ");
	registryNames[19](0x20,"ADXL362_REG_THRESH_ACT_L      ");
	registryNames[20](0x21,"ADXL362_REG_THRESH_ACT_H      ");
	registryNames[21](0x22,"ADXL362_REG_TIME_ACT          ");
	registryNames[22](0x23,"ADXL362_REG_THRESH_INACT_L    ");
	registryNames[23](0x24,"ADXL362_REG_THRESH_INACT_H    ");
	registryNames[24](0x25,"ADXL362_REG_TIME_INACT_L      ");
	registryNames[25](0x26,"ADXL362_REG_TIME_INACT_H      ");
	registryNames[26](0x27,"ADXL362_REG_ACT_INACT_CTL     ");
	registryNames[27](0x28,"ADXL362_REG_FIFO_CTL          ");
	registryNames[28](0x29,"ADXL362_REG_FIFO_SAMPLES      ");
	registryNames[29](0x2A,"ADXL362_REG_INTMAP1           ");
	registryNames[30](0x2B,"ADXL362_REG_INTMAP2           ");
	registryNames[31](0x2C,"ADXL362_REG_FILTER_CTL        ");
	registryNames[32](0x2D,"ADXL362_REG_POWER_CTL         ");
	registryNames[33](0x2E,"ADXL362_REG_SELF_TEST         ");

	for (int i = 0;i < NR_OF_REGISTERS ;i++) {
		if (!avoidIntteruptAcknowledgement || i != 7) {
			Serial.print(registryNames[i].getHash(),HEX);
			Serial.print(" ");
			Serial.print(registryNames[i].getValue());
			Serial.print(" = ");
			Serial.println(getReg((byte)registryNames[i].getHash()),printBinary ? BIN : HEX);
		}
	}
}
#else
void ADXL362::printRegisters(bool avoidIntteruptAcknowledgement, bool printBinary) {
	Serial.println(F("\r\nto print all registers please install: https://playground.arduino.cc/Code/HashMap/ and uncomment #define ADXL362_HASHMAPLIBINSTALLED\r\n"));
}
#endif
#else
void ADXL362::printRegisters(bool avoidIntteruptAcknowledgement, bool printBinary) {
}
#endif
