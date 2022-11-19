Winkel ICT ADXL362
==================

COMPLETE and fully tested implementation of the ADXL362 datasheet.

Library for ADXL362 accelerometer: unique because of its:
- ultralow (lowest of +/- 62 reviewed accelerometer datasheets) power (0,270uA) usage
- autonomous motion switch functionality. 

Together allowing for extremely long battery life.

Thoroughly tested low memory footprint library, complete implementation of datasheet / functionality.
Focus on ease of use and easy debugging (every error has a unique negative status code returned by most functions)

Start measuring as easy as:

	acceleroMeter.init();
	acceleroMeter.activateMeasure();
	MeasurementInMg xyz = acceleroMeter.getXYZ(ad_range_2G);
	
RAM usage: 1 byte, flash usage: 1162 bytes (measured on Pro Mini 3.3V) / debug mode off
	
Or use it as an autonomous motion switch with zero configuration (settings of example page 36 of datasheet):

	acceleroMeter.init();
	acceleroMeter.activateAutonomousMotionSwitch();

RAM usage: 5 bytes, flash usage: 2714 bytes (measured on Pro Mini 3.3V) / debug mode off

UPDATE 29/Jan/22: all FIFO modes fully tested and working, see v1.2.0
	 
## Hardware
- ADXL362 datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/adxl362.pdf

- SparkFun Triple Axis Accelerometer Breakout - ADXL362 (SEN-11446) https://www.sparkfun.com/products/11446
- SparkFun Wake on Shake (SEN-11447) - https://www.sparkfun.com/products/11447
- Tested with Pro Mini 3.3V 8Mhz and Seeeduino ARM Cortex-M0+

The ADXL362 uses the least power on lower voltages and can handle a maximum of 3.6V.
The sparkfun breakout board does not have a voltage regulator so **be careful** using lithium-ion / LiPo batteries.

Possible options
- Use a +-2.0V power source
- Use a LiFePO4 power source which stays under 3.6V fully charged (no regulation needed) (charger: https://www.ebay.com/itm/192121823376 (untested))
- Use an ultra low power voltage regulator, examples:

ST offers some ultra low power regulators: https://www.st.com/en/power-management/low-iq-ldo-regulators.html
- Lowest power: STLQ020 (0,3uA quiescent) , SOT323-5L, not available in SOT23-5L afaik
- SOT23-5L: STLQ015 (1.0uA quiescent), SOT23-5L (pick one with a dropout voltage below 3.0 ?)

My (test) setup:
- Sparkfun breakout (SEN-11446)
- LiPo battery
- STLQ020C22R (2.2volt), capacitors: Vin: 1uF ceramic / Vout: 1uf ceramic + 10uF tantalum 

## Guide
* [ Quick start. ](#quickstart)
* [ Debugging settings. ](#debug)
* [ Measuring XYZ/Temp. ](#measurexyz)
* [ Motion Switch. ](#motionswitch)
* [ Freefall detection. ](#freefall)
* [ Custom detection. ](#custom)
* [ Advanced detection. ](#advanced)
* [ FIFO: oldestsaved and streaming mode (normal measuring). ](#fifo)
* [ FIFO: triggered mode (use with movement detection). ](#fifotriggered)
* [ Self-test. ](#selftest)
* [ Misc settings. ](#misc)
* [ Goals / Philosophy. ](#goals)
* [ TODO. ](#todo)

<a name="quickstart"></a>
## Quick start
Important deviations from datasheet terminology:
- link/loop is called sequential (see configureSequential)
- wakeup mode (0,270 uA) is implemented as just a bandwidth / odr setting as nearly almost functionality is still available (except for minimum activity time)


	ad_bandwidth_hz_6_wakeup_ultralowpower

For most use-cases the 4 main functions cover all functionality:
- activateMeasure
- activateAutonomousMotionSwitch
- activateFreeFallDetection
- activateCustomDetection

In combination with:

For measuring (activateMeaure):
- getXYZ
- getTemperature

Or for working with interrupts:
- isActInterrupt
- isInactInterrupt
- isAwake

<a name="debug"></a>
### Debugging settings (ADXL362.h)
Uncomment to disable debugging, default=debug enabled

	#define ADXL362_DEBUG

In debug mode extra checks are done for parameter combinations, in rare cases (when no status code can be returned) a serial monitor print.

Also the printregisters function is disabled, see:

To use the printregisters function for debugging (prints all register values by hex address + name), will need to install https://playground.arduino.cc/Code/HashMap/

And uncomment:

	//#define ADXL362_HASHMAPLIBINSTALLED

This setting will cause all written registry values to be read back and checked if correctly written. Will slow down performance / energy usage so might not be suitable for production.

	#define ADXL362_VERIFY_REG_READBACK

<a name="measurexyz"></a>
### Measuring XYZ/Temp
See example ino file: **BasicMeassure**

As pointed out in the introduction:

	acceleroMeter.init();
	acceleroMeter.activateMeasure();
	MeasurementInMg xyz = acceleroMeter.getXYZ(ad_range_2G);

Recommended implementation would be to use a function to check the return status of each function call, like:

	void check(short code) {
	  if (code <= 0) {
	    if (code == -110)
	      //do something,print when debugging?: device not connected / bad power supply?
	    else if (code >= -104 && code <= -102)
	      //do somethin,print when debugging?: bad power supply?
	    else
	      //lookup the negative status code in ADXL362.cpp to find the root cause
	  }
	}
	
**Do not continue when a function returns a negative or 0 status**, there is something wrong with your setup or code: fix that first.

Using a function of this type the recommended way to setup measurement is:

	check(acceleroMeter.init());
	check(acceleroMeter.activateMeasure());
	
	MeasurementInMg xyz = acceleroMeter.getXYZ(ad_range_2G);

This way you do not end up in a debugging nightmare.

The cause of the error can be identified by looking up the negative status code in the source code (ADXL362.cpp)

<a name="motionswitch"></a>
### Motion Switch
See example ino file: **AutonomousMotionSwitch**

The easiest way to set up an autonomous motion switch is to call activateAutonomousMotionSwitch without parameters.
This way it configures the ADXL362 like on example page 36 of the datasheet (threshold values might be slightly different, see Misc settings):

	acceleroMeter.init();
	acceleroMeter.activateAutonomousMotionSwitch();
	
*Optionally execute executeTimeCalibration() after init, please see [ Misc settings. ](#misctiming) for calibration instructions*

Again, check for returned error codes! (see Measuring XYZ/Temp).
Interrupts will occur sequential: Activity->Inactivity interrupts will always occur in this order. Awake=0 (falling) will always follow awake=1 (rising) interrupt.

The library functions are set up in such a way that the activate/configure functions have as many defaults as possible.
The order of the function arguments are in order of likely to be most used to least used.
For example, activateAutonomousMotionSwitch has the following parameters:

	short activateAutonomousMotionSwitch(uint16_t minForceInMg = 250, uint16_t maxFroceInMg = 150
	  									, uint32_t inActminTimeInMs = SEC_TO_MS(5), bool linkMode=false, bool autoSleep = false, bandwidth bandwidthInHz = ad_bandwidth_hz_6_wakeup_ultralowpower);

Most likely to be changed are the activity / inactivity settings, like so:

	acc.activateAutonomousMotionSwitch(150,250,2000);
	
Which means: activate (awake=1) when a force detected of at least 150mg and deactivate when no force more than 250mg detected for at least 2.0 seconds.
Attention: when using a bandwidth other than ad_bandwidth_hz_6_wakeup_ultralowpower also a minimum time to activate can be set, for this see: custom detection.

The following 2 settings are linkMode and autoSleep: acc.activateAutonomousMotionSwitch(150,250,2000,linkmode,autosleep)
- linkMode: true: the adxl362 functions as an autonomous motion switch but awake 0/1 has to be acknowledged by calling:
 
 
	acceleroMeter.isAwake();
	
- linkMode: false: functioning in loop mode, the accelerometer will automatically lower the interrupt pin when inactivity is detected for a certain amount of time, regardless if the Arduino did anything.
- autoSleep: when inactive (looking for activity) the chip will be in wakeup mode: ad_bandwidth_hz_6_wakeup_ultralowpower, when active it will be in the bandwidth set in the next argument: bandwidth.
  An error will occur when autosleep = true and bandwidth = ad_bandwidth_hz_6_wakeup_ultralowpower, as this is the active bandwidth it will equal wake up mode instead of autosleep.

The bandwidth setting is the last one, this can be used together with autoSleep to set the bandwidth to the accuracy you need when the ADXL362 is active (awake = 1).

<a name="freefall"></a>
### Freefall detection
See example ino file: **FreeFall**

Can be called without arguments, settings will be equal to Datasheet page 37 (except for small differences in threshold, see Misc settings).

	acceleroMeter.init();
	acceleroMeter.activateFreeFallDetection();

*Optionally execute executeTimeCalibration() after init, please see [ Misc settings. ](#misctiming) for calibration instructions*

Just like the autonomous motion switch more customizations are possible:

	short activateFreeFallDetection(uint16_t maxForceInMg = 600, uint32_t minTimeInMs = 30, bandwidth bandwidthInHz = ad_bandwidth_hz_50_lowpower);

For example, freefall detection in wake up mode:

	acceleroMeter.activateFreeFallDetection(600,30,ad_bandwidth_hz_6_wakeup_ultralowpower);
	
<a name="custom"></a>
### Custom detection
See example ino file: **AdvancedDetection**

	short activateCustomDetection(bandwidth bandwidthInHz, sequentialMode smode, bool autoSleep
	    					, uint16_t minForceInMg, uint32_t actminTimeInMs
							, uint16_t maxForceInMg, uint32_t inActminTimeInMs
							, status int1Status = ad_status_active, status int2Status = ad_status_inactive, bool activeLow = false, bool absoluteMode = false
							, measurementRange measurementRangeInG = ad_range_2G, noiseMode noiseMode = ad_noise_normal);

Minimum arguments:

	acceleroMeter.activateCustomDetection(ad_bandwidth_hz_25, ad_seq_none, false, 100, 0, 150, 4000);
	
Interrupts can be configured manually here:
- int1Status and int2Status can be manually selected, see status enum in ADXL362.h
- both interrupts can be active low or high
- both interrupts can be in absolute or referenced mode
- measurement range can be selected
- noise mode can be selected

Some parameter combinations might result in a negative status code as the chip cannot execute them, so please check for the resulting status code. 

This function should cover most use-cases, when more specific interrupt configuration is needed, see the next chapter:

<a name="advanced"></a>
### Advanced detection
Use the library this way only if:
- You need different active low/high configuration for each interrupt
- You need a different absolute/referenced configuration for each interrupt
- You need to use an external clock or sample trigger using the interrupt pins as input (UNTESTED!)
- You will be using FIFO functionality (UNTESTED)

Use the implementation of the activateCustomDetection in ADXL362.cpp as a guide so you are able to adjust:
- [EXTERNALODRINHZ] (see datasheet for how to calculate)
- [ABSOLUTE?]
- [ACTIVELOW?]
- [EXTERNALCLOCK?]
- [EXTERNALSAMPLETRIGGER?]
- set all other variables

When using the [EXTERNAL..] parameters pass NULL for status. 

	sequentialMode smode = seq_none;
	short status=1;
	ADXL362Config config;

	if (smode != ad_seq_none)
		config =  configureSequentialMode((smode == ad_seq_link), bandwidthInHz, autoSleep, measurementRangeInG, noiseMode, [EXTERNALODRINHZ]);
	else
		config =  configure(bandwidthInHz, measurementRangeInG, noiseMode, [EXTERNALODRINHZ]);

	if (config.status <= 0) return config.status;

	status = configureActivity(config, minForceInMg, actminTimeInMs, [ABSOLUTE?]);
	if (status <= 0) return status;

	status = configureInActivity(config, maxForceInMg, inActminTimeInMs, [ABSOLUTE?]);
	if (status <= 0) return status;

	status = configureInterrupt1(config, int1Status, [ACTIVELOW?],[EXTERNALCLOCK?]);
	if (status <= 0) return status;

	status = configureInterrupt2(config, int2Status, [ACTIVELOW?],[EXTERNALSAMPLETRIGGER?]);
	if (status <= 0) return status;

	status = activateMode(config);

activateMode always has to be called to activate your configuration (uses POWER_CTL register).

<a name="fifo"></a>
### FIFO: oldestsaved and streaming mode (normal measuring)
See example ino file: **FIFO**

The FIFO functionality can be used in combination with all other active[] and configure[] functions.
FIFO has its own functions to configure interrupts, do not configure an interrupt twice using the other interrupt functions.

Configure the FIFO as follows:


	accelerometer.configureFIFO(ad_fifo_oldestsaved,FIFO_SIZE, FIFO_TEMP_ENABLED);

ad_fifo_oldestsaved here means that no new data will be written to the FIFO when it's full, a full FIFO will trigger a FIFO overrun status. Configuration for this has been included in the example ino file:


	accelerometer.configureFIFOInterrupt2(ad_status_fifo_overrun);
	
Also configure the watermark interrupt (see FIFO_SIZE):


	accelerometer.configureFIFOInterrupt1(ad_status_fifo_watermark);
	
ad_fifo_stream can also be used instead of ad_fifo_oldest saved, no overrun interrupt will occur as data in FIFO is always replaced with newest data.

The simplest way to use the FIFO buffer is in normal measurement mode, call this after the previous FIFO configuration to start measuring using FIFO:


	accelerometer.activateMeasure();

It is good to use macro definitions as in the example above:
- FIFO_SIZE: maximum size before a watermark interrupt is occured, this can in turn also be used to define a buffer with the right size to read the FIFO data into.
- FIFO_TEMP_ENABLED: when reading the FIFO data these functions need to know if temperature has been stored:

The FIFO buffer can be read like this:


    uint16_t fifoEntries[FIFO_SIZE];
    uint16_t nrOfEntries = accelerometer.readFIFO(fifoEntries,FIFO_SIZE);
    uint16_t* entriesptr = fifoEntries;
    uint16_t buflen = nrOfEntries;
    while (buflen > 0) {
      FifoMeasurement m = accelerometer.parseFIFOMeasurement(ad_range_2G, &entriesptr, &buflen, FIFO_TEMP_ENABLED);
      MeasurementInMg mxyz = m.forceInMg;
      float temp = m.tempInC;
      //do something with the measurement
    }

The entriesptr and buflen sizes will be changed by the parse function each iteration.

<a name="fifotriggered"></a>
### FIFO: triggered mode (use with movement detection)
See example ino file: **AutonomousMotionSwitchFIFO**

Similarly as the normal/measurement FIFO configuration, the FIFO buffer can also be used to capture data surrounding, for example, an awake interrupt:
Before using one of the activate[] functions first configure the FIFO in triggered mode:

	acc.configureFIFO(ad_fifo_trigger,FIFO_SIZE, FIFO_TEMP_ENABLED);

When an interrupt occured the FIFO will contain a maximum of FIFO_SIZE of measurements surrounding the event.

*See previous chapter for explanation of macro definitions and how to read the FIFO buffer*

<a name="selftest"></a>
### Self-test

Execute SelfTest.ino in the example files.

The self-test will average both before self-test and self-test x,y,z values over 16 samples and return the **difference** in x,y,z.

See datasheet page 41 / Table 22 to interpret results: https://www.analog.com/media/en/technical-documentation/data-sheets/adxl362.pdf

My test results per voltage (in g's):
- 2.0V, x : 1.21 y : -1.26 z : 1.08
- 2.2V, x : 1.30 y : -1.46 z : 1.29
- 3.3V, x : 1.61 y : -1.74 z : 1.49

<a name="misc"></a>
### Misc settings (ADXL362.h)
Enable build in hamming error detection

	//#define ADXL362_VERIFY_REG_WRITES_HAMMING

Skip writing the most significant byte of all MSB registers (_H registers) (UNTESTED)

	//ADXL362_POWERSAVE_SKIPWRITE_MSB

At request can make an SPI stub available which will simulate the accelerometer (not included).

	//#define ADXL362_USE_SPI_STUB

My measurements identified some corrections to the wakeup mode ODR and the time thresholds so that act/inact times passed to the functions will be accurate.
The library default's to the following settings which might or might not be accurate:

	#define ADXL362_WAKEUPMODE_ACTUALODR 6.0f
	
	#define ADXL362_TIMECORRECTION_INPERCENT 0.0f

<a name="misctiming"></a>
The timing can be adjusted run-time by executing by calling this function right after init():


	executeTimeCalibration(false);

Measure and print chip specific #define values and copy paste them into the .h/header file for compile time adjustment using:


	executeTimeCalibration(true);

Measurements for 3 different chips in this order:

 
	- Chip revision 	3 / 2 / 2
	- ODR: 			5 / 6.3 / 7.16
	- time perc:  		-14 / -5 /-13

<a name="goals"></a>
## Goals / Philosophy
- Functions / parameters should not allow options/combinations that the accelerometer cannot execute
- As many default values for parameters as possible, in order from most to least used (estimated) allowing for quick implementation for most use cases while still enabling all functionality.
- Intuitive usage of all code, functions, parameters
  - when not in conflict with first rule, use datasheet terminology
- All functions return a unique negative status (if possible) value when an error occurred, and exit the function immediately after that
  - this makes it easy to look for which problem occurred and fix it
- Development: start with debug mode 'on', allowing for extra checks of correct parameter combinations.
- as little RAM usage as possible (+-5 bytes)
- as little flash usage as possible (1162 measuring only, 2714 bytes for autonomous motion switch)

<a name="todo"></a>
## TODO's
- test external clock / sample trigger parameters of activity and inactivity functions
- test setting ADXL362_POWERSAVE_SKIPWRITE_MSB