# Arduino EEPROM Library
http://github.com/JChristensen/JC_EEPROM  
README file  


## License
Arduino EEPROM Library Copyright (C) 2022-2025 Jack Christensen GNU GPL v3.0

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License v3.0 as published by the Free Software Foundation.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/gpl.html>

## Introduction
**Arduino EEPROM Library**

This library will work with most I2C serial EEPROM chips between 2k bits and 2048k bits (2M bits) in size. Multiple EEPROMs on the bus are supported as a single address space. I/O across block, page and device boundaries is supported. Certain assumptions are made regarding the EEPROM device addressing. These assumptions should be true for most EEPROMs but there are exceptions, so read the datasheet and know your hardware. 

The library should also work for EEPROMs smaller than 2k bits, assuming that there is only one EEPROM on the bus and also that the user is careful to not exceed the maximum address for the EEPROM. 

The **JC_EEPROM Library** has been tested with:
- Microchip 24AA02E48 (2k bit)
- 24xx32 (32k bit, thanks to Richard M)
- Microchip 24LC256 (256k bit)
- Microchip 24FC1026 (1M bit, thanks to Gabriele B on the Arduino forum)
- ST Micro M24M02 (2M bit)
- Atmel AT24C256C (thanks to searobin)
- Microchip 24C16B, 24C32 (thanks to Diogko)
- Microchip AT24C02 and ST Micro M24C04-W with ESP-12F/ESP8266 (thanks to Thorsten)

The **JC_EEPROM Library** will **NOT** work with Microchip 24xx1025 as its control byte does not conform to the following assumptions.

**Device addressing assumptions:**
- The I2C address sequence consists of a control byte followed by one address byte (for EEPROMs <= 16k bits) or two address bytes (for EEPROMs > 16k bits).
- The three least-significant bits in the control byte (excluding the R/W bit) comprise the three most-significant bits for the entire address space, i.e. all chips on the bus. As such, these may be chip-select bits or block-select bits (for individual chips that have an internal block organization), or a combination of both (in which case the block-select bits must be of lesser significance than the chip-select bits).
- Regardless of the number of bits needed to address the entire address space, the three most-significant bits always go in the control byte. Depending on EEPROM device size, this may result in one or more of the most significant bits in the I2C address bytes being unused (or "don't care" bits).
- An EEPROM contains an integral number of pages.

Note that the Arduino Wire library has a buffer size of 32 bytes. This limits the size of physical I/Os that can be done to EEPROM. For writes, one or two bytes are used for the address, so writing is therefore limited to 31 or 30 bytes. Because the **JC_EEPROM Library** will handle I/O across block, page and device boundaries, the only consequence this has for the user is one of efficiency; arbitrarily large blocks of data can be written and read; however, carefully chosen block sizes may reduce the number of physical I/Os needed.

## Installation
The library can be installed using the Arduino Library Manager. To install manually:
- Go to http://github.com/JChristensen/JC_EEPROM, click **Code > Download ZIP** and save the ZIP file to a convenient location on your PC.
- Uncompress the downloaded file.  This will result in a folder containing all the files for the library, that has a name that includes the branch name, usually **JC_EEPROM-master**.
- Rename the folder to just **JC_EEPROM**.
- Copy the renamed folder to the Arduino sketchbook/libraries folder.

## Examples
The following example sketch is included with the **JC_EEPROM Library**:
- **eepromTest:** Writes 32-bit integers to the entire EEPROM address space, starting at address 0 and continuing to the topmost address. These are then read back in and verified; any discrepancies are reported to the serial monitor.
- **struct:** Demonstrates writing a `struct` to EEPROM and reading it back.

## Enumerations

### eeprom_size_t
##### Description
EEPROM device size in k-bits. Many manufacturers' EEPROM part numbers are designated in k-bits.
##### Values
- JC_EEPROM::kbits_2
- JC_EEPROM::kbits_4
- JC_EEPROM::kbits_8
- JC_EEPROM::kbits_16
- JC_EEPROM::kbits_32
- JC_EEPROM::kbits_64
- JC_EEPROM::kbits_128
- JC_EEPROM::kbits_256
- JC_EEPROM::kbits_512
- JC_EEPROM::kbits_1024
- JC_EEPROM::kbits_2048

### twiClockFreq_t
##### Description
I2C bus speed.
##### Values
- JC_EEPROM::twiClock100kHz
- JC_EEPROM::twiClock400kHz

## Constructor

### JC_EEPROM(eeprom_size_t devCap, uint8_t nDev, uint16_t pgSize, uint8_t busAddr)
##### Description
Instantiates an external EEPROM object.
##### Syntax
`JC_EEPROM myEEPROM(eeprom_size_t devCap, uint8_t nDev, uint16_t pgSize, uint8_t busAddr));`
##### Parameters
**devCap** *(eeprom_size_t)*: The size of one EEPROM device in k-bits. Choose a value from the eeprom_size_t enumeration above.  
**nDev** *(uint8_t)*: The number of EEPROM devices on the bus. Note that if there are multiple EEPROM devices on the bus, they must be identical and each must have its address pins strapped properly.  
**pgSize** *(uint16_t)*: The EEPROM page size in bytes. Consult the datasheet if you are unsure of the page size.  
**busAddr** *(uint8_t)*: The base I2C bus address for the EEPROM(s). 0x50 is a common value and this parameter can be omitted, in which case 0x50 will be used as the default.  
##### Example
```c++
JC_EEPROM myEEPROM(JC_EEPROM::kbits_256, 2, 64);			//two 24LC256 EEPROMS on the bus
JC_EEPROM oddEEPROM(JC_EEPROM::kbits_8, 1, 16, 0x42);		//an EEPROM with a non-standard I2C address
```

## Methods
### begin(twiClockFreq_t freq)
##### Description
Initializes the library. Call this method once in the setup code. begin() does a dummy I/O so that the user may interrogate the return status to ensure the EEPROM is operational.
##### Syntax
`myEEPROM.begin(twiClockFreq_t freq);`
##### Parameters
**freq** *(twiClockFreq_t)*: The desired I2C bus speed, `JC_EEPROM::twiClock100kHz` or `JC_EEPROM::twiClock400kHz`. Can be omitted in which case it will default to `twiClock100kHz`. **NOTE:** When using 400kHz, if there are other devices on the bus they must all support a 400kHz bus speed. **Secondly**, the other devices should be initialized first, as other libraries may not support adjusting the bus speed. To ensure the desired speed is set, call the JC_EEPROM.begin() function *after* initializing all other I2C devices.
##### Returns
I2C I/O status, zero if successful *(uint8_t)*. See the [Arduino Wire.endTransmission() function](https://www.arduino.cc/reference/en/language/functions/communication/wire/endtransmission/) for a description of other return codes.
##### Example
```c++
JC_EEPROM myEEPROM(JC_EEPROM::kbits_256, 2, 64);
uint8_t i2cStat = myEEPROM.begin(JC_EEPROM::twiClock400kHz);
if ( i2cStat != 0 ) {
	//there was a problem
}
```
### write(uint32_t addr, uint8_t* values, uint16_t nBytes)
##### Description
Write one or more bytes to external EEPROM.
##### Syntax
`myEEPROM.write(uint32_t addr, uint8_t* values, uint16_t nBytes);`
##### Parameters
**addr** *(uint32_t)*: The beginning EEPROM location to write.  
**values** _(uint8_t*)_: Pointer to an array containing the data to write.  
**nBytes** *(uint16_t)*: The number of bytes to write.  
##### Returns
I2C I/O status, zero if successful *(uint8_t)*. See the [Arduino Wire.endTransmission() function](https://www.arduino.cc/reference/en/language/functions/communication/wire/endtransmission/) for a description of other return codes. Returns a status of EEPROM_ADDR_ERR if the I/O would extend past the top of the EEPROM address space.
##### Example
```c++
uint8_t myData[10];
//write 10 bytes starting at location 42
uint8_t i2cStat = myEEPROM.write(42, myData, 10);
if ( i2cStat != 0 ) {
	//there was a problem
	if ( i2cStat == EEPROM_ADDR_ERR) {
		//bad address
	}
	else {
		//some other I2C error
	}
}
```
### write(uint32_t addr, uint8_t value)
##### Description
Writes a single byte to external EEPROM.
##### Syntax
`myEEPROM.write(uint32_t addr, uint8_t value);`
##### Parameters
**addr** *(uint32_t)*: The EEPROM location to write.  
**values** _(uint8_t)_: The value to write.  
##### Returns
Same as multiple-byte write() above.
##### Example
```c++
//write the value 16 to EEPROM location 314.
uint8_t i2cStat = myEEPROM.write(314, 16);
```
### read(uint32_t addr, uint8_t* values, uint16_t nBytes)
##### Description
Reads one or more bytes from external EEPROM into an array supplied by the caller.
##### Syntax
`myEEPROM.read(uint32_t addr, uint8_t* values, uint16_t nBytes);`
##### Parameters
**addr** *(uint32_t)*: The beginning EEPROM location to read from.  
**values** _(uint8_t*)_: Pointer to an array to receive the data.  
**nBytes** *(uint16_t)*: The number of bytes to read.  
##### Returns
I2C I/O status, zero if successful *(uint8_t)*. See the [Arduino Wire.endTransmission() function](https://www.arduino.cc/reference/en/language/functions/communication/wire/endtransmission/) for a description of other return codes. Returns a status of EEPROM_ADDR_ERR if the I/O would extend past the top of the EEPROM address space.
##### Example
```c++
uint8_t myData[10];
//read 10 bytes starting at location 42
uint8_t i2cStat = myEEPROM.read(42, myData, 10);
if ( i2cStat != 0 ) {
	//there was a problem
	if ( i2cStat == EEPROM_ADDR_ERR) {
		//bad address
	}
	else {
		//some other I2C error
	}
}
```
### read(uint32_t addr)
##### Description
Reads a single byte from external EEPROM.
##### Syntax
`myEEPROM.read(uint32_t addr);`
##### Parameters
**addr** *(uint32_t)*: The EEPROM location to read from.
##### Returns
The data read from EEPROM or an error code *(int16_t)*. To distinguish error values from valid data, error values are returned as negative numbers. See the [Arduino Wire.endTransmission() function](https://www.arduino.cc/reference/en/language/functions/communication/wire/endtransmission/) for a description of return codes. Returns a status of EEPROM_ADDR_ERR if the I/O would extend past the top of the EEPROM address space.

##### Example
```c++
//read a byte from location 42
int16_t readValue = myEEPROM.read(42);
if ( readValue < 0 ) {
	//there was a problem
	if ( -readValue == EEPROM_ADDR_ERR) {
		//bad address
	}
	else {
		//some other I2C error
	}
}
else {
	//data read ok
}
```

### update(uint32_t addr, uint8_t value)
##### Description
Updates a single byte in external EEPROM. Like `write(addr, value)` except first reads the location from EEPROM and only writes `value` if it differs from the current value stored at the given location, to reduce wear on the EEPROM.
##### Syntax
`myEEPROM.update(uint32_t addr, uint8_t value);`
##### Parameters
**addr** *(uint32_t)*: The EEPROM location to update.  
**values** _(uint8_t)_: The value to write.  
##### Returns
Same as multiple-byte write() above.
##### Example
```c++
// update the value in EEPROM location 314 to 16.
uint8_t i2cStat = myEEPROM.update(314, 16);
```