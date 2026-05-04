// Arduino JC_EEPROM Library
// https://github.com/JChristensen/JC_EEPROM
// Copyright (C) 2022 by Jack Christensen and licensed under
// GNU GPL v3.0, https://www.gnu.org/licenses/gpl.html
//
// Arduino library to support external I2C EEPROMs.
//
// This library will work with most I2C serial EEPROM chips between 2k bits
// and 2048k bits (2M bits) in size. Multiple EEPROMs on the bus are supported
// as a single address space. I/O across block, page and device boundaries
// is supported. Certain assumptions are made regarding the EEPROM
// device addressing. These assumptions should be true for most EEPROMs
// but there are exceptions, so read the datasheet and know your hardware.
//
// The library should also work for EEPROMs smaller than 2k bits, assuming
// that there is only one EEPROM on the bus and also that the user is careful
// to not exceed the maximum address for the EEPROM.
//
// Library tested with:
//      Microchip 24AA02E48 (2k bit)
//      24xx32 (32k bit, thanks to Richard M)
//      Microchip 24LC256 (256k bit)
//      Microchip 24FC1026 (1M bit, thanks to Gabriele B on the Arduino forum)
//      ST Micro M24M02 (2M bit)
//
// Library will NOT work with Microchip 24xx1025 as its control byte does not
// conform to the following assumptions.
//
// Device addressing assumptions:
//  1. The I2C address sequence consists of a control byte followed by one
//     address byte (for EEPROMs <= 16k bits) or two address bytes (for
//     EEPROMs > 16k bits).
//  2. The three least-significant bits in the control byte (excluding the R/W
//     bit) comprise the three most-significant bits for the entire address
//     space, i.e. all chips on the bus. As such, these may be chip-select
//     bits or block-select bits (for individual chips that have an internal
//     block organization), or a combination of both (in which case the
//     block-select bits must be of lesser significance than the chip-select
//     bits).
//  3. Regardless of the number of bits needed to address the entire address
//     space, the three most-significant bits always go in the control byte.
//     Depending on EEPROM device size, this may result in one or more of the
//     most significant bits in the I2C address bytes being unused (or "don't
//     care").
//  4. An EEPROM contains an integral number of pages.
//
// To use the extEEPROM library, the Arduino Wire library must also
// be included.
//
// Jack Christensen 23Mar2013 v1
// 29Mar2013 v2 - Updated to span page boundaries (and therefore also
//      device boundaries, assuming an integral number of pages per device)
// 08Jul2014 v3 - Generalized for 2kb - 2Mb EEPROMs.
// 11Jan2022 v1.0.0 - Rename to JC_EEPROM, conform to Arduino library standards.

#ifndef JC_EEPROM_H_INCLUDED
#define JC_EEPROM_H_INCLUDED

#include <Arduino.h>
#include <Wire.h>

// a horrible and limiting kludge for architectures that
// do not define BUFFER_LENGTH
#ifndef BUFFER_LENGTH
#define BUFFER_LENGTH 32
#endif

class JC_EEPROM
{
    public:
        // EEPROM size in kilobits.
        // EEPROM part numbers are usually designated in k-bits.
        enum eeprom_size_t {
            kbits_2 = 2,
            kbits_4 = 4,
            kbits_8 = 8,
            kbits_16 = 16,
            kbits_32 = 32,
            kbits_64 = 64,
            kbits_128 = 128,
            kbits_256 = 256,
            kbits_512 = 512,
            kbits_1024 = 1024,
            kbits_2048 = 2048
        };

        // I2C clock frequencies
        enum twiClockFreq_t
            { twiClock100kHz = 100000, twiClock400kHz = 400000 };

        // EEPROM addressing error, returned by write() or read() if
        // upper address bound is exceeded
        static const uint8_t EEPROM_ADDR_ERR {9};

        JC_EEPROM(eeprom_size_t deviceCapacity, uint8_t nDevice,
            uint16_t pageSize, uint8_t eepromAddr = 0x50);
        uint8_t begin(twiClockFreq_t twiFreq = twiClock100kHz);
        uint8_t write(uint32_t addr, uint8_t* values, uint16_t nBytes);
        uint8_t write(uint32_t addr, uint8_t value);
        uint8_t read(uint32_t addr, uint8_t* values, uint16_t nBytes);
        int16_t read(uint32_t addr);
        uint8_t update(uint32_t addr, uint8_t value)
            {return (read(addr) == value) ? 0 : write(addr, &value, 1); }

    private:
        uint8_t  m_eepromAddr;          // eeprom i2c address
        uint16_t m_dvcCapacity;         // capacity of one EEPROM device, in kbits
        uint8_t  m_nDevice;             // number of devices on the bus
        uint16_t m_pageSize;            // page size in bytes
        uint8_t  m_csShift;             // number of bits to shift address for chip select bits in control byte
        uint16_t m_nAddrBytes;          // number of address bytes (1 or 2)
        uint32_t m_totalCapacity;       // capacity of all EEPROM devices on the bus, in bytes
};

#endif
