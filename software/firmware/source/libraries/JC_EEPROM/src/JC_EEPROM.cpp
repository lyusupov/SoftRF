// Arduino JC_EEPROM Library
// https://github.com/JChristensen/JC_EEPROM
// Copyright (C) 2022 by Jack Christensen and licensed under
// GNU GPL v3.0, https://www.gnu.org/licenses/gpl.html
//
// Arduino library to support external I2C EEPROMs.

#include <JC_EEPROM.h>
#include <Wire.h>

// Constructor.
// - deviceCapacity is the capacity of a single EEPROM device in
//   kilobits (kb) and should be one of the values defined in the
//   eeprom_size_t enumeration in the extEEPROM.h file. (Most
//   EEPROM manufacturers use kbits in their part numbers.)
// - nDevice is the number of EEPROM devices on the I2C bus (all must
//   be identical).
// - pageSize is the EEPROM's page size in bytes.
// - eepromAddr is the EEPROM's I2C address and defaults to 0x50 which is common.
JC_EEPROM::JC_EEPROM(eeprom_size_t deviceCapacity, uint8_t nDevice, uint16_t pageSize, uint8_t eepromAddr)
{
    m_dvcCapacity = deviceCapacity;
    m_nDevice = nDevice;
    m_pageSize = pageSize;
    m_eepromAddr = eepromAddr;
    m_totalCapacity = m_nDevice * m_dvcCapacity * 1024UL / 8;
    m_nAddrBytes = deviceCapacity > kbits_16 ? 2 : 1;   // two address bytes needed for eeproms > 16kbits

    // determine the bitshift needed to isolate the chip select bits from
    // the address to put into the control byte
    uint16_t kb = m_dvcCapacity;
    if ( kb <= kbits_16 ) m_csShift = 8;
    else if ( kb >= kbits_512 ) m_csShift = 16;
    else {
        kb >>= 6;
        m_csShift = 12;
        while ( kb >= 1 ) {
            ++m_csShift;
            kb >>= 1;
        }
    }
}

// initialize the I2C bus and do a dummy write (no data sent)
// to the device so that the caller can determine whether it is responding.
// when using a 400kHz bus speed and there are multiple I2C devices on the
// bus (other than EEPROM), call extEEPROM::begin() after any initialization
// calls for the other devices to ensure the intended I2C clock speed is set.
uint8_t JC_EEPROM::begin(twiClockFreq_t twiFreq)
{
    Wire.begin();
    Wire.setClock(twiFreq);
    Wire.beginTransmission(m_eepromAddr);
    if (m_nAddrBytes == 2) Wire.write(0);   // high addr byte
    Wire.write(0);                          // low addr byte
    return Wire.endTransmission();
}

// Write bytes to external EEPROM.
// If the I/O would extend past the top of the EEPROM address space,
// a status of EEPROM_ADDR_ERR is returned. For I2C errors, the status
// from the Arduino Wire library is passed back through to the caller.
uint8_t JC_EEPROM::write(uint32_t addr, uint8_t* values, uint16_t nBytes)
{
    if (addr + nBytes > m_totalCapacity) {  // will this write go past the top of the EEPROM?
        return EEPROM_ADDR_ERR;             // yes, tell the caller
    }

    uint8_t txStatus {0};   // transmit status
    while (nBytes > 0) {
        uint16_t nPage = m_pageSize - ( addr & (m_pageSize - 1) );
        // find min(nBytes, nPage, BUFFER_LENGTH) -- BUFFER_LENGTH is defined in the Wire library.
        uint16_t nWrite = nBytes < nPage ? nBytes : nPage;
        nWrite = BUFFER_LENGTH - m_nAddrBytes < nWrite ? BUFFER_LENGTH - m_nAddrBytes : nWrite;
        uint8_t ctrlByte = m_eepromAddr | static_cast<uint8_t>(addr >> m_csShift);
        Wire.beginTransmission(ctrlByte);
        if (m_nAddrBytes == 2) Wire.write(static_cast<uint8_t>(addr >> 8)); // high addr byte
        Wire.write(static_cast<uint8_t>(addr));                             // low addr byte
        Wire.write(values, static_cast<uint8_t>(nWrite));
        txStatus = Wire.endTransmission();
        if (txStatus != 0) return txStatus;

        // wait up to 50ms for the write to complete
        for (uint8_t i=100; i; --i) {
            delayMicroseconds(500);                 // no point in waiting too fast
            Wire.beginTransmission(ctrlByte);
            if (m_nAddrBytes == 2) Wire.write(0);   // high addr byte
            Wire.write(0);                          // low addr byte
            txStatus = Wire.endTransmission();
            if (txStatus == 0) break;
        }
        if (txStatus != 0) return txStatus;

        addr += nWrite;         // increment the EEPROM address
        values += nWrite;       // increment the input data pointer
        nBytes -= nWrite;       // decrement the number of bytes left to write
    }
    return txStatus;
}

// Read bytes from external EEPROM.
// If the I/O would extend past the top of the EEPROM address space,
// a status of EEPROM_ADDR_ERR is returned. For I2C errors, the status
// from the Arduino Wire library is passed back through to the caller.
uint8_t JC_EEPROM::read(uint32_t addr, uint8_t* values, uint16_t nBytes)
{
    if (addr + nBytes > m_totalCapacity) {  // will this read take us past the top of the EEPROM?
        return EEPROM_ADDR_ERR;             // yes, tell the caller
    }

    while (nBytes > 0) {
        uint16_t nPage = m_pageSize - ( addr & (m_pageSize - 1) );
        uint16_t nRead = nBytes < nPage ? nBytes : nPage;
        nRead = BUFFER_LENGTH < nRead ? BUFFER_LENGTH : nRead;
        uint8_t ctrlByte = m_eepromAddr | static_cast<uint8_t>(addr >> m_csShift);
        Wire.beginTransmission(ctrlByte);
        if (m_nAddrBytes == 2) Wire.write(static_cast<uint8_t>(addr >> 8)); // high addr byte
        Wire.write(static_cast<uint8_t>(addr));                             // low addr byte
        uint8_t rxStatus = Wire.endTransmission();
        if (rxStatus != 0) return rxStatus;     // read error

        Wire.requestFrom(ctrlByte, static_cast<uint8_t>(nRead));
        for (byte i=0; i<nRead; i++) values[i] = Wire.read();

        addr += nRead;          // increment the EEPROM address
        values += nRead;        // increment the input data pointer
        nBytes -= nRead;        // decrement the number of bytes left to write
    }
    return 0;
}

// Write a single byte to external EEPROM.
// If the I/O would extend past the top of the EEPROM address space,
// a status of EEPROM_ADDR_ERR is returned. For I2C errors, the status
// from the Arduino Wire library is passed back through to the caller.
uint8_t JC_EEPROM::write(uint32_t addr, uint8_t value)
{
    return write(addr, &value, 1);
}

// Read a single byte from external EEPROM.
// If the I/O would extend past the top of the EEPROM address space,
// a status of EEPROM_ADDR_ERR is returned. For I2C errors, the status
// from the Arduino Wire library is passed back through to the caller.
// To distinguish error values from valid data, error values are returned as negative numbers.
int16_t JC_EEPROM::read(uint32_t addr)
{
    uint8_t data;

    int16_t ret = read(addr, &data, 1);
    return ret == 0 ? data : -ret;
}
