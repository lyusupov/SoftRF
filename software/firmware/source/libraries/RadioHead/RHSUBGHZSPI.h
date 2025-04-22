// RHSUBGHZSPI.h
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2023 Mike McCauley

#ifndef RHSUBGHZSPI_h
#define RHSUBGHZSPI_h

#include <RHGenericSPI.h>

// Are we building for a suitable STM processor
#if defined(SUBGHZSPI_BASE)

/////////////////////////////////////////////////////////////////////
/// \class RHSUBGHZSPI RHSUBGHZSPI.h <RHSUBGHZSPI.h>
/// \brief Base class for SPI interfacesInterface for SUBGHZSPIClass
/// as used on eg Wio-E5 mini and other boards that use the Seeed LoRa-E5-HF, LoRa-E5-LF modules
/// which use the  STM32WLE5JC family processors.
///
/// SubGhz is the SPI interface used on the STM32WLE5JC to communicate with the internal SX1261 radio,
/// and it has some special methods needed to talk to the chip.
///
class RHSUBGHZSPI : public RHGenericSPI
{
public:
    uint8_t transfer(uint8_t data) { return SubGhz.SPI.transfer(data); };

    /// Initialise the software SPI library
    /// Call this after configuring the SPI interface and before using it to transfer data.
    /// Initializes the SPI bus by setting SCK, MOSI, and SS to outputs, pulling SCK and MOSI low, and SS high. 
    void begin() { SubGhz.SPI.begin(); };

    /// Disables the SPI bus usually, in this case
    /// there is no hardware controller to disable.
    void end() { SubGhz.SPI.end(); };

    void beginTransaction() {
	SubGhz.SPI.beginTransaction(SubGhz.spi_settings);
	SubGhz.setNssActive(true);
	while (SubGhz.isBusy()) /* wait */;
    };
    void endTransaction() {
	SubGhz.setNssActive(false);
	SubGhz.SPI.endTransaction();
    };
};

#endif
#endif
