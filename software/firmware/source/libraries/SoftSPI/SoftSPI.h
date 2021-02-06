#ifndef _SOFTSPI_H
#define _SOFTSPI_H

# include <Arduino.h>
#include <SPI.h>

class SoftSPI : public SPIClass
{

public:
    SoftSPI(uint8_t mosi, uint8_t miso, uint8_t sck);
    void begin();
    void end();
    void setBitOrder(uint8_t);
    void setDataMode(uint8_t);
    void setClockDivider(uint32_t);
    uint8_t transfer(uint8_t);
    uint16_t transfer16(uint16_t data);
    void transfer(void *buf, size_t count);
    /* compatibility with Adafruit_nRF52_Arduino */
    void transfer(const void *tx_buf, void *rx_buf, size_t count);
    // Transaction Functions
    void beginTransaction(SPISettings settings);
    void endTransaction(void);
private:
    uint8_t _miso;
    uint8_t _mosi;
    uint8_t _sck;
};

extern SoftSPI SPI2;
#endif
