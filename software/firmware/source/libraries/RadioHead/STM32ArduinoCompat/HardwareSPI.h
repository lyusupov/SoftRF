// ArduinoCompat/HardwareSPI.h
// STM32 implementattion of Arduino compatible SPI class

#ifndef _HardwareSPI_h
#define _HardwareSPI_h

#include <stdint.h>

typedef enum SPIFrequency {
    SPI_21_0MHZ      = 0, /**< 21 MHz */
    SPI_10_5MHZ      = 1, /**< 10.5 MHz */
    SPI_5_25MHZ      = 2, /**< 5.25 MHz */
    SPI_2_625MHZ     = 3, /**< 2.625 MHz */
    SPI_1_3125MHZ    = 4, /**< 1.3125 MHz */
    SPI_656_25KHZ    = 5, /**< 656.25 KHz */
    SPI_328_125KHZ   = 6, /**< 328.125 KHz */
} SPIFrequency;

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

class HardwareSPI
{
public:
    HardwareSPI(uint32_t spiPortNumber); // Only port SPI1 is currently supported
    void begin(SPIFrequency frequency, uint32_t bitOrder, uint32_t mode);
    void end(void);
    uint8_t transfer(uint8_t data);

private:
    uint32_t _spiPortNumber; // Not used yet.
};
extern HardwareSPI SPI;


#endif
