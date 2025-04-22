// ArduinoCompat/HardwareSPI.h
// STM32 implementattion of Arduino compatible SPI class

#ifndef _HardwareSPI_h
#define _HardwareSPI_h
#include <mgos.h>
#include <mgos_spi.h>
#include <stdint.h>

extern "C"
{
    struct mgos_spi *mgos_spi_get_global(void);
    bool mgos_spi_run_txn(struct mgos_spi *spi, bool full_duplex, const struct mgos_spi_txn *txn);
}

//Not used on MGOS as SPI config is set in mos.yml
#define SPI_MODE0 0x00
#define SPI_MODE1 0x01
#define SPI_MODE2 0x03
#define SPI_MODE3 0x02

#define  SPI_TX_BUFFER_SIZE 64
#define  SPI_RX_BUFFER_SIZE 64

class HardwareSPI
{
public:
    HardwareSPI(uint32_t spiPortNumber); // Only port SPI1 is currently supported
    void begin(int frequency, uint32_t bitOrder, uint32_t mode);
    void end(void);
    uint8_t reverseBits(uint8_t value);
    int8_t getCSGpio();
    uint8_t transfer(uint8_t data);
    uint8_t transfer2B(uint8_t byte0, uint8_t byte1);
    uint8_t spiBurstRead(uint8_t reg, uint8_t* dest, uint8_t len);
    uint8_t spiBurstWrite(uint8_t reg, const uint8_t* src, uint8_t len);
private:
    uint32_t spiPortNumber; // Not used
    struct   mgos_spi_txn txn;
    uint32_t bitOrder;
    //Define spi TX and RX buffers.This is a little wasteful of memory
    //but no dynamic memory allocation fits with the RadioHead library.
    uint8_t spiTXBuf[SPI_TX_BUFFER_SIZE];
    uint8_t spiRXBuf[SPI_RX_BUFFER_SIZE];
};
extern HardwareSPI SPI;


#endif
