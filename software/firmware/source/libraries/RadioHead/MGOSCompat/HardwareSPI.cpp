// ArduinoCompat/HardwareSPI.cpp
//
// Interface between Arduino-like SPI interface and STM32F4 Discovery and similar
// using STM32F4xx_DSP_StdPeriph_Lib_V1.3.0

#include <RadioHead.h>
#if (RH_PLATFORM == RH_PLATFORM_MONGOOSE_OS)

#include <mgos.h>
#include <mgos_spi.h>
#include <HardwareSPI.h>

HardwareSPI::HardwareSPI(uint32_t spiPortNumber) : spiPortNumber(spiPortNumber)
{
}

void HardwareSPI::begin(int frequency, uint32_t bitOrder, uint32_t mode)
{
    //Set the SPI tx/rx buffer pointers.
    txn.fd.tx_data = spiTXBuf;
    txn.fd.rx_data = spiRXBuf;

    txn.freq       = frequency;
    this->bitOrder = bitOrder;
    txn.mode       = mode;
#ifdef RH_USE_SPI
    txn.cs         = mgos_sys_config_get_rh_spi_cs();
#else
    txn.cs         = -1;
#endif
}

void HardwareSPI::end(void)
{
    struct mgos_spi *spi = mgos_spi_get_global();
    mgos_spi_close(spi);
}

uint8_t HardwareSPI::reverseBits(uint8_t value)
{
    value = (value & 0xF0) >> 4 | (value & 0x0F) << 4;
    value = (value & 0xCC) >> 2 | (value & 0x33) << 2;
    value = (value & 0xAA) >> 1 | (value & 0x55) << 1;
    return value;
}

uint8_t HardwareSPI::transfer(uint8_t data)
{
    uint8_t status=0;
    txn.fd.len=1;
    spiTXBuf[0]=data;
    if( bitOrder != MSBFIRST ) {
        spiTXBuf[0]=reverseBits(spiTXBuf[0]);
    }
    bool success = mgos_spi_run_txn( mgos_spi_get_global(), true, &txn);
    if( !success ) {
        LOG(LL_INFO, ("%s: Failed SPI transfer()", __FUNCTION__) );
    }
    status = spiRXBuf[0];
    if( bitOrder != MSBFIRST ) {
        status = reverseBits(status);
    }
    return status;
}

uint8_t HardwareSPI::transfer2B(uint8_t byte0, uint8_t byte1)
{
    uint8_t status=0;
    txn.fd.len=2;
    spiTXBuf[0]=byte0;
    spiTXBuf[1]=byte1;
    if( bitOrder != MSBFIRST ) {
        spiTXBuf[0]=reverseBits(spiTXBuf[0]);
        spiTXBuf[1]=reverseBits(spiTXBuf[1]);
    }
    bool success = mgos_spi_run_txn( mgos_spi_get_global(), true, &txn);
    if( !success ) {
        LOG(LL_INFO, ("%s: Failed SPI transfer()", __FUNCTION__) );
    }

    status = spiRXBuf[0];
    if( bitOrder != MSBFIRST ) {
        status = reverseBits(status);
    }
    return status;
}

uint8_t HardwareSPI::spiBurstRead(uint8_t reg, uint8_t* dest, uint8_t len) {
    uint8_t status=0;
    if( len+1 <= SPI_RX_BUFFER_SIZE ) {
        txn.fd.len=len+1;
        memset(spiTXBuf, 0, SPI_RX_BUFFER_SIZE);
        spiTXBuf[0]=reg;
        if( bitOrder != MSBFIRST ) {
            spiTXBuf[0]=reverseBits(spiTXBuf[0]);
        }
        bool success = mgos_spi_run_txn( mgos_spi_get_global(), true, &txn);
        if( !success ) {
            LOG(LL_INFO, ("%s: Failed SPI transfer()", __FUNCTION__) );
        }
        if( bitOrder != MSBFIRST ) {
            uint8_t index=0;
            for( index=0 ; index<len+1 ; index++) {
                spiRXBuf[0]=reverseBits(spiRXBuf[0]);
            }
        }
        memcpy(dest, spiRXBuf+1, len); //copy all but the status byte to the data read buffer
        status = spiRXBuf[0]; //return the status byte
    }
    else {
        LOG(LL_INFO, ("%s: RX buffer not large enough (rx buf length = %d bytes message length = %d bytes).", __FUNCTION__, SPI_RX_BUFFER_SIZE, len) );
    }
    return status;
}

uint8_t HardwareSPI::spiBurstWrite(uint8_t reg, const uint8_t* src, uint8_t len) {
    uint8_t status=0;
    txn.fd.len=len+1;
    memcpy(spiTXBuf+1, src, len);
    spiTXBuf[0]=reg;
    if( bitOrder != MSBFIRST ) {
        uint8_t index=0;
        for( index=0 ; index<len+1 ; index++) {
            spiTXBuf[index]=reverseBits(spiTXBuf[index]);
        }
    }
    memset(spiRXBuf, 0, SPI_RX_BUFFER_SIZE);
    bool success = mgos_spi_run_txn( mgos_spi_get_global(), true, &txn);
    if( !success ) {
        LOG(LL_INFO, ("%s: Failed SPI transfer()", __FUNCTION__) );
    }
    status = spiRXBuf[0];
    if( bitOrder != MSBFIRST ) {
        status = reverseBits(status);
    }
    return status;
}

int8_t HardwareSPI::getCSGpio() {
    uint8_t rhSPICSPin=-1;

    if( txn.cs == 0 ) {
        rhSPICSPin = mgos_sys_config_get_spi_cs0_gpio();
    }
    else if ( txn.cs == 1 ) {
        rhSPICSPin = mgos_sys_config_get_spi_cs1_gpio();
    }
    else if ( txn.cs == 2 ) {
        rhSPICSPin = mgos_sys_config_get_spi_cs2_gpio();
    }
    return rhSPICSPin;
}

#endif
