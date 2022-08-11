#include <SoftSPI.h>

#if defined(ARDUINO_ARCH_NRF52)
SoftSPI::SoftSPI(uint8_t mosi, uint8_t miso, uint8_t sck) : SPIClass(NRF_SPIM0, 0, 0, 0)
#elif defined(ARDUINO_ARCH_ESP32)
SoftSPI::SoftSPI(uint8_t mosi, uint8_t miso, uint8_t sck) : SPIClass()
#else
#error "This build architecture is not supported!"
#endif /* ARDUINO_ARCH_NRF52 || ARDUINO_ARCH_ESP32 */
{
    _mosi = mosi;
    _miso = miso;
    _sck = sck;
}

void SoftSPI::begin()
{
    pinMode(_mosi, OUTPUT);
    if (_miso != _mosi) pinMode(_miso, INPUT);
    pinMode(_sck, OUTPUT);
}

void SoftSPI::end()
{
    pinMode(_mosi, INPUT);
    if (_miso != _mosi) pinMode(_miso, INPUT);
    pinMode(_sck, INPUT);
}

void SoftSPI::setBitOrder(uint8_t order)
{
    
}

void SoftSPI::setDataMode(uint8_t mode)
{
}

void SoftSPI::setClockDivider(uint32_t div)
{

}

uint8_t SoftSPI::transfer(uint8_t val)
{
    uint8_t out = 0;
    for (int i = 0; i < 8 ; i++) {
        digitalWrite(_sck, LOW);
        delayMicroseconds(10);
        out <<= 1;
        if (digitalRead(_miso)) {
            out |= 0x01;
        }
        if (val & 0x80) {
            digitalWrite(_mosi, HIGH );
        } else {
            digitalWrite(_mosi, LOW);
        }
        val <<= 1;
        digitalWrite(_sck, HIGH);
        delayMicroseconds(10);
    }
    return out;
}

uint8_t SoftSPI::transfer_in()
{
    uint8_t out = 0;
    for (int i = 0; i < 8 ; i++) {
        digitalWrite(_sck, LOW);
        delayMicroseconds(10);
        out <<= 1;
        if (digitalRead(_miso)) {
            out |= 0x01;
        }
        digitalWrite(_sck, HIGH);
        delayMicroseconds(10);
    }
    return out;
}

void SoftSPI::transfer_out(uint8_t val)
{
    for (int i = 0; i < 8 ; i++) {
        digitalWrite(_sck, LOW);
        delayMicroseconds(10);
        if (val & 0x80) {
            digitalWrite(_mosi, HIGH );
        } else {
            digitalWrite(_mosi, LOW);
        }
        val <<= 1;
        digitalWrite(_sck, HIGH);
        delayMicroseconds(10);
    }
}

uint16_t SoftSPI::transfer16(uint16_t data)
{
    union {
        uint16_t val;
        struct {
            uint8_t lsb;
            uint8_t msb;
        };
    } in, out;

    in.val = data;
    out.msb = transfer(in.msb);
    out.lsb = transfer(in.lsb);
    return out.val;
}

void SoftSPI::transfer(void *buf, size_t count)
{
    uint8_t *buffer = (uint8_t *) buf;
    for (size_t i = 0; i < count; i++) {
        buffer[i] = transfer(buffer[i]);
    }
}

/* compatibility with Adafruit_nRF52_Arduino */
void SoftSPI::transfer(const void *tx_buf, void *rx_buf, size_t count)
{
    transfer(rx_buf, count);
}

void SoftSPI::beginTransaction(SPISettings settings)
{
}

void SoftSPI::endTransaction(void)
{
}
