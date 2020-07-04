#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2)

#ifndef CC13XX_H
#define CC13XX_H

#define static_assert(x,y) ({ })

#ifndef nullptr
#define nullptr            ((u1_t *) 0)
#endif

#ifdef __cplusplus

class SPISettings {
public:
  SPISettings() :_clock(1000000), _bitOrder(LSBFIRST), _dataMode(SPI_MODE0){}
  SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) :_clock(clock), _bitOrder(bitOrder), _dataMode(dataMode){}
  uint32_t _clock;
  uint8_t  _bitOrder;
  uint8_t  _dataMode;
};

#endif /* __cplusplus */

#endif // CC13XX_H
#endif // ENERGIA_ARCH_CC13XX || ENERGIA_ARCH_CC13X2
