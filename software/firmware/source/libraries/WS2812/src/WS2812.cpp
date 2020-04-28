#include "WS2812.h"
#include <SPI.h>

#if defined(__MSP430G2553)
#define SPIDIV     SPI_CLOCK_DIV2       // 16 MHz/2 gives 125 ns for each on bit in byte
#define SPILONG    0b11111100           // 750 ns (acceptable "on" range 550 to 850 ns)
#define SPISHORT   0b11100000           // 375 ns (acceptable "on" range 200 to 500 ns)
#elif defined(__MSP430F5529)
#define SPIDIV     SPI_CLOCK_DIV4       // 25.6 MHz/4 gives 156.25 ns for each on bit in byte
#define SPILONG    0b11110000           // 625 ns (acceptable "on" range 550 to 850 ns)
#define SPISHORT   0b11000000           // 312.5 ns (acceptable "on" range 200 to 500 ns)
#elif defined(__MSP432P401R__)
#define SPIDIV     3                    // 16 MHz/2 gives 125 ns for each on bit in byte
#define SPILONG    0b00001111           // 750 ns (acceptable "on" range 550 to 850 ns)
#define SPISHORT   0b00000011           // 375 ns (acceptable "on" range 200 to 500 ns)
#elif defined(BOARD_CC1310_LAUNCHXL) || defined(BOARD_CC1352R1_LAUNCHXL)
#define SPIDIV     3                    // 16 MHz/2 gives 125 ns for each on bit in byte
#define SPILONG    0b00001111           // 750 ns (acceptable "on" range 550 to 850 ns)
#define SPISHORT   0b00000011           // 375 ns (acceptable "on" range 200 to 500 ns)
#elif defined(ENERGIA_EK_TM4C123GXL)
#define SPIDIV     8                    // 80 MHz/4 gives 125 ns for each on bit in byte
#define SPILONG    0b00111111           // 750 ns (acceptable "on" range 550 to 850 ns)
#define SPISHORT   0b00000111           // 375 ns (acceptable "on" range 200 to 500 ns)
#elif defined(ENERGIA_EK_TM4C1294XL)
#define SPIDIV     12                   // 80 MHz/4 gives 125 ns for each on bit in byte
#define SPILONG    0b00111111           // 750 ns (acceptable "on" range 550 to 850 ns)
#define SPISHORT   0b00000111           // 375 ns (acceptable "on" range 200 to 500 ns)
#else
#error This microcontroller is not supported
#endif

WS2812::WS2812(uint16_t pixels)
{
  _pixels = pixels;
}

void WS2812::begin()
{
  uint16_t i;

 SPI.begin();
 SPI.setBitOrder(MSBFIRST);
 SPI.setDataMode(SPI_MODE1);
 SPI.setClockDivider(SPIDIV);

  for(i = 0; i < _pixels; i++) {
   sendPixel(0x00, 0x00, 0x00);
  }

  delayMicroseconds(50);
}

void WS2812::sendBuffer(uint8_t (*ptr)[3], uint8_t len)
{
  uint8_t buf[24*len];
  uint8_t i, j=0, k, n;
  uint8_t bit;

  for (i = 0; i < len; i++) {
    for(k=0; k < 3; k++) {

      for (bit = 0; bit < 8; bit++) {
        n = 2;
        if(k == 0) n = 1;
        if(k == 1) n = 0;
        if (ptr[i][n] & (0x80 >> bit)) {
          buf[j] = SPILONG;
        } else {
          buf[j] = SPISHORT;
        }
        j++;
      }
    }
  }
#if defined(__MSP430__)
  SPI.transmit(buf, 24 * len);
#else
  SPI.transfer(buf, 24 * len);
#endif

  delayMicroseconds(50);
}

inline void WS2812::sendPixel(uint8_t r, uint8_t g, uint8_t b)
{
  uint8_t rgb[] = {g, r, b, g, r, b};
  uint8_t buf[24];
  uint8_t i, j=0;
  uint8_t bit;

  for (i = 0; i < 3; i++) {
    for (bit = 0; bit < 8; bit++) {
      if (rgb[i] & 0x80)
        buf[j] = SPILONG;
      else
        buf[j] = SPISHORT;
      rgb[i] <<= 1;
      j++;
    }
  }
  SPI.transfer(buf, 24);
}

void WS2812::fill(uint8_t r, uint8_t g, uint8_t b)
{
  uint8_t color[_pixels][3];
  uint8_t i;

  for (i = 0; i < _pixels; i++) {
      color[i][0] = r;
      color[i][1] = g;
      color[i][2] = b;
  }
  sendBuffer(color, _pixels);
}

uint16_t WS2812::numPixels(void) const {
  return _pixels;
}

// Convert separate R,G,B into packed 32-bit RGB color.
// Packed format is always RGB, regardless of LED strand color order.
uint32_t WS2812::Color(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g <<  8) | b;
}
