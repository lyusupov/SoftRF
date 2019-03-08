#ifndef WS2812_h
#define WS2812_h

#include <Arduino.h>
#define DIRECTION_NONE 0
#define DIRECTION_CCW 1
#define DIRECTION_CW 2

class WS2812 {
  public:
  /**
   * @brief Instantiate the WS2812 library.
   *
   * @param[in] pixels The number of pixels the string contains.
   */
  WS2812(uint16_t pixels = 1);
  /**
   * @brief Start the WS2812 library.
   */
  void begin();
  void sendPixel(uint8_t r, uint8_t g, uint8_t b);
  void sendBuffer(uint8_t (*ptr)[3], uint8_t len);
  void color(uint8_t r, uint8_t g, uint8_t b);
  void off(){ color(0, 0, 0); };
  private:
  uint16_t _pixels;
};

#endif
