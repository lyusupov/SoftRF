#include <WS2812.h>
WS2812 myLEDs(4);
#define PIXELS 4
uint8_t red[PIXELS][3] = {{255, 0, 0}, {255, 0, 0}, {255, 0, 0}, {255, 0, 0}};
uint8_t off[PIXELS][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

//uint8_t red[PIXELS][3] = {{255, 0, 0}};
//uint8_t off[PIXELS][3] = {{0, 0, 0}};

void setup() {
  Serial.begin(115200);
  myLEDs.begin();
}

void loop() {
  myLEDs.sendBuffer(red, PIXELS);
  delay(1000);
  myLEDs.sendBuffer(off, PIXELS);
  delay(1000);
}
