#include <WS2812.h>
WS2812 myLEDs(4);
#define PIXELS 4
uint8_t red[PIXELS][3] = {{255, 0, 0}, {255, 0, 0}, {255, 0, 0}, {255, 0, 0}};

void setup() {
  myLEDs.begin();
}

void loop() {
  uint8_t i;
  static uint8_t j = 0;

  // fade in from min to max in increments of 5 points:
  for(int fadeValue = 0 ; fadeValue <= 255; fadeValue +=5) {
    // sets the value (range from 0 to 255):
    for(i=0; i < PIXELS; i++) {
      red[i][j] = fadeValue;
    }
    myLEDs.sendBuffer(red, PIXELS);

    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }

  // fade out from max to min in increments of 5 points:
  for(int fadeValue = 255 ; fadeValue >= 0; fadeValue -=5) {
    // sets the value (range from 0 to 255):
    for(i=0; i < PIXELS; i++) {
      red[i][j] = fadeValue;
    }

    myLEDs.sendBuffer(red, PIXELS);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }

  j++;
  if(j=3) j=0;
}
