/*
 * A demo of a simplest AceButton that has a visible effect. One button is
 * connected to the digital pin BUTTON_PIN. It uses the internal pull-up
 * resistor (INPUT_PULLUP). Pressing the button turns on the built-in LED.
 * Releasing the button turns off the LED.
 */

#include <AceButton.h>

using namespace ace_button;

// Some ESP32 boards have multiple builtin LEDs so don't define LED_BUILTIN.
#if defined(ESP32)
  const int LED_PIN = 2;
#else
  const int LED_PIN = LED_BUILTIN;
#endif

const int BUTTON_PIN = 2;
const int LED_ON = HIGH;
const int LED_OFF = LOW;

AceButton button(BUTTON_PIN);

void handleEvent(AceButton*, uint8_t, uint8_t);

void setup() {
  delay(2000);

#if defined(ARDUINO_AVR_LEONARDO)
  RXLED0; // LED off
  TXLED0; // LED off
#endif

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  button.setEventHandler(handleEvent);
}

void loop() {
  button.check();
}

void handleEvent(AceButton* /* button */, uint8_t eventType,
    uint8_t /* buttonState */) {
  switch (eventType) {
    case AceButton::kEventPressed:
      digitalWrite(LED_PIN, LED_ON);
      break;
    case AceButton::kEventReleased:
      digitalWrite(LED_PIN, LED_OFF);
      break;
  }
}
