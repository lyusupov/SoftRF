/*
 * A demo that distinguishes a "Clicked" from a DoubleClicked by using a
 * Released event instead of a Clicked. Released turns on the LED. A DoubleClick
 * turns off the LED.
 *
 * Normally, AceButton cannot separate a Clicked from a DoubleClicked because
 * the Clicked event will always trigger if a DoubleClicked occurs. We cannot
 * suppress the first Clicked because it has already occurred by the time the
 * DoubleClicked occurs, and the first Clicked cannot predict the future.
 *
 * This version uses a Released event instead of a Clicked to turn the LED on,
 * while suppressing the Released after a DoubleClicked, and we ignore the
 * Clicked event that we can't suppress. The disadvantage of this version is
 * that if a user accidentally makes a normal Clicked event (a rapid
 * Pressed/Released), nothing happens to the LED. Depending on the application,
 * this may or may not be the desirable result.
 *
 * See Also:
 *    examples/ClickVersusDoubleClickUsingSuppression/
 *      - uses the kFeatureSuppressClickBeforeDoubleClick
 */

#include <AceButton.h>
using namespace ace_button;

// The pin number attached to the button.
const int BUTTON_PIN = 2;

#ifdef ESP32
  // Different ESP32 boards use different pins
  const int LED_PIN = 2;
#else
  const int LED_PIN = LED_BUILTIN;
#endif

// LED states. Some microcontrollers wire their built-in LED the reverse.
const int LED_ON = HIGH;
const int LED_OFF = LOW;

// One button wired to the pin at BUTTON_PIN. Automatically uses the default
// ButtonConfig. The alternative is to call the AceButton::init() method in
// setup() below.
AceButton button(BUTTON_PIN);

void handleEvent(AceButton*, uint8_t, uint8_t);

void setup() {
  // initialize built-in LED as an output
  pinMode(LED_PIN, OUTPUT);

  // Button uses the built-in pull up register.
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  ButtonConfig* buttonConfig = button.getButtonConfig();
  buttonConfig->setEventHandler(handleEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
}

void loop() {
  // Should be called every 20ms or faster for the default debouncing time
  // of ~50ms.
  button.check();
}

// The event handler for the button.
void handleEvent(AceButton* /* button */, uint8_t eventType,
    uint8_t /* buttonState */) {
  switch (eventType) {
    case AceButton::kEventReleased:
      digitalWrite(LED_PIN, LED_ON);
      break;
    case AceButton::kEventDoubleClicked:
      digitalWrite(LED_PIN, LED_OFF);
      break;
  }
}
