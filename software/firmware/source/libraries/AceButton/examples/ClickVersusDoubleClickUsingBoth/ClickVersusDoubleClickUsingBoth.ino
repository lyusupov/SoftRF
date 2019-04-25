/*
 * A demo that combines the techniques of ClickVersusDoubleClickUsingReleased
 * and ClickVersusDoubleClickUsingSuppression by using both event types to
 * trigger a "Clicked". I think this is the best of both worlds. If someone does
 * a simple Press/Release, the Release gets triggered. If someone does a quick
 * click, then a Click gets triggers (after a delay to wait for the potential
 * DoubleClick).
 *
 * See Also:
 *    examples/ClickVersusDoubleClickUsingReleased/
 *    examples/ClickVersusDoubleClickUsingSuppression/
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
  buttonConfig->setFeature(
      ButtonConfig::kFeatureSuppressClickBeforeDoubleClick);
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
    case AceButton::kEventClicked:
    case AceButton::kEventReleased:
      digitalWrite(LED_PIN, LED_ON);
      break;
    case AceButton::kEventDoubleClicked:
      digitalWrite(LED_PIN, LED_OFF);
      break;
  }
}
