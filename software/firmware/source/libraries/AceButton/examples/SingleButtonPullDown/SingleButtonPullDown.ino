/*
 * A demo of a simple AceButton used to handle the events from one button.
 * Pretty much the same as SingleButton.ino example but the button is wired with
 * an external pull-down resistor, instead of using the built-in pull-up
 * resistor.
 */

#include <AceButton.h>
using namespace ace_button;

// The pin number attached to the button.
const int BUTTON_PIN = 7;

#ifdef ESP32
  // Different ESP32 boards use different pins
  const int LED_PIN = 2;
#else
  const int LED_PIN = LED_BUILTIN;
#endif

// LED states. Some microcontrollers wire their built-in LED the reverse.
const int LED_ON = HIGH;
const int LED_OFF = LOW;

// Automatically uses the default ButtonConfig. We will configure this later
// using AceButton::init() method in setup() below.
AceButton button;

void handleEvent(AceButton*, uint8_t, uint8_t);

void setup() {
  delay(1000); // some microcontrollers reboot twice
  Serial.begin(115200);
  while (! Serial); // Wait until Serial is ready - Leonardo/Micro
  Serial.println(F("setup(): begin"));

  // initialize built-in LED as an output
  pinMode(LED_PIN, OUTPUT);

  // Button uses an external 10k resistor.
  pinMode(BUTTON_PIN, INPUT);

  // We use the AceButton::init() method here instead of using the constructor
  // to show an alternative. Using init() allows the configuration of the
  // hardware pin and the button to be placed closer to each other.
  button.init(BUTTON_PIN, LOW);

  // Configure the ButtonConfig with the event handler, and enable the LongPress
  // and RepeatPress events which are turned off by default.
  ButtonConfig* buttonConfig = button.getButtonConfig();
  buttonConfig->setEventHandler(handleEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  buttonConfig->setFeature(ButtonConfig::kFeatureRepeatPress);

  Serial.println(F("setup(): ready"));
}

void loop() {
  // Should be called every 20ms or faster for the default debouncing time
  // of ~50ms.
  button.check();
}

// The event handler for the button.
void handleEvent(AceButton* /* button */, uint8_t eventType,
    uint8_t buttonState) {

  // Print out a message for all events.
  Serial.print(F("handleEvent(): eventType: "));
  Serial.print(eventType);
  Serial.print(F("; buttonState: "));
  Serial.println(buttonState);

  // Control the LED only for the Pressed and Released events.
  // Notice that if the MCU is rebooted while the button is pressed down, no
  // event is triggered and the LED remains off.
  switch (eventType) {
    case AceButton::kEventReleased:
      digitalWrite(LED_PIN, LED_ON);
      break;
    case AceButton::kEventPressed:
      digitalWrite(LED_PIN, LED_OFF);
      break;
  }
}
