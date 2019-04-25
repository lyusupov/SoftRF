/*
 * A demo sketch that uses the AceButton Library
 * (https://github.com/bxparks/AceButton) and the CapitiveSense Library
 * (https://github.com/PaulStoffregen/CapacitiveSensor) to detect touches and
 * clicks of a capacitive switch.
 *
 * Prompted by questions from Gaston Loos.
 *
 * Brian T. Park 2018
 */

#if defined(ESP32) || defined(ESP8266)
  #error ESP32 or ESP8266 not supported
#endif

#include <CapacitiveSensor.h>
#include <AceButton.h>
using namespace ace_button;

/**
 * A subclass of ButtonConfig that emulates a mechanical switch connected to a
 * pull-up resistor on the input pin. A "touch" sends a LOW signal, just like a
 * mechnical switch.
 */
class CapacitiveConfig: public ButtonConfig {
  public:
    CapacitiveConfig(CapacitiveSensor& sensor):
      mSensor(sensor) {}

  protected:
    // Number of iterations to sample the capacitive switch. Higher number
    // provides better smoothing but increases the time taken for a single read.
    static const uint8_t kSamples = 30;

    // The threshold value which is considered to be a "touch" on the switch.
    static const long kTouchThreshold = 100;

    int readButton(uint8_t /*pin*/) override {
      long total =  mSensor.capacitiveSensor(kSamples);
      return (total > kTouchThreshold) ? LOW : HIGH;
    }

  private:
    CapacitiveSensor& mSensor;
};

// Timeout for a single read of the capacitive switch.
static const unsigned long TIMEOUT_MILLIS = 10;

// I used a 1M resistor between pins 4 (send) & metal plate, and a 1K resistor
// between the plate and pin 6 (receive). Try adjusting the
// CapacitiveConfig::kTouchThreshold value for other resistor values.
CapacitiveSensor capSensor(4, 6);

CapacitiveConfig buttonConfig(capSensor);
AceButton button(&buttonConfig);

void handleEvent(AceButton* /* button */, uint8_t eventType,
    uint8_t /* buttonState */) {
  switch (eventType) {
    case AceButton::kEventPressed:
      Serial.println(F("Pressed"));
      break;
    case AceButton::kEventClicked:
      Serial.println(F("Clicked"));
      break;
    case AceButton::kEventDoubleClicked:
      Serial.println(F("DoubleClicked"));
      break;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial); // Leonardo/Micro

  // Set the timeout to 10 millisecond so that AceButton::check()
  // can have about 4-5 iterations during the 50 millisecond debouncing time.
  capSensor.set_CS_Timeout_Millis(TIMEOUT_MILLIS);

  // Configure the button using CapacitiveConfig.
  buttonConfig.setFeature(ButtonConfig::kFeatureClick);
  buttonConfig.setFeature(ButtonConfig::kFeatureDoubleClick);
  buttonConfig.setEventHandler(handleEvent);
}

void loop() {
  unsigned long start = millis();
  button.check();

  // check on performance in milliseconds
  unsigned long duration = millis() - start;
  if (duration > TIMEOUT_MILLIS) {
    Serial.print(F("duration: "));
    Serial.println(duration);
  }
}
