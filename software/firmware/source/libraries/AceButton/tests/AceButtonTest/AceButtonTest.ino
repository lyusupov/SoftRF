#line 2 "AceButtonTest.ino"
/*
MIT License

Copyright (c) 2018 Brian T. Park

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
 * Using Multiple Cpp Files:
 *
 * I tried to spread the tests of this .ino file into 6 .cpp files (and one
 * .h file), matching the 6 logical sections indicated below. Unfortunately,
 * on the AVR platform (Arduino Nano), the flash memory consumption increased
 * from 26768 (87%) to 28630 (93%), a difference of 1862 bytes. Since an Arduino
 * Nano has only 32720 bytes in flash, the difference of 1862 bytes (5.7%) is
 * signficant. I'm not sure where the flash consumption is coming from. Maybe
 * the compiler is includeing debugging information to the 6 additional .cpp
 * file names, or maybe the compiler/linker is using 4-bytes references to
 * various global variables instead of 2-bytes? For now, let's leave all the
 * tests in this single .ino file. I also noticed that changing from "const int
 * PIN = 13" to "#define PIN 13" (and the same with BUTTON_ID) in the .h header
 * file, the #define saved 210 bytes.
 *
 * On the Teensy-ARM platform (Teensy LC), using 6 separate .cpp files instead
 * of one giant .ino file caused the flash memory to *decrease* from 31408 to
 * 31204 bytes(!). And on the ARM platform, there was no difference in flash
 * memory size between "const int PIN = 13" and "#define PIN 13".
 *
 * Conclusion, this seems to be a problem with the avr-gcc compiler, or a
 * suboptimal compiler flags set by the Arduino IDE.
 */

#define USE_AUNIT 1

#if USE_AUNIT == 1
#include <AUnit.h>
#else
#include <ArduinoUnit.h>
#endif

#include <AceButton.h>
#include <ace_button/testing/TestableButtonConfig.h>
#include <ace_button/testing/EventTracker.h>
#include <ace_button/testing/TestHelper.h>

using namespace ace_button;
using namespace ace_button::testing;

const uint8_t PIN = 13;
const uint8_t BUTTON_ID = 1;

ButtonConfig buttonConfig;

TestableButtonConfig testableConfig;
AceButton button(&testableConfig);
EventTracker eventTracker;
TestHelper helper(&testableConfig, &button, &eventTracker);

// The event handler takes the arguments sent with the event and stored them
// into the EventTracker circular buffer.
void handleEvent(AceButton* /* button */, uint8_t eventType,
    uint8_t buttonState) {
  eventTracker.addEvent(eventType, buttonState);
}

void setup() {
  delay(1000); // Wait for stability on some boards, otherwise garage on Serial
  Serial.begin(115200); // ESP8266 default 74880 not supported on Linux
  while (!Serial); // for the Arduino Leonardo/Micro only

  testableConfig.setEventHandler(handleEvent);

  // The default was 50 ms (not the current 20 ms) when these tests were written
  // and some of the timing delays are hardcoded to assume that, so we have to
  // revert back to the old value.
  testableConfig.setDebounceDelay(50);

  Serial.print(F("sizeof(AceButton): "));
  Serial.println(sizeof(AceButton));
  Serial.print(F("sizeof(ButtonConfig): "));
  Serial.println(sizeof(ButtonConfig));
  Serial.print(F("sizeof(TestableButtonConfig): "));
  Serial.println(sizeof(TestableButtonConfig));

  /*
  aunit::TestRunner::exclude("*");
  aunit::TestRunner::include("suppress_click_before_double_click");
  */
}

void loop() {
#if USE_AUNIT == 1
  aunit::TestRunner::run();
#else
  Test::run();
#endif
}

// ------------------------------------------------------------------
// Basic tests
// ------------------------------------------------------------------

// Test that the pin is properly set and retrieved.
test(pin) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;

  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);
  assertEqual(PIN, button.getPin());
}

// Test that the custom id is properly set and retrieved.
test(custom_id) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;

  // reset the button
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);
  assertEqual(BUTTON_ID, button.getId());
}

// Test that the getLastButtonPressed() returns BUTTON_STATE_UKNOWN initially.
test(button_state_unknown) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;

  // reset the button
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);

  uint8_t expected = AceButton::kButtonStateUnknown;
  assertEqual(expected, button.getLastButtonState());
}

test(feature_flags_off_by_default) {
  assertFalse(buttonConfig.isFeature(ButtonConfig::kFeatureClick));
  assertFalse(buttonConfig.isFeature(ButtonConfig::kFeatureDoubleClick));
  assertFalse(buttonConfig.isFeature(ButtonConfig::kFeatureLongPress));
  assertFalse(buttonConfig.isFeature(ButtonConfig::kFeatureRepeatPress));

  assertFalse(buttonConfig.isFeature(
      ButtonConfig::kFeatureSuppressAfterClick));
  assertFalse(buttonConfig.isFeature(
      ButtonConfig::kFeatureSuppressAfterDoubleClick));
  assertFalse(buttonConfig.isFeature(
      ButtonConfig::kFeatureSuppressAfterLongPress));
  assertFalse(buttonConfig.isFeature(
      ButtonConfig::kFeatureSuppressAfterRepeatPress));
}

// Test that the button transitions out of the kButtonStateUnknown after
// getDebounceDelay() time.
test(init_while_released) {
  uint8_t expected;
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;

  // reset the button
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);

  // button is released when the board is rebooted, should trigger an immediate
  // debouncing
  helper.releaseButton(0);
  assertEqual(0, eventTracker.getNumEvents());
  expected = AceButton::kButtonStateUnknown;
  assertEqual(expected, button.getLastButtonState());

  // button is bouncing pressed/released, but must wait to debounce
  helper.pressButton(40);
  assertEqual(0, eventTracker.getNumEvents());
  expected = AceButton::kButtonStateUnknown;
  assertEqual(expected, button.getLastButtonState());

  // button is bouncing pressed/released, but must wait to debounce
  helper.releaseButton(45);
  assertEqual(0, eventTracker.getNumEvents());
  expected = AceButton::kButtonStateUnknown;
  assertEqual(expected, button.getLastButtonState());

  // finally button is known to be released, this doesn't not trigger event
  helper.releaseButton(60);
  assertEqual(0, eventTracker.getNumEvents());
  assertEqual(HIGH, button.getLastButtonState());
}

// Test that the button transitions out of the kButtonStateUnknown when
// rebooted with the button pressed.
test(init_while_pressed) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;
  // reset the button
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);
  uint8_t expected;

  // button is pressed when the board is rebooted, should trigger an immediate
  // debouncing
  helper.pressButton(0);
  assertEqual(0, eventTracker.getNumEvents());
  expected = AceButton::kButtonStateUnknown;
  assertEqual(expected, button.getLastButtonState());

  // button is bouncing pressed/released, but must wait to debounce
  helper.releaseButton(40);
  assertEqual(0, eventTracker.getNumEvents());
  expected = AceButton::kButtonStateUnknown;
  assertEqual(expected, button.getLastButtonState());

  // button is bouncing pressed/released, but must wait to debounce
  helper.pressButton(45);
  assertEqual(0, eventTracker.getNumEvents());
  expected = AceButton::kButtonStateUnknown;
  assertEqual(expected, button.getLastButtonState());

  // finally button is known to be released, this doesn't not trigger event
  helper.pressButton(60);
  assertEqual(0, eventTracker.getNumEvents());
  assertEqual(LOW, button.getLastButtonState());
}

// Test that the TestableButtonConfig overrides the corresponding
// parameters on AceButton properly.
test(testable_config) {
  testableConfig.setClock(0);
  assertEqual(0UL, button.getButtonConfig()->getClock());

  testableConfig.setClock(40);
  assertEqual(40UL, button.getButtonConfig()->getClock());

  testableConfig.setButtonState(HIGH);
  assertEqual(HIGH, button.getButtonConfig()->readButton(0));

  testableConfig.setButtonState(LOW);
  assertEqual(LOW, button.getButtonConfig()->readButton(0));
}

// Test that the ButtonConfig parameters are mutable, just like the
// original AdjustableButtonConfig.
test(adjustable_config) {
  buttonConfig.setDebounceDelay(1);
  assertEqual((uint16_t)1, buttonConfig.getDebounceDelay());

  buttonConfig.setClickDelay(2);
  assertEqual((uint16_t)2, buttonConfig.getClickDelay());

  buttonConfig.setDoubleClickDelay(3);
  assertEqual((uint16_t)3, buttonConfig.getDoubleClickDelay());

  buttonConfig.setLongPressDelay(4);
  assertEqual((uint16_t)4, buttonConfig.getLongPressDelay());

  buttonConfig.setRepeatPressDelay(5);
  assertEqual((uint16_t)5, buttonConfig.getRepeatPressDelay());

  buttonConfig.setRepeatPressInterval(6);
  assertEqual((uint16_t)6, buttonConfig.getRepeatPressInterval());
}

// Detect if a button is pressed while the device is booted.
test(is_released_raw) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;
  button.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);
  testableConfig.init();

  testableConfig.setButtonState(HIGH);
  assertFalse(button.isPressedRaw());

  testableConfig.setButtonState(LOW);
  assertTrue(button.isPressedRaw());
}

// ------------------------------------------------------------------
// Press and Release tests
// ------------------------------------------------------------------

// We assume this will be the common case because of the Aruino boards provide
// internal pullup resistors on the digital input pins.
test(press_and_release_pullup) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;
  uint8_t expected;

  // reset the button
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);

  // initial button state
  helper.releaseButton(0);
  assertEqual(0, eventTracker.getNumEvents());

  // must wait until the initial debouncing
  helper.releaseButton(50);
  assertEqual(0, eventTracker.getNumEvents());

  // button pressed, but must wait to debounce
  helper.pressButton(100);
  assertEqual(0, eventTracker.getNumEvents());

  // still in debouncing period, so no event yet
  helper.releaseButton(110);
  assertEqual(0, eventTracker.getNumEvents());

  // after more than 50 ms, we should get an event
  helper.pressButton(190);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button
  helper.releaseButton(1000);
  assertEqual(0, eventTracker.getNumEvents());

  // wait more than 50 ms
  helper.releaseButton(1060);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
}

// Do the same test as press_and_release_pullup, but using
// the logic levels of an external pulldown resistor.
test(press_and_release_pulldown) {
  const uint8_t DEFAULT_RELEASED_STATE = LOW;
  uint8_t expected;

  // reset the button
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);

  // initial button state
  helper.releaseButton(0);
  assertEqual(0, eventTracker.getNumEvents());

  // must wait until the initial debouncing
  helper.releaseButton(50);
  assertEqual(0, eventTracker.getNumEvents());

  // button pressed, but must wait to debounce
  helper.pressButton(100);
  assertEqual(0, eventTracker.getNumEvents());

  // still in debouncing period, so no event yet
  helper.pressButton(110);
  assertEqual(0, eventTracker.getNumEvents());

  // after more than 50 ms, we should get an event
  helper.pressButton(190);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());

  // release the button
  helper.releaseButton(1000);
  assertEqual(0, eventTracker.getNumEvents());

  // wait more than 50 ms
  helper.releaseButton(1060);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());
}

// The AceButton class uses 16-bit timer variables for memory efficiency.
// Verify that we can rollover those variables without affecting the logic.
test(clock_rollover) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;
  const unsigned long BASE_TIME = 65500; // rolls over in 36 milliseconds
  uint8_t expected;

  // reset the button
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);

  // initial button state
  helper.releaseButton(BASE_TIME + 0);
  assertEqual(0, eventTracker.getNumEvents());

  // initialization phase, so no event yet
  helper.releaseButton(BASE_TIME + 60);
  assertEqual(0, eventTracker.getNumEvents());

  // press after the initialization phase, no event, must wait for debouncing
  helper.pressButton(BASE_TIME + 100);
  assertEqual(0, eventTracker.getNumEvents());

  // after more than 50 ms, we should get an event
  helper.pressButton(BASE_TIME + 190);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button
  helper.releaseButton(BASE_TIME + 1000);
  assertEqual(0, eventTracker.getNumEvents());

  // wait more than 50 ms
  helper.releaseButton(BASE_TIME + 1060);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
}

// ------------------------------------------------------------------
// Click tests
// ------------------------------------------------------------------

// Test a single click.
test(click_without_suppression) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;
  const unsigned long BASE_TIME = 65500;
  uint8_t expected;

  // reset the button
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);
  testableConfig.setFeature(ButtonConfig::kFeatureClick);

  // initial button state
  helper.releaseButton(BASE_TIME + 0);
  assertEqual(0, eventTracker.getNumEvents());

  // initilization phase
  helper.releaseButton(BASE_TIME + 50);
  assertEqual(0, eventTracker.getNumEvents());

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 140);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event
  helper.pressButton(BASE_TIME + 190);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button within 200 ms for a click
  helper.releaseButton(BASE_TIME + 300);
  assertEqual(0, eventTracker.getNumEvents());

  // Wait another 50 ms to get event
  helper.releaseButton(BASE_TIME + 350);
  assertEqual(2, eventTracker.getNumEvents());
  expected = AceButton::kEventClicked;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(1).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(1).getButtonState());
}

// Test a single click.
test(click_with_suppression) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;
  const unsigned long BASE_TIME = 65500;
  uint8_t expected;

  // reset the button
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);
  testableConfig.setFeature(ButtonConfig::kFeatureClick);
  testableConfig.setFeature(ButtonConfig::kFeatureSuppressAfterClick);

  // initial button state
  helper.releaseButton(BASE_TIME + 0);
  assertEqual(0, eventTracker.getNumEvents());

  // initilization phase
  helper.releaseButton(BASE_TIME + 50);
  assertEqual(0, eventTracker.getNumEvents());

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 140);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event
  helper.pressButton(BASE_TIME + 190);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button within 200 ms for a click
  helper.releaseButton(BASE_TIME + 300);
  assertEqual(0, eventTracker.getNumEvents());

  // Wait another 50 ms to get event
  helper.releaseButton(BASE_TIME + 350);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventClicked;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
}

// Test that no click generated with isFeature() flag off.
test(no_click_without_feature_flag) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;
  const unsigned long BASE_TIME = 65500;
  uint8_t expected;

  // reset the button
  // make sure isFeatureClick flag is cleared
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);
  testableConfig.clearFeature(ButtonConfig::kFeatureClick);

  // initial button state
  helper.releaseButton(BASE_TIME + 0);
  assertEqual(0, eventTracker.getNumEvents());

  // initilization phase
  helper.releaseButton(BASE_TIME + 50);
  assertEqual(0, eventTracker.getNumEvents());

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 140);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event
  helper.pressButton(BASE_TIME + 190);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button within 200 ms for a click
  helper.releaseButton(BASE_TIME + 300);
  assertEqual(0, eventTracker.getNumEvents());

  // Wait another 50 ms to get event. Only a Released event should be generated.
  helper.releaseButton(BASE_TIME + 350);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
}

// ------------------------------------------------------------------
// DoubleClick tests
// ------------------------------------------------------------------

// Test a double click. Verify also that a triple-click does not generate a
// spurious second double click. It should generate only the following:
//  Pressed, Clicked, Pressed, DoubleClicked, Pressed, Clicked
// because we have suppressed the Released events.
test(double_click_suppressed) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;
  const unsigned long BASE_TIME = 65500;
  uint8_t expected;

  // reset the button
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);
  testableConfig.setFeature(ButtonConfig::kFeatureDoubleClick);
  testableConfig.setFeature(ButtonConfig::kFeatureSuppressAfterClick);
  testableConfig.setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);

  // initial button state
  helper.releaseButton(BASE_TIME + 0);
  assertEqual(0, eventTracker.getNumEvents());

  // initilization phase
  helper.releaseButton(BASE_TIME + 50);
  assertEqual(0, eventTracker.getNumEvents());

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 140);
  assertEqual(0, eventTracker.getNumEvents());

  // generate first click

  // after 50 ms or more, we should get an event
  helper.pressButton(BASE_TIME + 190);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button within 200 ms for a click, but must wait for debounce
  helper.releaseButton(BASE_TIME + 300);
  assertEqual(0, eventTracker.getNumEvents());

  // Wait another 50 ms for debounce.
  helper.releaseButton(BASE_TIME + 350);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventClicked;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());

  // generate generate second click within 400 ms of the CLICK event (which
  // occurred at +350 ms) for a double click

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 500);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event
  helper.pressButton(BASE_TIME + 550);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button within 200 ms for a click, but must wait for debounce
  helper.releaseButton(BASE_TIME + 650);
  assertEqual(0, eventTracker.getNumEvents());

  // Wait another 50 ms for debounce. Should get a double-click.
  helper.releaseButton(BASE_TIME + 700);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventDoubleClicked;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());

  // generate third click within 400 ms of the DoubleClicked event (which
  // occurred at +700 ms)

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 900);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event,
  helper.pressButton(BASE_TIME + 950);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button within 200 ms for a click, but must wait for debounce
  helper.releaseButton(BASE_TIME + 1050);
  assertEqual(0, eventTracker.getNumEvents());

  // Wait another 50 ms for debounce.
  // Verify that we get only 1 Clicked event not another DoubleClicked.
  helper.releaseButton(BASE_TIME + 1100);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventClicked;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
}

// Test a double click without kFeatureSuppressAfterDoubleClick.
// Three rapid clicks should generate the following:
//    Pressed, Released, Clicked, Pressed, Released, DoubleClicked, Pressed,
//    Released, Clicked.
test(double_click_not_suppressed) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;
  const unsigned long BASE_TIME = 65500;
  uint8_t expected;

  // reset the button
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);
  testableConfig.setFeature(ButtonConfig::kFeatureDoubleClick);

  // initial button state
  helper.releaseButton(BASE_TIME + 0);
  assertEqual(0, eventTracker.getNumEvents());

  // initilization phase
  helper.releaseButton(BASE_TIME + 50);
  assertEqual(0, eventTracker.getNumEvents());

  // generate first click

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 140);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event
  helper.pressButton(BASE_TIME + 190);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button within 200 ms for a click, but must wait for debounce
  helper.releaseButton(BASE_TIME + 300);
  assertEqual(0, eventTracker.getNumEvents());

  // Wait another 50 ms for debounce.
  helper.releaseButton(BASE_TIME + 350);
  assertEqual(2, eventTracker.getNumEvents());
  expected = AceButton::kEventClicked;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(1).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(1).getButtonState());

  // generate second click within 400 ms of the Clicked event (which
  // occurred at +350 ms) for a double click

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 500);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event
  helper.pressButton(BASE_TIME + 550);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button within 200 ms for a click, but must wait for debounce
  helper.releaseButton(BASE_TIME + 650);
  assertEqual(0, eventTracker.getNumEvents());

  // Wait another 50 ms for debounce. Should get a DoubleClicked, and a
  // Released because we don't suppress.
  helper.releaseButton(BASE_TIME + 700);
  assertEqual(2, eventTracker.getNumEvents());
  expected = AceButton::kEventDoubleClicked;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(1).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(1).getButtonState());

  // generate third click within 400 ms of the DoubleClicked event (which
  // occurred at +700 ms)

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 900);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event,
  helper.pressButton(BASE_TIME + 950);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button within 200 ms for a click, but must wait for debounce
  helper.releaseButton(BASE_TIME + 1050);
  assertEqual(0, eventTracker.getNumEvents());

  // Wait another 50 ms for debounce.
  // Verify that we get only 1 Clicked event not another DoubleClicked,
  // and an unsuppressed Released.
  helper.releaseButton(BASE_TIME + 1100);
  assertEqual(2, eventTracker.getNumEvents());
  expected = AceButton::kEventClicked;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(1).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(1).getButtonState());
}

// Test that no double clicks generated with isFeature() flag off.
test(no_double_click_without_feature_flag) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;
  const unsigned long BASE_TIME = 65500;
  uint8_t expected;

  // reset the button
  // make sure isFeatureDoubleClick flag is cleared
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);
  testableConfig.setFeature(ButtonConfig::kFeatureClick);
  testableConfig.clearFeature(ButtonConfig::kFeatureDoubleClick);

  // initial button state
  helper.releaseButton(BASE_TIME + 0);
  assertEqual(0, eventTracker.getNumEvents());

  // initilization phase
  helper.releaseButton(BASE_TIME + 50);
  assertEqual(0, eventTracker.getNumEvents());

  // generate first click

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 140);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event
  helper.pressButton(BASE_TIME + 190);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button within 200 ms for a click, but must wait for debounce
  helper.releaseButton(BASE_TIME + 300);
  assertEqual(0, eventTracker.getNumEvents());

  // Wait another 50 ms for debounce to get Released
  helper.releaseButton(BASE_TIME + 350);
  assertEqual(2, eventTracker.getNumEvents());
  expected = AceButton::kEventClicked;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(1).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(1).getButtonState());

  // generate second click within 400 ms of the Clicked event (which
  // occurred at +350 ms) for a double click

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 500);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event
  helper.pressButton(BASE_TIME + 550);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button within 200 ms for a click, but must wait for debounce
  helper.releaseButton(BASE_TIME + 650);
  assertEqual(0, eventTracker.getNumEvents());

  // Wait another 50 ms for debounce. Should get just another click since
  // double-click is turned off.
  helper.releaseButton(BASE_TIME + 700);
  assertEqual(2, eventTracker.getNumEvents());
  expected = AceButton::kEventClicked;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(1).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(1).getButtonState());
}

// Test that an orphaned click is properly removed to prevent spurious
// double-click if the second click happens slightly over 65.536 seconds later.
test(orphaned_click_cleared) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;
  const unsigned long BASE_TIME = 65500;
  const unsigned long ROLLOVER_TIME = 65536;
  uint8_t expected;

  // reset the button, and enable double-click
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);
  testableConfig.setFeature(ButtonConfig::kFeatureClick);
  testableConfig.setFeature(ButtonConfig::kFeatureDoubleClick);

  // initial button state
  helper.releaseButton(BASE_TIME + 0);
  assertEqual(0, eventTracker.getNumEvents());

  // initilization phase
  helper.releaseButton(BASE_TIME + 50);
  assertEqual(0, eventTracker.getNumEvents());

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 140);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event
  helper.pressButton(BASE_TIME + 190);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button within 200 ms for a click, but must wait for debounce
  helper.releaseButton(BASE_TIME + 300);
  assertEqual(0, eventTracker.getNumEvents());

  // Wait another 50 ms to get event
  helper.releaseButton(BASE_TIME + 350);
  assertEqual(2, eventTracker.getNumEvents());
  expected = AceButton::kEventClicked;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(1).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(1).getButtonState());

  // Move time forward, so that the orphaned click is cleared.
  // If AceButton.checkOrphanedClick() is disabled, or this statement is removed
  // (thereby preventing a call to checkOrphanedClick()), then the asserts below
  // will fail.
  helper.checkTime(BASE_TIME + 5000);
  assertEqual(0, eventTracker.getNumEvents());

  // Generate another click between (65.535s, 65.535s + 400 ms) of the first
  // CLICK event (i.e. +250 ms). If the first orphaned click was not properly
  // reset, then this will genearte a double click instead of a single click.

  // button pressed, but must wait to debounce
  helper.pressButton(ROLLOVER_TIME + BASE_TIME + 400);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event
  helper.pressButton(ROLLOVER_TIME + BASE_TIME + 450);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button within 200 ms for a click, but must wait for debounce
  helper.releaseButton(ROLLOVER_TIME + BASE_TIME + 550);
  assertEqual(0, eventTracker.getNumEvents());

  // Wait another 50 ms for debounce. Should get a single click, not
  // a double click.
  helper.releaseButton(ROLLOVER_TIME + BASE_TIME + 600);
  assertEqual(2, eventTracker.getNumEvents());
  expected = AceButton::kEventClicked;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(1).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(1).getButtonState());
}

// Test that an orphaned click generates a double click if not cleared.
test(orphaned_click_causes_double_click_if_not_cleared) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;
  const unsigned long BASE_TIME = 65500;
  const unsigned long ROLLOVER_TIME = 65536;
  uint8_t expected;

  // reset the button, and enable double-click
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);
  testableConfig.setFeature(ButtonConfig::kFeatureClick);
  testableConfig.setFeature(ButtonConfig::kFeatureDoubleClick);

  // initial button state
  helper.releaseButton(BASE_TIME + 0);
  assertEqual(0, eventTracker.getNumEvents());

  // initilization phase
  helper.releaseButton(BASE_TIME + 50);
  assertEqual(0, eventTracker.getNumEvents());

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 140);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event
  helper.pressButton(BASE_TIME + 190);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button within 200 ms for a click, but must wait for debounce
  helper.releaseButton(BASE_TIME + 300);
  assertEqual(0, eventTracker.getNumEvents());

  // Wait another 50 ms to get event
  helper.releaseButton(BASE_TIME + 350);
  assertEqual(2, eventTracker.getNumEvents());
  expected = AceButton::kEventClicked;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(1).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(1).getButtonState());

  // Simulate an orphaned click not getting cleared by not calling
  // AceButton.check() for 65536 milliseconds.

  // Generate another click between (65.535s, 65.535s + 400 ms) of the first
  // CLICK event (i.e. +250 ms). If the first orphaned click was not properly
  // reset, then this will genearte a double click instead of a single click.

  // button pressed, but must wait to debounce
  helper.pressButton(ROLLOVER_TIME + BASE_TIME + 400);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event
  helper.pressButton(ROLLOVER_TIME + BASE_TIME + 450);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button within 200 ms for a click, but must wait for debounce
  helper.releaseButton(ROLLOVER_TIME + BASE_TIME + 550);
  assertEqual(0, eventTracker.getNumEvents());

  // Wait another 50 ms for debounce. Should get a double click because the
  // orphaned click was not removed before the 16-bit integer overflowed .
  helper.releaseButton(ROLLOVER_TIME + BASE_TIME + 600);
  assertEqual(2, eventTracker.getNumEvents());
  expected = AceButton::kEventDoubleClicked;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(1).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(1).getButtonState());
}

// Test that an orphaned click is removed if Click is enabled.
test(orphaned_click_removed_if_click_enabled) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;
  const unsigned long BASE_TIME = 65500;
  const unsigned long ROLLOVER_TIME = 65536;
  uint8_t expected;

  // reset the button, and enable double-click
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);
  testableConfig.setFeature(ButtonConfig::kFeatureClick);

  // initial button state
  helper.releaseButton(BASE_TIME + 0);
  assertEqual(0, eventTracker.getNumEvents());

  // initilization phase
  helper.releaseButton(BASE_TIME + 50);
  assertEqual(0, eventTracker.getNumEvents());

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 140);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event
  helper.pressButton(BASE_TIME + 190);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button within 200 ms for a click, but must wait for debounce
  helper.releaseButton(BASE_TIME + 300);
  assertEqual(0, eventTracker.getNumEvents());

  // Wait another 50 ms to get event
  helper.releaseButton(BASE_TIME + 350);
  assertEqual(2, eventTracker.getNumEvents());
  expected = AceButton::kEventClicked;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(1).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(1).getButtonState());

  // Move time forward, so that the orphaned click is cleared.
  // If AceButton.checkOrphanedClick() is disabled, or this statement is removed
  // (thereby preventing a call to checkOrphanedClick()), then the asserts below
  // will fail.
  helper.checkTime(BASE_TIME + 5000);
  assertEqual(0, eventTracker.getNumEvents());

  // Turn on DoubleClick in the middle of click processing. If we called
  // checkOrphanedClick() only if DoubleClick was enabled (instead of checking
  // it when Click is enabled as well), then this change in ButtonConfig will
  // cause this test to fail.
  testableConfig.setFeature(ButtonConfig::kFeatureDoubleClick);

  // Generate another click between (65.535s, 65.535s + 400 ms) of the first
  // CLICK event (i.e. +250 ms). If the first orphaned click was not properly
  // reset, then this will genearte a double click instead of a single click.

  // button pressed, but must wait to debounce
  helper.pressButton(ROLLOVER_TIME + BASE_TIME + 400);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event
  helper.pressButton(ROLLOVER_TIME + BASE_TIME + 450);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button within 200 ms for a click, but must wait for debounce
  helper.releaseButton(ROLLOVER_TIME + BASE_TIME + 550);
  assertEqual(0, eventTracker.getNumEvents());

  // Wait another 50 ms for debounce. Should get a single click, not
  // a double click.
  helper.releaseButton(ROLLOVER_TIME + BASE_TIME + 600);
  assertEqual(2, eventTracker.getNumEvents());
  expected = AceButton::kEventClicked;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(1).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(1).getButtonState());
}

// Test that kFeatureSuppressClickBeforeDoubleClick causes the first Clicked to
// be postponed until it can determine if a DoubleClick actually occurred.
test(suppress_click_before_double_click) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;
  const unsigned long BASE_TIME = 65500;
  uint8_t expected;

  // reset the button
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);
  testableConfig.setFeature(ButtonConfig::kFeatureDoubleClick);
  testableConfig.setFeature(
      ButtonConfig::kFeatureSuppressClickBeforeDoubleClick);

  // initial button state
  helper.releaseButton(BASE_TIME + 0);
  assertEqual(0, eventTracker.getNumEvents());

  // initilization phase
  helper.releaseButton(BASE_TIME + 50);
  assertEqual(0, eventTracker.getNumEvents());

  // generate first click

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 140);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event
  helper.pressButton(BASE_TIME + 190);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button within 200 ms for a click, but must wait for debounce
  helper.releaseButton(BASE_TIME + 300);
  assertEqual(0, eventTracker.getNumEvents());

  // Wait another 50 ms for debounce. Check that this first Click is
  // postponed, so we get only the Released.
  helper.releaseButton(BASE_TIME + 350);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());

  // generate second click within 400 ms of the Clicked event (which
  // occurred at +350 ms) for a double click

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 500);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get a Pressed
  helper.pressButton(BASE_TIME + 550);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button within 200 ms for a click, but must wait for debounce
  helper.releaseButton(BASE_TIME + 650);
  assertEqual(0, eventTracker.getNumEvents());

  // Wait another 50 ms for debounce. Should get a (DoubleClicked, Released) but
  // not a Clicked because we suppressed the first postponed Clicked.
  helper.releaseButton(BASE_TIME + 700);
  assertEqual(2, eventTracker.getNumEvents());
  expected = AceButton::kEventDoubleClicked;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(1).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(1).getButtonState());

  // generate third click within 400 ms of the DoubleClicked event (which
  // occurred at +700 ms)

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 900);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get a Pressed event
  helper.pressButton(BASE_TIME + 950);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // release the button within 200 ms for a click, but must wait for debounce
  helper.releaseButton(BASE_TIME + 1050);
  assertEqual(0, eventTracker.getNumEvents());

  // Wait another 50 ms for debounce.
  // Verify that we get only a Released event not another DoubleClicked.
  // The Clicked event is postponed again.
  helper.releaseButton(BASE_TIME + 1100);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());

  // Wait 300 ms, nothing should happen.
  helper.checkTime(BASE_TIME + 1400);
  assertEqual(0, eventTracker.getNumEvents());

  // Wait 400 ms to get the long postponed Clicked.
  helper.checkTime(BASE_TIME + 1500);
  assertEqual(1, eventTracker.getNumEvents());
  assertEqual(+AceButton::kEventClicked,
      eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
}

// ------------------------------------------------------------------
// LongPress tests
// ------------------------------------------------------------------

// Test a long press without suppression should generate a released event at
// the end.
test(long_press_without_suppression) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;
  const unsigned long BASE_TIME = 65500;
  uint8_t expected;

  // reset the button
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);
  testableConfig.setFeature(ButtonConfig::kFeatureLongPress);

  // initial button state
  helper.releaseButton(BASE_TIME + 0);
  assertEqual(0, eventTracker.getNumEvents());

  // initilization phase
  helper.releaseButton(BASE_TIME + 50);
  assertEqual(0, eventTracker.getNumEvents());

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 140);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event
  helper.pressButton(BASE_TIME + 190);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // keeping holding the button
  helper.pressButton(BASE_TIME + 1100);
  assertEqual(0, eventTracker.getNumEvents());

  // keeping holding the button longer than 1000 ms
  helper.pressButton(BASE_TIME + 1200);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventLongPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // finally release the button
  helper.releaseButton(BASE_TIME + 1600);
  assertEqual(0, eventTracker.getNumEvents());

  // Must wait for debouncing for the kEventReleased.
  helper.releaseButton(BASE_TIME + 1660);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
}

// Test a long press with suppression should produce no released event.
test(long_press_with_supression) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;
  const unsigned long BASE_TIME = 65500;
  uint8_t expected;

  // reset the button
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);
  testableConfig.setFeature(ButtonConfig::kFeatureLongPress);
  testableConfig.setFeature(ButtonConfig::kFeatureSuppressAfterLongPress);

  // initial button state
  helper.releaseButton(BASE_TIME + 0);
  assertEqual(0, eventTracker.getNumEvents());

  // initilization phase
  helper.releaseButton(BASE_TIME + 50);
  assertEqual(0, eventTracker.getNumEvents());

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 140);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event
  helper.pressButton(BASE_TIME + 190);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // keeping holding the button
  helper.pressButton(BASE_TIME + 1100);
  assertEqual(0, eventTracker.getNumEvents());

  // keeping holding the button longer than 1000 ms
  helper.pressButton(BASE_TIME + 1200);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventLongPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // finally release the button
  helper.releaseButton(BASE_TIME + 1600);
  assertEqual(0, eventTracker.getNumEvents());

  // Must wait for debouncing. We elected kFeatureSuppressAfterLongPress so
  // no kEventReleased is generated.
  helper.releaseButton(BASE_TIME + 1660);
  assertEqual(0, eventTracker.getNumEvents());
}

// Test that no LongPress generated with isFeature() flag off.
test(no_long_press_without_feature_flag) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;
  const unsigned long BASE_TIME = 65500;
  uint8_t expected;

  // reset the button
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);
  testableConfig.clearFeature(ButtonConfig::kFeatureLongPress);

  // initial button state
  helper.releaseButton(BASE_TIME + 0);
  assertEqual(0, eventTracker.getNumEvents());

  // initilization phase
  helper.releaseButton(BASE_TIME + 50);
  assertEqual(0, eventTracker.getNumEvents());

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 140);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event
  helper.pressButton(BASE_TIME + 190);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // keeping holding the button
  helper.pressButton(BASE_TIME + 1100);
  assertEqual(0, eventTracker.getNumEvents());

  // keeping holding the button longer than 1000 ms
  helper.pressButton(BASE_TIME + 1200);
  assertEqual(0, eventTracker.getNumEvents());

  // finally release the button
  helper.releaseButton(BASE_TIME + 1600);
  assertEqual(0, eventTracker.getNumEvents());

  // Must wait for debouncing. Only a Released event should be generated.
  helper.releaseButton(BASE_TIME + 1660);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
}

// ------------------------------------------------------------------
// RepeatPress tests
// ------------------------------------------------------------------

// Test repeated press
test(repeat_press_without_suppression) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;
  const unsigned long BASE_TIME = 65500;
  uint8_t expected;

  // reset the button
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);
  testableConfig.setFeature(ButtonConfig::kFeatureRepeatPress);

  // initial button state
  helper.releaseButton(BASE_TIME + 0);
  assertEqual(0, eventTracker.getNumEvents());

  // initilization phase
  helper.releaseButton(BASE_TIME + 50);
  assertEqual(0, eventTracker.getNumEvents());

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 140);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event
  helper.pressButton(BASE_TIME + 190);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // keeping holding the button
  helper.pressButton(BASE_TIME + 1100);
  assertEqual(0, eventTracker.getNumEvents());

  // keeping holding the button longer than 1000 ms, the kEventRepeatPressed
  // should trigger immediately after this duration
  helper.pressButton(BASE_TIME + 1200);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventRepeatPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // keeping holding the button for longer than repeat interval (200ms)
  helper.pressButton(BASE_TIME + 1400);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventRepeatPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // finally release the button
  helper.releaseButton(BASE_TIME + 1700);
  assertEqual(0, eventTracker.getNumEvents());

  // Must wait for debouncing for the kEventReleased.
  helper.releaseButton(BASE_TIME + 1760);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
}

// Test repeated press
test(repeat_press_with_suppression) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;
  const unsigned long BASE_TIME = 65500;
  uint8_t expected;

  // reset the button
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);
  testableConfig.setFeature(ButtonConfig::kFeatureRepeatPress);
  testableConfig.setFeature(ButtonConfig::kFeatureSuppressAfterRepeatPress);

  // initial button state
  helper.releaseButton(BASE_TIME + 0);
  assertEqual(0, eventTracker.getNumEvents());

  // initilization phase
  helper.releaseButton(BASE_TIME + 50);
  assertEqual(0, eventTracker.getNumEvents());

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 140);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event
  helper.pressButton(BASE_TIME + 190);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // keeping holding the button
  helper.pressButton(BASE_TIME + 1100);
  assertEqual(0, eventTracker.getNumEvents());

  // keeping holding the button longer than 1000 ms, the kEventRepeatPressed
  // should trigger immediately after this duration
  helper.pressButton(BASE_TIME + 1200);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventRepeatPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // keeping holding the button for longer than repeat interval (200ms)
  helper.pressButton(BASE_TIME + 1400);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventRepeatPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // finally release the button
  helper.releaseButton(BASE_TIME + 1700);
  assertEqual(0, eventTracker.getNumEvents());

  // Must wait for debouncing for the kEventReleased.
  // But there is no Released event because of suppression.
  helper.releaseButton(BASE_TIME + 1760);
  assertEqual(0, eventTracker.getNumEvents());
}

// Test that no RepeatPress generated with isFeature() flag off.
test(no_repeat_press_without_feature_flag) {
  const uint8_t DEFAULT_RELEASED_STATE = HIGH;
  const unsigned long BASE_TIME = 65500;
  uint8_t expected;

  // reset the button
  helper.init(PIN, DEFAULT_RELEASED_STATE, BUTTON_ID);
  testableConfig.clearFeature(ButtonConfig::kFeatureRepeatPress);

  // initial button state
  helper.releaseButton(BASE_TIME + 0);
  assertEqual(0, eventTracker.getNumEvents());

  // initilization phase
  helper.releaseButton(BASE_TIME + 50);
  assertEqual(0, eventTracker.getNumEvents());

  // button pressed, but must wait to debounce
  helper.pressButton(BASE_TIME + 140);
  assertEqual(0, eventTracker.getNumEvents());

  // after 50 ms or more, we should get an event
  helper.pressButton(BASE_TIME + 190);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventPressed;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(LOW, eventTracker.getRecord(0).getButtonState());

  // keeping holding the button
  helper.pressButton(BASE_TIME + 1100);
  assertEqual(0, eventTracker.getNumEvents());

  // keeping holding the button longer than 1000 ms, nothing should
  // should trigger
  helper.pressButton(BASE_TIME + 1200);
  assertEqual(0, eventTracker.getNumEvents());

  // keeping holding the button for longer than repeat interval (200ms)
  helper.pressButton(BASE_TIME + 1400);
  assertEqual(0, eventTracker.getNumEvents());

  // finally release the button
  helper.releaseButton(BASE_TIME + 1700);
  assertEqual(0, eventTracker.getNumEvents());

  // Must wait for debouncing. Only a Released event should be generated.
  helper.releaseButton(BASE_TIME + 1760);
  assertEqual(1, eventTracker.getNumEvents());
  expected = AceButton::kEventReleased;
  assertEqual(expected, eventTracker.getRecord(0).getEventType());
  assertEqual(HIGH, eventTracker.getRecord(0).getButtonState());
}
