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

#ifndef ACE_BUTTON_TEST_HELPER_H
#define ACE_BUTTON_TEST_HELPER_H

#include <AceButton.h>
#include "TestableButtonConfig.h"
#include "EventTracker.h"

namespace ace_button {
namespace testing {

/**
 * A wrapper class that sends emulated button presses and released to the the
 * underlying AceButton class, and captures the resulting events in the
 * provided EventTracker.
 */
class TestHelper {
  public:
    TestHelper(
        TestableButtonConfig* testableConfig,
        AceButton* button,
        EventTracker* eventTracker):
      mTestableConfig(testableConfig),
      mButton(button),
      mEventTracker(eventTracker) {}

    /** Reinitilize to its pristine state. */
    void init(uint8_t pin, uint8_t defaultReleasedState, uint8_t id) {
      mPin = pin;
      mDefaultReleasedState = defaultReleasedState;
      mId = id;
      mButton->init(mPin, mDefaultReleasedState, mId);
      mTestableConfig->init();
      mTestableConfig->setButtonState(defaultReleasedState);
    }

    /**
     * Simulate a press of the button and run the button.check() processing. The
     * defaultReleasedState is determined by whether the button has a pullup
     * (HIGH) or pulldown (LOW) resistor.
     */
    void pressButton(unsigned long time) {
      uint8_t targetState = (HIGH == mDefaultReleasedState) ? LOW : HIGH;
      mTestableConfig->setClock(time);
      mTestableConfig->setButtonState(targetState);
      mEventTracker->clear();
      mButton->check();
    }

    /**
     * Simulate a release of the button and run the button.check() processing.
     */
    void releaseButton(unsigned long time) {
      uint8_t targetState = (HIGH == mDefaultReleasedState) ? HIGH : LOW;
      mTestableConfig->setClock(time);
      mTestableConfig->setButtonState(targetState);
      mEventTracker->clear();
      mButton->check();
    }

    /**
     * Simply move the time forward and check the button. No changes to button.
     */
    void checkTime(unsigned long time) {
      mTestableConfig->setClock(time);
      mEventTracker->clear();
      mButton->check();
    }

  private:
    // Disable copy-constructor and assignment operator
    TestHelper(const TestHelper&) = delete;
    TestHelper& operator=(const TestHelper&) = delete;

    TestableButtonConfig* mTestableConfig;
    AceButton* mButton;
    EventTracker* mEventTracker;

    uint8_t mPin;
    uint8_t mDefaultReleasedState;
    uint8_t mId;
};

}
}
#endif
