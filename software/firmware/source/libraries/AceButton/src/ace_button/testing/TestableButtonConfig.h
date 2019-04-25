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

#ifndef ACE_BUTTON_TESTABLE_BUTTON_CONFIG_H
#define ACE_BUTTON_TESTABLE_BUTTON_CONFIG_H

#include "../ButtonConfig.h"

namespace ace_button {
namespace testing {

/**
 * A subclass of ButtonConfig which overrides getClock() and readButton() so
 * that their values can be controlled manually. This is intended to be used for
 * unit testing.
 */
class TestableButtonConfig: public ButtonConfig {
  public:
    TestableButtonConfig():
        mMillis(0),
        mButtonState(HIGH) {}

    /**
     * Initialize to its pristine state. This method is needed because
     * ArduinoUnit does not create a new instance of the Test class for each
     * test case, so we have to reuse objects between test cases, so we need a
     * way to reinitialize this object to its pristine state just after
     * construction.
     */
    void init() override {
      ButtonConfig::init();
      mMillis = 0;
      mButtonState = HIGH;
    }

    /** Read the time of the fake clock. */
    unsigned long getClock() override { return mMillis; }

    /** Read the fake physical button. */
    int readButton(uint8_t /* pin */) override { return mButtonState; }

    /** Set the time of the fake clock. */
    void setClock(unsigned long millis) { mMillis = millis; }

    /** Set the state of the fake physical button. */
    void setButtonState(int buttonState) { mButtonState = buttonState; }

  private:
    // Disable copy-constructor and assignment operator
    TestableButtonConfig(const TestableButtonConfig&) = delete;
    TestableButtonConfig& operator=(const TestableButtonConfig&) = delete;

    unsigned long mMillis;
    int mButtonState;
};

}
}
#endif
