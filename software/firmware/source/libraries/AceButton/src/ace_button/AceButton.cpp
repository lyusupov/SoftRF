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

#include "TimingStats.h"
#include "AceButton.h"

namespace ace_button {

// Check that the Arduino constants HIGH and LOW are defined to be 1 and 0,
// respectively. Otherwise, this library won't work.
#if HIGH != 1
  #error HIGH must be defined to be 1
#endif
#if LOW != 0
  #error LOW must be defined to be 0
#endif

AceButton::AceButton(uint8_t pin, uint8_t defaultReleasedState, uint8_t id):
    mButtonConfig(ButtonConfig::getSystemButtonConfig()) {
  init(pin, defaultReleasedState, id);
}

AceButton::AceButton(ButtonConfig* buttonConfig):
    mButtonConfig(buttonConfig) {
  init(0, HIGH, 0);
}

void AceButton::init(uint8_t pin, uint8_t defaultReleasedState, uint8_t id) {
  mPin = pin;
  mId = id;
  mFlags = 0;
  mLastButtonState = kButtonStateUnknown;
  mLastDebounceTime = 0;
  mLastClickTime = 0;
  setDefaultReleasedState(defaultReleasedState);
}

void AceButton::setDefaultReleasedState(uint8_t state) {
  if (state == HIGH) {
    mFlags |= kFlagDefaultReleasedState;
  } else {
    mFlags &= ~kFlagDefaultReleasedState;
  }
}

uint8_t AceButton::getDefaultReleasedState() {
  return (mFlags & kFlagDefaultReleasedState) ? HIGH : LOW;
}

// NOTE: It would be interesting to rewrite the check() method using a Finite
// State Machine.
void AceButton::check() {
  // Get the micros.
  uint16_t nowMicros = mButtonConfig->getClockMicros();

  // Retrieve the current time just once and use that in the various checkXxx()
  // functions below. This provides some robustness of the various timing
  // algorithms even if any of the event handlers takes more time than the
  // threshold time limits such as 'debounceDelay' or longPressDelay'.
  uint16_t now = mButtonConfig->getClock();

  uint8_t buttonState = mButtonConfig->readButton(mPin);

  // debounce the button
  if (checkDebounced(now, buttonState)) {
    // check if the button was initialized (i.e. UNKNOWN state)
    if (checkInitialized(buttonState)) {
      checkEvent(now, buttonState);
    }
  }

  TimingStats* stats = mButtonConfig->getTimingStats();
  if (stats != nullptr) {
    uint16_t elapsedMicros = mButtonConfig->getClockMicros() - nowMicros;
    stats->update(elapsedMicros);
  }
}

void AceButton::checkEvent(uint16_t now, uint8_t buttonState) {
  // We need to remove orphaned clicks even if just Click is enabled. It is not
  // sufficient to do this for just DoubleClick. That's because it's possible
  // for a Clicked event to be generated, then 65.536 seconds later, the
  // ButtonConfig could be changed to enable DoubleClick. (Such real-time change
  // of ButtonConfig is not recommended, but is sometimes convenient.) If the
  // orphaned click is not cleared, then the next Click would be errorneously
  // considered to be a DoubleClick. Therefore, we must clear the orphaned click
  // even if just the Clicked event is enabled.
  //
  // We also need to check of any postponed clicks that got generated when
  // kFeatureSuppressClickBeforeDoubleClick was enabled.
  if (mButtonConfig->isFeature(ButtonConfig::kFeatureClick) ||
      mButtonConfig->isFeature(ButtonConfig::kFeatureDoubleClick)) {
    checkPostponedClick(now);
    checkOrphanedClick(now);
  }

  if (mButtonConfig->isFeature(ButtonConfig::kFeatureLongPress)) {
    checkLongPress(now, buttonState);
  }
  if (mButtonConfig->isFeature(ButtonConfig::kFeatureRepeatPress)) {
    checkRepeatPress(now, buttonState);
  }
  if (buttonState != getLastButtonState()) {
    checkChanged(now, buttonState);
  }
}

bool AceButton::checkDebounced(uint16_t now, uint8_t buttonState) {
  if (isDebouncing()) {

    // NOTE: This is a bit tricky. The elapsedTime will be valid even if the
    // uint16_t representation of 'now' rolls over so that (now <
    // mLastDebounceTime). This is true as long as the 'unsigned long'
    // representation of 'now' is < (65536 + mLastDebounceTime). We need to cast
    // this expression into an uint16_t before doing the '>=' comparison below
    // for compatability with processors whose sizeof(int) == 4 instead of 2.
    // For those processors, the expression (now - mLastDebounceTime >=
    // getDebounceDelay()) won't work because the terms in the expression get
    // promoted to an (int).
    uint16_t elapsedTime = now - mLastDebounceTime;

    bool isDebouncingTimeOver =
        (elapsedTime >= mButtonConfig->getDebounceDelay());

    if (isDebouncingTimeOver) {
      clearDebouncing();
      return true;
    } else {
      return false;
    }
  } else {
    // Currently not in debouncing phase. Check for a button state change. This
    // will also detect a transition from kButtonStateUnknown to HIGH or LOW.
    if (buttonState == getLastButtonState()) {
      // no change, return immediately
      return true;
    }

    // button has changed so, enter debouncing phase
    setDebouncing();
    mLastDebounceTime = now;
    return false;
  }
}

bool AceButton::checkInitialized(uint16_t buttonState) {
  if (mLastButtonState != kButtonStateUnknown) {
    return true;
  }

  // If transitioning from the initial "unknown" button state, just set the last
  // valid button state, but don't fire off the event handler. This handles the
  // case where a momentary switch is pressed down, then the board is rebooted.
  // When the board comes up, it should not fire off the event handler. This
  // also handles the case of a 2-position switch set to the "pressed"
  // position, and the board is rebooted.
  mLastButtonState = buttonState;
  return false;
}

void AceButton::checkLongPress(uint16_t now, uint8_t buttonState) {
  if (buttonState == getDefaultReleasedState()) {
    return;
  }

  if (isPressed() && !isLongPressed()) {
    uint16_t elapsedTime = now - mLastPressTime;
    if (elapsedTime >= mButtonConfig->getLongPressDelay()) {
      setLongPressed();
      handleEvent(kEventLongPressed);
    }
  }
}

void AceButton::checkRepeatPress(uint16_t now, uint8_t buttonState) {
  if (buttonState == getDefaultReleasedState()) {
    return;
  }

  if (isPressed()) {
    if (isRepeatPressed()) {
      uint16_t elapsedTime = now - mLastRepeatPressTime;
      if (elapsedTime >= mButtonConfig->getRepeatPressInterval()) {
        handleEvent(kEventRepeatPressed);
        mLastRepeatPressTime = now;
      }
    } else {
      uint16_t elapsedTime = now - mLastPressTime;
      if (elapsedTime >= mButtonConfig->getRepeatPressDelay()) {
        setRepeatPressed();
        // Trigger the RepeatPressed immedidately, instead of waiting until the
        // first getRepeatPressInterval() has passed.
        handleEvent(kEventRepeatPressed);
        mLastRepeatPressTime = now;
      }
    }
  }
}

void AceButton::checkChanged(uint16_t now, uint8_t buttonState) {
  mLastButtonState = buttonState;
  checkPressed(now, buttonState);
  checkReleased(now, buttonState);
}

void AceButton::checkPressed(uint16_t now, uint8_t buttonState) {
  if (buttonState == getDefaultReleasedState()) {
    return;
  }

  // button was pressed
  mLastPressTime = now;
  setPressed();
  handleEvent(kEventPressed);
}

void AceButton::checkReleased(uint16_t now, uint8_t buttonState) {
  if (buttonState != getDefaultReleasedState()) {
    return;
  }

  // Check for click (before sending off the Released event).
  // Make sure that we don't clearPressed() before calling this.
  if (mButtonConfig->isFeature(ButtonConfig::kFeatureClick)
      || mButtonConfig->isFeature(ButtonConfig::kFeatureDoubleClick)) {
    checkClicked(now);
  }

  // check if Released events are suppressed
  bool suppress =
      ((isLongPressed() &&
          mButtonConfig->
              isFeature(ButtonConfig::kFeatureSuppressAfterLongPress)) ||
      (isRepeatPressed() &&
          mButtonConfig->
              isFeature(ButtonConfig::kFeatureSuppressAfterRepeatPress)) ||
      (isClicked() &&
          mButtonConfig->isFeature(ButtonConfig::kFeatureSuppressAfterClick)) ||
      (isDoubleClicked() &&
          mButtonConfig->
              isFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick)));

  // button was released
  clearPressed();
  clearDoubleClicked();
  clearLongPressed();
  clearRepeatPressed();

  if (!suppress) {
    handleEvent(kEventReleased);
  }
}

void AceButton::checkClicked(uint16_t now) {
  if (!isPressed()) {
    // Not a Click unless the previous state was a Pressed state.
    // This can happen if the chip was rebooted with the button Pressed. Upon
    // Release, it shouldn't generated a click, even accidentally due to a
    // spurious value in mLastPressTime.
    clearClicked();
    return;
  }
  uint16_t elapsedTime = now - mLastPressTime;
  if (elapsedTime >= mButtonConfig->getClickDelay()) {
    clearClicked();
    return;
  }

  // check for double click
  if (mButtonConfig->isFeature(ButtonConfig::kFeatureDoubleClick)) {
    checkDoubleClicked(now);
  }

  // Suppress a second click (both buttonState change and event message) if
  // double-click detected, which has the side-effect of preventing 3 clicks
  // from generating another double-click at the third click.
  if (isDoubleClicked()) {
    clearClicked();
    return;
  }

  // we got a single click
  mLastClickTime = now;
  setClicked();
  if (mButtonConfig->isFeature(
      ButtonConfig::kFeatureSuppressClickBeforeDoubleClick)) {
    setClickPostponed();
  } else {
    handleEvent(kEventClicked);
  }
}

void AceButton::checkDoubleClicked(uint16_t now) {
  if (!isClicked()) {
    clearDoubleClicked();
    return;
  }

  uint16_t elapsedTime = now - mLastClickTime;
  if (elapsedTime >= mButtonConfig->getDoubleClickDelay()) {
    clearDoubleClicked();
    // There should be no postponed Click at this point because
    // checkPostponedClick() should have taken care of it.
    return;
  }

  // If there was a postponed click, suppress it because it could only have been
  // postponed if kFeatureSuppressClickBeforeDoubleClick was enabled. If we got
  // to this point, there was a DoubleClick, so we must suppress the first
  // Click as requested.
  if (isClickPostponed()) {
    clearClickPostponed();
  }
  setDoubleClicked();
  handleEvent(kEventDoubleClicked);
}

void AceButton::checkOrphanedClick(uint16_t now) {
  // The amount of time which must pass before a click is determined to be
  // orphaned and reclaimed. If only DoubleClicked is supported, then I think
  // just getDoubleClickDelay() is correct. No other higher level event uses the
  // first Clicked event. If TripleClicked becomes supported, I think
  // orphanedClickDelay will be either (2 * getDoubleClickDelay()) or
  // (getDoubleClickDelay() + getTripleClickDelay()), depending on whether the
  // TripleClick has an independent delay time, or reuses the DoubleClick delay
  // time. But I'm not sure that I've thought through all the details.
  uint16_t orphanedClickDelay = mButtonConfig->getDoubleClickDelay();

  uint16_t elapsedTime = now - mLastClickTime;
  if (isClicked() && (elapsedTime >= orphanedClickDelay)) {
    clearClicked();
  }
}

void AceButton::checkPostponedClick(uint16_t now) {
  uint16_t postponedClickDelay = mButtonConfig->getDoubleClickDelay();
  uint16_t elapsedTime = now - mLastClickTime;
  if (isClickPostponed() && elapsedTime >= postponedClickDelay) {
    handleEvent(kEventClicked);
    clearClickPostponed();
  }
}

void AceButton::handleEvent(uint8_t eventType) {
  ButtonConfig::EventHandler eventHandler = mButtonConfig->getEventHandler();
  if (eventHandler) {
    eventHandler(this, eventType, getLastButtonState());
  }
}

}
