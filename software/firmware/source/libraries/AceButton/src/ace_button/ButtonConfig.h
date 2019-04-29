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

#ifndef ACE_BUTTON_BUTTON_CONFIG_H
#define ACE_BUTTON_BUTTON_CONFIG_H

#if !defined(RASPBERRY_PI)
#include <Arduino.h>
#else
#include <raspi/raspi.h>
#endif /* RASPBERRY_PI */

// TODO: Verify if this is actually needed. The program size seems to be
// identical with or without it on the Arduino IDE (which uses gcc).
#define ACE_BUTTON_INLINE __attribute__((always_inline))

namespace ace_button {

class AceButton;
class TimingStats;

/**
 * Class that defines the timing parameters and event handler of an AceButton or
 * a group of AceButton instances. It is assumed that in many cases, a group of
 * multiple buttons will need to be assigned the same configuration parameters.
 * For example, various timing delays and the EventHandler. Instead of storing
 * these parameters in each instance of AceButton (which consumes static
 * memory), we save space by collecting them into a separate ButtonConfig class.
 * Each AceButton instance contains a pointer to an instance of ButtonConfig,
 * and an instance of ButtonConfig will be shared among multiple AceButtons.
 *
 * Various timing parameters are given default values. They can be
 * overridden by the user.
 *
 * A single default "System" ButtonConfig instance is created automatically and
 * can be accessed using the ButtConfig::getSystemButtonConfig() static method.
 * For convenience and ease of use, every instance of AceButton is attached to
 * this "System" ButtonConfig by default. The client code can override this
 * association by attaching another ButtonConfig instance using the
 * AceButton(ButtonConfig*) constuctor (the recommended solution) or the
 * AceButton::setButtonConfig() method.
 */
class ButtonConfig {
  public:
    // Various timing constants, in milliseconds.
    //
    // Note that the timing constants are stored as uint16_t (2
    // bytes) instead of unsigned long (4 bytes) which is the type returned by
    // the millis() system method. It turns out that we can store and perform
    // all timing calculations using uint16_t without ill effect, as long as the
    // polling of AceButton::check() happens more frequently than the rollover
    // time of a uint16_t (i.e. 65.536 seconds) and certain precautions (e.g.
    // AceButton::checkOrphanedClick()) are taken before a uint16_t rollover
    // happens. In theory, these additional precautions would be needed even if
    // an 'unsigned long' is used but almost no one does them because they
    // assume that their code won't be running continuously for the rollover
    // time of an 'unsigned long' (i.e. 49.7 days).

    /** Default value returned by getDebounceDelay(). */
    static const uint16_t kDebounceDelay = 20;

    /** Default value returned by getClickDelay(). */
    static const uint16_t kClickDelay = 200;

    /** Default value returned by getDoubleClickDelay(). */
    static const uint16_t kDoubleClickDelay = 400;

    /** Default value returned by getLongPressDelay(). */
    static const uint16_t kLongPressDelay = 1000;

    /** Default value returned by getRepeatPressDelay(). */
    static const uint16_t kRepeatPressDelay = 1000;

    /** Default value returned by getRepeatPressInterval(). */
    static const uint16_t kRepeatPressInterval = 200;

    // Various features controlled by feature flags.

    /**
     * Type of the feature flag. It used to be a uint8_t but got changed to a
     * uint16_t when more than 8 flags were needed. Let's define a typedef to
     * make it easier to change this in the future.
     */
    typedef uint16_t FeatureFlagType;

    /** Flag to activate the AceButton::kEventClicked event. */
    static const FeatureFlagType kFeatureClick = 0x01;

    /**
     * Flag to activate the AceButton::kEventDoubleClicked event.
     * Activating this automatically activates kEventClicked since there is
     * no double-click without a click.
     */
    static const FeatureFlagType kFeatureDoubleClick = 0x02;

    /** Flag to activate the AceButton::kEventLongPress event. */
    static const FeatureFlagType kFeatureLongPress = 0x04;

    /** Flag to activate the AceButton::kEventRepeatPressed event. */
    static const FeatureFlagType kFeatureRepeatPress = 0x08;

    /** Flag to suppress kEventReleased after a kEventClicked. */
    static const FeatureFlagType kFeatureSuppressAfterClick = 0x10;

    /**
     * Flag to suppress kEventReleased after a kEventDoubleClicked. A
     * kEventClicked is _always_ suppressed after a kEventDoubleClicked to
     * prevent generating 2 double-clicks if the user performed a triple-click.
     */
    static const FeatureFlagType kFeatureSuppressAfterDoubleClick = 0x20;

    /** Flag to suppress kEventReleased after a kEventLongPressed. */
    static const FeatureFlagType kFeatureSuppressAfterLongPress = 0x40;

    /** Flag to suppress kEventReleased after a kEventRepeatPressed. */
    static const FeatureFlagType kFeatureSuppressAfterRepeatPress = 0x80;

    /**
     * Flag to suppress kEventClicked before a kEventDoubleClicked. This causes
     * the notification of a kEventClicked to be delayed until the delay time of
     * getDoubleClickDelay() has passed so that we can determine if there was a
     * kEventDoubleClicked.
     */
    static const FeatureFlagType kFeatureSuppressClickBeforeDoubleClick = 0x100;

    /**
     * Convenience flag to suppress all suppressions. Calling
     * setFeature(kFeatureSuppressAll) suppresses all and
     * clearFeature(kFeatureSuppressAll) clears all suppression. Note however
     * that isFeature(kFeatureSuppressAll) currently means "is ANY feature
     * enabled?" not "are ALL features enabled?".
     */
    static const FeatureFlagType kFeatureSuppressAll =
        (kFeatureSuppressAfterClick |
        kFeatureSuppressAfterDoubleClick |
        kFeatureSuppressAfterLongPress |
        kFeatureSuppressAfterRepeatPress |
        kFeatureSuppressClickBeforeDoubleClick);

    /**
     * The event handler signature.
     *
     * @param button pointer to the AceButton that generated the event
     * @param eventType the event type which trigger the call
     * @param buttonState the state of the button that triggered the event
     */
    typedef void (*EventHandler)(AceButton* button, uint8_t eventType,
        uint8_t buttonState);

    /** Constructor. */
    ButtonConfig() {}

    // These configuration methods are virtual so that they can be overriddden.
    // Subclasses can override at the class-level by defining a new virtual
    // function in the subclass, or by defining an instance variable and storing
    // the parameter with each instance of this class.

    /** Milliseconds to wait for debouncing. */
    uint16_t getDebounceDelay() { return mDebounceDelay; }

    /** Milliseconds to wait for a possible click. */
    uint16_t getClickDelay() { return mClickDelay; }

    /**
     * Milliseconds between the first and second click to register as a
     * double-click.
     */
    uint16_t getDoubleClickDelay() {
      return mDoubleClickDelay;
    }

    /** Milliseconds for a long press event. */
    uint16_t getLongPressDelay() {
      return mLongPressDelay;
    }

    /**
     * Milliseconds that a button needs to be Pressed down before the start of
     * the sequence of RepeatPressed events. The first event will fire as soon
     * as this delay has passed. Subsequent events will fire after
     * getRepeatPressInterval() time.
     */
    uint16_t getRepeatPressDelay() {
      return mRepeatPressDelay;
    }

    /**
     * Milliseconds between two successive RepeatPressed events.
     */
    uint16_t getRepeatPressInterval() {
      return mRepeatPressInterval;
    }

    /** Set the debounceDelay. */
    void setDebounceDelay(uint16_t debounceDelay) {
      mDebounceDelay = debounceDelay;
    }

    /** Set the clickDelay. */
    void setClickDelay(uint16_t clickDelay) {
      mClickDelay = clickDelay;
    }

    /** Set the doubleClickDelay. */
    void setDoubleClickDelay(uint16_t doubleClickDelay) {
      mDoubleClickDelay = doubleClickDelay;
    }

    /** Set the longPressDelay. */
    void setLongPressDelay(uint16_t longPressDelay) {
      mLongPressDelay = longPressDelay;
    }

    /** Set the repeatPressDelay. */
    void setRepeatPressDelay(uint16_t repeatPressDelay) {
      mRepeatPressDelay = repeatPressDelay;
    }

    /** Set the repeatPressInterval. */
    void setRepeatPressInterval(uint16_t repeatPressInterval) {
      mRepeatPressInterval = repeatPressInterval;
    }

    // The getClock() and readButton() are external dependencies that normally
    // would be injected using separate classes, but in the interest of saving
    // RAM in an embedded environment, we expose them in this class instead.

    /**
     * Return the milliseconds of the internal clock. Override to use something
     * other than millis(). The return type is 'unsigned long' instead of
     * uint16_t because that's the return type of millis().
     */
    virtual unsigned long getClock() { return millis(); }

    /**
     * Return the microseconds of the internal clock. Can be overridden
     * for testing purposes.
     */
    virtual unsigned long getClockMicros() { return micros(); }

    /**
     * Return the HIGH or LOW state of the button. Override to use something
     * other than digitalRead(). The return type is 'int' instead of uint16_t
     * because that's the return type of digitalRead().
     */
    virtual int readButton(uint8_t pin) {
      return digitalRead(pin);
    }

    // These methods return the various feature flags that control the
    // functionality of the AceButton.

    /** Check if the given features are enabled. */
    bool isFeature(FeatureFlagType features) ACE_BUTTON_INLINE {
      return mFeatureFlags & features;
    }

    /** Enable the given features. */
    void setFeature(FeatureFlagType features) ACE_BUTTON_INLINE {
      mFeatureFlags |= features;
    }

    /** Disable the given features. */
    void clearFeature(FeatureFlagType features) ACE_BUTTON_INLINE {
      mFeatureFlags &= ~features;
    }

    // EventHandler

    /** Return the eventHandler. */
    EventHandler getEventHandler() ACE_BUTTON_INLINE {
      return mEventHandler;
    }

    /**
     * Install the event handler. The event handler must be defined for the
     * AceButton to be useful.
     */
    void setEventHandler(EventHandler eventHandler) ACE_BUTTON_INLINE {
      mEventHandler = eventHandler;
    }

    // TimingStats

    /** Set the timing stats object. The timingStats can be nullptr. */
    void setTimingStats(TimingStats* timingStats) {
      mTimingStats = timingStats;
    }

    /** Get the timing stats. Can return nullptr. */
    TimingStats* getTimingStats() { return mTimingStats; }

    /**
     * Return a pointer to the singleton instance of the ButtonConfig
     * which is attached to all AceButton instances by default.
     */
    static ButtonConfig* getSystemButtonConfig() ACE_BUTTON_INLINE {
      return &sSystemButtonConfig;
    }

  protected:
    /**
     * Initialize to its pristine state, except for the EventHandler which is
     * unchanged. This is intended mostly for testing purposes.
     */
    virtual void init() {
      mFeatureFlags = 0;
      mTimingStats = nullptr;
    }

  private:
    /**
     * A single static instance of ButtonConfig provided by default to all
     * AceButton instances.
     */
    static ButtonConfig sSystemButtonConfig;

    // Disable copy-constructor and assignment operator
    ButtonConfig(const ButtonConfig&) = delete;
    ButtonConfig& operator=(const ButtonConfig&) = delete;

    /** The event handler for all buttons associated with this ButtonConfig. */
    EventHandler mEventHandler = nullptr;

    /** A bit mask flag that activates certain features. */
    FeatureFlagType mFeatureFlags = 0;

    /** The timing stats object. */
    TimingStats* mTimingStats = nullptr;

    uint16_t mDebounceDelay = kDebounceDelay;
    uint16_t mClickDelay = kClickDelay;
    uint16_t mDoubleClickDelay = kDoubleClickDelay;
    uint16_t mLongPressDelay = kLongPressDelay;
    uint16_t mRepeatPressDelay = kRepeatPressDelay;
    uint16_t mRepeatPressInterval = kRepeatPressInterval;
};

}
#endif
