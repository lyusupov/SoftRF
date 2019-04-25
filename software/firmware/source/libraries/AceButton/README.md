# AceButton

An adjustable, compact, event-driven button library for Arduino platforms.

Version: 1.3.3 (2019-03-10)

[![AUniter Jenkins Badge](https://us-central1-xparks2018.cloudfunctions.net/badge?project=AceButton)](https://github.com/bxparks/AUniter)

## Summary

This library provides classes which accept inputs from a mechanical button
connected to a digital input pin on the Arduino. The library should be able to
handle momentary buttons, maintained buttons, and switches, but it was designed
primarily for momentary buttons.

The library is called the ACE Button Library (or AceButton Library) because:

* many configurations of the button are **adjustable**, either at compile-time
  or run-time
* the library is optimized to create **compact** objects which take up
  a minimal amount of static memory
* the library detects changes in the button state and sends **events** to
  a user-defined `EventHandler` callback function

Most of the features of the library can be accessed through 2 classes and
1 callback function:

* `AceButton` (class)
* `ButtonConfig` (class)
* `EventHandler` (typedef)

The `AceButton` class contains the logic for debouncing and determining if a
particular event has occurred. The `ButtonConfig` class holds various timing
parameters, the event handler, code for reading the button, and code for
getting the internal clock. The `EventHandler` is a user-defined callback
function with a specific signature which is registered with the `ButtonConfig`
object. When the library detects interesting events, the callback function is
called by the library, allowing the client code to handle the event.

The supported events are:

* `AceButton::kEventPressed`
* `AceButton::kEventReleased`
* `AceButton::kEventClicked`
* `AceButton::kEventDoubleClicked`
* `AceButton::kEventLongPressed`
* `AceButton::kEventRepeatPressed`

(TripleClicked is not supported but can be easily added to the library if
requested.)

### Features

Here are the high-level features of the AceButton library:

* debounces the mechanical contact
* supports both pull-up and pull-down wiring
* event-driven through a user-defined `EventHandler` callback funcition
* supports 6 event types:
    * Pressed
    * Released
    * Clicked
    * DoubleClicked
    * LongPressed
    * RepeatPressed
* can distinguish between Clicked and DoubleClicked
* adjustable configurations at runtime or compile-time
    * timing parameters
    * `digitalRead()` button read function can be overridden
    * `millis()` clock function can be overridden
* small memory footprint
    * each `AceButton` consumes 14 bytes
    * each `ButtonConfig` consumes 20 bytes
    * one System `ButtonConfig` instance created automatically by the library
* thoroughly unit tested using [AUnit](https://github.com/bxparks/AUnit)
* properly handles reboots while the button is pressed
* properly handles orphaned clicks, to prevent spurious double-clicks
* only 13-15 microseconds (on 16MHz ATmega328P) per polling call to `AceButton::check()`
* can be instrumented to extract profiling numbers
* tested on Arduino AVR (UNO, Nano, etc), Teensy ARM (LC
  and 3.2), ESP8266 and ESP32 platforms

Compared to other Arduino button libraries, I think the unique or exceptional
features of the AceButton library are:

* many supported event types (e.g. LongPressed and RepeatPressed)
* able to distinguish between Clicked and DoubleClicked
* small memory usage
* thorough unit testing
* proper handling of orphaned clicks
* proper handling of a reboot while button is pressed

### Non-goals

An Arduino UNO or Nano has 16 times more flash memory (32KB) than static memory
(2KB), so the library is optimized to minimize the static memory usage. The
AceButton library is not optimized to create a small program size (i.e. flash
memory), or for small CPU cycles (i.e. high execution speed). I assumed that if
you are seriously optimizing for program size or CPU cycles, you will probably
want to write everything yourself from scratch.

That said, the [examples/AutoBenchmark](examples/AutoBenchmark) program
shows that `AceButton::check()` takes between 13-15 microseconds on a 16MHz
ATmega328P chip on average. Hopefully that is fast enough for the vast
majority of people.

### HelloButton

Here is a simple program (see `examples/HelloButton.ino`) which controls
the builtin LED on the Arduino board using a momentary button connected
to PIN 2.

```C++
#include <AceButton.h>
using namespace ace_button;

const int BUTTON_PIN = 2;
const int LED_ON = HIGH;
const int LED_OFF = LOW;

AceButton button(BUTTON_PIN);

void handleEvent(AceButton*, uint8_t, uint8_t);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
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
      digitalWrite(LED_BUILTIN, LED_ON);
      break;
    case AceButton::kEventReleased:
      digitalWrite(LED_BUILTIN, LED_OFF);
      break;
  }
}
```

(The `button` and `buttonState` parameters are commented out to avoid an `unused
parameter` warning from the compiler. We can't remove the parameters completely
because the method signature is defined by the `EventHandler` typedef.)

## Installation

The latest stable release is available in the Arduino IDE Library Manager.
Search for "AceButton". Click install.

The development version can be installed by cloning the
[GitHub repository](https://github.com/bxparks/AceButton), checking out the
`develop` branch, then manually copying over the contents to the `./libraries`
directory used by the Arduino IDE. (The result is a directory named
`./libraries/AceButton`.) The `master` branch contains the stable release.

### Source Code

The source files are organized as follows:
* `src/AceButton.h` - main header file
* `src/ace_button/` - all implementation files
* `src/ace_button/testing/` - internal testing files
* `tests/` - unit tests which require [AUnit](https://github.com/bxparks/AUnit)
* `examples/` - example sketches

### Docs

Besides this README.md file, the [docs/](docs/) directory contains the
[Doxygen docs published on GitHub Pages](https://bxparks.github.io/AceButton/html/).
It can help you navigate an unfamiliar code base.

### Examples

The following example sketches are provided:

* [HelloButton.ino](examples/HelloButton)
    * minimal program that reads a switch and control the built-in LED
* [SingleButton.ino](examples/SingleButton)
    * controls a single button wired with a pull-up resistor
    * prints out a status line for every supported event
* [SingleButtonPullDown.ino](examples/SingleButtonPullDown)
    * same as SingleButton.ino but with an external pull-down resistor
* [Stopwatch.ino](examples/Stopwatch)
    * measures the speed of `AceButton:check()` with a start/stop/reset button
    * uses `kFeatureLongPress`
* [TunerButtons.ino](examples/TunerButtons)
    * implements 5 radio buttons (tune-up, tune-down, and 3 presets)
    * shows multiple `ButtonConfig` instances
    * shows multiple `EventHandler`s
    * shows an example of how to use `getId()`
    * uses `kFeatureLongPress`, `kFeatureRepeatPress`,
      `kFeatureSuppressAfterLongPress`, and `kFeatureSuppressAfterRepeatPress`
* [ClickVersusDoubleClickUsingReleased.ino](examples/ClickVersusDoubleClickUsingReleased)
    * a way to distinguish between a `kEventClicked` from a
      `kEventDoubleClicked` using a `kEventReleased` instead
* [ClickVersusDoubleClickUsingSuppression.ino](examples/ClickVersusDoubleClickUsingSuppression)
    * another way to dstinguish between a `kEventClicked` from a
      `kEventDoubleClicked` using the `kFeatureSuppressClickBeforeDoubleClick`
      flag at the cost of increasing the response time of the `kEventClicked`
      event
* [ClickVersusDoubleClickUsingBoth.ino](examples/ClickVersusDoubleClickUsingBoth)
    * an example that combines both the "UsingPressed" and "UsingSuppression"
      techniques
* [CapacitiveButton](examples/CapacitiveButton)
    * reads a capacitive button using the
      [CapacitiveSensor](https://github.com/PaulStoffregen/CapacitiveSensor)
      library
* [AutoBenchmark.ino](examples/AutoBenchmark)
    * generates the timing stats (min/average/max) for the `AceButton::check()`
      method for various types of events (idle, press/release, click,
      double-click, and long-press)

## Usage

There are 2 classes and one typedef that a user will normally interact with:

* `AceButton` (class)
* `ButtonConfig` (class)
* `EventHandler` (typedef)

We explain how to use these below.

### Include Header and Use Namespace

Only a single header file `AceButton.h` is required to use this library.
To prevent name clashes with other libraries that the calling code may use, all
classes are defined in the `ace_button` namespace. To use the code without
prepending the `ace_button::` prefix, use the `using` directive:

```C++
#include <AceButton.h>
using namespace ace_button;
```

If you are dependent on just `AceButton`, the following might be sufficient:

```C++
#include <AceButton.h>
using ace_button::AceButton;
```

### Pin Wiring and Initialization

An Arduino microcontroller pin can be in an `OUTPUT` mode, an `INPUT` mode, or
an `INPUT_PULLUP` mode. This mode is controlled by the `pinMode()` method.

By default upon boot, the pin is set to the `INPUT` mode. However, this `INPUT`
mode puts the pin into a high impedance state, which means that if there is no
wire connected to the pin, the voltage on the pin is indeterminant. When the
input pin is read (using `digitalRead()`), the boolean value will be a random
value. If you are using the pin in `INPUT` mode, you *must* connect an external
pull-up resistor (connected to Vcc) or pull-down resistor (connected to ground)
so that the voltage level of the pin is defined when there is nothing connected
to the pin (i.e. when the button is not pressed).

The `INPUT_PULLUP` mode is a special `INPUT` mode which tells the
microcontroller to connect an internal pull-up resistor to the pin. It is
activated by calling `pintMode(pin, INPUT_PULLUP)` on the given `pin`. This mode
is very convenient because it eliminates the external resistor, making the
wiring simpler.

The AceButton library itself does *not* call the `pinMode()` function. The
calling application is responsible for calling `pinMode()`. Normally, this
happens in the global `setup()` method but the call can happen somewhere else if
the application requires it. The reason for decoupling the hardware
configuration from the AceButton library is mostly because the library does not
actually care about the specific hardware wiring of the button. It does not care
whether an external resistor is used, or the internal resistor is used. It only
cares about whether the resistor is a pull-up or a pull-down.

See https://www.arduino.cc/en/Tutorial/DigitalPins for additional information
about the I/O pins on an Arduino.

### AceButton Class

Each physical button will be handled by an instance of `AceButton`. At a
minimum, the instance needs to be told the pin number of the button. This can
be done through the constructor:

```C++
const uint8_t BUTTON_PIN = 2;

AceButton button(BUTTON_PIN);

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  ...
}
```

Or we can use the `init()` method in the `setup()`:

```C++
AceButton button;

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  button.init(BUTTON_PIN);
  ...
}
```

Both the constructor and the `init()` function take 3 optional parameters:
```C++
AceButton(uint8_t pin = 0, uint8_t defaultReleasedState = HIGH, uint8_t id = 0);

void init(uint8_t pin = 0, uint8_t defaultReleasedState = HIGH, uint8_t id = 0);
```

* `pin`: the I/O pin number assigned to the button
* `defaultReleasedState`: the logical value of the button when it is in its
  default "released" state (`HIGH` using a pull-up resistor,
  `LOW` for a pull-down pull-down resistor)
* `id`: an optional, user-defined identifier for the the button,
  for example, an index into an array with additional information

The `pin` must be defined either through the constructor or the `init()` method.
But the other two parameters may be optional in many cases.

Finally, the `AceButton::check()` method should be called from the `loop()`
method periodically. Roughly speaking, this should be about 4 times faster than
the value of `getDebounceDelay()` so that the various event detection logic can
work properly. (If the debounce delay is 20 ms, `AceButton::check()` should be
called every 5 ms or faster.)

```C++
void loop() {
  ...
  button.check();
  ...
}
```

### ButtonConfig Class

The core concept of the AceButton library is the separation of the
button (`AceButton`) from its configuration (`ButtonConfig`).

* The `AceButton` class has the logic for debouncing and detecting the various
  events (Pressed, Released, etc), and the various bookkeeping variables
  needed to implement the logic. These variables are associated with the
  specific instance of that `AceButton`.
* The `ButtonConfig` class has the various timing parameters which control
  how much time is needed to detect certain events. This class also has the
  ability to override the default methods for reading the pin (`readButton()`)
  and the clock (`getClock()`). This ability allows unit tests to be written.

The `ButtonConfig` can be created and assigned to one or more `AceButton`
instances using dependency injection through the `AceButton(ButtonConfig*)`
constructor. If this constructor is used, then the `AceButton::init()` method
must be used to set the pin number of the button. For example:

```C++
const uint8_t PIN1 = 2;
const uint8_t PIN2 = 4;

ButtonConfig buttonConfig;
AceButton button1(&buttonConfig);
AceButton button2(&buttonConfig);

void setup() {
  pinMode(PIN1, INPUT_PULLUP);
  button1.init(PIN1);

  pinMode(PIN2, INPUT_PULLUP);
  button2.init(PIN2);
  ...
}
```

Another way to inject the `ButtonConfig` dependency is to use the
`AceButton::setButtonConfig()` method but it is recommended that you use the
constructor instead because the dependency is easier to follow.

#### System ButtonConfig

A single instance of `ButtonConfig` called the "System ButtonConfig" is
automatically created by the library at startup. By default, all instances of
`AceButton` are automatically assigned to this singleton instance. We explain in
the _Single Button Simplifications_ section below how this simplifies the code
needed to handle a single button.

#### Configuring the EventHandler

The `ButtonConfig` class provides a number of methods which are mostly
used internally by the `AceButton` class. The one method which is expected
to be used by the calling client code is `setEventHandler()` which
assigns the user-defined `EventHandler` callback function to the `ButtonConfig`
instance. This is explained in more detail below in the
_EventHandler Callback_ section.

#### Timing Parameters

Here are the methods to retrieve the timing parameters:

* `uint16_t getDebounceDelay();` (default: 20 ms)
* `uint16_t getClickDelay();` (default: 200 ms)
* `uint16_t getDoubleClickDelay();` (default: 400 ms)
* `uint16_t getLongPressDelay();` (default: 1000 ms)
* `uint16_t getRepeatPressDelay();` (default: 1000 ms)
* `uint16_t getRepeatPressInterval();` (default: 200 ms)

The default values of each timing parameter can be changed at run-time using
the following methods:

* `void setDebounceDelay(uint16_t debounceDelay);`
* `void setClickDelay(uint16_t clickDelay);`
* `void setDoubleClickDelay(uint16_t doubleClickDelay);`
* `void setLongPressDelay(uint16_t longPressDelay);`
* `void setRepeatPressDelay(uint16_t repeatPressDelay);`
* `void setRepeatPressInterval(uint16_t repeatPressInterval);`

#### Hardware Dependencies

The `ButtonConfig` class has 2 methods which provide hooks to its external
hardware dependencies:

* `virtual unsigned long getClock();`
* `virtual int readButton(uint8_t pin);`

By default these are mapped to the underlying Arduino system functions respectively:

* `millis()`
* `digitalRead()`

Unit tests are possible because these methods are `virtual` and the hardware
dependencies can be swapped out with fake ones.

#### Multiple ButtonConfig Instances

We have assumed that there is a 1-to-many relationship between a `ButtonConfig`
and the `AceButton`. In other words, multiple buttons will normally be
associated with a single configuration. Each `AceButton` has a pointer to an
instance of `ButtonConfig`. So the cost of separating the `ButtonConfig` from
`AceButton` is 2 bytes in each instance of `AceButton`. Note that this is
equivalent to adding virtual methods to `AceButton` (which would add 2 bytes),
so in terms of static RAM size, this is a wash.

The library is designed to handle multiple buttons, and it assumes that the
buttons are normally grouped together into a handful of types. For example,
consider the buttons of a car radio. It has several types of buttons:

* the tuner buttons (2, up and down)
* the preset buttons (6)
* the AM/FM band button (1)

In this example, there are 9 buttons, but only 3 instances of `ButtonConfig`
would be needed.

### EventHandler

The event handler is a callback function that gets called when the `AceButton`
class determines that an interesting event happened on the button. The
advantage of this mechanism is that all the complicated logic of determining
the various events happens inside the `AceButton` class, and the user will
normally not need to worry about the details.

#### EventHandler Signature

The event handler has the following signature:

```C++
typedef void (*EventHandler)(AceButton* button, uint8_t eventType,
    uint8_t buttonState);
```

The event handler is registered with the `ButtonConfig` object, not with the
`AceButton` object, although the convenience method
`AceButton::setEventHandler()` is provided as a pass-through to the underlying
`ButtonConfig` (see the _Single Button Simplifications_ section below):

```C++
ButtonConfig buttonConfig;

void handleEvent(AceButton* button, uint8_t eventType, uint8_t buttonState) {
  ...
}

void setup() {
  ...
  buttonConfig.setEventHandler(handleEvent);
  ...
}
```

The motivation for this design is to save static memory. If multiple buttons
are associated with a single `ButtonConfig`, then it is not necessary for every
button of that type to hold the same pointer to the `EventHandler` function. It
is only necessary to save that information once, in the `ButtonConfig` object.

**Pro Tip**: Comment out the unused parameter(s) in the `handleEvent()` method
to avoid the `unused parameter` compiler warning:
```C++
void handleEvent(AceButton* /* button */, uint8_t eventType,
    uint8_t /* buttonState */) {
  ...
}
```
The Arduino sketch compiler can get confused with the parameters commented out,
so you may need to add a forward declaration for the `handleEvent()` method
before the `setup()` method:
```C++
void handleEvent(AceButton*, uint8_t, uint8_t);
```

#### EventHandler Parameters

The `EventHandler` function receives 3 parameters from the `AceButton`:

* `aceButton`
    * pointer to the `AceButton` instance that generated this event
    * can be used to retrieve the `getPin()` or the `getId()`
* `eventType`
    * the type of this event given by the various `AceButton::kEventXxx`
      constants
* `buttonState`
    * the `HIGH` or `LOW` button state that generated this event

The `aceButton` pointer should be used only to extract information about the
button that triggered the event. It should **not** be used to modify the
button's internal variables in any way within the eventHandler. The logic in
`AceButton::check()` assumes that those internal variable are held constant,
and if they are changed by the eventHandler, unpredictable results may occur.
(I was tempted to make the `aceButton` a pointer to a `const AceButton`
but this cause too many viral changes to the code which seemed to increase
the complexity without too much benefit.)

If you are using only a single button, then you should need to check
only the `eventType`.

It is not expected that `buttonState` will be needed very often. It should be
sufficient to examine just the `eventType` to determine the action that needs
to be performed. Part of the difficulty with this parameter is that it has the
value of `LOW` or `HIGH`, but the physical interpretation of those values depends
on whether the button was wired with a pull-up or pull-down resistor. The helper
function `AceButton::isReleased(uint8_t buttonState)` is provided to make this
determination if you need it.

#### One EventHandler

Only a single `EventHandler` per `ButtonConfig` is supported. An alternative
would have been to register a separate event handler for each of the 6
`kEventXxx` events. But each callback function requires 2 bytes of memory, and
it was assumed that in most cases, the calling client code would be interested
in only a few of these event types, so it seemed wasteful to allocate 12 bytes
when most of these would be unused. If the client code really wanted separate
event handlers, it can be easily emulated by invoking them through the main
event handler:

```C++
void handleEvent(AceButton* button, uint8_t eventType, uint8_t buttonState) {
  switch (eventType) {
    case AceButton:kEventPressed:
      handleEventPressed(button, eventType, buttonState);
      break;
    case AceButton::kEventReleased:
      handleEventReleased(button, eventType, buttonState);
      break;
    ...
  }
}
```

#### EventHandler Tips

The Arduino runtime environment is single-threaded, so the `EventHandler` is
called in the middle of the `AceButton::check()` method, in the same thread as
the `check()` method. It is therefore important to write the `EventHandler`
code to run somewhat quickly, so that the delay doesn't negatively impact the
logic of the `AceButton::check()` algorithm. Since `AceButton::check()` should
run approximately every 5 ms, the user-provided `EventHandler` should run
somewhat faster than 5 ms. Given a choice, it is probably better to use the
`EventHandler` to set some flags or variables and return quickly, then do
additional processing from the `loop()` method.

Speaking of threads, the API of the AceButton Library was designed to work in a
multi-threaded environment, if that situation were to occur in the Arduino
world.

### Event Types

The supported events are defined by a list of constants in `AceButton`:

* `AceButton::kEventPressed` (always enabled)
* `AceButton::kEventReleased` (conditionally enabled)
* `AceButton::kEventClicked` (default: disabled)
* `AceButton::kEventDoubleClicked` (default: disabled)
* `AceButton::kEventLongPressed` (default: disabled)
* `AceButton::kEventRepeatPressed` (default: disabled)

These values are sent to the `EventHandler` in the `eventType` parameter.

Two of the events are enabled by default, four are disabled by default but can
be enabled by using a Feature flag described below.

### ButtonConfig Feature Flags

There are 9 flags defined in `ButtonConfig` which can
control the behavior of `AceButton` event handling:

* `ButtonConfig::kFeatureClick`
* `ButtonConfig::kFeatureDoubleClick`
* `ButtonConfig::kFeatureLongPress`
* `ButtonConfig::kFeatureRepeatPress`
* `ButtonConfig::kFeatureSuppressAfterClick`
* `ButtonConfig::kFeatureSuppressAfterDoubleClick`
* `ButtonConfig::kFeatureSuppressAfterLongPress`
* `ButtonConfig::kFeatureSuppressAfterRepeatPress`
* `ButtonConfig::kFeatureSuppressClickBeforeDoubleClick`
* `ButtonConfig::kFeatureSuppressAll`

These constants are used to set or clear the given flag:

```C++
ButtonConfig* config = button.getButtonConfig();

config->setFeature(ButtonConfig::kFeatureLongPress);

config->clearFeature(ButtonConfig::kFeatureLongPress);

if (config->isFeature(ButtonConfig::kFeatureLongPress)) {
  ...
}
```

The meaning of these flags are described below.

#### Event Activation

Of the 6 event types, 4 are disabled by default:

* `AceButton::kEventClicked`
* `AceButton::kEventDoubleClicked`
* `AceButton::kEventLongPressed`
* `AceButton::kEventRepeatPressed`

To receive these events, call `ButtonConfig::setFeature()` with the following
flags respectively:

* `ButtonConfig::kFeatureClick`
* `ButtonConfig::kFeatureDoubleClick`
* `ButtonConfig::kFeatureLongPress`
* `ButtonConfig::kFeatureRepeatPress`

To disable these events, call `ButtonConfig::clearFeature()` with one of these
flags.

Enabling `kFeatureDoubleClick` automatically enables `kFeatureClick`, because we
need to have a Clicked event before a DoubleClicked event can be detected.

It seems unlikely that both `LongPress` and `RepeatPress` events would be
useful at the same time, but both event types can be activated if you need it.

#### Event Suppression

Event types can be considered to be built up in layers, starting with the
lowest level primitive events: Pressed and Released. Higher level events are
built on top of the lower level events through various timing delays. When a
higher level event is detected, it is sometimes useful to suppress the lower
level event that was used to detect the higher level event.

For example, a Clicked event requires a Pressed event followed by a Released
event within a `ButtonConfig::getClickDelay()` milliseconds (200 ms by
default). The Pressed event is always generated. If a Clicked event is
detected, we could choose to generate both a Released event and a Clicked
event, and this is the default behavior.

However, many times, it is useful to suppress the Released event if the Clicked
event is detected. The `ButtonConfig` can be configured to suppress these lower
level events. Call the `setFeature(feature)` method passing the various
`kFeatureSuppressXxx` constants:

* `ButtonConfig::kFeatureSuppressAfterClick`
    * suppresses the Released event after a Clicked event is detected
    * also suppresses the Released event from the *first* Clicked of a
      DoubleClicked, since `kFeatureDoubleClick` automatically enables
      `kFeatureClick`
* `ButtonConfig::kFeatureSuppressAfterDoubleClick`
    * suppresses the Released event and the *second* Clicked event if a
      DoubleClicked event is detected
* `ButtonConfig::kFeatureSuppressAfterLongPress`
    * suppresses the Released event if a LongPressed event is detected
* `ButtonConfig::kFeatureSuppressAfterRepeatPress`
    * suppresses the Released event after the last RepeatPressed event
* `ButtonConfig::kFeatureSuppressAll`
    * a convenience parameter that is the equivalent of suppressing all of the
      previous events
* `ButtonConfig::kFeatureSuppressClickBeforeDoubleClick`
    * The *first* Clicked event is postponed by `getDoubleClickDelay()`
      millis until the code can determine if a DoubleClick has occurred. If so,
      then the postponed Clicked message to the `EventHandler` is suppressed.
    * See the section ___Distinguishing Between a Clicked and DoubleClicked___
      for more info.

By default, no suppression is performed.

As an example, to suppress the `Released` event after a `LongPressed` event
(this is actually often the case), you would do this:

```C++
ButtonConfig* config = button.getButtonConfig();
config->setFeature(ButtonConfig::kFeatureSuppressAfterLongPress);
```

The special convenient constant `kFeatureSuppressAll` is equivalent of using all
suppression constants:

```C++
ButtonConfig* config = button.getButtonConfig();
config->setFeature(ButtonConfig::kFeatureSuppressAll);
```

All suppressions can be cleared by using:
```C++
ButtonConfig* config = button.getButtonConfig();
config->clearFeature(ButtonConfig::kFeatureSuppressAll);
```

Note, however, that the `isFeature(ButtonConfig::kFeatureSuppressAll)` currently
means "isAnyFeature() implemented?" not "areAllFeatures() implemented?" We don't
expect `isFeature()` to be used often (or at all) for `kFeatureSuppressAll`.

### Distinguishing Between a Clicked and DoubleClicked

On a project using only a small number of buttons (due to physical limits or the
limited availability of pins), it may be desirable to distinguish between a
single Clicked event and a DoubleClicked event from a single button. This is a
challenging problem to solve because fundamentally, a DoubleClicked event *must
always* generate a Clicked event, because a Clicked event must happen before it
can become a DoubleClicked event.

Notice that on a desktop computer (running Windows, MacOS or Linux), a
double-click on a mouse always generates both a Clicked and a DoubleClicked. The
first Click selects the given desktop object (e.g. an icon or a window), and
the DoubleClick performs some action on the selected object (e.g. open the
icon, or resize the window).

The AceButton Library provides 3 solutions which may work for some projects:

**Method 1:** The `kFeatureSuppressClickBeforeDoubleClick` flag causes the first
Clicked event to be detected, but the posting of the event message (i.e. the
call to the `EventHandler`) is postponed until the state of the DoubleClicked
can be determined. If the DoubleClicked happens, then the first Clicked event
message is suppressed. If DoubleClicked does not occur, the long delayed
Clicked message is sent via the `EventHandler`.

There are two noticeable disadvantages of this method. First, the response time
of all Clicked events is delayed by about 600 ms (`kClickDelay +
kDoubleClickDelay`) whether or not the DoubleClicked event happens. Second, the
user may not be able to accurately produce a Clicked event (due to the physical
characteristics of the button, or the user's dexterity).

It may also be worth noting that only the Clicked event is postponed.
The accompanying Released event of the Clicked event is not postponed. So a
single click action (without a DoubleClick) produces the following sequence of
events to the EventHandler:

1. `kEventPressed` - at time 0ms
1. `kEventReleased` - at time 200ms
1. `kEventClicked` - at time 600ms (200ms + 400ms)

The `ButtonConfig` configuration looks like this:
```C++
ButtonConfig* buttonConfig = button.getButtonConfig();
buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
buttonConfig->setFeature(
    ButtonConfig::kFeatureSuppressClickBeforeDoubleClick);
```

See the example code at
`examples/ClickVersusDoubleClickUsingSuppression/`.

**Method 2:** A viable alternative is to use the Released event instead of the
Clicked event to distinguish it from the DoubleClicked. For this method to work,
we need to suppress the Released event after both Clicked and DoubleClicked.

The advantage of using this method is that there is no response time lag in the
handling of the Released event. To the user, there is almost no difference
between triggering on the Released event, versus triggering on the Clicked
event.

The disadvantage of this method is that the Clicked event must be be ignored
(because of the spurious Clicked event generated by the DoubleClicked). If the
user accidentally presses and releases the button to quickly, it generates a
Clicked event, which will cause the program to do nothing.

The `ButtonConfig` configuration looks like this:
```C++
ButtonConfig* buttonConfig = button.getButtonConfig();
buttonConfig->setEventHandler(handleEvent);
buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
```

See the example code at
`examples/ClickVersusDoubleClickUsingReleased/`.

**Method 3:** We could actually combine both Methods 1 and 2 so that either
Released or a delayed Click is considered to be a "Click". This may be the best
of both worlds.

The `ButtonConfig` configuration looks like this:
```C++
ButtonConfig* buttonConfig = button.getButtonConfig();
buttonConfig->setEventHandler(handleEvent);
buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
buttonConfig->setFeature(
    ButtonConfig::kFeatureSuppressClickBeforeDoubleClick);
buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
```

See the example code at
`examples/ClickVersusDoubleClickUsingBoth/`.

### Single Button Simplifications

Although the AceButton library is designed to shine for multiple buttons, you
may want to use it to handle just one button. The library provides some features
to make this simple case easy.

1. The library automatically creates one instance of `ButtonConfig`
   called a "System ButtonConfig". This System ButtonConfig can be retrieved
   using the class static method `ButtonConfig::getSystemButtonConfig()`.
1. Every instance of `AceButton` is assigned an instance of the System
   ButtonConfig by default (which can be overridden manually).
1. A convenience method allows the `EventHandler` for the System
   ButtonConfig to be set easily through `AceButton` itself, instead of having
   to get the System ButtonConfig first, then set the event handler. In other
   words, `button.setEventHandler(handleEvent)` is a synonym for
   `button.getButtonConfig()->setEventHandler(handleEvent)`.

These simplifying features allow a single button to be configured and used like
this:

```C++
AceButton button(BUTTON_PIN);

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  button.setEventHandler(handleEvent);
  ...
}

void loop() {
  button.check();
}

void handleEvent(AceButton* button, uint8_t eventType, uint8_t buttonState) {
  ...
}
```

To configure the System ButtonConfig, you may need to add something like
this to the `setup()` section:

```C++
  button.getButtonConfig()->setFeature(ButtonConfig::kFeatureLongPress);
```

### Multiple Buttons

When transitioning from a single button to multiple buttons, it's important to
remember what's happening underneath the convenience methods. The single
`AceButton` button is assigned to the System ButtonConfig that was created
automatically. When an `EventHandler` is assigned to the button, it is actually
assigned to the System ButtonConfig. All subsequent instances of `AceButton`
will also be associated with this event handler, unless another `ButtonConfig`
is explicitly assigned.

See the example sketch `TunerButtons.ino` to see how to use multiple
`ButtonConfig` instances with multiple `AceButton` instances.

### Events After Reboot

A number of edge cases occur when the microcontroller is rebooted:

* if the button is held down, should the Pressed event be triggered?
* if the button is in its natural Released state, should the Released event
  happen?
* if the button is Pressed down, and `ButtonConfig` is configured to
  support RepeatPress events, should the `kEventRepeatPressed` events
  be triggered initially?

I think most users would expect that in all these cases, the answer is no, the
microcontroller should not trigger an event until the button undergoes a
human-initiated change in state. The AceButton library implements this logic.
(It might be useful to make this configurable using a `ButtonConfig` feature
flag but that is not implemented.)

On the other hand, it is sometimes useful to perform some special action if a
button is pressed while the device is rebooted. To support this use-case, call
the `AceButton::isPressedRaw()` in the global `setup()` method (after the
button is configured). It will directly call the `digitalRead()` method
associated with the button pin and return `true` if the button is in the
Pressed state.

### Orphaned Clicks

When a Clicked event is generated, the `AceButton` class looks for a
second Clicked event within a certain time delay (default 400 ms) to
determine if the second Clicked event is actually a DoubleClicked event.

All internal timestamps in `AceButton` are stored as `uint16_t`
(i.e. an unsigned integer of 16 bits) in millisecond units. A 16-bit
unsigned counter rolls over after 65536 iterations. Therefore, if the second
Clicked event happens between (65.636 seconds, 66.036 seconds) after the first
Clicked event, a naive-logic would erroneously consider the (long-delayed)
second click as a double-click.

The `AceButton` contains code that prevents this from happening.

Note that even if the `AceButton` class uses an `unsigned long` type (a 32-bit
integer on the Arduino), the overflow problem would still occur after `2^32`
milliseconds (i.e. 49.7 days). To be strictly correct, the `AceButton` class
would still need logic to take care of orphaned Clicked events.

## Resource Consumption

Here are the sizes of the various classes on the 8-bit AVR microcontrollers
(Arduino Uno, Nano, etc):

* sizeof(AceButton): 14
* sizeof(ButtonConfig): 20

(An early version of `AceButton`, with only half of the functionality, consumed
40 bytes. It got down to 11 bytes before additional functionality increased it
to 14.)

**Program size:**

[LibrarySizeBenchmark](examples/LibrarySizeBenchmark/) was used to determine
the size of the library. For a single button, the library consumed:
* flash memory: 1100-1330 bytes
* static memory: 14-28 bytes

depending on the target board. See the README.md in the above link for more
details.

**CPU cycles:**

The profiling numbers for `AceButton::check()` can be found in
[examples/AutoBenchmark](examples/AutoBenchmark).

In summary, the average numbers for various boards are:
* Arduino Nano: 13-15 microsesconds
* Teensy 3.2: 3 microseconds
* ESP8266: 8-9 microseconds
* ESP32: 2-3 microseconds

## System Requirements

This library was developed and tested using:
* [Arduino IDE 1.8.5 - 1.8.7](https://www.arduino.cc/en/Main/Software)
* [Teensyduino 1.41](https://www.pjrc.com/teensy/td_download.html)
* [ESP8266 Arduino Core 2.4.1 - 2.4.2](https://arduino-esp8266.readthedocs.io/en/2.4.2/)
* [arduino-esp32](https://github.com/espressif/arduino-esp32)

I used MacOS 10.13.3 and Ubuntu Linux 17.10 for most of my development.

The library has been verified to work on the following hardware:

* Arduino Nano clone (16 MHz ATmega328P)
* Arduino UNO R3 clone (16 MHz ATmega328P)
* Arduino Pro Micro clone (16 MHz ATmega32U4)
* Teensy LC (48 MHz ARM Cortex-M0+)
* Teensy 3.2 (72 MHz ARM Cortex-M4)
* NodeMCU 1.0 clone (ESP-12E module, 80MHz ESP8266)
* ESP32 Dev Module (ESP-WROOM-32 module, 240MHz dual core Tensilica LX6)

## Background Motivation

There are numerous "button" libraries out there for the Arduino. Why write
another one? I wanted to add a button to an addressable strip LED controller,
which was being refreshed at 120 Hz. I had a number of requirements:

* the button needed to support a LongPress event, in addition to the simple
  Press and Release events
* the button code must not interfere with the LED refresh code which was
  updating the LEDs at 120 Hz
* well-tested, I didn't want to be hunting down random and  obscure bugs

Since the LED refresh code needed to run while the button code was waiting for
a "LongPress" delay, it seemed that the cleanest API for a button library
would use an event handler callback mechanism. This reduced the number of
candidate libraries to a handful. Of these, only a few of them supported a
LongPress event. I did not find the remaining ones flexible enough for my
button needs in the future. Finally, I knew that it was tricky to write correct
code for debouncing and detecting various events (e.g. DoubleClick, LongPress,
RepeatPress). I looked for a library that contained unit tests, and I found
none.

I decided to write my own and use the opportunity to learn how to create and
publish an Arduino library.

## Changelog

See [CHANGELOG.md](CHANGELOG.md).

## License

* Versions 1.0 to 1.0.6: [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0)
* Versions 1.1 and above: [MIT License](https://opensource.org/licenses/MIT)

I changed to the MIT License starting with version 1.1 because the MIT License
is so simple to understand. I could not be sure that I understood what the
Apache License 2.0 meant.

## Feedback and Support

If you have any questions, comments, bug reports, or feature requests, please
file a GitHub ticket or send me an email. I'd love to hear about how this
software and its documentation can be improved. Instead of forking the
repository to modify or add a feature for your own projects, let me have a
chance to incorporate the change into the main repository so that your external
dependencies are simpler and so that others can benefit. I can't promise that I
will incorporate everything, but I will give your ideas serious consideration.

## Author

Created by Brian T. Park (brian@xparks.net).
