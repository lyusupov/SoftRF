# AceButtonTest

These unit tests depend on
[AUnit](https://github.com/bxparks/AUnit), which is a derivative of
[ArduinoUnit](https://github.com/mmurdoch/arduinounit). As explained
in [issue #70](https://github.com/mmurdoch/arduinounit/issues/70),
the original ArduinoUnit consumes too much flash memory. The `AceButtonTest.ino`
sketch generates 53kB of flash with ArduinoUnit, which no longer fits
inside the 32kB space of an Arduino Nano (ATmega328P). AUnit
decreases the flash size for `AceButtonTest` by 66%, consuming only
18kB.

I tried splitting the tests into 6 smaller sketches, which worked for a while.
But when I started testing the library on multiple platforms (e.g. Arduino,
Teensy, ESP8266), it became too cumbersome to repeatedly run 6 sketches across
these platforms.
