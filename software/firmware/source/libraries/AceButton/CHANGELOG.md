# Changelog

* Unreleased
* 1.3.3 (2019-03-10)
    * Add blurb about using `pinMode()` and button wiring configurations in
      README.md based on feedback from
      [Issue #19](https://github.com/bxparks/AceButton/issues/19).
    * Add `AceButton::isPressedRaw()` to determine the state of the button upon
      booting the device.
      (See [Issue #21](https://github.com/bxparks/AceButton/issues/21))
* 1.3.2 (2018-12-30)
    * Year-end maintenance release. No functional change.
    * Fix minor spelling and grammar mistakes in README.md.
    * Remove `virtual` when using `override` per C++ core style guide.
    * Update auniter.ini for compatibility with latest ESP32 and Sparkfun cores.
* 1.3.1 (2018-09-30)
    * Fix botched release on GitHub. Same as v1.3.
* 1.3 (2018-09-30)
    * Merge `AdjustableButtonConfig` into `ButtonConfig` and deprecated
      `AdjustableButtonConfig`. See
      [Issue #13](https://github.com/bxparks/AceButton/issues/13) for
      benchmarks which show that the theoretical increase of static RAM
      consumption does not often happen in practice because of compiler
      optimization.
    * Reduce default value of `getDebounceDelay()` from 50 ms to 20 ms
      to improve perceived responsiveness of buttons when they are rapidly
      pressed on and off. See
      [Issue #14](https://github.com/bxparks/AceButton/issues/14)
      for details.
    * Update `tests/auniter.ini` and `Jenkinsfile` for compatibility with
      AUniter v1.7. Add `CapacitiveSensor` to the exclude list for
      `env:esp8266` and `env:esp32` because it doesn't compile under those
      environments.
    * Remove leading zero in `ACE_BUTTON_VERSION` because that I forgot that it
      means an octal number.
* 1.2 (2018-08-14)
    * Add `AceButton(ButtonConfig*)` constructor to support constructor
      dependency injection. Recommended over `setButtonConfig()`.
      Should be 100% backwards compatible.
    * Add examples/CapacitiveSwitch program to demonstrate integration
      with CapacitiveSensor library to support capacitive switches.
    * Add continuous integration using AUniter/Jenkins.
* 1.1.1 (2018-06-21)
    * Fix compiler warnings about unused parameters in eventHandler callback.
    * Move AutoBenchmark results into AutoBenchmark/README.md.
    * Update various links in AceButton/README.md.
    * No functional or API change.
* 1.1.0 (2018-05-03)
    * Change to MIT License.
    * Add instrumentation of AceButton.check() using TimingStats to measure
      performance. (Fixes #9)
    * Add `examples/AutoBenchmark` sketch to auto-generate benchmarks for
      various microcontrollers.
    * Verify that library and examples compile, and the tests pass for ESP32.
    * Add a third method for distinguishing Clicked from DoubleClicked using
      both techniques described in 1.0.6.
* 1.0.6 (2018-03-25)
    * Add `kFeatureSuppressClickBeforeDoubleClick` flag to suppress
      Clicked event before a DoubleClicked event, at the cost of slower
      response time of the Clicked event. Added 2 more examples to demonstrate 2
      methods to distinguish between a Clicked and DoubleClicked.
    * Publish [doxygen docs](https://bxparks.github.io/AceButton/html/)
      on GitHub Pages.
* 1.0.5 (2018-03-17)
    * Migrate unit tests to [AUnit](https://github.com/bxparks/AUnit).
    * Fix various typos in README.md.
* 1.0.4 (2018-03-07)
    * Support ESP8266.
        * Split `loop()` in `Stopwatch.ino` into inner and outer loops, to
          allow `loop()` to return periodically.
        * Perform manual testing, since ArduinoUnit does not work on ESP8266.
    * Optimize `check()` so that `checkOrphanedClick()` is called only when
      needed.
    * README.md: add  benchmark numbers for ESP8266, fix typos.
    * Fix various compiler warnings about unused variables.
* 1.0.3 (2018-02-13)
    * Make library work on Teensy LC and 3.2.
        * Fix `elapsedTime` expression that breaks on 32-bit processors
          (whose `int` is 4 bytes instead of 2).
* 1.0.2 (2018-02-07)
    * Add documentation and unit tests for `AdjustableButtonConfig`.
    * Reduce `orphanClickDelay` to 1X `getDoubleClickDelay()` instead of 10X.
* 1.0.1 (2018-02-03)
    * Fix typo in 'library.properties'.
* 1.0.0 (2018-02-03)
    * Initial public release.
