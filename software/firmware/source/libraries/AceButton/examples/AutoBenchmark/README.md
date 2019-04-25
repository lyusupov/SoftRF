# AutoBenchmark

This sketch measures the amount of time consumed by the `AceButton::check()`
method when processing various button events. It uses a special
`ProfilingButtonConfig` object that allows the program to inject button events
into the library. The profiling numbers come from activating the `TimingStats`
object that has been instrumented into the `AceButton::check()` method.

Note that `ProfilingButtonConfig` class generates synthetic button events,
bypassing the actual `digitalRead()` function. The `digitalRead()` function on
an Arduino AVR platform (UNO, Nano, etc) is
[known to be slow](https://forum.arduino.cc/index.php?topic=337578)
which will add to the timing values in actual usage.
The [digitalWriteFast library](https://github.com/NicksonYap/digitalWriteFast)
might be an alternative if speed is critical.

## Benchmark Results

In all of the tests, the **min** time for the "idle" case is larger than any of
the other button events. This is because when a button event occurs, the
`AceButton::checkDebounced()` method returns immediately until the debouncing
time is over which brings down the minimum time. No debouncing is done in the
"idle" case so the minimum code path takes more CPU cycles.

All times are in microseconds. The "samples" column is the number of
`TimingStats::update()` calls that were made.

### Arduino Nano

* 16MHz ATmega328P

```
------------------------+-------------+---------+
button event            | min/avg/max | samples |
------------------------+-------------+---------+
idle                    |  12/ 13/ 20 | 1934    |
press/release           |   8/ 14/ 20 | 1925    |
click                   |   8/ 14/ 24 | 1925    |
double click            |   8/ 13/ 24 | 1925    |
long press/repeat press |   8/ 15/ 24 | 1927    |
------------------------+-------------+---------+
```

### Arduino Pro Micro

* 16MHz ATmega32U4

```
------------------------+-------------+---------+
button event            | min/avg/max | samples |
------------------------+-------------+---------+
idle                    |  12/ 13/ 24 | 1935    |
press/release           |   8/ 14/ 24 | 1928    |
click                   |   8/ 13/ 24 | 1928    |
double click            |   8/ 13/ 24 | 1926    |
long press/repeat press |   8/ 15/ 28 | 1928    |
------------------------+-------------+---------+
```

### Teensy 3.2

* 96 MHz ARM Cortex-M4

```
------------------------+-------------+---------+
button event            | min/avg/max | samples |
------------------------+-------------+---------+
idle                    |   3/  3/  5 | 1985    |
press/release           |   1/  3/  6 | 1983    |
click                   |   1/  3/  6 | 1984    |
double click            |   1/  3/  6 | 1984    |
long press/repeat press |   1/  3/  6 | 1983    |
------------------------+-------------+---------+
```

### NodeMCU 1.0 clone

* 80MHz ESP8266

```
------------------------+-------------+---------+
button event            | min/avg/max | samples |
------------------------+-------------+---------+
idle                    |   7/  8/ 24 | 1922    |
press/release           |   6/  8/ 53 | 1919    |
click                   |   6/  8/ 50 | 1920    |
double click            |   6/  8/ 67 | 1910    |
long press/repeat press |   6/  9/ 60 | 1894    |
------------------------+-------------+---------+
```

The large **max** times for "double click" and "long press" seem to be
reproducible. I have not researched this but my speculation is that the system
WiFi code interrupts the `AceButton::check()` method right when the "double
click" and "long press" samples are taken, causing the extra latency.

### ESP32-01 Dev Board

* 240 MHz Tensilica LX6

```
------------------------+-------------+---------+
button event            | min/avg/max | samples |
------------------------+-------------+---------+
idle                    |   3/  3/  3 | 2002    |
press/release           |   2/  2/  8 | 2002    |
click                   |   2/  2/  7 | 2002    |
double click            |   2/  2/  4 | 2002    |
long press/repeat press |   2/  2/  4 | 2002    |
------------------------+-------------+---------+
```
