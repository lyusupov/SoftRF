# LibrarySizeBenchmark

A small sketch to determine the size of the AceButton library.
First we compile it with `#define USE_ACE_BUTTON 1` to include the library.
Then we compile it with `#define USE_ACE_BUTTON 0` to exclude the library.
The difference should give us a rough idea of the size of the library.
(The compiler will produce slightly difference results for different programs.)

```
-------------+--------------+---------------+------------+
board        | AceButton    | w/o AceButton | Difference |
-------------+--------------+---------------+------------+
ATmega328P   |   2282/   55 | 1126   /  41  | 1156/14    |
ESP8266      | 249364/28076 | 248032/28048  | 1332/28    |
ESP32        | 195588/14044 | 194424/14028  | 1164/16    |
Teensy 3.2   |   9852/ 3476 |   8760/ 3460  | 1092/16    |
-------------+--------------+---------------+------------+
```
