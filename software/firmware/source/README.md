# SoftRF firmware build instructions

[NodeMCU](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#nodemcu)<br>
[ESP32](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source#esp32)<br>

## NodeMCU

1. Follow [these official instructions](https://github.com/esp8266/Arduino#installing-with-boards-manager)
  to install Arduino IDE and latest Arduino ESP8266 Core
2. Become familiar with IDE and **NodeMCU** by building and uploading of a basic **Blink** sketch:<br>

    _File_ -> _Examples_ -> _ESP8266_ -> _Blink_ <br>

    then<br>

    _Sketch_ -> _Upload_

3. When you are done with the lesson, close your **Arduino** application
4. open ``<My Documents>`` (Windows) or ``<Home>`` (Linux) directory
5. create **Arduino** sub-directory
6. transfer full content of **SoftRF** and **libraries** GitHub folders into the sub-directory:

    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) **-->** ``<My Documents>``/Arduino/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<My Documents>``/Arduino/libraries <br>

7. start **Arduino** application again
8. open **SoftRF** sketch from _File_ -> _Open_ menu
9. Select _Tools_ -> _Board_ -> _NodeMCU_ _1.0_
10. in _Tools_ -> _lwIP Variant_  -> _v2 Higher Bandwidth_
11. _Sketch_ -> _Upload_

## ESP32

1. Follow [these official instructions](https://github.com/espressif/arduino-esp32#installation-instructions)
  to install Arduino IDE and latest Arduino ESP32 Core
2. Become familiar with IDE and **DoIt ESP32 DevKit** by building and uploading of a basic **Blink** sketch:<br>
```
int ledPin = 2;

void setup()
{
    pinMode(ledPin, OUTPUT);
    Serial.begin(115200);
}

void loop()
{
    Serial.println("Hello, world!");
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
}
```

3. When you are done with the lesson, close your **Arduino** application
4. open ``<My Documents>`` (Windows) or ``<Home>`` (Linux) directory
5. create **Arduino** sub-directory
6. transfer full content of **SoftRF** and **libraries** GitHub folders into the sub-directory:

    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) **-->** ``<My Documents>``/Arduino/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<My Documents>``/Arduino/libraries <br>

7. take **WebServer_tng** library from [this location](https://github.com/bbx10/WebServer_tng)<br>
    and deploy into ``<My Documents>``/Arduino/hardware/espressif/esp32/libraries
8. take **libbt.a** binary from [this location](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/binaries/ESP32/misc)<br>
    and overwrite existing one in ``<My Documents>``/Arduino/hardware/espressif/esp32/tools/sdk/lib
9. start **Arduino** application again
10. open **SoftRF** sketch from _File_ -> _Open_ menu
11. Select _Tools_ -> _Board_ ->  _ESP32_ _Dev_ _Module_
12. Select _Tools_ -> _Flash_ _Mode_ ->  _DIO_
13. Select _Tools_ -> _Flash_ _Size_ ->  _4MB_
14. Select _Tools_ -> _Partition_ _Scheme_ ->  _Minimal_ _SPIFFS_
15. Select _Tools_ -> _Flash_ _Frequency_ ->  _80MHz_
16. _Sketch_ -> _Upload_
