# SoftRF firmware build instructions

## NodeMCU

1. Follow [these official instructions](https://github.com/esp8266/Arduino#installing-with-boards-manager)
  to install Arduino IDE and latest Arduino ESP8266 Core
2. Become familiar with IDE and **NodeMCU** by building and uploading of a basic **Blink** sketch:<br>

    _File_ -> _Examples_ -> _ESP8266_ -> _Blink_ <br>

    then<br>

    _Sketch_ -> _Upload_

3. When you are done with the lesson, close your Arduino application
4. open ``<My Documents>`` (Windows) or ``<Home>`` (Linux) directory
5. create **Arduino** sub-directory
6. transfer full content of **SoftRF** and **libraries** GitHub folders into the sub-directory:

    [SoftRF](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/SoftRF) **-->** ``<My Documents>``/Arduino/SoftRF <br>
    [libraries](https://github.com/lyusupov/SoftRF/tree/master/software/firmware/source/libraries) **-->** ``<My Documents>``/Arduino/libraries <br>

7. start Arduino application again
8. open SoftRF sketch from _File_ -> _Open_ menu
9. Select _Tools_ -> _Board_ -> _NodeMCU_ _1.0_
10. in _Tools_ -> _lwIP_ ... select _1.4...._
11. _Sketch_ -> _Upload_
