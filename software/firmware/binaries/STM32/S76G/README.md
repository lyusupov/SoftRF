# Firmware files for AcSiP S7xG

This folder contains firmware binaries for S76G.<br>
AcSiP 'stock' firmware is avilable in _factory_ sub-folder.

SoftRF files are available both in **.bin** and **.dfu** formats.<br>
Although these files have been built from the same source code, the build options were different:
* **.bin** is mostly targeted for "TTGO T-Watch LoRa+GPS bottom board" which primary data output interface is UART1 <sub>(on PA9/PA10 pins)</sub> ;
* **.dfu** is targeted for "LilyGO&#174; & SoftRF T-Motion" USB dongle. It's primary data output interface is USB CDC <sub>(on PA11/PA12 pins)</sub>.<br>

Firmware uploading method is also different:
* **.bin** files are typically used by Serial or SWD flash memory loaders ;
* **.dfu** - for use with USB flashing tools.

For example of SWD method - please, read [this page](https://github.com/lyusupov/SoftRF/wiki/S76G-flashing-and-test-instructions).<br>
For DFU method - open [this link](https://github.com/lyusupov/SoftRF/wiki/AcSiP-S7xG-flashing-instructions).
