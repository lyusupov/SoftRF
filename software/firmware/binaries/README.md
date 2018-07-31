# First-time firmware installation procedure

- [NodeMCU](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#nodemcu) (ESP8266)
- [ESP32](https://github.com/lyusupov/SoftRF/blob/master/software/firmware/binaries/README.md#esp32)

## NodeMCU

### Select NodeMCU COM port
![](https://github.com/lyusupov/SoftRF/blob/master/documents/images/NodeMCU-Flasher-1.GIF)



### Select firmware file
![](https://github.com/lyusupov/SoftRF/blob/master/documents/images/NodeMCU-Flasher-2.GIF)



### Start flashing cycle
![](https://github.com/lyusupov/SoftRF/blob/master/documents/images/NodeMCU-Flasher-3.GIF)



### Wait for completion
![](https://github.com/lyusupov/SoftRF/blob/master/documents/images/NodeMCU-Flasher-4.GIF)

## ESP32

1. Take ESP32 flash download tool from this location: https://www.espressif.com/en/support/download/other-tools <br>
You might also need to install a [driver for the CP210X USB to UART bridge from Silicon Labs](https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers) prior to first use of the ESP32 tool.
2. Select COM port, enter partition files and addresses, select options.<br>

   Here is an example:<br>

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/ESP32-Flasher-1.JPG)



3. Press **START** button and wait for completion.

For some boards you may need to push **BOOT** button in order to activate flash download mode.<br>
"Stock" modules may also require to apply full flash memory erase (use **ERASE** UI "button") prior to first flashing with SoftRF's firmware.    
