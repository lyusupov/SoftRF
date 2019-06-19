# Bluetooth Serial library for ESP32

This code is based on [Espressif Bluetooth Serial library](https://github.com/espressif/arduino-esp32/tree/master/libraries/BluetoothSerial).

It has been modified so that it allows using ESP32 as an initiator or client of a SPP Bluetooth Classic device. The example uses a BT GPS to get possitioning information.

All changes are been made so that it is compatible with original implementation on Espressif repository.

In order to activate classic mode you only need to call `begin()` with BT address or BT name.

If name is used it searches for device MAC address using GAP.

If address is used, it directly connects the device. This has priority over name, so if address is used name is ignored.

No security mechanism is implemented so if your device requires a PIN it will not work without adding that feature to client part.