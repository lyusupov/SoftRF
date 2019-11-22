AXP202X_Library
=====================================
- axp192 partial support, the function is not fully tested, please refer to the manual
- The `setPowerOutPut` function has forced DCDC3 to be turned on and cannot be controlled because T-Watch uses DCDC3 as the esp32 to power the main chip. If it is turned off, the hardware cannot be programmed.
  
  
TTGO invests time and resources to provide this open source code, please support TTGO and open source hardware by purchasing products from TTGO!

Written by Lewis He for TTGO. MIT license, all text above must be included in any redistribution

## Chip resource table
| CHIP     | AXP173           | AXP192           | AXP202           |
| -------- | ---------------- | ---------------- | ---------------- |
| DC1      | 0v7~3v5  /1200mA | 0v7~3v5  /1200mA | X                |
| DC2      | 0v7~2v275/1600mA | 0v7~2v275/1600mA | 0v7~2v275/1600mA |
| DC3      | X                | 0v7~3v5  /700mA  | 0v7~3v5  /1200mA |
| LDO1     | 3v3      /30mA   | 3v3      /30mA   | 3v3      /30mA   |
| LDO2     | 1v8~3v3  /200mA  | 1v8~3v3  /200mA  | 1v8~3v3  /200mA  |
| LDO3     | 1v8~3v3  /200mA  | 1v8~3v3  /200mA  | 0v7~3v3  /200mA  |
| LDO4     | 0v7~3v5  /500mA  | X                | 1v8~3v3  /200mA  |
| LDO5/IO0 | X                | 1v8~3v3  /50mA   | 1v8~3v3  /50mA   |
