# LOG

```bash
« ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0x15 (USB_UART_CHIP_RESET),boot:0x29 (SPI_FAST_FLASH_BOOT)
Saved PC:0x420294da
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce3808,len:0x4bc
load:0x403c9700,len:0xbd8
load:0x403cc700,len:0x2a0c
entry 0x403c98d0
Initializing Sensors...
[   134][I][esp32-hal-i2c.c:75] i2cInit(): Initialising I2C Master: sda=3 scl=2 freq=100000
[   155][I][SensorBHI260AP.cpp:1083] initImpl(): BHI260/BHA260 found. Product ID read 0x89
[   165][D][SensorBHI260AP.cpp:357] uploadFirmware(): Upload Firmware ...
[   172][D][SensorBHI260AP.cpp:380] uploadFirmware(): Loading firmware into RAM.
[   179][D][SensorBHI260AP.cpp:381] uploadFirmware(): upload size = 109252
[ 10546][D][SensorBHI260AP.cpp:384] uploadFirmware(): Loading firmware into RAM Done
[ 10554][D][SensorBHI260AP.cpp:400] uploadFirmware(): Booting from RAM.
[ 10714][D][SensorBHI260AP.cpp:271] getKernelVersion(): Boot successful. Kernel version 5991.
[ 10722][I][SensorBHI260AP.cpp:1143] initImpl(): Boot successful. Kernel version 5991.
[ 10733][I][SensorBHI260AP.cpp:982] parseMetaEvent(): [META EVENT WAKE UP] Firmware initialized. Firmware version 5991
[ 10746][I][SensorBHI260AP.cpp:982] parseMetaEvent(): [META EVENT] Firmware initialized. Firmware version 5991
Initializing the sensor successfully!
Product ID     : 89
Kernel version : 5991
User version   : 5991
ROM version    : 5166
Power state    : sleeping
Host interface : I2C
Feature status : 0x4a
Boot Status : 0x38: 	No flash installed. 
	Host interface ready. 
	Firmware verification done. 
Virtual sensor list.
Sensor ID |                          Sensor Name |  ID | Ver |  Min rate |  Max rate |
----------+--------------------------------------+-----+-----+-----------+-----------|
        1 |            Accelerometer passthrough | 205 |   1 |   12.5000 |  400.0000 |
        3 |           Accelerometer uncalibrated | 203 |   1 |    1.5625 |  400.0000 |
        4 |              Accelerometer corrected | 241 |   1 |    1.5625 |  400.0000 |
        5 |                 Accelerometer offset | 209 |   1 |    1.0000 |    1.0000 |
        6 |      Accelerometer corrected wake up | 192 |   1 |    1.5625 |  400.0000 |
        7 |   Accelerometer uncalibrated wake up | 204 |   1 |    1.5625 |  400.0000 |
       10 |                Gyroscope passthrough | 207 |   1 |   25.0000 |  400.0000 |
       12 |               Gyroscope uncalibrated | 244 |   1 |    1.5625 |  400.0000 |
       13 |                  Gyroscope corrected | 243 |   1 |    1.5625 |  400.0000 |
       14 |                     Gyroscope offset | 208 |   1 |    1.0000 |    1.0000 |
       15 |                    Gyroscope wake up | 194 |   1 |    1.5625 |  400.0000 |
       16 |       Gyroscope uncalibrated wake up | 195 |   1 |    1.5625 |  400.0000 |
       28 |                       Gravity vector | 247 |   1 |    1.5625 |  400.0000 |
       29 |               Gravity vector wake up | 198 |   1 |    1.5625 |  400.0000 |
       31 |                  Linear acceleration | 246 |   1 |    1.5625 |  400.0000 |
       32 |          Linear acceleration wake up | 197 |   1 |    1.5625 |  400.0000 |
       37 |                 Game rotation vector | 252 |   1 |    1.5625 |  400.0000 |
       38 |         Game rotation vector wake up | 200 |   1 |    1.5625 |  400.0000 |
       48 |                        Tilt detector | 236 |   1 |    1.0000 |    1.0000 |
       50 |                        Step detector | 248 |   1 |    1.0000 |    1.0000 |
       52 |                         Step counter | 249 |   1 |    1.0000 |    1.0000 |
       53 |                 Step counter wake up | 231 |   1 |    0.0005 |   25.0000 |
       55 |                   Significant motion | 250 |   1 |    1.0000 |    1.0000 |
       57 |                         Wake gesture | 232 |   1 |    1.0000 |    1.0000 |
       59 |                       Glance gesture | 234 |   1 |    1.0000 |    1.0000 |
       61 |                       Pickup gesture | 233 |   1 |    1.0000 |    1.0000 |
       63 |                 Activity recognition | 235 |   1 |    1.0000 |    1.0000 |
       67 |                   Wrist tilt gesture | 162 |   1 |    1.0000 |    1.0000 |
       69 |                   Device orientation | 163 |   1 |    1.0000 |    1.0000 |
       70 |           Device orientation wake up | 164 |   1 |    1.0000 |    1.0000 |
       75 |                    Stationary detect | 161 |   1 |    1.0000 |    1.0000 |
       77 |                        Motion detect | 160 |   1 |    1.0000 |    1.0000 |
       94 |                Step detector wake up | 230 |   1 |    1.0000 |    1.0000 |
[ 11177][D][SensorBHI260AP.cpp:440] configure(): Enable Device orientation at 1.00Hz.
Initializing Sensors...
[ 11185][I][esp32-hal-i2c.c:75] i2cInit(): Initialising I2C Master: sda=11 scl=12 freq=100000
[ 11205][I][SensorBHI260AP.cpp:1083] initImpl(): BHI260/BHA260 found. Product ID read 0x89
[ 11216][D][SensorBHI260AP.cpp:357] uploadFirmware(): Upload Firmware ...
[ 11223][D][SensorBHI260AP.cpp:380] uploadFirmware(): Loading firmware into RAM.
[ 11230][D][SensorBHI260AP.cpp:381] uploadFirmware(): upload size = 109252
[ 21597][D][SensorBHI260AP.cpp:384] uploadFirmware(): Loading firmware into RAM Done
[ 21605][D][SensorBHI260AP.cpp:400] uploadFirmware(): Booting from RAM.
[ 21765][D][SensorBHI260AP.cpp:271] getKernelVersion(): Boot successful. Kernel version 5991.
[ 21773][I][SensorBHI260AP.cpp:1143] initImpl(): Boot successful. Kernel version 5991.
[ 21784][I][SensorBHI260AP.cpp:982] parseMetaEvent(): [META EVENT WAKE UP] Firmware initialized. Firmware version 5991
[ 21797][I][SensorBHI260AP.cpp:982] parseMetaEvent(): [META EVENT] Firmware initialized. Firmware version 5991
Initializing the sensor successfully!
Product ID     : 89
Kernel version : 5991
User version   : 5991
ROM version    : 5166
Power state    : sleeping
Host interface : I2C
Feature status : 0x4a
Boot Status : 0x38: 	No flash installed. 
	Host interface ready. 
	Firmware verification done. 
Virtual sensor list.
Sensor ID |                          Sensor Name |  ID | Ver |  Min rate |  Max rate |
----------+--------------------------------------+-----+-----+-----------+-----------|
        1 |            Accelerometer passthrough | 205 |   1 |   12.5000 |  400.0000 |
        3 |           Accelerometer uncalibrated | 203 |   1 |    1.5625 |  400.0000 |
        4 |              Accelerometer corrected | 241 |   1 |    1.5625 |  400.0000 |
        5 |                 Accelerometer offset | 209 |   1 |    1.0000 |    1.0000 |
        6 |      Accelerometer corrected wake up | 192 |   1 |    1.5625 |  400.0000 |
        7 |   Accelerometer uncalibrated wake up | 204 |   1 |    1.5625 |  400.0000 |
       10 |                Gyroscope passthrough | 207 |   1 |   25.0000 |  400.0000 |
       12 |               Gyroscope uncalibrated | 244 |   1 |    1.5625 |  400.0000 |
       13 |                  Gyroscope corrected | 243 |   1 |    1.5625 |  400.0000 |
       14 |                     Gyroscope offset | 208 |   1 |    1.0000 |    1.0000 |
       15 |                    Gyroscope wake up | 194 |   1 |    1.5625 |  400.0000 |
       16 |       Gyroscope uncalibrated wake up | 195 |   1 |    1.5625 |  400.0000 |
       28 |                       Gravity vector | 247 |   1 |    1.5625 |  400.0000 |
       29 |               Gravity vector wake up | 198 |   1 |    1.5625 |  400.0000 |
       31 |                  Linear acceleration | 246 |   1 |    1.5625 |  400.0000 |
       32 |          Linear acceleration wake up | 197 |   1 |    1.5625 |  400.0000 |
       37 |                 Game rotation vector | 252 |   1 |    1.5625 |  400.0000 |
       38 |         Game rotation vector wake up | 200 |   1 |    1.5625 |  400.0000 |
       48 |                        Tilt detector | 236 |   1 |    1.0000 |    1.0000 |
       50 |                        Step detector | 248 |   1 |    1.0000 |    1.0000 |
       52 |                         Step counter | 249 |   1 |    1.0000 |    1.0000 |
       53 |                 Step counter wake up | 231 |   1 |    0.0005 |   25.0000 |
       55 |                   Significant motion | 250 |   1 |    1.0000 |    1.0000 |
       57 |                         Wake gesture | 232 |   1 |    1.0000 |    1.0000 |
       59 |                       Glance gesture | 234 |   1 |    1.0000 |    1.0000 |
       61 |                       Pickup gesture | 233 |   1 |    1.0000 |    1.0000 |
       63 |                 Activity recognition | 235 |   1 |    1.0000 |    1.0000 |
       67 |                   Wrist tilt gesture | 162 |   1 |    1.0000 |    1.0000 |
       69 |                   Device orientation | 163 |   1 |    1.0000 |    1.0000 |
       70 |           Device orientation wake up | 164 |   1 |    1.0000 |    1.0000 |
       75 |                    Stationary detect | 161 |   1 |    1.0000 |    1.0000 |
       77 |                        Motion detect | 160 |   1 |    1.0000 |    1.0000 |
       94 |                Step detector wake up | 230 |   1 |    1.0000 |    1.0000 |
[ 22229][D][SensorBHI260AP.cpp:440] configure(): Enable Device orientation at 1.00Hz.
All sensor init done!
isReadyFlag1:
[ 22241][I][SensorBHI260AP.cpp:951] parseMetaEvent(): [META EVENT] Power mode changed for sensor id 69
[ 22250][I][SensorBHI260AP.cpp:948] parseMetaEvent(): [META EVENT] Sample rate changed for sensor id 69
Sensor1
3

  ________________  
 |                | 
 |                | 
 |                | 
 |                | 
 |                | 
 |             *  | 
 |________________| 

isReadyFlag1:
Sensor1
1

  ________________  
 |                | 
 |  *             | 
 |                | 
 |                | 
 |                | 
 |                | 
 |________________| 

isReadyFlag2:
[ 22316][I][SensorBHI260AP.cpp:951] parseMetaEvent(): [META EVENT] Power mode changed for sensor id 69
[ 22325][I][SensorBHI260AP.cpp:948] parseMetaEvent(): [META EVENT] Sample rate changed for sensor id 69
Sensor2
1

  ________________  
 |                | 
 |  *             | 
 |                | 
 |                | 
 |                | 
 |                | 
 |________________| 

isReadyFlag1:
isReadyFlag1:
Sensor1
2

  ________________  
 |                | 
 |                | 
 |                | 
 |                | 
 |                | 
 |  *             | 
 |________________| 

isReadyFlag1:
isReadyFlag1:
Sensor1
1

  ________________  
 |                | 
 |  *             | 
 |                | 
 |                | 
 |                | 
 |                | 
 |________________| 

isReadyFlag1:
isReadyFlag2:
Sensor2
2

  ________________  
 |                | 
 |                | 
 |                | 
 |                | 
 |                | 
 |  *             | 
 |________________| 

isReadyFlag2:
isReadyFlag2:
Sensor2
1

  ________________  
 |                | 
 |  *             | 
 |                | 
 |                | 
 |                | 
 |                | 
 |________________| 

isReadyFlag2:
isReadyFlag2:
Sensor2
2

  ________________  
 |                | 
 |                | 
 |                | 
 |                | 
 |                | 
 |  *             | 
 |________________| 

isReadyFlag2:
isReadyFlag2:
Sensor2
1

  ________________  
 |                | 
 |  *             | 
 |                | 
 |                | 
 |                | 
 |                | 
 |________________| 

isReadyFlag2:
isReadyFlag1:
Sensor1
2

  ________________  
 |                | 
 |                | 
 |                | 
 |                | 
 |                | 
 |  *             | 
 |________________| 

isReadyFlag1:
isReadyFlag1:
Sensor1
1

  ________________  
 |                | 
 |  *             | 
 |                | 
 |                | 
 |                | 
 |                | 
 |________________| 

isReadyFlag1:


```