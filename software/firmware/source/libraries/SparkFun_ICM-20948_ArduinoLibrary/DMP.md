# InvenSense Digital Motion Processor (DMP™)

## What is the Digital Motion Processor (DMP™)?

In version 1.2 of this library, we added _partial_ support for the InvenSense Digital Motion Processor (DMP™). The DMP is firmware which runs on the
ICM-20948 and which "offloads computation of motion processing algorithms from the host processor, improving system power performance".

"The DMP enables ultra-low power run-time and background calibration of the accelerometer, gyroscope, and compass, maintaining optimal performance of
the sensor data for both physical and virtual sensors generated through sensor fusion."

The DMP allows the accelerometer, gyro and magnetometer data to be combined (fused) so that Quaternion data can be produced.

The DMP firmware binary has been available for quite some time. It is included in InvenSense's "MotionLink" and "Embedded Motion Driver (eMD)" examples
which can be downloaded from the InvenSense Developers Corner. However, the code is opaque and difficult to follow.

Users like [@ericalbers](https://github.com/ericalbers/ICM20948_DMP_Arduino) and [@ZaneL](https://github.com/ZaneL/Teensy-ICM-20948) have ported the
InvenSense example code to the Arduino environment previously. We are grateful to Eric and Zane as their code allowed us to reverse-engineer some of the
ICM-20948 configuration steps.

We are also grateful to InvenSense themselves for sharing with us a _confidential & proprietary_ document called "_Application Note: Programming Sequence for
ICM-20648 DMP Hardware Function_". InvenSense admit that the document is not complete and have asked us not to share it openly.

The InvenSense document and the bus traffic we captured using Eric's port have allowed us to add _partial_ support for the DMP to this library, using our
own functions. We say _partial_ because, at the time of writing, our library does not support: activity recognition, step counting, pick-up and tap-detection.
It does however support:
- Raw and calibrated accelerometer, gyro and compass data and accuracy
- 6-axis and 9-axis Quaternion data (including Game Rotation Vector data)
- Geomagnetic Rotation Vector data
- and [more...](#which-dmp-features-are-currently-supported)

We have added [five new examples](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/tree/master/examples/Arduino) to show how to configure the DMP and read:
9-axis Quaternion data; 6-axis Quaternion converted to Euler angles (roll, pitch & yaw); raw accelerometer data.

## Is DMP support enabled by default?

No. The DMP occupies 14kBytes of program memory and so, to allow the library to continue to run on processors with limited memory, DMP support is disabled by default.

You can enable it by editing the file called ```ICM_20948_C.h``` and uncommenting [line 29](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/master/src/util/ICM_20948_C.h#L29):

Change:

```
//#define ICM_20948_USE_DMP
```

to:

```
#define ICM_20948_USE_DMP
```

You will find ```ICM_20948_C.h``` in the library _src\util_ folder. If you are using Windows, you will find it in _Documents\Arduino\libraries\SparkFun_ICM-20948_ArduinoLibrary\src\util_.

## How is the DMP loaded and started?

In version 1.2.5 we added a new helper function named ```initializeDMP```. This is a weak function which you can overwrite e.g. if you want to change the sample rate
(see [Example10](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/master/examples/Arduino/Example10_DMP_FastMultipleSensors/Example10_DMP_FastMultipleSensors.ino) for details).
```initializeDMP``` does most of the heavy lifting for you: it downloads the DMP firmware; and configures all of the registers with the appropriate values. The only things
you need to do manually are: select which DMP sensors to enable; reset and start both FIFO and DMP.

The DMP firmware is loaded into the ICM-20948's processor memory space via three special Bank 0 registers:
- **AGB0_REG_MEM_START_ADDR** (0x7C) - the address which AGB0_REG_MEM_R_W reads from or writes to (it auto-increments after each read or write)
- **AGB0_REG_MEM_R_W** (0x7D) - the memory read/write register
- **AGB0_REG_MEM_BANK_SEL** (0x7E) - the memory bank select. The complete read/write address is: (AGB0_REG_MEM_BANK_SEL * 256) + AGB0_REG_MEM_START_ADDR

The firmware binary (14290 or 14301 Bytes) is written into processor memory starting at address 0x90. ```loadDMPFirmware``` automatically breaks the code up into 256 byte blocks and increments
**AGB0_REG_MEM_BANK_SEL** during the writing.

Before the DMP is enabled, the 16-bit register **AGB2_REG_PRGM_START_ADDRH** (Bank 2, 0x50) needs to be loaded with the program start address. ```setDMPstartAddress``` does this for you.

The DMP is enabled or reset by setting bits in the Bank 0 register **AGB0_REG_USER_CTRL** (0x03). ```enableDMP``` and ```resetDMP``` do this for you.

The helper functions ```readDMPmems``` and ```writeDMPmems``` will let you read and write data directly from the DMP memory space.

## How do I access the DMP data?

The DMP data is returned via the FIFO (First In First Out). ```readDMPdataFromFIFO``` checks if any data is present in the FIFO (by calling ```getFIFOcount``` which reads the 16-bit register
**AGB0_REG_FIFO_COUNT_H** (0x70)). If data is present, it is copied into a ```icm_20948_DMP_data_t``` struct.

```readDMPdataFromFIFO``` will return:
- ```ICM_20948_Stat_FIFONoDataAvail``` if no data or incomplete data is available
- ```ICM_20948_Stat_Ok``` if a valid frame was read
- ```ICM_20948_Stat_FIFOMoreDataAvail``` if a valid frame was read _and_ the FIFO contains more (unread) data
- ```ICM_20948_Stat_FIFOIncompleteData``` if a frame was present in the FIFO but it was incomplete

You can examine the 16-bit ```icm_20948_DMP_data_t data.header``` to see what data the frame contained. ```data.header``` is a bit field; each bit indicates what data is present:
- **DMP_header_bitmap_Compass_Calibr** (0x0020)
- **DMP_header_bitmap_Gyro_Calibr** (0x0040)
- **DMP_header_bitmap_Geomag** (0x0100)
- **DMP_header_bitmap_PQuat6** (0x0200)
- **DMP_header_bitmap_Quat9** (0x0400)
- **DMP_header_bitmap_Quat6** (0x0800)
- **DMP_header_bitmap_ALS** (0x1000)
- **DMP_header_bitmap_Compass** (0x2000)
- **DMP_header_bitmap_Gyro** (0x4000)
- **DMP_header_bitmap_Accel** (0x8000)

**DMP_header_bitmap_Header2** (0x0008) indicates if any secondary data was included. If the **DMP_header_bitmap_Header2** bit is set, the frame also contained one or more of:
- **DMP_header2_bitmap_Compass_Accuracy** (0x1000)
- **DMP_header2_bitmap_Gyro_Accuracy** (0x2000)
- **DMP_header2_bitmap_Accel_Accuracy** (0x4000)

## Which DMP features are currently supported?

All of the following _should_ work, but we have not tested them all:

```
INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)
```

## What changes did you make in v1.2.5?

In v1.2.5 we added some critical missing configuration steps:
- We use I2C_SLV0 and I2C_SLV1 to request the magnetometer data and trigger the next Single Measurement. We no longer use the 100Hz continuous mode for the DMP
- We now read ten bytes of data from the magnetometer, starting at register 0x03; instead of reading nine bytes, starting at register 0x10
  - Register 0x03 is reserved and the other nine registers are undocumented. They appear to contain the raw magnetometer reading in big-endian format (instead of little-endian)
  - We had to dig deep into InvenSense's Icm20948AuxCompassAkm.c to find this out...
- We configure the I2C Master ODR which reduces the magnetometer read rate from a silly 1100Hz to a sensible 69Hz
  - We had to monitor the Aux I2C pins and study the AK09916 traffic to figure this out...

The DMP configuration code was becoming so verbose that we decided to move it into its own function called ```initializeDMP```. This is a weak function which you can overwrite
e.g. if you want to change the sample rate
(see [Example10](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/master/examples/Arduino/Example10_DMP_FastMultipleSensors/Example10_DMP_FastMultipleSensors.ino) for details).
```initializeDMP``` does most of the heavy lifting for you: it downloads the DMP firmware; and configures all of the registers with the appropriate values. The only things
you need to do manually are: select which DMP sensors to enable; reset and start both FIFO and DMP. Please see the revised DMP examples for more details.

## Where are the DMP registers defined?

You will find the definitions in ```ICM_20948_DMP.h```.

That file also includes the definition for the ```icm_20948_DMP_data_t``` struct which is loaded with DMP data from the FIFO.

```const int``` declarations (including the DMP firmware image) are in ```ICM_20948_C.c```

## Can the DMP generate interrupts?

Yes it can, but you might find that they are not fully supported as we have not tested them. The main functions you will need to experiment with are ```intEnableDMP``` and ```enableDMPSensorInt```.

## How is the DMP data rate set?

It is a _combination_ of the raw sensor rate (set by ```setSampleRate```) and the multiple DMP Output Data Rate (ODR) registers
(set by ```setDMPODRrate```). There are other settings that need to be changed to match the sample rate too.
Please see [examples 9 & 10](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/tree/master/examples/Arduino) for more details.

The DMP is capable of outputting multiple sensor data at different rates to the FIFO.

## Can I contribute to this library?

Absolutely! Please see [CONTRIBUTING.md](./CONTRIBUTING.md) for further details.

## Can I see the full DMP configuration captured from @ericalbers code?

Brace yourself. Here it is:

- **.....** indicates where I've omitted some of the bus transactions. There are _many_ writes to the Power Management 1 register to enable and disable low power mode. I have omitted many of those.
- **#####** indicates an interval in the I<sup>2</sup>C bus traffic.

```
/*
 *  From @ericalbers port of the InvenSense example (https://github.com/ericalbers/ICM20948_DMP_Arduino)
 *  The eric + InvenSense code configures the ICM-20948 (via I2C) as follows:
 *  
 *  Select and read register 0x00 (WhoAmI)
 *  ICM returns 0xEA
 *  Set register 0x06 (Power Management 1) to 0x01 (Auto clock)
 *  Select and read register 0x03 (User Control)
 *  ICM returns 0xC0
 *  Set register 0x07 (Power Management 2) to 0x47 (Reserved bit 6 set. Disable all Gyro axes)
 *  Select and read register 0x00 (WhoAmI)
 *  ICM returns 0xEA
 *  Set register 0x05 (Low Power Configuration) to 0x70 (I2C Master, Accel and Gyro in duty cycled mode)
 *  Set register 0x03 (User Control) to 0x00 (Disable DMP, disable FIFO, disable I2C Master)
 *  Set register 0x7C (Memory Start Address) to 0x90
 *  .....
 *  Write DMP firmware data to register 0x7D (Memory Read/Write) in blocks of 16 bytes
 *  During write, increment register 0x7E (Memory Bank Select) every 256 bytes
 *  .....
 *  Verify DMP firmware by reading it all back again
 *  .....
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x00
 *  Set register 0x7F (Register Bank Select) to 0x20 == Bank 2
 *  Write 0x1000 to register 0x50 (Program Start Address High) (Magic Number)
 *  Set register 0x7F (Register Bank Select) to 0x00 == Bank 0
 *  Set register 0x7E (Memory Bank Select) to 0x00
 *  Set register 0x7C (Memory Start Address) to 0x40
 *  Write 0x0000 to memory (Data Out Control 1) (Disable all DMP features)
 *  Set register 0x7C (Memory Start Address) to 0x42
 *  Write 0x0000 to memory (Data Out Control 2) (Disable all "header2" features)
 *  Set register 0x7C (Memory Start Address) to 0x4C
 *  Write 0x0000 to memory (Data Interrupt Control) (Disable DMP interrupts)
 *  Set register 0x7C (Memory Start Address) to 0x4E
 *  Write 0x0000 to memory (FIFO Watermark) (Set watermark to zero)
 *  Set register 0x7C (Memory Start Address) to 0x8A
 *  Write 0x0000 to memory (Data Ready Status) (Disable all data ready indicators)
 *  Set register 0x7E (Memory Bank Select) to 0x01
 *  Set register 0x7C (Memory Start Address) to 0xFE
 *  Write 0x0320 to memory (31 * 16 + 14 = FIFO Watermark) (Set FIFO watermark to 800)
 *  Set register 0x10 (Reg Interrupt Enable) to 0x02 (Enable DMP interrupt to interrupt pin)
 *  Set register 0x12 (Reg Interrupt Enable 2) to 0x01 (Enable interrupt for FIFO overflow - 'channel' 0 only)
 *  Set register 0x26 (Single FIFO Priority Select) to 0xE4 (Worth investigating?)
 *  Select and read register 0x75 (Hardware Fix Disable)
 *  ICM returns 0x48
 *  Set register 0x75 (Hardware Fix Disable) to 0x48 (Worth investigating?)
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x00
 *  Set register 0x7F (Register Bank Select) to 0x20 == Bank 2
 *  Set register 0x00 (Gyro Sample Rate Divider) to 0x13
 *  Write 0x0013 to 0x10 (Accelerometer Sample Rate Divider 1&2)
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x20
 *  Set register 0x7F (Register Bank Select) to 0x00 == Bank 0
 *  Set register 0x7E (Memory Bank Select) to 0x03
 *  Set register 0x7C (Memory Start Address) to 0x0A
 *  Write 0x0000 to memory (48 * 16 + 10 = BAC Rate)
 *  Set register 0x7C (Memory Start Address) to 0x08
 *  Write 0x0000 to memory (48 * 16 + 8 = B2S Rate)
 *  Set register 0x76 (FIFO Config) to 0x00
 *  Set register 0x68 (FIFO Reset) to 0x1F
 *  Set register 0x68 (FIFO Reset) to 0x1E
 *  Set register 0x66 (FIFO Enable 1) to 0x00
 *  Set register 0x67 (FIFO Enable 2) to 0x00
 *  Set register 0x06 (Power Management 1) to 0x21 (Turn on low power, auto clock)
 *  Set register 0x07 (Power Management 2) to 0x7F (Reserved bit 6 set, disable all accel and gyro axes)
 *  Set register 0x06 (Power Management 1) to 0x61 (Sleep mode on, low power on, auto clock)
 *  Set register 0x06 (Power Management 1) to 0x21 (Turn on low power, auto clock)
 *  Set register 0x06 (Power Management 1) to 0x01 (Auto clock)
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x00
 *  Set register 0x7F (Register Bank Select) to 0x30 == Bank 3
 *  Set register 0x05 (Peripheral 0 Control) to 0x00
 *  Set register 0x09 (Peripheral 1 Control) to 0x00
 *  Set register 0x0D (Peripheral 2 Control) to 0x00
 *  Set register 0x11 (Peripheral 3 Control) to 0x00
 *  Set register 0x01 (Master Control) to 0x10
 *  Set register 0x00 (Master ODR Config) to 0x04
 *  Set register 0x03 (Master Delay Control) to 0x8C
 *  Set register 0x04 (Peripheral 0 Register) to 0x00
 *  Set register 0x05 (Peripheral 0 Control) to 0x81
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x30
 *  Set register 0x7F (Register Bank Select) to 0x00 == Bank 0
 *  Set register 0x03 (User Control) to 0x20
 *  #####
 *  #####
 *  #####
 *  Set register 0x03 (User Control) to 0x00
 *  Select and read register 0x3B (Ext Peripheral Sensor Data)
 *  ICM returns 0x48
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x00
 *  Set register 0x7F (Register Bank Select) to 0x30 == Bank 3
 *  Set register 0x05 (Peripheral 0 Control) to 0x00
 *  Set register 0x07 (Peripheral 1 Address) to 0x0C
 *  Set register 0x08 (Peripheral 1 Register) to 0x31
 *  Set register 0x0A (Peripheral 1 Data) to 0x00
 *  Set register 0x09 (Peripheral 1 Control) to 0x81
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x30
 *  Set register 0x7F (Register Bank Select) to 0x00 == Bank 0
 *  Set register 0x03 (User Control) to 0x20
 *  #####
 *  #####
 *  #####
 *  Set register 0x03 (User Control) to 0x00
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x00
 *  Set register 0x7F (Register Bank Select) to 0x30 == Bank 3
 *  Set register 0x09 (Peripheral 1 Control) to 0x00
 *  Set register 0x05 (Peripheral 0 Control) to 0x00
 *  Set register 0x09 (Peripheral 1 Control) to 0x00
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x30
 *  Set register 0x7F (Register Bank Select) to 0x00 == Bank 0
 *  Set register 0x03 (User Control) to 0x00
 *  Set register 0x7E (Memory Bank Select) to 0x01
 *  Set register 0x7C (Memory Start Address) to 0x70
 *  Write 0x09999999 to memory (23 * 16 + 0 = Compass Matrix 00)
 *  Set register 0x7C (Memory Start Address) to 0x74
 *  Write 0x00000000 to memory (23 * 16 + 4 = Compass Matrix 01)
 *  Set register 0x7C (Memory Start Address) to 0x78
 *  Write 0x00000000 to memory (23 * 16 + 8 = Compass Matrix 02)
 *  Set register 0x7C (Memory Start Address) to 0x7C
 *  Write 0x00000000 to memory (23 * 16 + 12 = Compass Matrix 10)
 *  Set register 0x7C (Memory Start Address) to 0x80
 *  Write 0xF6666667 to memory (24 * 16 + 0 = Compass Matrix 11)
 *  Set register 0x7C (Memory Start Address) to 0x84
 *  Write 0x00000000 to memory (24 * 16 + 4 = Compass Matrix 12)
 *  Set register 0x7C (Memory Start Address) to 0x88
 *  Write 0x00000000 to memory (24 * 16 + 8 = Compass Matrix 20)
 *  Set register 0x7C (Memory Start Address) to 0x8C
 *  Write 0x00000000 to memory (24 * 16 + 12 = Compass Matrix 21)
 *  Set register 0x7C (Memory Start Address) to 0x90
 *  Write 0xF6666667 to memory (25 * 16 + 0 = Compass Matrix 22)
 *  Set register 0x06 (Power Management 1) to 0x21 (Turn on low power, auto clock)
 *  Set register 0x06 (Power Management 1) to 0x01 (Auto clock)
 *  Set register 0x7E (Memory Bank Select) to 0x0D
 *  Set register 0x7C (Memory Start Address) to 0x00
 *  Write 0x40000000 to memory (208 * 16 + 0 = B2S Mounting Matrix 00)
 *  Set register 0x06 (Power Management 1) to 0x21 (Turn on low power, auto clock)
 *  Set register 0x06 (Power Management 1) to 0x01 (Auto clock)
 *  Set register 0x7C (Memory Start Address) to 0x04
 *  Write 0x00000000 to memory (208 * 16 + 4 = B2S Mounting Matrix 01)
 *  Set register 0x06 (Power Management 1) to 0x21 (Turn on low power, auto clock)
 *  Set register 0x06 (Power Management 1) to 0x01 (Auto clock)
 *  Set register 0x7C (Memory Start Address) to 0x08
 *  Write 0x00000000 to memory (208 * 16 + 8 = B2S Mounting Matrix 02)
 *  Set register 0x06 (Power Management 1) to 0x21 (Turn on low power, auto clock)
 *  Set register 0x06 (Power Management 1) to 0x01 (Auto clock)
 *  Set register 0x7C (Memory Start Address) to 0x0C
 *  Write 0x00000000 to memory (208 * 16 + 12 = B2S Mounting Matrix 10)
 *  Set register 0x06 (Power Management 1) to 0x21 (Turn on low power, auto clock)
 *  Set register 0x06 (Power Management 1) to 0x01 (Auto clock)
 *  Set register 0x7C (Memory Start Address) to 0x10
 *  Write 0x40000000 to memory (209 * 16 + 0 = B2S Mounting Matrix 11)
 *  Set register 0x06 (Power Management 1) to 0x21 (Turn on low power, auto clock)
 *  Set register 0x06 (Power Management 1) to 0x01 (Auto clock)
 *  Set register 0x7C (Memory Start Address) to 0x14
 *  Write 0x00000000 to memory (209 * 16 + 4 = B2S Mounting Matrix 12)
 *  Set register 0x06 (Power Management 1) to 0x21 (Turn on low power, auto clock)
 *  Set register 0x06 (Power Management 1) to 0x01 (Auto clock)
 *  Set register 0x7C (Memory Start Address) to 0x18
 *  Write 0x00000000 to memory (209 * 16 + 8 = B2S Mounting Matrix 20)
 *  Set register 0x06 (Power Management 1) to 0x21 (Turn on low power, auto clock)
 *  Set register 0x06 (Power Management 1) to 0x01 (Auto clock)
 *  Set register 0x7C (Memory Start Address) to 0x1C
 *  Write 0x00000000 to memory (209 * 16 + 12 = B2S Mounting Matrix 21)
 *  Set register 0x06 (Power Management 1) to 0x21 (Turn on low power, auto clock)
 *  Set register 0x06 (Power Management 1) to 0x01 (Auto clock)
 *  Set register 0x7C (Memory Start Address) to 0x20
 *  Write 0x40000000 to memory (210 * 16 + 0 = B2S Mounting Matrix 22)
 *  Set register 0x06 (Power Management 1) to 0x21 (Turn on low power, auto clock)
 *  Set register 0x06 (Power Management 1) to 0x01 (Auto clock)
 *  Set register 0x7C (Memory Start Address) to 0x00
 *  Write 0x40000000 to memory (208 * 16 + 0 = B2S Mounting Matrix 00)
 *  .....
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x00
 *  Set register 0x7F (Register Bank Select) to 0x20 == Bank 2
 *  Select and read register 0x14 (Accel Config)
 *  ICM returns 0x02
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x20
 *  Set register 0x7F (Register Bank Select) to 0x00 == Bank 0
 *  Set register 0x06 (Power Management 1) to 0x21 (Turn on low power, auto clock)
 *  Set register 0x06 (Power Management 1) to 0x01 (Auto clock)
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x00
 *  Set register 0x7F (Register Bank Select) to 0x20 == Bank 2
 *  Set register 0x14 (Accel Config) to 0x02
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x20
 *  Set register 0x7F (Register Bank Select) to 0x00 == Bank 0
 *  Set register 0x06 (Power Management 1) to 0x21 (Turn on low power, auto clock)
 *  Set register 0x06 (Power Management 1) to 0x01 (Auto clock)
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x00
 *  Set register 0x7F (Register Bank Select) to 0x20 == Bank 2
 *  Select and read register 0x15 (Accel Config 2)
 *  ICM returns 0x00
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x20
 *  Set register 0x7F (Register Bank Select) to 0x00 == Bank 0
 *  Set register 0x06 (Power Management 1) to 0x21 (Turn on low power, auto clock)
 *  Set register 0x06 (Power Management 1) to 0x01 (Auto clock)
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x00
 *  Set register 0x7F (Register Bank Select) to 0x20 == Bank 2
 *  Set register 0x15 (Accel Config 2) to 0x00
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x20
 *  Set register 0x7F (Register Bank Select) to 0x00 == Bank 0
 *  Set register 0x06 (Power Management 1) to 0x21 (Turn on low power, auto clock)
 *  Set register 0x06 (Power Management 1) to 0x01 (Auto clock)
 *  Set register 0x7E (Memory Bank Select) to 0x01
 *  Set register 0x7C (Memory Start Address) to 0xE0
 *  Write 0x40000000 to memory (30 * 16 + 0 = Accelerometer Scale)
 *  Set register 0x06 (Power Management 1) to 0x21 (Turn on low power, auto clock)
 *  Set register 0x06 (Power Management 1) to 0x01 (Auto clock)
 *  Set register 0x7E (Memory Bank Select) to 0x04
 *  Set register 0x7C (Memory Start Address) to 0xF4
 *  Write 0x00040000 to memory (79 * 16 + 4 = Accelerometer Scale 2)
 *  Set register 0x06 (Power Management 1) to 0x21 (Turn on low power, auto clock)
 *  Set register 0x06 (Power Management 1) to 0x01 (Auto clock)
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x00
 *  Set register 0x7F (Register Bank Select) to 0x20 == Bank 2
 *  Select and read register 0x14 (Accel Config)
 *  ICM returns 0x02
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x20
 *  Set register 0x7F (Register Bank Select) to 0x00 == Bank 0
 *  Set register 0x06 (Power Management 1) to 0x21 (Turn on low power, auto clock)
 *  Set register 0x06 (Power Management 1) to 0x01 (Auto clock)
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x00
 *  Set register 0x7F (Register Bank Select) to 0x20 == Bank 2
 *  Set register 0x14 (Accel Config) to 0x02
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x20
 *  Set register 0x7F (Register Bank Select) to 0x00 == Bank 0
 *  Set register 0x06 (Power Management 1) to 0x21 (Turn on low power, auto clock)
 *  Set register 0x06 (Power Management 1) to 0x01 (Auto clock)
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x00
 *  Set register 0x7F (Register Bank Select) to 0x20 == Bank 2
 *  Select and read register 0x15 (Accel Config 2)
 *  ICM returns 0x00
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x20
 *  Set register 0x7F (Register Bank Select) to 0x00 == Bank 0
 *  Set register 0x06 (Power Management 1) to 0x21 (Turn on low power, auto clock)
 *  Set register 0x06 (Power Management 1) to 0x01 (Auto clock)
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x00
 *  Set register 0x7F (Register Bank Select) to 0x20 == Bank 2
 *  Set register 0x15 (Accel Config 2) to 0x00
 *  .....
 *  Set register 0x7E (Memory Bank Select) to 0x01
 *  Set register 0x7C (Memory Start Address) to 0xE0
 *  Write 0x04000000 to memory (30 * 16 + 0 = Accelerometer Scale)
 *  .....
 *  Set register 0x7E (Memory Bank Select) to 0x04
 *  Set register 0x7C (Memory Start Address) to 0xF4
 *  Write 0x00040000 to memory (79 * 16 + 4 = Accelerometer Scale 2)
 *  .....
 *  Select and read register 0x7F (Register Bank Select)
 *  ICM returns 0x00
 *  Set register 0x7F (Register Bank Select) to 0x20 == Bank 2
 *  Select and read register 0x01 (Gyro Config 1)
 *  ICM returns 0x07
 *  .....
 *  Set register 0x01 (Gyro Config 1) to 0x07
 *  .....
 *  Select and read register 0x02 (Gyro Config 2)
 *  ICM returns 0x00
 *  .....
 *  Set register 0x02 (Gyro Config 2) to 0x00
 *  .....
 *  Set register 0x7F (Register Bank Select) to 0x10 == Bank 1
 *  Select and read register 0x28 (Timebase Correction PLL)
 *  ICM returns 0x18
 *  .....
 *  Set register 0x7E (Memory Bank Select) to 0x01
 *  Set register 0x7C (Memory Start Address) to 0x30
 *  Write 0x26FAB4B1 to memory (19 * 16 + 0 = Gyro Scaling Factor)
 *  .....
 *  Set register 0x7E (Memory Bank Select) to 0x04
 *  Set register 0x7C (Memory Start Address) to 0x8C
 *  Write 0x10000000 to memory (72 * 16 + 12 = Gyro Fullscale)
 *  .....
 *  Set register 0x7F (Register Bank Select) to 0x20 == Bank 2
 *  Select and read register 0x01 (Gyro Config 1)
 *  ICM returns 0x07
 *  .....
 *  Set register 0x01 (Gyro Config 1) to 0x07
 *  .....
 *  Select and read register 0x02 (Gyro Config 2)
 *  ICM returns 0x00
 *  .....
 *  Set register 0x02 (Gyro Config 2) to 0x00
 *  .....
 *  Set register 0x7C (Memory Start Address) to 0x8C
 *  Write 0x10000000 to memory (72 * 16 + 12 = Gyro Fullscale)
 *  .....
 *  Set register 0x7F (Register Bank Select) to 0x20 == Bank 2
 *  Select and read register 0x01 (Gyro Config 1)
 *  ICM returns 0x07
 *  .....
 *  Set register 0x01 (Gyro Config 1) to 0x07
 *  .....
 *  Select and read register 0x02 (Gyro Config 2)
 *  ICM returns 0x00
 *  .....
 *  Set register 0x02 (Gyro Config 2) to 0x00
 *  .....
 *  Set register 0x7F (Register Bank Select) to 0x00 == Bank 0
 *  .....
 *  Set register 0x7C (Memory Start Address) to 0x8C
 *  Write 0x10000000 to memory (72 * 16 + 12 = Gyro Fullscale)
 *  Set register 0x06 (Power Management 1) to 0x21
 *  .....
 *  Set register 0x07 (Power Management 2) to 0x47
 *  Set register 0x06 (Power Management 1) to 0x01
 *  Set register 0x03 (User Control) to 0xC0
 *  Set register 0x7E (Memory Bank Select) to 0x00
 *  Set register 0x7C (Memory Start Address) to 0x40
 *  Write 0x4048 to memory (4 * 16 + 0 == Data Out Control 1)
 *  Set register 0x7C (Memory Start Address) to 0x4C
 *  Write 0x4048 to memory (4 * 16 + 12 == Data Interrupt Control)
 *  Set register 0x7C (Memory Start Address) to 0x42
 *  Write 0x2000 to memory (4 * 16 + 2 == Data Out Control 2)
 *  Set register 0x7C (Memory Start Address) to 0x4E
 *  Write 0x0100 to memory (4 * 16 + 14 == Motion Event Control)
 *  .....
 *  Set register 0x7F (Register Bank Select) to 0x20 == Bank 2
 *  Set register 0x00 (Gyro Sample Rate Divider) to 0xE0
 *  .....
 *  Set register 0x7F (Register Bank Select) to 0x00 == Bank 0
 *  Set register 0x7C (Memory Start Address) to 0xBA
 *  Write 0x0000 to memory (11 * 16 + 10 == ODR Gyro)
 *  Set register 0x7C (Memory Start Address) to 0xB8
 *  Write 0x0000 to memory (11 * 16 + 8 == ODR Gyro Calibr)
 *  Set register 0x7E (Memory Bank Select) to 0x01
 *  Set register 0x7C (Memory Start Address) to 0x30
 *  Write 0x7FFFFFFF to memory (19 * 16 + 0 == Gyro Scaling Factor)
 *  Set register 0x07 (Power Management 2) to 0x78
 *  Set register 0x7E (Memory Bank Select) to 0x00
 *  Set register 0x7C (Memory Start Address) to 0x8A
 *  Write 0x0001 to memory (8 * 16 + 10 == Data Ready Status)
 *  .....
 *  Set register 0x7C (Memory Start Address) to 0x40
 *  Write 0xC048 to memory (4 * 16 + 0 == Data Out Control 1)
 *  Set register 0x7C (Memory Start Address) to 0x4C
 *  Write 0xC048 to memory (4 * 16 + 12 == Data Interrupt Control)
 *  Set register 0x7C (Memory Start Address) to 0x42
 *  Write 0x6000 to memory (4 * 16 + 2 == Data Out Control 2)
 *  Set register 0x7C (Memory Start Address) to 0x4E
 *  Write 0x0300 to memory (4 * 16 + 14 == FIFO Watermark)
 *  Set register 0x7E (Memory Bank Select) to 0x01
 *  Set register 0x7C (Memory Start Address) to 0x0C
 *  Write 0x00E8BA2E to memory (16 * 16 + 12 == Accel Only Gain)
 *  Set register 0x7E (Memory Bank Select) to 0x05
 *  Set register 0x7C (Memory Start Address) to 0xB0
 *  Write 0x06666666 to memory (91 * 16 + 0 == Accel Alpha Var)
 *  Set register 0x7C (Memory Start Address) to 0xC0
 *  Write 0x3999999A to memory (92 * 16 + 0 == Accel A Var)
 *  Set register 0x7C (Memory Start Address) to 0xE4
 *  Write 0x0000 to memory (94 * 16 + 4 == Accel Cal Rate)
 *  .....
 *  Set register 0x7F (Register Bank Select) to 0x20 == Bank 2
 *  Set register 0x10 (Accel Sample Rate Divider 1&2) to 0x00E0
 *  .....
 *  Set register 0x7F (Register Bank Select) to 0x00 == Bank 0
 *  Set register 0x7E (Memory Bank Select) to 0x00
 *  Set register 0x7C (Memory Start Address) to 0xBE
 *  Write 0x0000 to memory (11 * 16 + 14 == ODR Accel)
 *  Set register 0x7C (Memory Start Address) to 0xBA
 *  Write 0x0000 to memory (11 * 16 + 10 == ODR Gyro)
 *  Set register 0x7C (Memory Start Address) to 0xB8
 *  Write 0x0000 to memory (11 * 16 + 8 == ODR Gyro Calibr)
 *  Set register 0x07 (Power Management 2) to 0x40
 *  Set register 0x7C (Memory Start Address) to 0x8A
 *  Write 0x0003 to memory (8 * 16 + 10 == Data Ready Status)
 *  .....
 *  Set register 0x7C (Memory Start Address) to 0x40
 *  Write 0xC848 to memory (4 * 16 + 0 == Data Out Control 1)
 *  Set register 0x7C (Memory Start Address) to 0x4C
 *  Write 0xC848 to memory (4 * 16 + 12 == Data Interrupt Control)
 *  Set register 0x7C (Memory Start Address) to 0x42
 *  Write 0x6000 to memory (4 * 16 + 2 == Data Out Control 2)
 *  Set register 0x7C (Memory Start Address) to 0x4E
 *  Write 0x0300 to memory (4 * 16 + 14 == FIFO Watermark)
 *  Set register 0x7C (Memory Start Address) to 0xBE
 *  Write 0x0000 to memory (11 * 16 + 14 == ODR Accel)
 *  Set register 0x7C (Memory Start Address) to 0xBA
 *  Write 0x0000 to memory (11 * 16 + 10 == ODR Gyro)
 *  Set register 0x7C (Memory Start Address) to 0xB8
 *  Write 0x0000 to memory (11 * 16 + 8 == ODR Gyro Calibr)
 *  Set register 0x07 (Power Management 2) to 0x40
 *  Set register 0x7C (Memory Start Address) to 0x8A
 *  Write 0x0003 to memory (8 * 16 + 10 == Data Ready Status)
 *  .....
 *  Set register 0x7C (Memory Start Address) to 0x40
 *  Write 0xC848 to memory (4 * 16 + 0 == Data Out Control 1)
 *  Set register 0x7C (Memory Start Address) to 0x4C
 *  Write 0xC848 to memory (4 * 16 + 12 == Data Interrupt Control)
 *  Set register 0x7C (Memory Start Address) to 0x42
 *  Write 0x6000 to memory (4 * 16 + 2 == Data Out Control 2)
 *  Set register 0x7C (Memory Start Address) to 0x4E
 *  Write 0x0300 to memory (4 * 16 + 14 == FIFO Watermark)
 *  Set register 0x7C (Memory Start Address) to 0xBE
 *  Write 0x0000 to memory (11 * 16 + 14 == ODR Accel)
 *  Set register 0x7C (Memory Start Address) to 0xBA
 *  Write 0x0000 to memory (11 * 16 + 10 == ODR Gyro)
 *  Set register 0x7C (Memory Start Address) to 0xB8
 *  Write 0x0000 to memory (11 * 16 + 8 == ODR Gyro Calibr)
 *  Set register 0x07 (Power Management 2) to 0x40
 *  Set register 0x7C (Memory Start Address) to 0x8A
 *  Write 0x0003 to memory (8 * 16 + 10 == Data Ready Status)
 *  Set register 0x06 (Power Management 1) to 0x21
 *  Select and read register 0x19 (Interrupt Status)
 *  ICM returns 0x0A
 *  Select and read register 0x18 (DMP Interrupt Status)
 *  ICM returns 0x01
 *  Select and read register 0x70 (FIFO Count Hi&Lo)
 *  ICM returns 0x0000
 *  Select and read register 0x19 (Interrupt Status)
 *  ICM returns 0x00
 *  Select and read register 0x18 (DMP Interrupt Status)
 *  ICM returns 0x00
 *  Select and read register 0x70 (FIFO Count Hi&Lo)
 *  ICM returns 0x0000
 *  .....
 *  .....
 *  Select and read register 0x19 (Interrupt Status)
 *  ICM returns 0x04
 *  Select and read register 0x18 (DMP Interrupt Status)
 *  ICM returns 0x00
 *  Select and read register 0x70 (FIFO Count Hi&Lo)
 *  ICM returns 0x0000
 *  Select and read register 0x19 (Interrupt Status)
 *  ICM returns 0x00
 *  Select and read register 0x18 (DMP Interrupt Status)
 *  ICM returns 0x00
 *  Select and read register 0x70 (FIFO Count Hi&Lo)
 *  ICM returns 0x0000
 *  .....
 *  .....
 *  Select and read register 0x19 (Interrupt Status)
 *  ICM returns 0x00
 *  Select and read register 0x18 (DMP Interrupt Status)
 *  ICM returns 0x00
 *  Select and read register 0x70 (FIFO Count Hi&Lo)
 *  ICM returns 0x001C
 *  Select and read register 0x72 (FIFO Read Write)
 *  Data Stream is:
 *  Header:  0xC048 : Accel & Gyro & Gyro_Calibr & Header2
 *  Header2: 0x6000 : Accel_Accuracy & Gyro_Accuracy
 *  Accel:   0x0130 0xFD3C 0x1FD0
 *  Gyro:    0xFFE7 0x000C 0x0007
 *  Gyro_Bias:      0x0000 0x0000 0x0000
 *  Accel_Accuracy: 0x0000
 *  Gyro_Accuracy:  0x0000
 *  Footer:  0x20DD
 *  .....
 *  Header:  0xC040 : Accel & Gyro & Gyro_Calibr
 *  Accel:   0x0178 0xFD84 0x1F44
 *  Gyro:    0xFFEE 0x0016 0x0000
 *  Gyro_Bias:      0x0000 0x0000 0x0000
 *  Footer:  0x70DD
 *  .....
 *  Header:  0xC040 : Accel & Gyro & Gyro_Calibr
 *  Accel:   0x0134 0xFDAC 0x1FC4
 *  Gyro:    0xFFF9 0x0013 0x0001
 *  Gyro_Bias:      0x0000 0x0000 0x0000
 *  Footer:  0x70DD
 *  .....
 *  Select and read register 0x19 (Interrupt Status)
 *  ICM returns 0x02
 *  Select and read register 0x18 (DMP Interrupt Status)
 *  ICM returns 0x01
 *  Select and read register 0x70 (FIFO Count Hi&Lo)
 *  ICM returns 0x0022
 *  Select and read register 0x72 (FIFO Read Write)
 *  Data Stream is:
 *  Header:  0xC840 : Accel & Gyro & Quat6 & Gyro_Calibr
 *  Accel:   0x0188 0xFDC4 0x1FA4
 *  Gyro:    0xFFF1 0x0019 0xFFFF
 *  Gyro_Bias:      0x0000 0x0000 0x0000
 *  Quat6:   0xFFC7A057 0x0030BD92 0x000A6646
 *  Footer:  0x00DB
*/
```
