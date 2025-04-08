/**
 *
 * @license MIT License
 *
 * Copyright (c) 2022 lewis he
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      SensorBHI260AP.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-06
 * @note      Most source code references come from the https://github.com/boschsensortec/BHY2-Sensor-API
 *            Simplification for Arduino
 */
#pragma once
#include "bosch/BoschSensorControl.hpp"
#include "bosch/BoschPhySensorInfo.hpp"
#include "bosch/BoschSensorInfo.hpp"
#include "SensorPlatform.hpp"
#include "bosch/BoschSensorID.hpp"
#include "bosch/BoschParseBase.hpp"
#include "bosch/BoschParseCallbackManager.hpp"


#define BHI260AP_SLAVE_ADDRESS_L          0x28
#define BHI260AP_SLAVE_ADDRESS_H          0x29

using SensorConfig = struct bhy2_virt_sensor_conf;
using ProcessCallback = void (*)(void *user_data, uint32_t total, uint32_t transferred);

class SensorBHI260AP : public BoschVirtualSensor, BoschParseBase
{
public:
    using SensorDebugMessageCallback = void (*)(const char *message);
    using SensorEventCallback = void (*)(uint8_t event, uint8_t sensor_id, uint8_t data);

    enum BoschOrientation {
        DIRECTION_TOP_RIGHT,
        DIRECTION_TOP_LEFT,
        DIRECTION_BOTTOM_LEFT,
        DIRECTION_BOTTOM_RIGHT,
    };

    enum SensorEvent {
        EVENT_FLUSH_COMPLETE           = 1,
        EVENT_SAMPLE_RATE_CHANGED,
        EVENT_POWER_MODE_CHANGED,
        EVENT_ALGORITHM_EVENTS         = 5,
        EVENT_SENSOR_STATUS,
        EVENT_BSX_DO_STEPS_MAIN,
        EVENT_BSX_DO_STEPS_CALIB,
        EVENT_BSX_GET_OUTPUT_SIGNAL,
        EVENT_SENSOR_ERROR             = 11,
        EVENT_FIFO_OVERFLOW,
        EVENT_DYNAMIC_RANGE_CHANGED,
        EVENT_FIFO_WATERMARK,
        EVENT_INITIALIZED              = 16,
        EVENT_TRANSFER_CAUSE,
        EVENT_SENSOR_FRAMEWORK,
        EVENT_RESET,
    };

    /**
    * @enum SensorRemap
    * @brief Enumeration representing different remapping options for the sensor's orientation.
    *
    * This enum defines various positions and orientations of the sensor chip. Each value corresponds
    * to a specific corner or location of the chip, which can be used to remap the axes of the sensor
    * according to its physical placement.
    *
    * Top view of the chip, where 'T' stands for top,
    * 'B' stands for bottom,
    * 'L' stands for left, and 'R' stands for right
    *  -------------
    * | TL         TR |
    * |               |
    * |               |
    * |               |
    * | BL         BR |
    *  -------------
    *
    * There is also a bottom view of the chip：
    *
    *  -------------
    * | BT         BB |
    * |               |
    * |               |
    * |               |
    * | LT         RT |
    *  -------------
    */
    enum SensorRemap {
        // Chip top view, upper left corner
        //  -------------
        // | *             |
        // |               |
        // |               |
        // |               |
        // |               |
        //  -------------
        TOP_LAYER_LEFT_CORNER,
        // Chip top view, upper right corner
        //  -------------
        // |             * |
        // |               |
        // |               |
        // |               |
        // |               |
        //  -------------
        TOP_LAYER_RIGHT_CORNER,
        // Chip top view, bottom right corner of the top
        //  -------------
        // |               |
        // |               |
        // |               |
        // |               |
        // |             * |
        //  -------------
        TOP_LAYER_BOTTOM_RIGHT_CORNER,
        // The top view of the chip, the lower left corner of the front bottom
        //  -------------
        // |               |
        // |               |
        // |               |
        // |               |
        // | *             |
        //  -------------
        TOP_LAYER_BOTTOM_LEFT_CORNER,
        // The bottom view of the chip, the upper left corner of the top
        //  -------------
        // | *             |
        // |               |
        // |               |
        // |               |
        // |               |
        //  -------------
        BOTTOM_LAYER_TOP_LEFT_CORNER,
        // The bottom view of the chip, the upper right corner of the top
        //  -------------
        // |             * |
        // |               |
        // |               |
        // |               |
        // |               |
        //  -------------
        BOTTOM_LAYER_TOP_RIGHT_CORNER,
        // The bottom view of the chip, the lower right corner of the bottom
        //  -------------
        // |               |
        // |               |
        // |               |
        // |               |
        // |             * |
        //  -------------
        BOTTOM_LAYER_BOTTOM_RIGHT_CORNER,
        // Chip bottom view, bottom left corner
        //  -------------
        // |               |
        // |               |
        // |               |
        // |               |
        // | *             |
        //  -------------
        BOTTOM_LAYER_BOTTOM_LEFT_CORNER,
    };

    // The pin names are named according to the sensor manual.
    enum BHI260AP_GPIO {
        MCSB1 = 1,
        RESV1 = 2,
        RESV2 = 3,
        MCSB2 = 4,  //It may be connected to the BMM150 sensor, select according to the actual situation
        MCSB3 = 5,
        MCSB4 = 6,

        QSPI_CLK = 8, // If BHI260 carries external flash, it is not available
        QSPI_CSN = 9, // If BHI260 carries external flash, it is not available
        QSPI_D0 = 10, // If BHI260 carries external flash, it is not available
        QSPI_D1 = 11, // If BHI260 carries external flash, it is not available
        QSPI_D2 = 12, // If BHI260 carries external flash, it is not available
        QSPI_D3 = 13, // If BHI260 carries external flash, it is not available

        M2SCX = 14,
        M2SDX = 15,
        M2SDI = 16,
        M3SCL = 17, //It may be connected to the BMM150 sensor, select according to the actual situation
        M3SDA = 18, //It may be connected to the BMM150 sensor, select according to the actual situation
        JTAG_CLK = 19,
        JTAG_DIO = 20,

        M1SCX = 127, // Invalid Pin
        M1SDX = 128, // Invalid Pin
        M1SDI = 129, // Invalid Pin
        RESV3 = 130, // Invalid Pin
    };

    ~SensorBHI260AP();

    SensorBHI260AP();

    /**
     * @brief  setPins
     * @note   Set the reset pin. reset pin is not set by default.
     * @param  rst:
     * @retval None
     */
    void setPins(int rst);

#if defined(ARDUINO)
    /**
     * @brief   begin
     * @note    Initialization using the Arduino Wire Interface
     * @param  &wire: TwoWire Class
     * @param  addr: Device address, default 0x28, can also be changed to 0x29
     * @param  sda: Set I2C SCL Pin, not set by default
     * @param  scl: Set I2C SDA Pin, not set by default
     * @retval bool true-> Success false-> failure
     */
    bool begin(TwoWire &wire, uint8_t addr = BHI260AP_SLAVE_ADDRESS_L, int sda = -1, int scl = -1);

    /**
     * @brief  begin
     * @note   Initialization using the Arduino SPI Interface
     * @param  &spi: SPIClass
     * @param  csPin: Set CS SCL Pin, not set by default
     * @param  mosi:  Set SPI MOSI SCL Pin, not set by default
     * @param  miso:  Set SPI MISO SCL Pin, not set by default
     * @param  sck:   Set SPI SCK SCL Pin, not set by default
     * @retval bool true-> Success false-> failure
     */
    bool begin(SPIClass &spi, uint8_t csPin, int mosi = -1, int miso = -1, int sck = -1);

#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
    /**
     * @brief  begin
     * @note   Initialization using the ESP-IDF I2C Legacy Interface
     * @param  port_num: I2C_NUM0 or I2C_NUM1
     * @param  addr: Device address, default 0x28, can also be changed to 0x29
     * @param  sda: Set I2C SCL Pin, not set by default
     * @param  scl: Set I2C SDA Pin, not set by default
     * @retval bool true-> Success false-> failure
     */
    bool begin(i2c_port_t port_num, uint8_t addr = BHI260AP_SLAVE_ADDRESS_L, int sda = -1, int scl = -1);
#else
    /**
     * @brief begin
     * @note    Initialization using the ESP-IDF I2C LL Interface idf version > 5.0.0
     * @param  handle: I2C Handle
     * @param  addr: Device address, default 0x28, can also be changed to 0x29
     * @retval bool true-> Success false-> failure
     */
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr = BHI260AP_SLAVE_ADDRESS_L);
#endif  //ESP_PLATFORM


    /**
     * @brief  begin
     * @note   Initialization using the ESP-IDF SPI Interface
     * @param  host: spi_host_device_t enum
     * @param  handle: spi_device_handle_t handle
     * @param  csPin: cs pin
     * @param  mosi: spi mosi pin
     * @param  miso: spi miso pin
     * @param  sck: spi sck pin
     * @retval bool true-> Success false-> failure
     */
    bool begin(spi_host_device_t host, spi_device_handle_t handle, uint8_t csPin, int mosi, int miso, int sck);

#endif  //ARDUINO

    /**
     * @brief  begin
     * @note   Custom callback interface, suitable for other platforms
     * @param  interface: Communication mode, COMM_SPI or COMM_I2C
     * @param  callback: Register read and write callback function
     * @param  hal_callback: Platform digital IO and delay callback function
     * @param  addr: Device address, default 0x28, can also be changed to 0x29
     * @retval bool true-> Success false-> failure
     */
    bool begin(CommInterface interface, SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_callback,
               uint8_t addr = BHI260AP_SLAVE_ADDRESS_L);

    /**
     * @brief  reset
     * @note   Reset sensor
     * @retval None
     */
    void reset();

    /**
     * @brief  update
     * @note   Update sensor fifo data
     * @retval None
     */
    void update();

    /**
     * @brief  setBootFromFlash
     * @note   Set whether to start from external flash
     * @param  boot_from_flash: true boot form flash or boot form ram
     * @retval None
     */
    void setBootFromFlash(bool boot_from_flash);

    /**
     * @brief  getHandler
     * @note   Get the native BHI API handle
     * @retval handle
     */
    bhy2_dev *getHandler();

    /**
     * @brief  setInterruptCtrl
     * @note   Set the interrupt control mask
     * @param  data:
     *               BHY2_ICTL_DISABLE_FIFO_W
     *               BHY2_ICTL_DISABLE_FIFO_NW
     *               BHY2_ICTL_DISABLE_STATUS_FIFO
     *               BHY2_ICTL_DISABLE_DEBUG
     *               BHY2_ICTL_DISABLE_FAULT
     *               BHY2_ICTL_ACTIVE_LOW
     *               BHY2_ICTL_EDGE
     *               BHY2_ICTL_OPEN_DRAIN
     *
     * @retval true is success , false is failed
     */
    bool setInterruptCtrl(uint8_t data);

    /**
     * @brief  getInterruptCtrl
     * @note   Get interrupt control info
     * @retval SensorBHI260APControl class
     */
    SensorBHI260APControl getInterruptCtrl();


    /**
     * @brief  isReady
     * @note   Query whether the sensor is ready
     * @retval true OK , false Not ready
     */
    bool isReady();

    /**
     * @brief  getKernelVersion
     * @note   Get the sensor firmware kernel version
     * @retval 2 bytes
     */
    uint16_t getKernelVersion();


    /**
     * @brief  onEvent
     * @note   Registered sensor event callback function
     * @param  callback: Callback Function
     * @retval None
     */
    void onEvent(SensorEventCallback callback);

    /**
     * @brief  removeEvent
     * @note   Remove sensor event callback function
     * @retval None
     */
    void removeEvent();

    /**
     * @brief  onResultEvent
     * @note   Registered sensor result callback function , The same sensor ID can register multiple event callbacks.
     *         Please note that you should not register the same event callback repeatedly.
     * @param  sensor_id: Sensor ID , see enum BoschSensorID
     * @param  callback: Callback Function
     * @param  *user_data: user data,can be null
     * @retval bool true-> Success false-> failure
     */
    bool onResultEvent(BoschSensorID sensor_id, SensorDataParseCallback callback, void *user_data = nullptr);

    /**
     * @brief  removeResultEvent
     * @note   Remove the registered result callback function
     * @param  sensor_id: Sensor ID , see enum BoschSensorID
     * @param  callback: Callback Function
     * @retval bool true-> Success false-> failure
     */
    bool removeResultEvent(BoschSensorID sensor_id, SensorDataParseCallback callback);

    /**
     * @brief  setProcessBufferSize
     * @note   The default value is 512Bytes , Must be called before initialization
     * @param  size: The set buffer size is requested by malloc, and if PSRAM is enabled, it is requested from PSRAM
     * @retval None
     */
    void setProcessBufferSize(uint32_t size);

    /**
     * @brief  uploadFirmware
     * @note   Update BHI sensor firmware
     * @param  *firmware: Firmware data address
     * @param  length: Firmware data length
     * @param  write2Flash: 1 is written to external flash, 0 is written to RAM
     * @retval bool true-> Success false-> failure
     */
    bool uploadFirmware(const uint8_t *firmware, uint32_t length, bool write2Flash = false);

    /**
     * @brief  getError
     * @note   Get the error status string
     * @retval string
     */
    const char *getError();

    /**
     * @brief  configure
     * @note   Sensor Configuration
     * @param  sensor_id: Sensor ID , see enum BoschSensorID
     * @param  sample_rate: Data output rate, unit: HZ
     * @param  report_latency_ms: Report interval in milliseconds
     * @return bool true-> Success false-> failure
     */
    bool configure(uint8_t sensor_id, float sample_rate, uint32_t report_latency_ms);

    /**
     * @brief  configureRange
     * @note   Set range of the sensor
     * @param  sensor_id: Sensor ID , see enum BoschSensorID
     * @param  range:     Range for selected SensorID. See Table 79 in BHY260 datasheet 109 page
     * @retval  bool true-> Success false-> failure
     */
    bool configureRange(uint8_t sensor_id, uint16_t range);


    /**
     * @brief  getConfigure
     * @note   Get sensor configuration
     * @param  sensor_id: Sensor ID , see enum BoschSensorID
     * @retval  SensorConfig
     */
    SensorConfig getConfigure(uint8_t sensor_id);

    /**
     * @brief  getScaling
     * @note   Get sensor scale factor
     * @param  sensor_id: Sensor ID , see enum BoschSensorID
     * @retval scale factor
     */
    float getScaling(uint8_t sensor_id);

    /**
     * @brief  setFirmware
     * @note   Set the firmware
     * @param  *image: firmware data address
     * @param  image_len: firmware length
     * @param  write_flash: true : write to flash otherwise ram
     * @param  force_update: true, rewrite to flash or ram regardless of whether there is firmware, false, do not write if firmware is detected
     * @retval None
     */
    void setFirmware(const uint8_t *image, size_t image_len, bool write_flash = false, bool force_update = false);

    /**
     * @brief  getSensorName
     * @note   Get sensor name
     * @param  sensor_id: Sensor ID , see enum BoschSensorID
     * @retval sensor name
     */
    const char *getSensorName(uint8_t sensor_id);

    /**
     * @brief  getAccuracy
     * @note   Get an accuracy report
     * @retval Current report accuracy
     */
    uint8_t getAccuracy();

    /**
     * @brief  digitalRead
     * @note   Read GPIO level, only for custom firmware
     * @param  pin: see BHI260AP_aux_BMM150_BME280_Expand_GPIO example
     * @param  pullup: true is set pullup or input mode
     * @retval 1 is high ,0 is low
     */
    uint8_t digitalRead(uint8_t pin, bool pullup = false);

    /**
     * @brief  digitalWrite
     * @note   Write GPIO level, only for custom firmware
     * @param  pin: see BHI260AP_aux_BMM150_BME280_Expand_GPIO example
     * @param  level: 1 is high ,0 is low
     * @retval None
     */
    void digitalWrite(uint8_t pin, uint8_t level);

    /**
     * @brief  disableGpio
     * @note   Disable GPIO function
     * @param  pin: see BHI260AP_aux_BMM150_BME280_Expand_GPIO example
     * @retval None
     */
    void disableGpio(uint8_t pin);

    /**
     * @brief  setDebug
     * @note   Whether to enable chip debug output
     * @param  enable: true Enable message debug , false disable debug , Requires firmware support, the default firmware will not output any messages
     * @param  &serial: Stream
     * @retval None
     */
    void setDebugKernel(bool enable);

    /**
     * @brief setDebugCallback
     * @param  cb: Sensor debug output callback function , Requires firmware support, the default firmware will not output any messages
     * @retval None
     */
    void setDebugCallback(SensorDebugMessageCallback cb);

    /**
     * @brief  getPhySensorInfo
     * @note   Get all information about physical sensors
     * @param  sens_id: ID of the physical sensor
     * @retval BoschPhySensorInfo Class
     */
    BoschPhySensorInfo getPhySensorInfo(uint8_t sens_id);

    /**
     * @brief  getSensorInfo
     * @note   Get all information about sensors
     * @retval BoschSensorInfo Class
     */
    BoschSensorInfo getSensorInfo();


    /**
     * @brief  setMaxiTransferSize
     * @note   Set the maximum number of bytes transmitted by the interface , Called before begin
     * @param  size_of_bytes: The maximum transmission bytes of different platforms are different.
     *                        Set it according to the platform. If not set, the default is I2C 32 bytes, SPI 256 bytes.
     * @retval None
     */
    void setMaxiTransferSize(uint16_t size_of_bytes);


    /**
     * @brief  setUpdateProcessCallback
     * @note   Set the callback function of the firmware update process to obtain the update progress
     * @param  callback: callback function
     * @param  *user_data: user data, can be nullptr
     * @retval None
     */
    void setUpdateProcessCallback(ProcessCallback callback, void *user_data = nullptr);

    /**
     * @brief  availableSensorNums
     * @note   Get the number of available sensors
     * @retval available sensors
     */
    uint8_t availableSensorNums();


    /**
    * @brief Set the axis remapping for the sensor based on the specified orientation.
    *
    * This function allows you to configure the sensor's axis remapping according to a specific
    * physical orientation of the chip. By passing one of the values from the SensorRemap enum,
    * you can ensure that the sensor data is correctly interpreted based on how the chip is placed.
    * [bst-bhi260ab-ds000.pdf](https://www.mouser.com/datasheet/2/783/bst-bhi260ab-ds000-1816249.pdf)
    * 20.3 Sensing axes and axes remapping
    * @param remap An enumeration value from SensorRemap that specifies the desired axis remapping.
    * @return Returns true if the axis remapping is successfully set; false otherwise.
    */
    bool setRemapAxes(SensorRemap remap);

protected:

    void parseData(const struct bhy2_fifo_parse_data_info *fifo, void *user_data) override;

    void parseMetaEvent(const struct bhy2_fifo_parse_data_info *callback_info, void *user_data) override;

    void parseDebugMessage(const struct bhy2_fifo_parse_data_info *callback_info, void *user_data) override;

private:

    /**
     * @brief  bootFromFlash
     * @note   Boot from external flash
     * @retval bool true-> Success false-> failure
     */
    bool bootFromFlash();

    void print_boot_status(uint8_t boot_status);

    bool initImpl(bhy2_intf interface);

protected:

    std::unique_ptr<SensorCommBase> comm;
    std::unique_ptr<SensorHal> hal;
    std::unique_ptr<SensorCommStatic> staticComm;
    std::unique_ptr<struct bhy2_dev> _bhy2;
    int                 _rst;
    int8_t              _error_code;
    uint8_t            *_processBuffer;
    size_t              _processBufferSize;
    const uint8_t      *_firmware_stream;
    size_t              _firmware_size;
    bool                _write_flash;
    bool                _boot_from_flash;
    bool                _force_update;
    int                 _max_rw_length;
    uint8_t             _accuracy;      /* Accuracy is reported as a meta event. */
    bool                _debug;
    ProcessCallback     _process_callback;
    void               *_process_callback_user_data;
    SensorEventCallback _event_callback;
    SensorDebugMessageCallback _debug_callback;
    BoschParseCallbackManager  _callback_manager;
    uint8_t             _sensor_available_nums;
    char                _err_buffer[128];

};
