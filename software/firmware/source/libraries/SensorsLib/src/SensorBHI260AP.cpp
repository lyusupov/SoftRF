/**
 *
 * @license MIT License
 *
 * Copyright (c) 2024 lewis he
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
 * @file      SensorBHI260AP.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-05-27
 *
 */
#include "SensorBHI260AP.hpp"
#include "bosch/BoschParseStatic.hpp"

#define BHY2_RLST_CHECK(ret, str, val) \
    do                                 \
    {                                  \
        if (ret)                       \
        {                              \
            log_e(str);                \
            return val;                 \
        }                               \
    } while (0)

static constexpr uint16_t max_process_buffer_size = 512;

SensorBHI260AP::SensorBHI260AP(): comm(nullptr),
    hal(nullptr),
    staticComm(nullptr),
    _rst(-1), _error_code(0),
    _processBuffer(nullptr),
    _processBufferSize(max_process_buffer_size),
    _firmware_stream(nullptr),
    _firmware_size(0),
    _write_flash(false),
    _boot_from_flash(false),
    _force_update(false),
    _max_rw_length(-1),
    _accuracy(0),
    _debug(false),
    _process_callback(nullptr),
    _process_callback_user_data(nullptr),
    _event_callback(nullptr),
    _debug_callback(nullptr)
{
}

SensorBHI260AP::~SensorBHI260AP()
{
    if (_processBuffer) {
        free(_processBuffer);
    }
    _processBuffer = NULL;
}

void SensorBHI260AP::setPins(int rst)
{
    _rst = rst;
}

#if defined(ARDUINO)
bool SensorBHI260AP::begin(TwoWire &wire, uint8_t addr, int sda, int scl)
{
    if (!beginCommonStatic<SensorCommI2C, HalArduino>(comm, staticComm, hal, wire, addr, sda, scl)) {
        return false;
    }
    return initImpl(BHY2_I2C_INTERFACE);
}

bool SensorBHI260AP::begin(SPIClass &spi, uint8_t csPin, int mosi, int miso, int sck)
{
    if (!beginCommonStatic<SensorCommSPI, HalArduino>(comm,
            staticComm, hal,
            spi, csPin, mosi, miso, sck)) {
        return false;
    }
    return initImpl(BHY2_SPI_INTERFACE);
}

#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
bool SensorBHI260AP::begin(i2c_port_t port_num, uint8_t addr, int sda, int scl)
{
    if (!beginCommonStatic<SensorCommI2C, HalEspIDF>(comm, staticComm, hal, port_num, addr, sda, scl)) {
        return false;
    }
    return initImpl(BHY2_I2C_INTERFACE);
}
#else   //USEING_I2C_LEGACY
bool SensorBHI260AP::begin(i2c_master_bus_handle_t handle, uint8_t addr)
{
    if (!beginCommonStatic<SensorCommI2C, HalEspIDF>(comm, staticComm, hal, handle, addr)) {
        return false;
    }
    return initImpl(BHY2_I2C_INTERFACE);
}
#endif  //USEING_I2C_LEGACY


bool SensorBHI260AP::begin(spi_host_device_t host, spi_device_handle_t handle, uint8_t csPin, int mosi, int miso, int sck)
{
    if (!beginCommonStatic<SensorCommSPI, HalEspIDF>(comm,
            staticComm, hal,
            host, handle, csPin, mosi, miso, sck)) {
        return false;
    }
    return initImpl(BHY2_SPI_INTERFACE);
}

#endif  //ARDUINO

bool SensorBHI260AP::begin(CommInterface interface,
                           SensorCommCustom::CustomCallback callback,
                           SensorCommCustomHal::CustomHalCallback hal_callback,
                           uint8_t addr)
{
    if (!beginCommCustomCallback<SensorCommCustom, SensorCommCustomHal>(interface,
            callback, hal_callback, addr, comm, hal)) {
        return false;
    }
    return initImpl(static_cast<bhy2_intf>(interface));
}

/**
 * @brief  reset
 * @note   Reset sensor
 * @retval None
 */
void SensorBHI260AP::reset()
{
    if (_rst != -1) {
        hal->digitalWrite(_rst, HIGH);
        hal->delay(5);
        hal->digitalWrite(_rst, LOW);
        hal->delay(10);
        hal->digitalWrite(_rst, HIGH);
        hal->delay(5);
    }
}

/**
 * @brief  update
 * @note   Update sensor fifo data
 * @retval None
 */
void SensorBHI260AP::update()
{
    if (!_processBuffer) {
        log_e("Process buffer is empty.");
        return;
    }
    bhy2_get_and_process_fifo(_processBuffer, _processBufferSize, _bhy2.get());
}

/**
 * @brief  setBootFromFlash
 * @note   Set whether to start from external flash
 * @param  boot_from_flash: true boot form flash or boot form ram
 * @retval None
 */
void SensorBHI260AP::setBootFromFlash(bool boot_from_flash)
{
    _boot_from_flash = boot_from_flash;
}

/**
 * @brief  getHandler
 * @note   Get the native BHI API handle
 * @retval handle
 */
bhy2_dev *SensorBHI260AP::getHandler()
{
    return _bhy2.get();
}

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
bool SensorBHI260AP::setInterruptCtrl(uint8_t data)
{
    _error_code = bhy2_set_host_interrupt_ctrl(data, _bhy2.get());
    if (_error_code != BHY2_OK) {
        return false;
    }
    return true;
}

/**
 * @brief  getInterruptCtrl
 * @note   Get interrupt control info
 * @retval SensorBHI260APCtrl class
 */
SensorBHI260APControl SensorBHI260AP::getInterruptCtrl()
{
    uint8_t data;
    _error_code = bhy2_get_host_interrupt_ctrl(&data, _bhy2.get());
    SensorBHI260APControl result;
    if (_error_code == BHY2_OK) {
        result.wakeUpFIFOEnabled = !(data & BHY2_ICTL_DISABLE_FIFO_W);
        result.nonWakeUpFIFOEnabled = !(data & BHY2_ICTL_DISABLE_FIFO_NW);
        result.statusFIFOEnabled = !(data & BHY2_ICTL_DISABLE_STATUS_FIFO);
        result.debuggingEnabled = !(data & BHY2_ICTL_DISABLE_DEBUG);
        result.faultEnabled = !(data & BHY2_ICTL_DISABLE_FAULT);
        result.interruptIsActiveLow = (data & BHY2_ICTL_ACTIVE_LOW);
        result.interruptIsPulseTriggered = (data & BHY2_ICTL_EDGE);
        result.interruptPinDriveIsOpenDrain = (data & BHY2_ICTL_OPEN_DRAIN);
    }
    return result;
}


/**
 * @brief  isReady
 * @note   Query whether the sensor is ready
 * @retval 1 OK , 0 Not ready
 */
bool SensorBHI260AP::isReady()
{
    uint8_t  boot_status = 0;
    _error_code = bhy2_get_boot_status(&boot_status, _bhy2.get());
    log_d("boot_status:0x%x", boot_status);
    if (_error_code != BHY2_OK) {
        return false;
    }
    return (boot_status & BHY2_BST_HOST_INTERFACE_READY);
}

/**
 * @brief  getKernelVersion
 * @note   Get the sensor firmware kernel version
 * @retval 2 bytes
 */
uint16_t SensorBHI260AP::getKernelVersion()
{
    uint16_t version = 0;
    _error_code = bhy2_get_kernel_version(&version, _bhy2.get());
    if ((_error_code != BHY2_OK) && (version == 0)) {
        return 0;
    }
    log_d("Boot successful. Kernel version %u.", version);
    return version;
}


/**
 * @brief  onEvent
 * @note   Registered sensor event callback function
 * @param  callback: Callback Function
 * @retval None
 */
void SensorBHI260AP::onEvent(SensorEventCallback callback)
{
    _event_callback = callback;
}

/**
 * @brief  removeEvent
 * @note   Remove sensor event callback function
 * @retval None
 */
void SensorBHI260AP::removeEvent()
{
    _event_callback = NULL;
}

/**
 * @brief  onResultEvent
 * @note   Registered sensor result callback function , The same sensor ID can register multiple event callbacks.
 *         Please note that you should not register the same event callback repeatedly.
 * @param  sensor_id: Sensor ID , see enum BoschSensorID
 * @param  callback: Callback Function
 * @param  *user_data: user data,can be null
 * @retval bool true-> Success false-> failure
 */
bool SensorBHI260AP::onResultEvent(BoschSensorID sensor_id, SensorDataParseCallback callback, void *user_data)
{
    if (!bhy2_is_sensor_available(sensor_id, _bhy2.get())) {
        log_e("%s not present", getSensorName(sensor_id)); return false;
    }
    return  _callback_manager.add(sensor_id, callback, user_data);
}

/**
 * @brief  removeResultEvent
 * @note   Remove the registered result callback function
 * @param  sensor_id: Sensor ID , see enum BoschSensorID
 * @param  callback: Callback Function
 * @retval bool true-> Success false-> failure
 */
bool SensorBHI260AP::removeResultEvent(BoschSensorID sensor_id, SensorDataParseCallback callback)
{
    if (!bhy2_is_sensor_available(sensor_id, _bhy2.get())) {
        log_e("%s not present", getSensorName(sensor_id)); return false;
    }
    return  _callback_manager.remove(sensor_id, callback);
}

/**
 * @brief  setProcessBufferSize
 * @note   The default value is 512Bytes , Must be called before initialization
 * @param  size: The set buffer size is requested by malloc, and if PSRAM is enabled, it is requested from PSRAM
 * @retval None
 */
void SensorBHI260AP::setProcessBufferSize(uint32_t size)
{
    if (_processBuffer) {
        log_e("The buffer size must be set before begin");
        return;
    }
    _processBufferSize = size;
}

/**
 * @brief  uploadFirmware
 * @note   Update BHI sensor firmware
 * @param  *firmware: Firmware data address
 * @param  length: Firmware data length
 * @param  write2Flash: 1 is written to external flash, 0 is written to RAM
 * @retval true success or failed
 */
bool SensorBHI260AP::uploadFirmware(const uint8_t *firmware, uint32_t length, bool write2Flash)
{
    uint8_t sensor_error;
    uint8_t boot_status;

    log_d("Upload Firmware ...");

    _error_code = bhy2_get_boot_status(&boot_status, _bhy2.get());
    BHY2_RLST_CHECK(_error_code != BHY2_OK, "bhy2_get_boot_status failed!", false);

    if (write2Flash) {
        if (boot_status & BHY2_BST_FLASH_DETECTED) {
            uint32_t start_addr = BHY2_FLASH_SECTOR_START_ADDR;
            uint32_t end_addr = start_addr + length;
            log_d("Flash detected. Erasing flash to upload firmware");
            _error_code = bhy2_erase_flash(start_addr, end_addr, _bhy2.get());
            BHY2_RLST_CHECK(_error_code != BHY2_OK, "bhy2_erase_flash failed!", false);
        } else {
            log_e("Flash not detected");
            return false;
        }
        log_d("Loading firmware into FLASH.");
        _error_code = bhy2_upload_firmware_to_flash(firmware, length, _bhy2.get(),
                      _process_callback,
                      _process_callback_user_data);
        BHY2_RLST_CHECK(_error_code != BHY2_OK, "bhy2_upload_firmware_to_flash failed!", false);
        log_d("Loading firmware into FLASH Done");
    } else {
        log_d("Loading firmware into RAM.");
        log_d("upload size = %lu", length);
        _error_code = bhy2_upload_firmware_to_ram(firmware, length, _bhy2.get());
        BHY2_RLST_CHECK(_error_code != BHY2_OK, "bhy2_upload_firmware_to_ram failed!", false);
        log_d("Loading firmware into RAM Done");
    }

    _error_code = bhy2_get_error_value(&sensor_error, _bhy2.get());
    BHY2_RLST_CHECK(_error_code != BHY2_OK, "bhy2_get_error_value failed!", false);
    if (sensor_error != BHY2_OK) {
        _error_code = bhy2_get_error_value(&sensor_error, _bhy2.get());
        log_e("%s", get_sensor_error_text(sensor_error));
        return false;
    }


    if (write2Flash) {
        log_d("Booting from FLASH.");
        _error_code = bhy2_boot_from_flash(_bhy2.get());
    } else {
        log_d("Booting from RAM.");
        _error_code = bhy2_boot_from_ram(_bhy2.get());
    }
    BHY2_RLST_CHECK(_error_code != BHY2_OK, "_bhy2 boot failed!", false);

    _error_code = bhy2_get_error_value(&sensor_error, _bhy2.get());
    if (sensor_error) {
        log_e("%s", get_sensor_error_text(sensor_error));
        return false;
    }
    return sensor_error == BHY2_OK;
}


/**
 * @brief  getError
 * @note   Get the error status string
 * @retval string
 */
const char *SensorBHI260AP::getError()
{
    snprintf(_err_buffer, 128, "API:%s\nSensor:%s\n", get_api_error(_error_code), get_sensor_error_text(_error_code));
    return static_cast<const char *>(_err_buffer);
}

/**
 * @brief  configure
 * @note   Sensor Configuration
 * @param  sensor_id: Sensor ID , see enum BoschSensorID
 * @param  sample_rate: Data output rate, unit: HZ
 * @param  report_latency_ms: Report interval in milliseconds
 * @return bool true-> Success false-> failure
 */
bool SensorBHI260AP::configure(uint8_t sensor_id, float sample_rate, uint32_t report_latency_ms)
{
    if (!bhy2_is_sensor_available(sensor_id, _bhy2.get())) {
        log_e("%s not present", getSensorName(sensor_id)); return false;
    }
    _error_code = bhy2_set_virt_sensor_cfg(sensor_id, sample_rate, report_latency_ms, _bhy2.get());
    BHY2_RLST_CHECK(_error_code != BHY2_OK, "bhy2_set_virt_sensor_cfg failed!", false);
    log_d("Enable %s at %.2fHz.", get_sensor_name(sensor_id), sample_rate);
    return true;
}

/**
 * @brief  configureRange
 * @note   Set range of the sensor
 * @param  sensor_id: Sensor ID , see enum BoschSensorID
 * @param  range:     Range for selected SensorID. See Table 79 in BHY260 datasheet 109 page
 * @retval  bool true-> Success false-> failure
 */
bool SensorBHI260AP::configureRange(uint8_t sensor_id, uint16_t range)
{
    _error_code = bhy2_set_virt_sensor_range(sensor_id, range, _bhy2.get());
    BHY2_RLST_CHECK(_error_code != BHY2_OK, "bhy2_set_virt_sensor_range failed!", false);
    return true;
}


/**
 * @brief  getConfigure
 * @note   Get sensor configuration
 * @param  sensor_id: Sensor ID , see enum BoschSensorID
 * @retval  SensorConfig
 */
SensorConfig SensorBHI260AP::getConfigure(uint8_t sensor_id)
{
    bhy2_virt_sensor_conf conf;
    bhy2_get_virt_sensor_cfg(sensor_id, &conf, _bhy2.get());
    log_d("range:%u sample_rate:%f latency:%lu sensitivity:%u\n", conf.range, conf.sample_rate, conf.latency, conf.sensitivity);
    return conf;
}

/**
 * @brief  getScaling
 * @note   Get sensor scale factor
 * @param  sensor_id: Sensor ID , see enum BoschSensorID
 * @retval scale factor
 */
float SensorBHI260AP::getScaling(uint8_t sensor_id)
{
    return get_sensor_default_scaling(sensor_id);
}

/**
 * @brief  setFirmware
 * @note   Set the firmware
 * @param  *image: firmware data address
 * @param  image_len: firmware length
 * @param  write_flash: true : write to flash otherwise ram
 * @param  force_update: true, rewrite to flash or ram regardless of whether there is firmware, false, do not write if firmware is detected
 * @retval None
 */
void SensorBHI260AP::setFirmware(const uint8_t *image, size_t image_len, bool write_flash, bool force_update)
{
    _firmware_stream = image;
    _firmware_size = image_len;
    _write_flash = write_flash;
    _force_update = force_update;
}

/**
 * @brief  getSensorName
 * @note   Get sensor name
 * @param  sensor_id: Sensor ID , see enum BoschSensorID
 * @retval sensor name
 */
const char *SensorBHI260AP::getSensorName(uint8_t sensor_id)
{
    return get_sensor_name(sensor_id);
}

// Get an accuracy report
uint8_t SensorBHI260AP::getAccuracy()
{
    return _accuracy;
}

/**
 * @brief  digitalRead
 * @note   Read GPIO level, only for custom firmware
 * @param  pin: see BHI260AP_aux_BMM150_BME280_Expand_GPIO example
 * @param  pullup: true is set pullup or input mode
 * @retval 1 is high ,0 is low
 */
uint8_t SensorBHI260AP::digitalRead(uint8_t pin, bool pullup)
{
    if (pin > JTAG_DIO)return 0;
    uint32_t pin_mask = pin   | BHY2_GPIO_SET;
    if (pullup) {
        pin_mask |= (BHY2_INPUT_PULLUP << 8);
    } else {
        pin_mask |= (BHY2_INPUT << 8);
    }
    bhy2_set_virt_sensor_cfg(BoschSensorID::GPIO_EXP, (float)pin_mask, 0, _bhy2.get());
    pin_mask = pin /*GetCmd*/;
    bhy2_set_virt_sensor_cfg(BoschSensorID::GPIO_EXP, (float)pin_mask, 0, _bhy2.get());
    bhy2_virt_sensor_conf conf;
    bhy2_get_virt_sensor_cfg(BoschSensorID::GPIO_EXP, &conf, _bhy2.get());
    uint8_t level = conf.sample_rate;
    return level;
}

/**
 * @brief  hal->digitalWrite
 * @note   Write GPIO level, only for custom firmware
 * @param  pin: see BHI260AP_aux_BMM150_BME280_Expand_GPIO example
 * @param  level: 1 is high ,0 is low
 * @retval None
 */
void SensorBHI260AP::digitalWrite(uint8_t pin, uint8_t level)
{
    if (pin > JTAG_DIO)return;
    uint32_t pin_mask = pin  | (BHY2_OUTPUT << 8) | (level << 6) | BHY2_GPIO_SET ;
    bhy2_set_virt_sensor_cfg(BoschSensorID::GPIO_EXP, (float)pin_mask, 0, _bhy2.get());
}

/**
 * @brief  disableGpio
 * @note   Disable GPIO function
 * @param  pin: see BHI260AP_aux_BMM150_BME280_Expand_GPIO example
 * @retval None
 */
void SensorBHI260AP::disableGpio(uint8_t pin)
{
    if (pin > JTAG_DIO)return;
    uint32_t pin_mask = pin  | (BHY2_OPEN_DRAIN << 8) | BHY2_GPIO_SET;
    bhy2_set_virt_sensor_cfg(BoschSensorID::GPIO_EXP, (float)pin_mask, 0, _bhy2.get());
}

/**
 * @brief  setDebugKernel
 * @note   Whether to enable chip debug output , Must be called before begin, otherwise it will be invalid
 * @param  enable: true Enable message debug , false disable debug , Requires firmware support, the default firmware will not output any messages
 * @retval None
 */
void SensorBHI260AP::setDebugKernel(bool enable)
{
    _debug = enable;
}

/**
 * @brief setDebugCallback
 * @param  cb: Sensor debug output callback function , Requires firmware support, the default firmware will not output any messages
 * @retval None
 */
void SensorBHI260AP::setDebugCallback(SensorDebugMessageCallback cb)
{
    _debug_callback = cb;
}


/**
 * @brief  getPhySensorInfo
 * @note   Get all information about physical sensors
 * @param  sens_id: ID of the physical sensor
 * @retval BoschPhySensorInfo Class
 */
#define INT4_TO_INT8(INT4)  ((int8_t)(((INT4) > 1) ? -1 : (INT4)))

BoschPhySensorInfo SensorBHI260AP::getPhySensorInfo(uint8_t sens_id)
{
    BoschPhySensorInfo result;

    struct bhy2_phys_sensor_info psi;

    memset(&psi, 0, sizeof(psi));

    if (!_bhy2.get()) return result;

    uint16_t param_id = (uint16_t)(0x0120 | sens_id);
    if (param_id >= 0x0121 && param_id <= 0x0160) {
        int8_t assert_rslt = (bhy2_get_phys_sensor_info(sens_id, &psi, _bhy2.get()));
        if (assert_rslt == BHY2_OK) {
            result.sensor_type = psi.sensor_type;
            result.driver_id = psi.driver_id;
            result.driver_version = psi.driver_version;
            result.power_current = psi.power_current / 10.f;
            result.curr_range = psi.curr_range.u16_val;
            result.irq_status = psi.flags & 0x01;
            result.master_intf = (psi.flags >> 1) & 0x0F;
            result.power_mode = (psi.flags >> 5) & 0x07;
            result.slave_address = psi.slave_address;
            result.gpio_assignment = psi.gpio_assignment;
            result.curr_rate = psi.curr_rate.f_val;
            result.num_axis = psi.num_axis;
            struct bhy2_orient_matrix ort_mtx = { 0 };

            ort_mtx.c[0] = INT4_TO_INT8(psi.orientation_matrix[0] & 0x0F);
            ort_mtx.c[1] = INT4_TO_INT8(psi.orientation_matrix[0] >> 8);
            ort_mtx.c[2] = INT4_TO_INT8(psi.orientation_matrix[1] & 0x0F);
            ort_mtx.c[3] = INT4_TO_INT8(psi.orientation_matrix[1] >> 8);
            ort_mtx.c[4] = INT4_TO_INT8(psi.orientation_matrix[2] & 0x0F);
            ort_mtx.c[5] = INT4_TO_INT8(psi.orientation_matrix[2] >> 8);
            ort_mtx.c[6] = INT4_TO_INT8(psi.orientation_matrix[3] & 0x0F);
            ort_mtx.c[7] = INT4_TO_INT8(psi.orientation_matrix[3] >> 8);
            ort_mtx.c[8] = INT4_TO_INT8(psi.orientation_matrix[4] & 0x0F);
            for (int i = 0; i < 9; ++i) {
                result.orientation_matrix[i] = ort_mtx.c[i];
            }
            result.reserved = psi.reserved;
        }
    }
    return result;
}

/**
 * @brief  getSensorInfo
 * @note   Get all information about sensors
 * @retval BoschSensorInfo Class
 */
BoschSensorInfo SensorBHI260AP::getSensorInfo()
{
    BoschSensorInfo sensorInfo;
    if (bhy2_get_product_id(&sensorInfo.product_id, _bhy2.get()) != BHY2_OK) {
        log_e("bhy2_get_product_id failed!");
    }
    if (bhy2_get_kernel_version(&sensorInfo.kernel_version, _bhy2.get()) != BHY2_OK) {
        log_e("bhy2_get_kernel_version failed!");
    }
    if (bhy2_get_user_version(&sensorInfo.user_version, _bhy2.get()) != BHY2_OK) {
        log_e("bhy2_get_user_version failed!");
    }
    if (bhy2_get_rom_version(&sensorInfo.rom_version, _bhy2.get()) != BHY2_OK) {
        log_e("bhy2_get_rom_version failed!");
    }
    if (bhy2_get_host_status(&sensorInfo.host_status, _bhy2.get()) != BHY2_OK) {
        log_e("bhy2_get_host_status failed!");
    }
    if (bhy2_get_feature_status(&sensorInfo.feat_status, _bhy2.get()) != BHY2_OK) {
        log_e("bhy2_get_feature_status failed!");
    }
    if (bhy2_get_boot_status(&sensorInfo.boot_status, _bhy2.get()) != BHY2_OK) {
        log_e("bhy2_get_boot_status failed!");
    }
    if (bhy2_get_error_value(&sensorInfo.sensor_error, _bhy2.get()) != BHY2_OK) {
        log_e("bhy2_get_error_value failed!");
    }
    if (sensorInfo.info) {
        for (uint8_t i = 0; i < BHY2_SENSOR_ID_MAX; i++) {
            if (bhy2_is_sensor_available(i, _bhy2.get())) {
                if (bhy2_get_sensor_info(i, &sensorInfo.info[i], _bhy2.get()) != BHY2_OK) {
                    log_e("bhy2_get_sensor_info [%u] failed!", i);
                }
            }
        }
    }
    sensorInfo.dev = _bhy2.get();
    return sensorInfo;
}


/**
 * @brief  setMaxiTransferSize
 * @note   Set the maximum number of bytes transmitted by the interface , Called before begin
 * @param  size_of_bytes: The maximum transmission bytes of different platforms are different.
 *                        Set it according to the platform. If not set, the default is I2C 32 bytes, SPI 256 bytes.
 * @retval None
 */
void SensorBHI260AP::setMaxiTransferSize(uint16_t size_of_bytes)
{
    if (_processBuffer) {
        log_e("Must be called before begin");
        return;
    }
    _max_rw_length = size_of_bytes;
}

/**
 * @brief  setUpdateProcessCallback
 * @note   Set the callback function of the firmware update process to obtain the update progress
 * @param  callback: callback function
 * @param  *user_data: user data, can be nullptr
 * @retval None
 */
void SensorBHI260AP::setUpdateProcessCallback(ProcessCallback callback, void *user_data)
{
    _process_callback = callback;
}

/**
 * @brief  availableSensorNums
 * @note   Get the number of available sensors
 * @retval available sensors
 */
uint8_t SensorBHI260AP::availableSensorNums()
{
    return _sensor_available_nums;
}

/**
* @brief Set the axis remapping for the sensor based on the specified orientation.
*
* This function allows you to configure the sensor's axis remapping according to a specific
* physical orientation of the chip. By passing one of the values from the SensorRemap enum,
* you can ensure that the sensor data is correctly interpreted based on how the chip is placed.
*
* @param remap An enumeration value from SensorRemap that specifies the desired axis remapping.
* @return Returns true if the axis remapping is successfully set; false otherwise.
*/
bool SensorBHI260AP::setRemapAxes(SensorRemap remap)
{
    if (remap > BOTTOM_LAYER_BOTTOM_LEFT_CORNER) {
        log_e("Invalid SensorRemap value passed to setRemapAxes!");
        return false;
    }

    // Acceleration - related orientation matrices for different mounting directions
    struct bhy2_orient_matrix acc_matrices[] = {
        // P0 mounting direction, default direction, axis direction is consistent with the default coordinate system
        {1, 0, 0, 0, 1, 0, 0, 0, 1},
        // P1 mounting direction, P0 is rotated 90° clockwise around the Z - axis
        {0, -1, 0, 1, 0, 0, 0, 0, 1},
        // P2 mounting direction, P0 is rotated 180° clockwise around the Z - axis
        {-1, 0, 0, 0, -1, 0, 0, 0, 1},
        // P3 mounting direction, P0 is rotated 270° clockwise around the Z - axis
        {0, 1, 0, -1, 0, 0, 0, 0, 1},
        // P4 mounting direction, P0 is flipped vertically (rotated 180° around the X - axis)
        {1, 0, 0, 0, -1, 0, 0, 0, -1},
        // P5 mounting direction, P4 is rotated 90° clockwise around the Z - axis
        {0, 1, 0, 1, 0, 0, 0, 0, -1},
        // P6 mounting direction, P4 is rotated 180° clockwise around the Z - axis
        {-1, 0, 0, 0, 1, 0, 0, 0, -1},
        // P7 mounting direction, P4 is rotated 270° clockwise around the Z - axis
        {0, -1, 0, -1, 0, 0, 0, 0, -1}
    };

    // Gyroscope - related orientation matrices for different mounting directions
    struct bhy2_orient_matrix gyro_matrices[] = {
        // P0 mounting direction, default direction, axis direction is consistent with the default coordinate system
        {1, 0, 0, 0, 1, 0, 0, 0, 1},
        // P1 mounting direction, P0 is rotated 90° clockwise around the Z - axis
        {0, -1, 0, 1, 0, 0, 0, 0, 1},
        // P2 mounting direction, P0 is rotated 180° clockwise around the Z - axis
        {-1, 0, 0, 0, -1, 0, 0, 0, 1},
        // P3 mounting direction, P0 is rotated 270° clockwise around the Z - axis
        {0, 1, 0, -1, 0, 0, 0, 0, 1},
        // P4 mounting direction, P0 is flipped vertically (rotated 180° around the X - axis)
        {1, 0, 0, 0, -1, 0, 0, 0, -1},
        // P5 mounting direction, P4 is rotated 90° clockwise around the Z - axis
        {0, 1, 0, 1, 0, 0, 0, 0, -1},
        // P6 mounting direction, P4 is rotated 180° clockwise around the Z - axis
        {-1, 0, 0, 0, 1, 0, 0, 0, -1},
        // P7 mounting direction, P4 is rotated 270° clockwise around the Z - axis
        {0, -1, 0, -1, 0, 0, 0, 0, -1}
    };

    // Set the orientation matrix for the accelerometer
    _error_code = bhy2_set_orientation_matrix(BHY2_PHYS_SENSOR_ID_ACCELEROMETER, acc_matrices[remap], _bhy2.get());
    if (_error_code != BHY2_OK) {
        log_e("Set acceleration orientation matrix failed!");
        return false;
    }
    // Set the orientation matrix for the gyroscope
    _error_code = bhy2_set_orientation_matrix(BHY2_PHYS_SENSOR_ID_GYROSCOPE, gyro_matrices[remap], _bhy2.get());
    if (_error_code != BHY2_OK) {
        log_e("Set gyroscope orientation matrix failed!");
        return false;
    }
    return true;
}

/**
 * @brief  bootFromFlash
 * @note   Boot from external flash
 * @retval bool true-> Success false-> failure
 */
bool SensorBHI260AP::bootFromFlash()
{
    int8_t rslt;
    uint8_t boot_status, feat_status;
    uint8_t error_val = 0;
    uint16_t tries = 300; /* Wait for up to little over 3s */

    log_d("Waiting for firmware verification to complete");
    do {
        _error_code = bhy2_get_boot_status(&boot_status, _bhy2.get());
        BHY2_RLST_CHECK(_error_code != BHY2_OK, "bhy2_get_boot_status failed!", false);
        if (boot_status & BHY2_BST_FLASH_VERIFY_DONE) {
            break;
        }
        hal->delay(10);
    } while (tries--);

    _error_code = bhy2_get_boot_status(&boot_status, _bhy2.get());
    BHY2_RLST_CHECK(_error_code != BHY2_OK, "bhy2_get_boot_status failed!", false);
    print_boot_status(boot_status);

    if (boot_status & BHY2_BST_HOST_INTERFACE_READY) {

        if (boot_status & BHY2_BST_FLASH_DETECTED) {

            /* If no firmware is running, boot from Flash */
            log_d("Booting from flash");
            rslt = bhy2_boot_from_flash(_bhy2.get());
            if (rslt != BHY2_OK) {
                log_e("%s. Booting from flash failed.\r\n", get_api_error(rslt));
                _error_code = bhy2_get_regs(BHY2_REG_ERROR_VALUE, &error_val, 1, _bhy2.get());
                BHY2_RLST_CHECK(_error_code != BHY2_OK, "bhy2_get_regs failed!", false);
                if (error_val) {
                    log_e("%s\r\n", get_sensor_error_text(error_val));
                }
                return false;
            }

            _error_code = bhy2_get_boot_status(&boot_status, _bhy2.get());
            BHY2_RLST_CHECK(_error_code != BHY2_OK, "bhy2_get_boot_status failed!", false);
            print_boot_status(boot_status);

            if (!(boot_status & BHY2_BST_HOST_INTERFACE_READY)) {
                /* hub is not ready, need reset hub */
                log_d("Host interface is not ready, triggering a reset");
                _error_code = bhy2_soft_reset(_bhy2.get());
                BHY2_RLST_CHECK(_error_code != BHY2_OK, "bhy2_soft_reset failed!", false);
            }

            _error_code = (bhy2_get_feature_status(&feat_status, _bhy2.get()));
            BHY2_RLST_CHECK(_error_code != BHY2_OK, "Reading Feature status failed, booting from flash failed!", false);

        } else {
            log_e("Can't detect external flash");
            return false;
        }
    } else {
        log_e("Host interface is not ready");
        return false;
    }

    log_d("Booting from flash successful");
    return true;
}


void SensorBHI260AP::print_boot_status(uint8_t boot_status)
{
    log_d("Boot Status : 0x%02x: ", boot_status);
    if (boot_status & BHY2_BST_FLASH_DETECTED) {
        log_d("Flash detected. ");
    }

    if (boot_status & BHY2_BST_FLASH_VERIFY_DONE) {
        log_d("Flash verify done. ");
    }

    if (boot_status & BHY2_BST_FLASH_VERIFY_ERROR) {
        log_d("Flash verification failed. ");
    }

    if (boot_status & BHY2_BST_NO_FLASH) {
        log_d("No flash installed. ");
    }

    if (boot_status & BHY2_BST_HOST_INTERFACE_READY) {
        log_d("Host interface ready. ");
    }

    if (boot_status & BHY2_BST_HOST_FW_VERIFY_DONE) {
        log_d("Firmware verification done. ");
    }

    if (boot_status & BHY2_BST_HOST_FW_VERIFY_ERROR) {
        log_d("Firmware verification error. ");
    }

    if (boot_status & BHY2_BST_HOST_FW_IDLE) {
        log_d("Firmware halted. ");
    }
}

void SensorBHI260AP::parseData(const struct bhy2_fifo_parse_data_info *fifo, void *user_data)
{
    if (user_data != this) {
        return;
    }
#ifdef BHI260AP_PARSE_DATA_DUMP
    log_i("ID:[%d]:%s: DATA LEN:%u", fifo->sensor_id, get_sensor_name(fifo->sensor_id), fifo->data_size);
    SensorLibDumpBuffer(fifo->data_ptr, fifo->data_size);
#endif
    _callback_manager.call(fifo->sensor_id, fifo->data_ptr, fifo->data_size, fifo->time_stamp);
}

void SensorBHI260AP::parseMetaEvent(const struct bhy2_fifo_parse_data_info *callback_info, void *user_data)
{
    if (user_data != this) {
        return;
    }
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
    const char *event_text;
    // Remove warning
    UNUSED(byte1);
    UNUSED(byte2);
    UNUSED(event_text);

    if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT) {
        event_text = "[META EVENT]";
    } else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU) {
        event_text = "[META EVENT WAKE UP]";
    } else {
        return;
    }

    switch (meta_event_type) {
    case BHY2_META_EVENT_FLUSH_COMPLETE:
        log_i("%s Flush complete for sensor id %u", event_text, byte1);
        break;
    case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
        log_i("%s Sample rate changed for sensor id %u", event_text, byte1);
        break;
    case BHY2_META_EVENT_POWER_MODE_CHANGED:
        log_i("%s Power mode changed for sensor id %u", event_text, byte1);
        break;
    case BHY2_META_EVENT_ALGORITHM_EVENTS:
        log_i("%s Algorithm event", event_text);
        break;
    case BHY2_META_EVENT_SENSOR_STATUS:
        log_i("%s Accuracy for sensor id %u changed to %u", event_text, byte1, byte2);
        _accuracy = byte2;
        break;
    case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
        log_i("%s BSX event (do steps main)", event_text);
        break;
    case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
        log_i("%s BSX event (do steps calib)", event_text);
        break;
    case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
        log_i("%s BSX event (get output signal)", event_text);
        break;
    case BHY2_META_EVENT_SENSOR_ERROR:
        log_i("%s Sensor id %u reported error 0x%02X", event_text, byte1, byte2);
        break;
    case BHY2_META_EVENT_FIFO_OVERFLOW:
        log_i("%s FIFO overflow", event_text);
        break;
    case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
        log_i("%s Dynamic range changed for sensor id %u", event_text, byte1);
        break;
    case BHY2_META_EVENT_FIFO_WATERMARK:
        log_i("%s FIFO watermark reached", event_text);
        break;
    case BHY2_META_EVENT_INITIALIZED:
        log_i("%s Firmware initialized. Firmware version %u", event_text, ((uint16_t)byte2 << 8) | byte1);
        break;
    case BHY2_META_TRANSFER_CAUSE:
        log_i("%s Transfer cause for sensor id %u", event_text, byte1);
        break;
    case BHY2_META_EVENT_SENSOR_FRAMEWORK:
        log_i("%s Sensor framework event for sensor id %u", event_text, byte1);
        break;
    case BHY2_META_EVENT_RESET:
        log_i("%s Reset event", event_text);
        break;
    case BHY2_META_EVENT_SPACER:
        return;
    default:
        log_i("%s Unknown meta event with id: %u", event_text, meta_event_type);
        break;
    }

    if (_event_callback) {
        _event_callback(meta_event_type, byte1, byte2);
    }
}

void SensorBHI260AP::parseDebugMessage(const struct bhy2_fifo_parse_data_info *callback_info, void *user_data)
{
    if (user_data != this) {
        return;
    }
    uint8_t msg_length = 0;
    uint8_t debug_msg[17] = { 0 }; /* Max payload size is 16 bytes, adds a trailing zero if the payload is full */
    if (!callback_info) {
        log_i("Null reference");
        return;
    }
    msg_length = callback_info->data_ptr[0];
    memcpy(debug_msg, &callback_info->data_ptr[1], msg_length);
    debug_msg[msg_length] = '\0'; /* Terminate the string */
    log_d("[DEBUG MSG]: %s", debug_msg);

    if (_debug_callback) {
        _debug_callback((const char *)debug_msg);
    }
}


bool SensorBHI260AP::initImpl(bhy2_intf interface)
{
    uint8_t product_id = 0;

    if (_rst != -1) {
        hal->pinMode(_rst, OUTPUT);
    }

    reset();

    _bhy2 = std::make_unique<struct bhy2_dev>();
    BHY2_RLST_CHECK(!_bhy2, " Device handler malloc failed!", false);

    if (_max_rw_length == -1) {

#if defined(ARDUINO_ARCH_ESP32)
        switch (interface) {
        case BHY2_I2C_INTERFACE:
            // esp32s3 test I2C maximum read and write is 64 bytes
            _max_rw_length = 64;
            break;
        case BHY2_SPI_INTERFACE:
            // esp32s3 test SPI maximum read and write is 256 bytes
            _max_rw_length = 256;
            break;
        default:
            return false;
        }
#else
        // Other platforms,I2C 32 Bytes , SPI 256 Bytes
        _max_rw_length = interface == BHY2_I2C_INTERFACE ? 32 : 256;
#endif
    }

    _error_code = bhy2_init(interface,
                            SensorCommStatic::sensor_static_read_data,
                            SensorCommStatic::sensor_static_write_data,
                            SensorCommStatic::sensor_static_delay_us,
                            _max_rw_length,
                            staticComm.get(),
                            _bhy2.get());

    BHY2_RLST_CHECK(_error_code != BHY2_OK, "bhy2_init failed!", false);


    _error_code = bhy2_soft_reset(_bhy2.get());
    BHY2_RLST_CHECK(_error_code != BHY2_OK, "reset _bhy2 failed!", false);

    _error_code = bhy2_get_product_id(&product_id, _bhy2.get());
    BHY2_RLST_CHECK(_error_code != BHY2_OK, "bhy2_get_product_id failed!", false);

    /* Check for a valid product ID */
    if (product_id != BHY2_PRODUCT_ID) {
        log_e("Product ID read 0x%02X. Expected 0x%02X", product_id, BHY2_PRODUCT_ID);
        return false;
    } else {
        log_i("BHI260/BHA260 found. Product ID read 0x%02X", product_id);
    }

    // Set default interrupt configure
    uint8_t data = 0, data_exp;
    bhy2_get_host_interrupt_ctrl(&data, _bhy2.get());
    data &= ~BHY2_ICTL_DISABLE_STATUS_FIFO;     /* Enable status interrupts */
    if (_debug) {
        data &= ~BHY2_ICTL_DISABLE_DEBUG;           /* Enable debug interrupts */
    } else {
        data |= BHY2_ICTL_DISABLE_DEBUG;           /* Disable debug interrupts */
    }
    data &= ~BHY2_ICTL_EDGE;                    /* Level */
    data &= ~BHY2_ICTL_ACTIVE_LOW;              /* Active high */
    data &= ~BHY2_ICTL_OPEN_DRAIN;              /* Push-pull */
    data_exp = data;
    bhy2_set_host_interrupt_ctrl(data, _bhy2.get());
    bhy2_get_host_interrupt_ctrl(&data, _bhy2.get());
    if (data != data_exp) {
        log_d("Expected Host Interrupt Control (0x07) to have value 0x%x but instead read 0x%x\r\n", data_exp, data);
    }
    /* Config status channel */
    bhy2_set_host_intf_ctrl(BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL, _bhy2.get());
    bhy2_get_host_intf_ctrl(&data, _bhy2.get());
    if (!(data & BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL)) {
        log_d("Expected Host Interface Control (0x06) to have bit 0x%x to be set\r\n", BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL);
    }

    if (_boot_from_flash) {
        if (!bootFromFlash()) {
            return false;
        }
        if (_force_update) {
            if ((_firmware_stream == NULL) || (_firmware_size == 0)) {
                log_e("No valid firmware is set. Please use the \"setFirmware\" method to set the valid firmware.");
                return false;
            }
            _error_code = bhy2_soft_reset(_bhy2.get());
            BHY2_RLST_CHECK(_error_code != BHY2_OK, "reset _bhy2 failed!", false);
            log_i("Force update firmware.");
            if (!uploadFirmware(_firmware_stream, _firmware_size, _write_flash)) {
                log_e("uploadFirmware failed!");
                return false;
            }
        }
    } else {
        if ((_firmware_stream == NULL) || (_firmware_size == 0)) {
            log_e("No valid firmware is set. Please use the \"setFirmware\" method to set the valid firmware.");
            return false;
        }

        // ** Upload firmware to RAM **//
        if (!uploadFirmware(_firmware_stream, _firmware_size, false)) {
            log_e("uploadFirmware failed!");
            return false;
        }
    }

    uint16_t version = getKernelVersion();
    BHY2_RLST_CHECK(!version, "getKernelVersion failed!", false);
    log_i("Boot successful. Kernel version %u.", version);

    _error_code = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, BoschParseStatic::parseMetaEvent, this, _bhy2.get());
    BHY2_RLST_CHECK(_error_code != BHY2_OK, "bhy2_register_fifo_parse_callback failed!", false);

    _error_code = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, BoschParseStatic::parseMetaEvent, this, _bhy2.get());
    BHY2_RLST_CHECK(_error_code != BHY2_OK, "bhy2_register_fifo_parse_callback failed!", false);

    if (_debug) {
        _error_code = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_DEBUG_MSG, BoschParseStatic::parseDebugMessage, this, _bhy2.get());
        BHY2_RLST_CHECK(_error_code != BHY2_OK, "bhy2_register_fifo_parse_callback parseDebugMessage failed!", false);
    }

    //Set process buffer
#if defined(ARDUINO_ARCH_ESP32)
    if (psramFound()) {
        _processBuffer = (uint8_t *)ps_malloc(_processBufferSize);
        // In older versions of esp-core, even if psramFound returns true, it may not initialize psram correctly.
        // This situation is common when OPI type SPI-RAM is selected as QSPI, or QSPI is selected as OPI.
        if (!_processBuffer) {
            log_e("malloc psram buffer failed,try to alloc sram buffer!");
            _processBuffer = (uint8_t *)malloc(_processBufferSize);
        }
    } else {
        _processBuffer = (uint8_t *)malloc(_processBufferSize);
    }
#else
    _processBuffer = (uint8_t *)malloc(_processBufferSize);
#endif
    BHY2_RLST_CHECK(!_processBuffer, "process buffer malloc failed!", false);

    _error_code = bhy2_get_and_process_fifo(_processBuffer, _processBufferSize, _bhy2.get());
    if (_error_code != BHY2_OK) {
        log_e("bhy2_get_and_process_fifo failed");
        return false;
    }

    /* Update the callback table to enable parsing of sensor hintr_ctrl */
    bhy2_update_virtual_sensor_list(_bhy2.get());

    /* Get present virtual sensor */
    bhy2_get_virt_sensor_list(_bhy2.get());

    // Only register valid sensor IDs
    for (uint8_t i = 0; i < BHY2_SENSOR_ID_MAX; i++) {
        if (bhy2_is_sensor_available(i, _bhy2.get())) {
            _sensor_available_nums++;
            bhy2_register_fifo_parse_callback(i, BoschParseStatic::parseData, this, _bhy2.get());
        }
    }
    return _error_code == BHY2_OK;
}
