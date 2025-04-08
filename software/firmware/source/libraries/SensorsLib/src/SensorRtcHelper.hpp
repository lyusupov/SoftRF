/**
 *
 * @license MIT License
 *
 * Copyright (c) 2025 lewis he
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
 * @file      SensorRtcHelper.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-02-24
 * @note      The SensorRtcHelper class supports automatic determination of the commonly
 *            used PCF8563 and PCF8503 real-time clock chips. There are some differences in the registers
 *            between the two. This class is added to facilitate switching between different chips without
 *            having to worry about the specific model used.
 */
#include "SensorPCF8563.hpp"
#include "SensorPCF85063.hpp"

/**
 * @brief Enumeration of different RTC (Real-Time Clock) driver types.
 *
 * This enumeration defines the possible types of RTC drivers that can be used.
 * Currently only supports NXP PCF85063 and PCF8563.
 */
typedef enum  {
    RtcDrv_UNKOWN,    // Represents an unknown or unsupported RTC driver type.
    RtcDrv_PCF85063,  // Represents the PCF85063 RTC driver type.
    RtcDrv_PCF8563,   // Represents the PCF8563 RTC driver type.
} SensorRTCType;

class SensorRtcHelper: public SensorRTC
{
public:
    using SensorRTC::setDateTime;
    using SensorRTC::getDateTime;

    SensorRtcHelper();

    ~SensorRtcHelper();

    /**
    * @brief Set the model of the RTC (real-time clock) driver.
    *
    * This method is used to specify the type of RTC driver model that the system should use.
    * If not set, the chip type is automatically determined by register information by default.
    *
    * @param model An enumeration value of the SensorRTCType type, indicating a specific
    * RTC driver model. Possible values ​​include RtcDrv_UNKOWN (unknown model),
    * RtcDrv_PCF85063 (PCF85063 model), RtcDrv_PCF8563 (PCF8563 model), etc.
    */
    void setRtcDrvModel(SensorRTCType model);

#if defined(ARDUINO)
    /**
     * @brief Initialize the RTC driver for Arduino platform using I2C.
     *
     * This method initializes the RTC driver on the Arduino platform using the specified I2C bus,
     * SDA (Serial Data Line), and SCL (Serial Clock Line) pins.
     *
     * @param wire A reference to the TwoWire object representing the I2C bus.
     * @param sda The GPIO pin number for the SDA line. The default setting is -1, which does not initialize any GPIO.
     * @param scl The GPIO pin number for the SCL line. The default setting is -1, which does not initialize any GPIO.
     * @return true if the initialization is successful, false otherwise.
     */
    bool begin(TwoWire &wire,  int sda = -1, int scl = -1);

#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
    /**
     * @brief Initialize the RTC driver for ESP platform using legacy I2C.
     *
     * This method initializes the RTC driver on the ESP platform using the legacy I2C interface.
     * It takes the I2C port number, SDA, and SCL pin numbers as parameters.
     *
     * @param port_num The I2C port number to use.
     * @param sda The GPIO pin number for the SDA line. The default setting is -1, which does not initialize any GPIO.
     * @param scl The GPIO pin number for the SCL line. The default setting is -1, which does not initialize any GPIO.
     * @return true if the initialization is successful, false otherwise.
     */
    bool begin(i2c_port_t port_num,  int sda, int scl);

#else

    /**
     * @brief Initialize the RTC driver for ESP platform using I2C master bus handle.
     *
     * This method initializes the RTC driver on the ESP platform using the provided I2C master bus handle.
     *
     * @param handle The I2C master bus handle to use.
     * @return true if the initialization is successful, false otherwise.
     */
    bool begin(i2c_master_bus_handle_t handle);

#endif  //USEING_I2C_LEGACY

#endif  // ESP_PLATFORM

    /**
     * @brief Initialize the RTC driver using a custom callback.
     *
     * This method allows the user to initialize the RTC driver using a custom callback function.
     * The callback can be used to perform custom communication or configuration operations.
     *
     * @param callback A pointer to the custom callback function.
     * @return true if the initialization is successful, false otherwise.
     */
    bool begin(SensorCommCustom::CustomCallback callback);

    /**
     * @brief Set the date and time on the RTC device.
     *
     * This method sets the specified date and time on the RTC device.
     *
     * @param datetime An RTC_DateTime object containing the date and time to set.
     */
    void setDateTime(RTC_DateTime datetime);

    /**
     * @brief Get the current date and time from the RTC device.
     *
     * This method retrieves the current date and time from the RTC device and returns it as an RTC_DateTime object.
     *
     * @return An RTC_DateTime object representing the current date and time.
     */
    RTC_DateTime getDateTime();

    /**
     * @brief Get the name of the RTC chip.
     * @return A pointer to a constant character array containing the name of the RTC chip.
     */
    const char *getChipName();

private:
    /**
     * @brief Create a new RTC driver instance based on the specified type.
     *
     * This private method creates a new RTC driver instance of the specified type using the driver creators array.
     *
     * @param type The type of the RTC driver to create, as defined in the SensorRTCType enumeration.
     * @return A unique pointer to the newly created SensorRTC object.
     */
    std::unique_ptr<SensorRTC> createDriver(SensorRTCType type);

    /**
     * @brief Function pointer type for creating a SensorRTC object.
     *
     * This typedef defines a function pointer type that can be used to create a SensorRTC object.
     * The function should return a unique pointer to a SensorRTC object.
     */
    using DriverCreator = std::unique_ptr<SensorRTC> (*)();

    /**
     * @brief The maximum number of driver creators in the array.
     *
     * This constant defines the maximum number of driver creator functions that can be stored in the driverCreators array.
     */
    static constexpr uint8_t driverCreatorMaxNum = 2;

    /**
     * @brief An array of driver creator functions.
     *
     * This static array stores function pointers to the driver creator functions.
     * Each function can create a specific type of SensorRTC object.
     */
    static DriverCreator driverCreators[driverCreatorMaxNum];

    /**
     * @brief The type of the currently used RTC driver.
     *
     * This member variable stores the type of the RTC driver that is currently in use,
     * as defined in the SensorRTCType enumeration.
     */
    SensorRTCType _drvType;

    /**
     * @brief A unique pointer to the RTC driver object.
     *
     * This member variable holds a unique pointer to the SensorRTC object that represents the RTC driver.
     * It is used to interact with the RTC device.
     */
    std::unique_ptr<SensorRTC>  _drv;
};