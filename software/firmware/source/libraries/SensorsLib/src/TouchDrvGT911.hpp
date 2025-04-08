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
 * @file      TouchDrvGT911.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-12
 *
 */
#pragma once

#include "REG/GT911Constants.h"
#include "TouchDrvInterface.hpp"

#if defined(ARDUINO_ARCH_NRF52)
// NRF52840 I2C BUFFER : 64 Bytes ,
#warning "NRF Platform I2C Buffer expansion is not implemented , GT911 requires at least 188 bytes to read all configurations"
#endif

#define GT911_GET_POINT(x)            (x & 0x0F)
#define GT911_GET_BUFFER_STATUS(x)    (x & 0x80)
#define GT911_GET_HAVE_KEY(x)         (x & 0x10)

class TouchDrvGT911 :  public TouchDrvInterface, public GT911Constants
{
    typedef struct {
        uint8_t trackID;
        int16_t x;
        int16_t y;
        int16_t size;
    } PointReg;

public:

    TouchDrvGT911() : comm(nullptr), hal(nullptr) {}

    ~TouchDrvGT911()
    {
        if (comm) {
            comm->deinit();
        }
    }

#if defined(ARDUINO)
    bool begin(TwoWire &wire, uint8_t addr = GT911_SLAVE_ADDRESS_H, int sda = -1, int scl = -1)
    {
        if (!beginCommon<SensorCommI2C, HalArduino>(comm, hal, wire, addr, sda, scl)) {
            return false;
        }
        return initImpl(addr);
    }

#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
    bool begin(i2c_port_t port_num, uint8_t addr = GT911_SLAVE_ADDRESS_H, int sda = -1, int scl = -1)
    {
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, port_num, addr, sda, scl)) {
            return false;
        }
        return initImpl(addr);
    }
#else
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr = GT911_SLAVE_ADDRESS_H)
    {
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, handle, addr)) {
            return false;
        }
        return initImpl(addr);
    }
#endif  //ESP_PLATFORM
#endif  //ARDUINO


    bool begin(SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_callback,
               uint8_t addr)
    {
        if (!beginCommCustomCallback<SensorCommCustom, SensorCommCustomHal>(COMM_CUSTOM,
                callback, hal_callback, addr, comm, hal)) {
            return false;
        }
        return initImpl(addr);
    }

    void reset()
    {
        if (_rst != -1) {
            hal->pinMode(_rst, OUTPUT);
            hal->digitalWrite(_rst, HIGH);
            hal->delay(10);
        }
        if (_irq != -1) {
            hal->pinMode(_irq, INPUT);
        }
        /*
        * If you perform a software reset on a board without a reset pin connected,
        * subsequent interrupt settings or re-writing of configurations will be invalid.
        * For example, when debugging a LilyGo T-Deck, resetting the interrupt mode will
        * be invalid after a software reset.
        * */
        // comm->writeRegister(GT911_COMMAND, 0x02);
        // writeCommand(0x02);
    }

    void sleep()
    {
        if (_irq != -1) {
            hal->pinMode(_irq, OUTPUT);
            hal->digitalWrite(_irq, LOW);
        }
        // comm->writeRegister(GT911_COMMAND, 0x05);
        writeCommand(0x05);

        /*
        * Depending on the chip and platform, setting it to input after removing sleep will affect power consumption.
        * The chip platform determines whether
        *
        * * */
        // if (_irq != -1) {
        //     hal->digitalWrite(_irq, INPUT);
        // }
    }



    void wakeup()
    {
        if (_irq != -1) {
            hal->pinMode(_irq, OUTPUT);
            hal->digitalWrite(_irq, HIGH);
            hal->delay(8);
            hal->pinMode(_irq, INPUT);
        } else {
            reset();
        }
    }

    void idle()
    {

    }

    uint8_t getSupportTouchPoint()
    {
        return 5;
    }

    uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t size = 1)
    {
        uint8_t buffer[39];
        uint8_t touchPoint = 0;
        PointReg p[5];

        if (!x_array || !y_array || size == 0)
            return 0;

        uint8_t val = readGT911(GT911_POINT_INFO);

        bool haveKey = GT911_GET_HAVE_KEY(val);
        // bool bufferStatus = GT911_GET_BUFFER_STATUS(val);
        // log_i("REG:0x%X S:0X%d K:%d\n", val,bufferStatus,haveKey);

        if (_HButtonCallback && haveKey) {
            _HButtonCallback(_userData);
        }

        clearBuffer();

        touchPoint = GT911_GET_POINT(val);
        if (touchPoint == 0) {
            return 0;
        }

        // GT911_POINT_1  0X814F
        uint8_t write_buffer[2] = {0x81, 0x4F};
        if (comm->writeThenRead(write_buffer, arraySize(write_buffer),
                                buffer, 39) == -1) {
            return 0;
        }

        for (uint8_t i = 0; i < size; i++) {
            p[i].trackID = buffer[i * 8];
            p[i].x =  buffer[0x01 + i * 8] ;
            p[i].x |= (buffer[0x02 + i * 8] << 8 );
            p[i].y =  buffer[0x03 + i * 8] ;
            p[i].y |= (buffer[0x04 + i * 8] << 8);
            p[i].size = buffer[0x05 + i * 8] ;
            p[i].size |= (buffer[0x06 + i * 8] << 8) ;

            x_array[i] = p[i].x;
            y_array[i] = p[i].y;
        }

        updateXY(touchPoint, x_array, y_array);

        return touchPoint;
    }


    bool isPressed()
    {
        if (_irq != -1) {
            if (_irq_mode == FALLING) {
                return hal->digitalRead(_irq) == LOW;
            } else if (_irq_mode == RISING ) {
                return hal->digitalRead(_irq) == HIGH;
            } else if (_irq_mode == LOW_LEVEL_QUERY) {
                return hal->digitalRead(_irq) == LOW;
            }  else if (_irq_mode == HIGH_LEVEL_QUERY) {
                return hal->digitalRead(_irq) == HIGH;
            }
        }
        return getPoint();
    }

    bool setInterruptMode(uint8_t mode)
    {
        // GT911_MODULE_SWITCH_1 0x804D
        uint8_t val = readGT911(GT911_MODULE_SWITCH_1);
        val &= 0XFC;
        if (mode == FALLING) {
            val |= 0x01;
        } else if (mode == RISING ) {
            val |= 0x00;
        } else if (mode == LOW_LEVEL_QUERY ) {
            val |= 0x02;
        } else if (mode == HIGH_LEVEL_QUERY ) {
            val |= 0x03;
        }
        _irq_mode = mode;
        writeGT911(GT911_MODULE_SWITCH_1, val);
        return reloadConfig();
    }

    /**
     * @retval
     *  * 0x0: Rising edge trigger
     *  * 0x1: Falling edge trigger
     *  * 0x2: Low level query
     *  * 0x3: High level query
     */
    uint8_t getInterruptMode()
    {
        uint8_t val = readGT911(GT911_MODULE_SWITCH_1);
        // return val & 0x03;
        val &= 0x03;
        if (val == 0x00) {
            _irq_mode = RISING;
        } else if (val == 0x01) {
            _irq_mode = FALLING;
        } else if (val == 0x02) {
            _irq_mode = LOW_LEVEL_QUERY;
        } else if (val == 0x03) {
            _irq_mode = HIGH_LEVEL_QUERY;
        }
        return val;
    }


    uint8_t getPoint()
    {
        // GT911_POINT_INFO 0X814E
        uint8_t val = readGT911(GT911_POINT_INFO);
        clearBuffer();
        return GT911_GET_POINT(val);
    }


    uint32_t getChipID()
    {
        char product_id[4] = {0};
        // GT911_PRODUCT_ID 0x8140
        for (int i = 0; i < 4; ++i) {
            product_id[i] = readGT911(GT911_PRODUCT_ID + i);
        }
        return atoi(product_id);
    }

    uint16_t getFwVersion()
    {
        uint8_t fw_ver[2] = {0};
        // GT911_FIRMWARE_VERSION 0x8144
        for (int i = 0; i < 2; ++i) {
            fw_ver[i] = readGT911(GT911_FIRMWARE_VERSION + i);
        }
        return fw_ver[0] | (fw_ver[1] << 8);
    }

    uint8_t getConfigVersion()
    {
        return readGT911(GT911_CONFIG_VERSION);
    }


    bool getResolution(int16_t *x, int16_t *y)
    {
        uint8_t x_resolution[2] = {0}, y_resolution[2] = {0};

        for (int i = 0; i < 2; ++i) {
            x_resolution[i] = readGT911(GT911_X_RESOLUTION + i);
        }
        for (int i = 0; i < 2; ++i) {
            y_resolution[i] = readGT911(GT911_Y_RESOLUTION + i);
        }

        *x = x_resolution[0] | (x_resolution[1] << 8);
        *y = y_resolution[0] | (y_resolution[1] << 8);
        return true;
    }

    //Range : 5 ~ 15 ms
    void updateRefreshRate(uint8_t rate_ms)
    {
        if ((rate_ms - 5) < 5) {
            rate_ms = 5;
        }
        if (rate_ms > 15) {
            rate_ms = 15;
        }
        rate_ms -= 5;
        writeGT911(GT911_REFRESH_RATE, rate_ms);
        reloadConfig();
    }

    uint8_t getRefreshRate()
    {
        uint8_t rate_ms  = readGT911(GT911_REFRESH_RATE);
        return rate_ms + GT911_BASE_REF_RATE ;
    }


    int getVendorID()
    {
        return readGT911(GT911_VENDOR_ID);
    }


    const char *getModelName()
    {
        return "GT911";
    }

    void  setGpioCallback(CustomMode mode_cb,
                          CustomWrite write_cb,
                          CustomRead read_cb)
    {
        SensorHalCustom::setCustomMode(mode_cb);
        SensorHalCustom::setCustomWrite(write_cb);
        SensorHalCustom::setCustomRead(read_cb);
    }

    void setHomeButtonCallback(HomeButtonCallback cb, void *user_data)
    {
        _HButtonCallback = cb;
        _userData = user_data;
    }

    bool writeConfig(const uint8_t *config_buffer, size_t buffer_size)
    {
#if 0   //TODO:
        uint8_t check_sum = 0;
        for (int i = 0; i < (GT911_REG_LENGTH - 2 ); i++) {
            check_sum += config_buffer[i];
        }
        check_sum =  (~check_sum) + 1;
        if (check_sum != config_buffer[GT911_REG_LENGTH - 2]) {
            log_e("Config checksum error !");
            return false;
        }
        log_d("Update touch config , write %lu Bytes check sum:0x%X", buffer_size, check_sum);
        uint8_t cmd[] = {lowByte(GT911_CONFIG_VERSION), highByte(GT911_CONFIG_VERSION)};
        int err =  comm->writeRegister(GT911_CONFIG_VERSION, (uint8_t *)config_buffer, buffer_size);


#if 0
        while (digitalRead(_irq)) {
            log_i("Wait irq.."); hal->delay(500);
        }
        int err =   comm->writeBuffer((uint8_t *)config_buffer, buffer_size);
#endif
        return err == 0;
#endif
        return false;
    }

    uint8_t *loadConfig(size_t *output_size, bool print_out = false)
    {
        *output_size = 0;
        uint8_t   *buffer = (uint8_t * )malloc(GT911_REG_LENGTH * sizeof(uint8_t));
        if (!buffer)return NULL;
        uint8_t write_buffer[2] = {highByte(GT911_CONFIG_VERSION), lowByte(GT911_CONFIG_VERSION)};
        if (comm->writeThenRead(write_buffer, arraySize(write_buffer), buffer, GT911_REG_LENGTH) == -1) {
            free(buffer);
            return NULL;
        }
        if (print_out) {
            printf("const unsigned char config[186] = {");
            for (int i = 0; i < GT911_REG_LENGTH; ++i) {
                if ( (i % 8) == 0) {
                    printf("\n");
                }
                printf(" 0x%02X", buffer[i]);
                if ((i + 1) < GT911_REG_LENGTH) {
                    printf(",");
                }
            }
            printf("};\n");
        }
        *output_size = GT911_REG_LENGTH;
        return buffer;
    }

    bool reloadConfig()
    {
        uint8_t buffer[GT911_REG_LENGTH] = {highByte(GT911_CONFIG_VERSION), lowByte(GT911_CONFIG_VERSION)};
        if (comm->writeThenRead(buffer, 2, buffer, GT911_REG_LENGTH - 2) == -1) {
            return false;
        }

        uint8_t check_sum = 0;
        for (int i = 0; i < (GT911_REG_LENGTH - 2 ); i++) {
            check_sum += buffer[i];
        }
        check_sum =  (~check_sum) + 1;
        log_d("reloadConfig check_sum : 0x%X\n", check_sum);
        writeGT911(GT911_CONFIG_CHKSUM, check_sum);
        writeGT911(GT911_CONFIG_FRESH, 0x01);
        return true;
    }

    void dumpRegister()
    {
        size_t output_size = 0;
        uint8_t *buffer = loadConfig(&output_size, true);
        if (output_size == 0) {
            return;
        }

        if (buffer == NULL)return;
        printf("----------Dump register------------\n");
        for (size_t  i = 0; i < output_size; ++i) {
            printf("[%d]  REG: 0x%X : 0x%02X\n", i, GT911_CONFIG_VERSION + i, buffer[i]);
        }
        free(buffer);
    }

    // Range : 1~5
    void setMaxTouchPoint(uint8_t num)
    {
        if (num < 1)num = 1;
        if (num > 5) num = 5;
        writeGT911(GT911_TOUCH_NUMBER, num);
        reloadConfig();
    }

    uint8_t getMaxTouchPoint()
    {
        uint8_t num = readGT911(GT911_TOUCH_NUMBER);
        return num & 0x0F;
    }

    void setConfigData(uint8_t *data, uint16_t length)
    {
        _config = data;
        _config_size = length;
    }

private:

    uint8_t readGT911(uint16_t cmd)
    {
        uint8_t value = 0x00;
        uint8_t write_buffer[2] = {highByte(cmd), lowByte(cmd)};
        comm->writeThenRead(write_buffer, arraySize(write_buffer),
                            &value, 1);
        return value;
    }

    int writeGT911(uint16_t cmd, uint8_t value)
    {
        uint8_t write_buffer[3] = {highByte(cmd), lowByte(cmd), value};
        return comm->writeBuffer(write_buffer, arraySize(write_buffer));
    }


    void writeCommand(uint8_t command)
    {
        // GT911_COMMAND 0x8040
        uint8_t write_buffer[3] = {0x80, 0x40, command};
        comm->writeBuffer(write_buffer, arraySize(write_buffer));
    }

    void inline clearBuffer()
    {
        writeGT911(GT911_POINT_INFO, 0x00);
    }

    bool probeAddress()
    {
        const uint8_t device_address[2]  = {GT911_SLAVE_ADDRESS_L, GT911_SLAVE_ADDRESS_H};
        for (size_t i = 0; i < arraySize(device_address); ++i) {
            I2CParam params(I2CParam::I2C_SET_ADDR, device_address[i]);
            comm->setParams(params);
            for (int retry = 0; retry < 3; ++retry) {
                _chipID = getChipID();
                if (_chipID == GT911_DEV_ID) {
                    log_i("Touch device address found is : 0x%X", device_address[i]);
                    return true;
                }
            }
        }
        log_e("GT911 not found, touch device 7-bit address should be 0x5D or 0x14");
        return false;
    }


    bool initImpl(uint8_t addr)
    {
        int16_t x = 0, y = 0;

        if (addr == GT911_SLAVE_ADDRESS_H  && _rst != -1 && _irq != -1) {

            log_i("Try using 0x14 as the device address");

            hal->pinMode(_rst, OUTPUT);
            hal->pinMode(_irq, OUTPUT);

            hal->digitalWrite(_rst, LOW);
            hal->digitalWrite(_irq, HIGH);
            hal->delayMicroseconds(120);
            hal->digitalWrite(_rst, HIGH);

#if   defined(ARDUINO)
            // In the Arduino ESP32 platform, the test delay is 8ms and the GT911
            // can be accessed correctly. If the time is too long, it will not be accessible.
            hal->delay(8);
#elif defined(ESP_PLATFORM)
            // For the variant of GPIO extended RST,
            // communication and delay are carried out simultaneously, and 18 ms is measured in T-RGB esp-idf new api
            hal->delay(18);
#endif

            hal->pinMode(_irq, INPUT);

        } else if (addr == GT911_SLAVE_ADDRESS_L && _rst != -1 && _irq != -1) {

            log_i("Try using 0x5D as the device address");

            hal->pinMode(_rst, OUTPUT);
            hal->pinMode(_irq, OUTPUT);

            hal->digitalWrite(_rst, LOW);
            hal->digitalWrite(_irq, LOW);
            hal->delayMicroseconds(120);
            hal->digitalWrite(_rst, HIGH);
#if   defined(ARDUINO)
            // In the Arduino ESP32 platform, the test hal->delay is 8ms and the GT911
            // can be accessed correctly. If the time is too long, it will not be accessible.
            hal->delay(8);
#elif defined(ESP_PLATFORM)
            // For the variant of GPIO extended RST,
            // communication and hal->delay are carried out simultaneously, and 18 ms is measured in T-RGB esp-idf new api
            hal->delay(18);
#endif
            hal->pinMode(_irq, INPUT);

        } else {
            if (!autoProbe()) {
                return false;
            }
        }

        // For variants where the GPIO is controlled by I2C, a hal->delay is required here
        hal->delay(20);


        /*
        * For the ESP32 platform, the default buffer is 128.
        * Need to re-apply for a larger buffer to fully read the configuration table.
        *
        * TODO: NEED FIX
        if (!this->reallocBuffer(GT911_REG_LENGTH + 2)) {
            log_e("realloc i2c buffer failed !");
            return false;
        }
         */

        _chipID = getChipID();

        if (_chipID != GT911_DEV_ID) {
            log_i("Not found device GT911,Try to found the GT911");
            if (!autoProbe()) {
                return false;
            }
        }

        log_i("Product id:%ld", _chipID);

#if 0
        /*If the configuration is not written, the touch screen may be damaged. */
        if (_config && _config_size != 0) {

            log_d("Current version char :%x", getConfigVersion());
            hal->delay(100);
            writeConfig(_config, _config_size);
            if (_irq != -1) {
                hal->pinMode(_irq, INPUT);
            }
            log_d("WriteConfig version char :%x", getConfigVersion());
            // hal->delay(1000);
            // size_t output_size;
            // loadConfig(&output_size, true);
            // log_d("loadConfig version char :%x", version_char);
        }
#endif


        log_i("Firmware version: 0x%x", getFwVersion());
        getResolution(&x, &y);
        log_i("Resolution : X = %d Y = %d", x, y);
        log_i("Vendor id:%d", getVendorID());
        log_i("Refresh Rate:%d ms", getRefreshRate());
        log_i("MaxTouchPoint:%d", getMaxTouchPoint());


        // Get the default interrupt trigger mode of the current screen
        getInterruptMode();

        if ( _irq_mode == RISING) {
            log_i("Interrupt Mode:  RISING");
        } else if (_irq_mode == FALLING) {
            log_i("Interrupt Mode:  FALLING");
        } else if (_irq_mode == LOW_LEVEL_QUERY) {
            log_i("Interrupt Mode:  LOW_LEVEL_QUERY");
        } else if (_irq_mode == HIGH_LEVEL_QUERY) {
            log_i("Interrupt Mode:  HIGH_LEVEL_QUERY");
        } else {
            log_e("UNKOWN");
        }

        if (x == -1 || y == -1) {
            log_e("The screen configuration is lost, please update the configuration file again !");
            return false;
        }

        return true;
    }

    bool autoProbe()
    {
        if (_rst != -1) {
            hal->pinMode(_rst, OUTPUT);
            hal->digitalWrite(_rst, HIGH);
            hal->delay(10);
        }

        // Automatically determine the current device
        // address when using the reset pin without connection
        if (!probeAddress()) {
            return false;
        }

        // Reset Config
        reset();

        if (_irq != -1) {
            hal->pinMode(_irq, INPUT);
        }

        return true;
    }

    static constexpr uint8_t LOW_LEVEL_QUERY  = 0x03;
    static constexpr uint8_t HIGH_LEVEL_QUERY = 0x04;

protected:
    std::unique_ptr<SensorCommBase> comm;
    std::unique_ptr<SensorHal> hal;

    int _irq_mode;
    uint8_t *_config = NULL;
    uint16_t _config_size = 0;
};
