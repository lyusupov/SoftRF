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
 * @file      TouchDrvCHSC5816.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-12
 *
 */
#pragma once

#include "REG/CHSC5816Constants.h"
#include "TouchDrvInterface.hpp"

#define CHSC5816_SLAVE_ADDRESS     (0x2E)

class TouchDrvCHSC5816 :  public TouchDrvInterface
{
    typedef struct {
        uint16_t fw_ver;
        uint16_t checksum;
        uint32_t sig;
        uint32_t vid_pid;
        uint16_t raw_offset;
        uint16_t dif_offset;
    } Header_t;

    union PointReg {
        struct {
            uint8_t status;
            uint8_t fingerNumber;
            uint8_t x_l8;
            uint8_t y_l8;
            uint8_t z;
            uint8_t x_h4: 4;
            uint8_t y_h4: 4;
            uint8_t id: 4;
            uint8_t event: 4;
            uint8_t p2;
        } report;
        unsigned char data[8];
    };

public:

    TouchDrvCHSC5816() : comm(nullptr), hal(nullptr) {}

    ~TouchDrvCHSC5816()
    {
        if (comm) {
            comm->deinit();
        }
    }

#if defined(ARDUINO)
    bool begin(TwoWire &wire, uint8_t addr = CHSC5816_SLAVE_ADDRESS, int sda = -1, int scl = -1)
    {
        if (!beginCommon<SensorCommI2C, HalArduino>(comm, hal, wire, addr, sda, scl)) {
            return false;
        }
        return initImpl();
    }
#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
    bool begin(i2c_port_t port_num, uint8_t addr = CHSC5816_SLAVE_ADDRESS, int sda = -1, int scl = -1)
    {
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, port_num, addr, sda, scl)) {
            return false;
        }
        return initImpl();
    }
#else
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr = CHSC5816_SLAVE_ADDRESS)
    {
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, handle, addr)) {
            return false;
        }
        return initImpl();
    }
#endif  //ESP_PLATFORM
#endif  //ARDUINO

    bool begin(SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_callback,
               uint8_t addr = CHSC5816_SLAVE_ADDRESS)
    {
        if (!beginCommCustomCallback<SensorCommCustom, SensorCommCustomHal>(COMM_CUSTOM,
                callback, hal_callback, addr, comm, hal)) {
            return false;
        }
        return initImpl();
    }

    void reset()
    {
        if (_rst != -1) {
            hal->digitalWrite(_rst, LOW);
            hal->delay(3);
            hal->digitalWrite(_rst, HIGH);
            hal->delay(5);
        }
    }

    uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point = 1)
    {
        PointReg touch;
        uint8_t CHSC5816_REG_POINT[] = {0x20, 0x00, 0x00, 0x2c};
        comm->writeThenRead(CHSC5816_REG_POINT, arraySize(CHSC5816_REG_POINT), touch.data, 8);
        if (touch.report.status == 0xFF && touch.report.fingerNumber == 0) {
            return 0;
        }
        if (x_array) {
            *x_array = (unsigned int)(touch.report.x_h4 << 8) | touch.report.x_l8;
        }
        if (y_array) {
            *y_array = (unsigned int)(touch.report.y_h4 << 8) | touch.report.y_l8;
        }

        updateXY(1, x_array, y_array);

        return 1;
    }

    bool isPressed()
    {
        if (_irq != -1) {
            return hal->digitalRead(_irq) == LOW;
        }
        return getPoint(NULL, NULL);
    }

    const char *getModelName()
    {
        return "CHSC5816";
    }

    //2uA
    void sleep()
    {
        uint8_t CHSC5816_REG_SLEEP[] = {
            0x20, 0x00, 0x00, 0x00, // CHSC5816_REG_CMD_BUFF
            0xF8, 0x16, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE9
        };
        comm->writeBuffer(CHSC5816_REG_SLEEP, arraySize(CHSC5816_REG_SLEEP));
    }

    void wakeup()
    {
        reset();
    }

    void idle()
    {
        uint8_t CHSC5816_REG_IDEL[] = {
            0x20, 0x00, 0x00, 0x00, // CHSC5816_REG_CMD_BUFF
            0x20, 0x16, 0x02, 0x00, 0xDB, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE9
        };
        comm->writeBuffer(CHSC5816_REG_IDEL, arraySize(CHSC5816_REG_IDEL));
    }

    uint8_t getSupportTouchPoint()
    {
        return 1;
    }

    bool getResolution(int16_t *width, int16_t *height)
    {
#if 0
        //TODO: NEED TEST
        uint8_t CHSC5816_REG_FW[] = {
            0x20, 0x00, 0x00, 0x00, // CHSC5816_REG_CMD_BUFF
            0xFC, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xe9
        };
        if (comm->writeThenRead(CHSC5816_REG_FW,
                                arraySize(CHSC5816_REG_FW),
                                CHSC5816_REG_FW,
                                arraySize(CHSC5816_REG_FW)) < 0) {
            return false;
        }

        SensorLibDumpBuffer(CHSC5816_REG_FW, arraySize(CHSC5816_REG_FW));

        int16_t res_w =  (CHSC5816_REG_FW[2] << 8) | CHSC5816_REG_FW[3];
        int16_t res_h =  (CHSC5816_REG_FW[4] << 8) | CHSC5816_REG_FW[5];
        if (width) {
            *width = res_w;
        }
        if (height) {
            *height = res_h;
        }
        return true;
#endif
        return false;
    }

    void  setGpioCallback(CustomMode mode_cb,
                          CustomWrite write_cb,
                          CustomRead read_cb)
    {
        SensorHalCustom::setCustomMode(mode_cb);
        SensorHalCustom::setCustomWrite(write_cb);
        SensorHalCustom::setCustomRead(read_cb);
    }

private:
    bool checkOnline()
    {
        Header_t first;
        Header_t second;

        memset(&second, 0, sizeof(Header_t));
        memset(&first, 0, sizeof(Header_t));

        // CHSC5816_REG_BOOT_STATE 0x20000018
        uint8_t CHSC5816_REG_BOOT_STATE[] = {0x20, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00};
        if (comm->writeBuffer(CHSC5816_REG_BOOT_STATE, arraySize(CHSC5816_REG_BOOT_STATE)) < 0) {
            log_e("comm->writeBuffer clean boot state failed!\n");
            return false;
        }

        reset();

        for (int i = 0; i < 10; ++i) {
            hal->delay(10);
            // CHSC5816_REG_IMG_HEAD 0x20000014
            uint8_t CHSC5816_REG_IMG_HEAD[] = {0x20, 0x00, 0x00, 0x14};
            if (comm->writeThenRead(CHSC5816_REG_IMG_HEAD,
                                    arraySize(CHSC5816_REG_IMG_HEAD),
                                    (uint8_t *)&first,
                                    sizeof(Header_t)) < 0) {
                return false;
            }

            if (comm->writeThenRead(CHSC5816_REG_IMG_HEAD,
                                    arraySize(CHSC5816_REG_IMG_HEAD),
                                    (uint8_t *)&second,
                                    sizeof(Header_t)) < 0) {
                return false;
            }

            if (memcmp(&second, &first, sizeof(Header_t)) != 0 ) {
                continue;
            }
            if (first.sig == CHSC5816_SIG_VALUE) {
                return true;
            }
        }
        return false;
    }

    bool initImpl()
    {

        if (_irq != -1) {
            hal->pinMode(_irq, INPUT);
        }

        if (_rst != -1) {
            hal->pinMode(_rst, OUTPUT);
        }

        reset();

        if (checkOnline()) {
            reset();
            return true;
        }

        return false;
    }


protected:
    std::unique_ptr<SensorCommBase> comm;
    std::unique_ptr<SensorHal> hal;

    static constexpr uint32_t CHSC5816_SIG_VALUE            = (0x43534843U);
};


// #define CHSC5816_REG_CMD_BUFF                  (0x20000000U)
// #define CHSC5816_REG_RSP_BUFF                  (0x20000000U)
// #define CHSC5816_REG_IMG_HEAD                  (0x20000014U)
// #define CHSC5816_REG_POINT                     (0x2000002CU)
// #define CHSC5816_REG_WR_BUFF                   (0x20002000U)
// #define CHSC5816_REG_RD_BUFF                   (0x20002400U)
// #define CHSC5816_REG_HOLD_MCU                  (0x40007000U)
// #define CHSC5816_REG_AUTO_FEED                 (0x40007010U)
// #define CHSC5816_REG_REMAP_MCU                 (0x40007000U)
// #define CHSC5816_REG_RELEASE_MCU               (0x40007000U)
// #define CHSC5816_REG_BOOT_STATE                (0x20000018U)
// #define CHSC5816_HOLD_MCU_VAL                  (0x12044000U)
// #define CHSC5816_AUTO_FEED_VAL                 (0x0000925aU)
// #define CHSC5816_REMAP_MCU_VAL                 (0x12044002U)
// #define CHSC5816_RELEASE_MCU_VAL               (0x12044003U)
// #define CHSC5816_REG_VID_PID_BACKUP            (40 * 1024 + 0x10U)
// /*ctp work staus*/
// #define CHSC5816_POINTING_WORK                 (0x00000000U)
// #define CHSC5816_READY_UPGRADE                 (1 << 1)
// #define CHSC5816_UPGRAD_RUNING                 (1 << 2)
// #define CHSC5816_SLFTEST_RUNING                (1 << 3)
// #define CHSC5816_SUSPEND_GATE                  (1 << 16)
// #define CHSC5816_GUESTURE_GATE                 (1 << 17)
// #define CHSC5816_PROXIMITY_GATE                (1 << 18)
// #define CHSC5816_GLOVE_GATE                    (1 << 19)
// #define CHSC5816_ORIENTATION_GATE              (1 << 20)

