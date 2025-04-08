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
 * @file      SensorDRV2605.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-03
 * @note      Source code from https://github.com/adafruit/Adafruit_DRV2605_Library
 */
#pragma once

#include "REG/DRV2605Constants.h"
#include "SensorPlatform.hpp"

class SensorDRV2605 : public DRV2605Constants
{
public:
    static constexpr uint8_t MODE_INTTRIG = 0x00;              //* Internal trigger mode
    static constexpr uint8_t MODE_EXTTRIGEDGE = 0x01;              //* External edge trigger mode
    static constexpr uint8_t MODE_EXTTRIGLVL = 0x02;              //* External level trigger mode
    static constexpr uint8_t MODE_PWMANALOG = 0x03;              //* PWM/Analog input mode
    static constexpr uint8_t MODE_AUDIOVIBE = 0x04;              //* Audio-to-vibe mode
    static constexpr uint8_t MODE_REALTIME = 0x05;              //* Real-time playback (RTP) mode
    static constexpr uint8_t MODE_DIAGNOS = 0x06;              //* Diagnostics mode
    static constexpr uint8_t MODE_AUTOCAL = 0x07;              //* Auto calibration mode

    SensorDRV2605() : comm(nullptr) {}

    ~SensorDRV2605()
    {
        if (comm) {
            comm->deinit();
        }
    }

#if defined(ARDUINO)
    bool begin(TwoWire &wire, int sda = -1, int scl = -1)
    {
        comm = std::make_unique<SensorCommI2C>(wire, DRV2605_SLAVE_ADDRESS, sda, scl);
        if (!comm) {
            return false;
        }
        comm->init();
        return initImpl();
    }
#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
    bool begin(i2c_port_t port_num, int sda = -1, int scl = -1)
    {
        comm = std::make_unique<SensorCommI2C>(port_num, DRV2605_SLAVE_ADDRESS, sda, scl);
        if (!comm) {
            return false;
        }
        comm->init();
        return initImpl();
    }
#else
    bool begin(i2c_master_bus_handle_t handle)
    {
        comm = std::make_unique<SensorCommI2C>(handle, DRV2605_SLAVE_ADDRESS);
        if (!comm) {
            return false;
        }
        comm->init();
        return initImpl();
    }
#endif  //ESP_PLATFORM
#endif  //ARDUINO

    bool begin(SensorCommCustom::CustomCallback callback)
    {
        comm = std::make_unique<SensorCommCustom>(callback, DRV2605_SLAVE_ADDRESS);
        if (!comm) {
            return false;
        }
        comm->init();
        return initImpl();
    }

    /**************************************************************************/
    /*!
      @brief Select the haptic waveform to use.
      @param slot The waveform slot to set, from 0 to 7
      @param w The waveform sequence value, refers to an index in the ROM library.

        Playback starts at slot 0 and continues through to slot 7, stopping if it
      encounters a value of 0. A list of available waveforms can be found in
      section 11.2 of the datasheet: http://www.adafruit.com/datasheets/DRV2605.pdf
    */
    /**************************************************************************/
    void setWaveform(uint8_t slot, uint8_t w)
    {
        comm->writeRegister(DRV2605_REG_WAVESEQ1 + slot, w);
    }

    /**************************************************************************/
    /*!
      @brief Select the waveform library to use.
      @param lib Library to use, 0 = Empty, 1-5 are ERM, 6 is LRA.

        See section 7.6.4 in the datasheet for more details:
      http://www.adafruit.com/datasheets/DRV2605.pdf
    */
    /**************************************************************************/
    void selectLibrary(uint8_t lib)
    {
        comm->writeRegister(DRV2605_REG_LIBRARY, lib);
    }

    /**************************************************************************/
    /*!
      @brief Start playback of the waveforms (start moving!).
    */
    /**************************************************************************/
    void run()
    {
        comm->writeRegister(DRV2605_REG_GO, 1);
    }

    /**************************************************************************/
    /*!
      @brief Stop playback.
    */
    /**************************************************************************/
    void stop()
    {
        comm->writeRegister(DRV2605_REG_GO, (uint8_t)0);
    }

    /**************************************************************************/
    /*!
      @brief Set the device mode.
      @param mode Mode value, see datasheet section 7.6.2:
      http://www.adafruit.com/datasheets/DRV2605.pdf

        0: Internal trigger, call run() to start playback\n
        1: External trigger, rising edge on IN pin starts playback\n
        2: External trigger, playback follows the state of IN pin\n
        3: PWM/analog input\n
        4: Audio\n
        5: Real-time playback\n
        6: Diagnostics\n
        7: Auto calibration
    */
    /**************************************************************************/
    void setMode(uint8_t mode)
    {
        comm->writeRegister(DRV2605_REG_MODE, mode);
    }

    /**************************************************************************/
    /*!
      @brief Set the realtime value when in RTP mode, used to directly drive the
      haptic motor.
      @param rtp 8-bit drive value.
    */
    /**************************************************************************/
    void setRealtimeValue(uint8_t rtp)
    {
        comm->writeRegister(DRV2605_REG_RTPIN, rtp);
    }

    /**************************************************************************/
    /*!
      @brief Use ERM (Eccentric Rotating Mass) mode.
    */
    /**************************************************************************/
    void useERM()
    {
        comm->writeRegister(DRV2605_REG_FEEDBACK, comm->readRegister(DRV2605_REG_FEEDBACK) & 0x7F);
    }

    /**************************************************************************/
    /*!
      @brief Use LRA (Linear Resonance Actuator) mode.
    */
    /**************************************************************************/
    void useLRA()
    {
        comm->writeRegister(DRV2605_REG_FEEDBACK, comm->readRegister(DRV2605_REG_FEEDBACK) | 0x80);
    }

private:
    bool initImpl()
    {
        int chipID = comm->readRegister(DRV2605_REG_STATUS);
        if (chipID < 0) {
            return false;
        }
        chipID >>= 5;

        if (chipID != DRV2604_CHIP_ID &&
                chipID != DRV2605_CHIP_ID &&
                chipID != DRV2604L_CHIP_ID &&
                chipID != DRV2605L_CHIP_ID ) {
            log_e("ChipID:0x%x should be 0x03 or 0x04 or 0x06 or 0x07\n", chipID);
            return false;
        }

        comm->writeRegister(DRV2605_REG_MODE, (uint8_t)0x00); // out of standby

        comm->writeRegister(DRV2605_REG_RTPIN, (uint8_t)0x00); // no real-time-playback

        comm->writeRegister(DRV2605_REG_WAVESEQ1, (uint8_t)1); // strong click
        comm->writeRegister(DRV2605_REG_WAVESEQ2, (uint8_t)0); // end sequence

        comm->writeRegister(DRV2605_REG_OVERDRIVE, (uint8_t)0); // no overdrive

        comm->writeRegister(DRV2605_REG_SUSTAINPOS, (uint8_t)0);
        comm->writeRegister(DRV2605_REG_SUSTAINNEG, (uint8_t)0);
        comm->writeRegister(DRV2605_REG_BREAK, (uint8_t)0);
        comm->writeRegister(DRV2605_REG_AUDIOMAX, (uint8_t)0x64);

        // ERM open loop

        // turn off N_ERM_LRA
        comm->writeRegister(DRV2605_REG_FEEDBACK,
                            comm->readRegister(DRV2605_REG_FEEDBACK) & 0x7F);
        // turn on ERM_OPEN_LOOP
        comm->writeRegister(DRV2605_REG_CONTROL3,
                            comm->readRegister(DRV2605_REG_CONTROL3) | 0x20);

        return true;
    }

protected:
    std::unique_ptr<SensorCommBase> comm;
};



