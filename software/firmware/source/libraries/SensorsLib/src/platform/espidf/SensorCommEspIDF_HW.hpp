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
 * @file      SensorCommEspIDF_HW.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-18
 *
 */
#pragma once

#include "../SensorCommBase.hpp"

#if !defined(ARDUINO)  && defined(ESP_PLATFORM)

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "esp_idf_version.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "driver/gpio.h"

class HalEspIDF: public SensorHal
{
public:
    void pinMode(uint8_t pin, uint8_t mode)
    {
        if (modeCallback) {
            modeCallback(pin, mode);
        } else {
            gpio_config_t config;
            memset(&config, 0, sizeof(config));
            config.pin_bit_mask = 1ULL << pin;
            switch (mode) {
            case INPUT:
                config.mode = GPIO_MODE_INPUT;
                break;
            case OUTPUT:
                config.mode = GPIO_MODE_OUTPUT;
                break;
            }
            config.pull_up_en = GPIO_PULLUP_DISABLE;
            config.pull_down_en = GPIO_PULLDOWN_DISABLE;
            config.intr_type = GPIO_INTR_DISABLE;
            ESP_ERROR_CHECK(gpio_config(&config));
        }
    }

    void digitalWrite(uint8_t pin, uint8_t level)
    {
        if (writeCallback) {
            writeCallback(pin, level);
        } else {
            gpio_set_level((gpio_num_t )pin, level);
        }
    }

    uint8_t digitalRead(uint8_t pin)
    {
        if (readCallback) {
            return readCallback(pin);
        }
        return gpio_get_level((gpio_num_t)pin);
    }

    void delay(uint32_t ms)
    {
        ets_delay_us((ms % portTICK_PERIOD_MS) * 1000UL);
    }

    uint32_t millis()
    {
        return (uint32_t) (esp_timer_get_time() / 1000LL);
    }

    uint32_t micros()
    {
        return (uint32_t) esp_timer_get_time();
    }

    void delayMicroseconds(uint32_t us)
    {
        ets_delay_us(us);
    }
};



#endif  //*ESP_PLATFORM
