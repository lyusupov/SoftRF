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
 * @file      BHI260APSensorInfo.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-22
 *
 */
#pragma once

#include <stdint.h>
#include <stdio.h>
#include "bhy2_parse.h"
#ifdef ARDUINO
#include <Stream.h>
#endif

class BoschPhySensorInfo
{
public:
    uint8_t sensor_type;
    uint8_t driver_id;
    uint8_t driver_version;
    float power_current;
    uint16_t curr_range;
    bool irq_status;
    int master_intf;
    int power_mode;
    uint8_t slave_address;
    uint8_t gpio_assignment;
    float curr_rate;
    uint8_t num_axis;
    int8_t orientation_matrix[9];
    uint8_t reserved;
    uint8_t flags;


    BoschPhySensorInfo() : sensor_type(0), driver_id(0), driver_version(0), power_current(0), curr_range(0), irq_status(false),
        master_intf(0), power_mode(0), slave_address(0), gpio_assignment(0), curr_rate(0), num_axis(0), reserved(0), flags(0)
    {
        memset(orientation_matrix, 0, sizeof(orientation_matrix));
    }

#if defined(ARDUINO) && !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
    void print(Stream &stream)
    {
        const char *irq_status_str[2] = { "Disabled", "Enabled" };
        const char *master_intf_str[5] = { "None", "SPI0", "I2C0", "SPI1", "I2C1" };
        const char *power_mode_str[8] = {
            "Sensor Not Present", "Power Down", "Suspend", "Self-Test", "Interrupt Motion", "One Shot",
            "Low Power Active", "Active"
        };

        stream.printf("Field Name            hex                    | Value (dec)\n");
        stream.printf("----------------------------------------------------------\n");
        stream.printf("Physical Sensor ID    %02X                     | %d\n", sensor_type, sensor_type);
        stream.printf("Driver ID             %02X                     | %d\n", driver_id, driver_id);
        stream.printf("Driver Version        %02X                     | %d\n", driver_version, driver_version);
        stream.printf("Current Consumption   %02X                     | %0.3f mA\n", (int)power_current, power_current);
        stream.printf("Dynamic Range         %04X                   | %d\n", curr_range, curr_range);
        stream.printf("Flags                 %02X                     | IRQ status       : %s\n", flags, irq_status_str[irq_status ? 1 : 0]);
        stream.printf("                                             | Master interface : %s\n", master_intf_str[master_intf]);
        stream.printf("                                             | Power mode       : %s\n", power_mode_str[power_mode]);
        stream.printf("Slave Address         %02X                     | %d\n", slave_address, slave_address);
        stream.printf("GPIO Assignment       %02X                     | %d\n", gpio_assignment, gpio_assignment);
        stream.printf("Current Rate          %08X               | %.3f Hz\n", (unsigned int)curr_rate, curr_rate);
        stream.printf("Number of axes        %02X                     | %d\n", num_axis, num_axis);
        stream.printf("Orientation Matrix    %02X%02X%02X%02X%02X             | %+02d %+02d %+02d |\n",
                      orientation_matrix[0],
                      orientation_matrix[1],
                      orientation_matrix[2],
                      orientation_matrix[3],
                      orientation_matrix[4],
                      orientation_matrix[0],
                      orientation_matrix[1],
                      orientation_matrix[2]);
        stream.printf("                                             | %+02d %+02d %+02d |\n",
                      orientation_matrix[3],
                      orientation_matrix[4],
                      orientation_matrix[5]);
        stream.printf("                                             | %+02d %+02d %+02d |\n",
                      orientation_matrix[6],
                      orientation_matrix[7],
                      orientation_matrix[8]);
        stream.printf("Reserved              %02X                     | %d\n", reserved, reserved);
        stream.printf("\n");
    }
#else
    void print()
    {
        const char *irq_status_str[2] = { "Disabled", "Enabled" };
        const char *master_intf_str[5] = { "None", "SPI0", "I2C0", "SPI1", "I2C1" };
        const char *power_mode_str[8] = {
            "Sensor Not Present", "Power Down", "Suspend", "Self-Test", "Interrupt Motion", "One Shot",
            "Low Power Active", "Active"
        };

        printf("Field Name            hex                    | Value (dec)\n");
        printf("----------------------------------------------------------\n");
        printf("Physical Sensor ID    %02X                     | %d\n", sensor_type, sensor_type);
        printf("Driver ID             %02X                     | %d\n", driver_id, driver_id);
        printf("Driver Version        %02X                     | %d\n", driver_version, driver_version);
        printf("Current Consumption   %02X                     | %0.3f mA\n", (int)power_current, power_current);
        printf("Dynamic Range         %04X                   | %d\n", curr_range, curr_range);
        printf("Flags                 %02X                     | IRQ status       : %s\n", flags, irq_status_str[irq_status ? 1 : 0]);
        printf("                                             | Master interface : %s\n", master_intf_str[master_intf]);
        printf("                                             | Power mode       : %s\n", power_mode_str[power_mode]);
        printf("Slave Address         %02X                     | %d\n", slave_address, slave_address);
        printf("GPIO Assignment       %02X                     | %d\n", gpio_assignment, gpio_assignment);
        printf("Current Rate          %08X               | %.3f Hz\n", (unsigned int)curr_rate, curr_rate);
        printf("Number of axes        %02X                     | %d\n", num_axis, num_axis);
        printf("Orientation Matrix    %02X%02X%02X%02X%02X             | %+02d %+02d %+02d |\n",
               orientation_matrix[0],
               orientation_matrix[1],
               orientation_matrix[2],
               orientation_matrix[3],
               orientation_matrix[4],
               orientation_matrix[0],
               orientation_matrix[1],
               orientation_matrix[2]);
        printf("                                             | %+02d %+02d %+02d |\n",
               orientation_matrix[3],
               orientation_matrix[4],
               orientation_matrix[5]);
        printf("                                             | %+02d %+02d %+02d |\n",
               orientation_matrix[6],
               orientation_matrix[7],
               orientation_matrix[8]);
        printf("Reserved              %02X                     | %d\n", reserved, reserved);
        printf("\n");
    }
#endif
};

