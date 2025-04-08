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
 * @file      SensorWireHelper.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-15
 *
 */
#include <SPI.h>
#include <SensorWireHelper.h>
#include <Commander.h>              //Deplib https://github.com/CreativeRobotics/Commander

#ifndef SerialMon
#define SerialMon   Serial
#endif

#ifndef SENSOR_SDA
#define SENSOR_SDA  2
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  3
#endif

Commander cmd;

bool dumpReg(Commander &Cmdr);
bool dumpDevices(Commander &Cmdr);
bool helpHandler(Commander &Cmdr);

//COMMAND ARRAY ------------------------------------------------------------------------------
const commandList_t masterCommands[] = {
    {"help",     helpHandler, "help"},
    {"dump reg", dumpReg, "dump reg"},
    {"dump devices", dumpDevices, "dump devices"},

};

bool helpHandler(Commander &Cmdr)
{
    SerialMon.println("Help:");
    SerialMon.println("\tdump reg [device address] [reg address] [request read len]");
    SerialMon.println("\tdump devices");
    return false;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Start!");

#if (defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_STM32)) && !defined(ARDUINO_ARCH_MBED)
    Wire.setSCL(SENSOR_SCL);
    Wire.setSDA(SENSOR_SDA);
    Wire.begin();
#elif defined(ARDUINO_ARCH_NRF52)
    Wire.setPins(SENSOR_SDA, SENSOR_SCL);
    Wire.begin();
#elif defined(ARDUINO_ARCH_ESP32)
    Wire.begin(SENSOR_SDA, SENSOR_SCL);
#else
    Wire.begin();
#endif

    // Initialise Commander
    cmd.begin(&SerialMon, masterCommands, sizeof(masterCommands));
    cmd.commandPrompt(ON); //enable the command prompt
    cmd.echo(true);     //Echo incoming characters to theoutput port
    cmd.errorMessages(ON); //error messages are enabled - it will tell us if we issue any unrecognised commands
    //Error messaged do NOT work for quick set and get commands

    cmd.printCommandPrompt();
}

void loop()
{
    //Call the update functions using the activeCommander pointer
    cmd.update();
}

uint8_t toInt(String data)
{
    uint8_t retVal = 0x0;
    if (data.startsWith("0x")) {
        retVal = (uint8_t)strtol(data.c_str(), NULL, 16);
    } else {
        retVal = atoi(data.c_str());
    }
    return retVal;
}

bool dumpReg(Commander &Cmdr)
{
    int items = Cmdr.countItems();
    if (items < 3) {
        return 0;
    }
    String dev_address_str, reg_str, request_read_len_str;
    uint8_t dev_address = 0, reg = 0, request_read_len = 0;

    Cmdr.getString(dev_address_str);
    Cmdr.getString(reg_str);
    Cmdr.getString(request_read_len_str);

    dev_address = toInt(dev_address_str);
    reg = toInt(reg_str);
    request_read_len = toInt(request_read_len_str);

    SerialMon.print("Reg : 0x");
    if (dev_address < 16) {
        SerialMon.print("0");
    }
    SerialMon.print(dev_address, HEX);
    SerialMon.print(" - start:0x");
    if (reg < 16) {
        SerialMon.print("0");
    }
    SerialMon.print(reg, HEX);
    SerialMon.print(" end:0x");
    if (request_read_len < 16) {
        SerialMon.print("0");
    }
    SerialMon.print(request_read_len, HEX);
    SerialMon.println();

    int  ret = SensorWireHelper::regdump(Wire, SerialMon, dev_address, reg, request_read_len);
    if (ret == -1) {
        SerialMon.println("ERROR!");
    }
    return false;
}

bool dumpDevices(Commander &Cmdr)
{
    SensorWireHelper::dumpDevices(Wire, SerialMon);
    return false;
}

