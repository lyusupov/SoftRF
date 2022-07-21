/////////////////////////////////////////////////////////////////
/*
           __   _______ ___   ___ ___
     /\    \ \ / /  __ \__ \ / _ \__ \
    /  \    \ V /| |__) | ) | | | | ) |
   / /\ \    > < |  ___/ / /| | | |/ /
  / ____ \  / . \| |    / /_| |_| / /_
 /_/    \_\/_/ \_\_|   |____|\___/____|


MIT License

Copyright (c) 2019 lewis he

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

axp20x.cpp - Arduino library for X-Power AXP202 chip.
Created by Lewis he on April 1, 2019.
github:https://github.com/lewisxhe/AXP202X_Libraries
*/
/////////////////////////////////////////////////////////////////

#include "axp20x.h"
#include <math.h>

#define ISCONNECETD(ret)       do{if(!_init)return ret;}while(0)

const uint8_t AXP20X_Class::startupParams[] = {
    0b00000000,
    0b01000000,
    0b10000000,
    0b11000000
};

const uint8_t AXP20X_Class::longPressParams[] = {
    0b00000000,
    0b00010000,
    0b00100000,
    0b00110000
};

const uint8_t AXP20X_Class::shutdownParams[] = {
    0b00000000,
    0b00000001,
    0b00000010,
    0b00000011
};

const uint8_t AXP20X_Class::targetVolParams[] = {
    0b00000000,
    0b00100000,
    0b01000000,
    0b01100000
};



// Power Output Control register
uint8_t AXP20X_Class::_outputReg;

int AXP20X_Class::_axp_probe(void)
{
    uint8_t data;
    if (_isAxp173) {
        //!Axp173 does not have a chip ID, read the status register to see if it reads normally
        _readByte(0x01, 1, &data);
        if (data == 0 || data == 0xFF) {
            return AXP_FAIL;
        }
        _chip_id = AXP173_CHIP_ID;
        _readByte(AXP202_LDO234_DC23_CTL, 1, &_outputReg);
        AXP_DEBUG("OUTPUT Register 0x%x\n", _outputReg);
        _init = true;
        return AXP_PASS;
    }
    _readByte(AXP202_IC_TYPE, 1, &_chip_id);
    AXP_DEBUG("chip id detect 0x%x\n", _chip_id);
    if (_chip_id == AXP202_CHIP_ID || _chip_id == AXP192_CHIP_ID) {
        AXP_DEBUG("Detect CHIP :%s\n", _chip_id == AXP202_CHIP_ID ? "AXP202" : "AXP192");
        _readByte(AXP202_LDO234_DC23_CTL, 1, &_outputReg);
        AXP_DEBUG("OUTPUT Register 0x%x\n", _outputReg);
        _init = true;
        return AXP_PASS;
    }
    return AXP_FAIL;
}

#ifdef ARDUINO
int AXP20X_Class::begin(TwoWire &port, uint8_t addr, bool isAxp173)
{
    _i2cPort = &port; //Grab which port the user wants us to use
    _address = addr;
    _isAxp173 = isAxp173;

    return _axp_probe();
}
#endif

int AXP20X_Class::begin(axp_com_fptr_t read_cb, axp_com_fptr_t write_cb, uint8_t addr, bool isAxp173)
{
    if (read_cb == nullptr || write_cb == nullptr)return AXP_FAIL;
    _read_cb = read_cb;
    _write_cb = write_cb;
    _address = addr;
    _isAxp173 = isAxp173;
    return _axp_probe();
}

//Only axp192 chip
bool AXP20X_Class::isDCDC1Enable(void)
{
    if (_chip_id == AXP192_CHIP_ID)
        return IS_OPEN(_outputReg, AXP192_DCDC1);
    else if (_chip_id == AXP173_CHIP_ID)
        return IS_OPEN(_outputReg, AXP173_DCDC1);
    return false;
}

bool AXP20X_Class::isExtenEnable(void)
{
    if (_chip_id == AXP192_CHIP_ID)
        return IS_OPEN(_outputReg, AXP192_EXTEN);
    else if (_chip_id == AXP202_CHIP_ID)
        return IS_OPEN(_outputReg, AXP202_EXTEN);
    else if (_chip_id == AXP173_CHIP_ID) {
        uint8_t data;
        _readByte(AXP173_EXTEN_DC2_CTL, 1, &data);
        return IS_OPEN(data, AXP173_CTL_EXTEN_BIT);
    }
    return false;
}

bool AXP20X_Class::isLDO2Enable(void)
{
    if (_chip_id == AXP173_CHIP_ID) {
        return IS_OPEN(_outputReg, AXP173_LDO2);
    }
    //axp192 same axp202 ldo2 bit
    return IS_OPEN(_outputReg, AXP202_LDO2);
}

bool AXP20X_Class::isLDO3Enable(void)
{
    if (_chip_id == AXP192_CHIP_ID)
        return IS_OPEN(_outputReg, AXP192_LDO3);
    else if (_chip_id == AXP202_CHIP_ID)
        return IS_OPEN(_outputReg, AXP202_LDO3);
    else if (_chip_id == AXP173_CHIP_ID)
        return IS_OPEN(_outputReg, AXP173_LDO3);
    return false;
}

bool AXP20X_Class::isLDO4Enable(void)
{
    if (_chip_id == AXP202_CHIP_ID)
        return IS_OPEN(_outputReg, AXP202_LDO4);
    if (_chip_id == AXP173_CHIP_ID)
        return IS_OPEN(_outputReg, AXP173_LDO4);
    return false;
}

bool AXP20X_Class::isDCDC2Enable(void)
{
    if (_chip_id == AXP173_CHIP_ID) {
        uint8_t data;
        _readByte(AXP173_EXTEN_DC2_CTL, 1, &data);
        return IS_OPEN(data, AXP173_CTL_DC2_BIT);
    }
    //axp192 same axp202 dc2 bit
    return IS_OPEN(_outputReg, AXP202_DCDC2);
}

bool AXP20X_Class::isDCDC3Enable(void)
{
    if (_chip_id == AXP173_CHIP_ID)
        return false;
    //axp192 same axp202 dc3 bit
    return IS_OPEN(_outputReg, AXP202_DCDC3);
}

int AXP20X_Class::setPowerOutPut(uint8_t ch, bool en)
{
    uint8_t data;
    uint8_t val = 0;
    if (!_init)
        return AXP_NOT_INIT;

    //! Axp173 cannot use the REG12H register to control
    //! DC2 and EXTEN. It is necessary to control REG10H separately.
    if (_chip_id == AXP173_CHIP_ID) {
        _readByte(AXP173_EXTEN_DC2_CTL, 1, &data);
        if (ch & AXP173_DCDC2) {
            data = en ? data | _BV(AXP173_CTL_DC2_BIT) : data & (~_BV(AXP173_CTL_DC2_BIT));
            ch &= (~_BV(AXP173_DCDC2));
            _writeByte(AXP173_EXTEN_DC2_CTL, 1, &data);
        } else if (ch & AXP173_EXTEN) {
            data = en ? data | _BV(AXP173_CTL_EXTEN_BIT) : data & (~_BV(AXP173_CTL_EXTEN_BIT));
            ch &= (~_BV(AXP173_EXTEN));
            _writeByte(AXP173_EXTEN_DC2_CTL, 1, &data);
        }
    }

    _readByte(AXP202_LDO234_DC23_CTL, 1, &data);
    if (en) {
        data |= (1 << ch);
    } else {
        data &= (~(1 << ch));
    }

    if (_chip_id == AXP202_CHIP_ID) {
        FORCED_OPEN_DCDC3(data); //! Must be forced open in T-Watch
    }

    _writeByte(AXP202_LDO234_DC23_CTL, 1, &data);

#ifdef ARDUINO
    delay(1);
#endif
    _readByte(AXP202_LDO234_DC23_CTL, 1, &val);
    if (data == val) {
        _outputReg = val;
        return AXP_PASS;
    }
    return AXP_FAIL;
}

bool AXP20X_Class::isChargeing(void)
{
    return isCharging();
}

bool AXP20X_Class::isCharging(void)
{
    uint8_t reg;
    if (!_init)
        return AXP_NOT_INIT;
    _readByte(AXP202_MODE_CHGSTATUS, 1, &reg);
    return IS_OPEN(reg, 6);
}

bool AXP20X_Class::isBatteryConnect(void)
{
    uint8_t reg;
    if (!_init)
        return AXP_NOT_INIT;
    _readByte(AXP202_MODE_CHGSTATUS, 1, &reg);
    return IS_OPEN(reg, 5);
}

float AXP20X_Class::getAcinVoltage(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_ACIN_VOL_H8, AXP202_ACIN_VOL_L4) * AXP202_ACIN_VOLTAGE_STEP;
}

float AXP20X_Class::getAcinCurrent(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_ACIN_CUR_H8, AXP202_ACIN_CUR_L4) * AXP202_ACIN_CUR_STEP;
}

float AXP20X_Class::getVbusVoltage(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_VBUS_VOL_H8, AXP202_VBUS_VOL_L4) * AXP202_VBUS_VOLTAGE_STEP;
}

float AXP20X_Class::getVbusCurrent(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_VBUS_CUR_H8, AXP202_VBUS_CUR_L4) * AXP202_VBUS_CUR_STEP;
}

float AXP20X_Class::getTemp(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    // Internal temperature
    // 000H => -144.7℃
    // STEP => 0.1℃
    // FFFH => 264.8℃
    return _getRegistResult(AXP202_INTERNAL_TEMP_H8, AXP202_INTERNAL_TEMP_L4)  * AXP202_INTERNAL_TEMP_STEP  - 144.7;
}

float AXP20X_Class::getTSTemp(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_TS_IN_H8, AXP202_TS_IN_L4) * AXP202_TS_PIN_OUT_STEP;
}

float AXP20X_Class::getGPIO0Voltage(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_GPIO0_VOL_ADC_H8, AXP202_GPIO0_VOL_ADC_L4) * AXP202_GPIO0_STEP;
}

float AXP20X_Class::getGPIO1Voltage(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_GPIO1_VOL_ADC_H8, AXP202_GPIO1_VOL_ADC_L4) * AXP202_GPIO1_STEP;
}

/*
Note: the battery power formula:
Pbat =2* register value * Voltage LSB * Current LSB / 1000.
(Voltage LSB is 1.1mV; Current LSB is 0.5mA, and unit of calculation result is mW.)
*/
float AXP20X_Class::getBattInpower(void)
{
    float rslt;
    uint8_t hv, mv, lv;
    if (!_init)
        return AXP_NOT_INIT;
    _readByte(AXP202_BAT_POWERH8, 1, &hv);
    _readByte(AXP202_BAT_POWERM8, 1, &mv);
    _readByte(AXP202_BAT_POWERL8, 1, &lv);
    rslt = (hv << 16) | (mv << 8) | lv;
    rslt = 2 * rslt * 1.1 * 0.5 / 1000;
    return rslt;
}

float AXP20X_Class::getBattVoltage(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_BAT_AVERVOL_H8, AXP202_BAT_AVERVOL_L4) * AXP202_BATT_VOLTAGE_STEP;
}

float AXP20X_Class::getBattChargeCurrent(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    switch (_chip_id) {
    case AXP202_CHIP_ID:
        return _getRegistResult(AXP202_BAT_AVERCHGCUR_H8, AXP202_BAT_AVERCHGCUR_L4) * AXP202_BATT_CHARGE_CUR_STEP;
    case AXP192_CHIP_ID:
        return _getRegistH8L5(AXP202_BAT_AVERCHGCUR_H8, AXP202_BAT_AVERCHGCUR_L5) * AXP202_BATT_CHARGE_CUR_STEP;
    default:
        return AXP_FAIL;
    }
}

float AXP20X_Class::getBattDischargeCurrent(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    return _getRegistH8L5(AXP202_BAT_AVERDISCHGCUR_H8, AXP202_BAT_AVERDISCHGCUR_L5) * AXP202_BATT_DISCHARGE_CUR_STEP;
}

float AXP20X_Class::getSysIPSOUTVoltage(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    return _getRegistResult(AXP202_APS_AVERVOL_H8, AXP202_APS_AVERVOL_L4) * AXP202_APS_VOLTAGE_STEP;
}

/*
Coulomb calculation formula:
C= 65536 * current LSB *（charge coulomb counter value - discharge coulomb counter value） /
3600 / ADC sample rate. Refer to REG84H setting for ADC sample rate；the current LSB is
0.5mA；unit of the calculation result is mAh. ）
*/
uint32_t AXP20X_Class::getBattChargeCoulomb(void)
{
    uint8_t buffer[4];
    if (!_init)
        return AXP_NOT_INIT;
    _readByte(0xB0, 4, buffer);
    return (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
}

uint32_t AXP20X_Class::getBattDischargeCoulomb(void)
{
    uint8_t buffer[4];
    if (!_init)
        return AXP_NOT_INIT;
    _readByte(0xB4, 4, buffer);
    return (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
}

float AXP20X_Class::getCoulombData(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    uint32_t charge = getBattChargeCoulomb(), discharge = getBattDischargeCoulomb();
    uint8_t rate = getAdcSamplingRate();
    float result = 65536.0 * 0.5 * ((float)charge - (float)discharge) / 3600.0 / rate;
    return result;
}


//-------------------------------------------------------
// New Coulomb functions  by MrFlexi
//-------------------------------------------------------

uint8_t AXP20X_Class::getCoulombRegister(void)
{
    uint8_t buffer;
    if (!_init)
        return AXP_NOT_INIT;
    _readByte(AXP202_COULOMB_CTL, 1, &buffer);
    return buffer;
}


int AXP20X_Class::setCoulombRegister(uint8_t val)
{
    if (!_init)
        return AXP_NOT_INIT;
    _writeByte(AXP202_COULOMB_CTL, 1, &val);
    return AXP_PASS;
}


int AXP20X_Class::EnableCoulombcounter(void)
{

    if (!_init)
        return AXP_NOT_INIT;
    uint8_t val = 0x80;
    _writeByte(AXP202_COULOMB_CTL, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::DisableCoulombcounter(void)
{

    if (!_init)
        return AXP_NOT_INIT;
    uint8_t val = 0x00;
    _writeByte(AXP202_COULOMB_CTL, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::StopCoulombcounter(void)
{

    if (!_init)
        return AXP_NOT_INIT;
    uint8_t val = 0xB8;
    _writeByte(AXP202_COULOMB_CTL, 1, &val);
    return AXP_PASS;
}


int AXP20X_Class::ClearCoulombcounter(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    uint8_t val = 0xA0;
    _writeByte(AXP202_COULOMB_CTL, 1, &val);
    return AXP_PASS;
}

//-------------------------------------------------------
// END
//-------------------------------------------------------



uint8_t AXP20X_Class::getAdcSamplingRate(void)
{
    //axp192 same axp202 aregister address 0x84
    if (!_init)
        return AXP_NOT_INIT;
    uint8_t val;
    _readByte(AXP202_ADC_SPEED, 1, &val);
    return 25 * (int)pow(2, (val & 0xC0) >> 6);
}

int AXP20X_Class::setAdcSamplingRate(axp_adc_sampling_rate_t rate)
{
    //axp192 same axp202 aregister address 0x84
    if (!_init)
        return AXP_NOT_INIT;
    if (rate > AXP_ADC_SAMPLING_RATE_200HZ)
        return AXP_FAIL;
    uint8_t val;
    _readByte(AXP202_ADC_SPEED, 1, &val);
    uint8_t rw = rate;
    val &= 0x3F;
    val |= (rw << 6);
    _writeByte(AXP202_ADC_SPEED, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::setTSfunction(axp_ts_pin_function_t func)
{
    //axp192 same axp202 aregister address 0x84
    if (!_init)
        return AXP_NOT_INIT;
    if (func > AXP_TS_PIN_FUNCTION_ADC)
        return AXP_FAIL;
    uint8_t val;
    _readByte(AXP202_ADC_SPEED, 1, &val);
    uint8_t rw = func;
    val &= 0xFA;
    val |= (rw << 2);
    _writeByte(AXP202_ADC_SPEED, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::setTScurrent(axp_ts_pin_current_t current)
{
    //axp192 same axp202 aregister address 0x84
    if (!_init)
        return AXP_NOT_INIT;
    if (current > AXP_TS_PIN_CURRENT_80UA)
        return AXP_FAIL;
    uint8_t val;
    _readByte(AXP202_ADC_SPEED, 1, &val);
    uint8_t rw = current;
    val &= 0xCF;
    val |= (rw << 4);
    _writeByte(AXP202_ADC_SPEED, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::setTSmode(axp_ts_pin_mode_t mode)
{
    //axp192 same axp202 aregister address 0x84
    if (!_init)
        return AXP_NOT_INIT;
    if (mode > AXP_TS_PIN_MODE_ENABLE)
        return AXP_FAIL;
    uint8_t val;
    _readByte(AXP202_ADC_SPEED, 1, &val);
    uint8_t rw = mode;
    val &= 0xFC;
    val |= rw;
    _writeByte(AXP202_ADC_SPEED, 1, &val);

    // TS pin ADC function enable/disable
    if (mode == AXP_TS_PIN_MODE_DISABLE)
        adc1Enable(AXP202_TS_PIN_ADC1, false);
    else
        adc1Enable(AXP202_TS_PIN_ADC1, true);
    return AXP_PASS;
}

int AXP20X_Class::adc1Enable(uint16_t params, bool en)
{
    if (!_init)
        return AXP_NOT_INIT;
    uint8_t val;
    _readByte(AXP202_ADC_EN1, 1, &val);
    if (en)
        val |= params;
    else
        val &= ~(params);
    _writeByte(AXP202_ADC_EN1, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::adc2Enable(uint16_t params, bool en)
{
    if (!_init)
        return AXP_NOT_INIT;
    uint8_t val;
    _readByte(AXP202_ADC_EN2, 1, &val);
    if (en)
        val |= params;
    else
        val &= ~(params);
    _writeByte(AXP202_ADC_EN2, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::enableIRQ(uint64_t params, bool en)
{
    if (!_init)
        return AXP_NOT_INIT;
    uint8_t val, val1;
    if (params & 0xFFUL) {
        val1 = params & 0xFF;
        _readByte(AXP202_INTEN1, 1, &val);
        if (en)
            val |= val1;
        else
            val &= ~(val1);
        AXP_DEBUG("%s [0x%x]val:0x%x\n", en ? "enable" : "disable", AXP202_INTEN1, val);
        _writeByte(AXP202_INTEN1, 1, &val);
    }
    if (params & 0xFF00UL) {
        val1 = params >> 8;
        _readByte(AXP202_INTEN2, 1, &val);
        if (en)
            val |= val1;
        else
            val &= ~(val1);
        AXP_DEBUG("%s [0x%x]val:0x%x\n", en ? "enable" : "disable", AXP202_INTEN2, val);
        _writeByte(AXP202_INTEN2, 1, &val);
    }

    if (params & 0xFF0000UL) {
        val1 = params >> 16;
        _readByte(AXP202_INTEN3, 1, &val);
        if (en)
            val |= val1;
        else
            val &= ~(val1);
        AXP_DEBUG("%s [0x%x]val:0x%x\n", en ? "enable" : "disable", AXP202_INTEN3, val);
        _writeByte(AXP202_INTEN3, 1, &val);
    }

    if (params & 0xFF000000UL) {
        val1 = params >> 24;
        _readByte(AXP202_INTEN4, 1, &val);
        if (en)
            val |= val1;
        else
            val &= ~(val1);
        AXP_DEBUG("%s [0x%x]val:0x%x\n", en ? "enable" : "disable", AXP202_INTEN4, val);
        _writeByte(AXP202_INTEN4, 1, &val);
    }

    if (params & 0xFF00000000ULL) {
        val1 = params >> 32;
        uint8_t reg = _chip_id == AXP192_CHIP_ID ? AXP192_INTEN5 : AXP202_INTEN5;
        _readByte(reg, 1, &val);
        if (en)
            val |= val1;
        else
            val &= ~(val1);
        AXP_DEBUG("%s [0x%x]val:0x%x\n", en ? "enable" : "disable", reg, val);
        _writeByte(reg, 1, &val);
    }
    return AXP_PASS;
}

int AXP20X_Class::readIRQ(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    switch (_chip_id) {
    case AXP192_CHIP_ID:
        for (int i = 0; i < 4; ++i) {
            _readByte(AXP192_INTSTS1 + i, 1, &_irq[i]);
        }
        _readByte(AXP192_INTSTS5, 1, &_irq[4]);
        return AXP_PASS;

    case AXP202_CHIP_ID:
        for (int i = 0; i < 5; ++i) {
            _readByte(AXP202_INTSTS1 + i, 1, &_irq[i]);
        }
        return AXP_PASS;
    default:
        return AXP_FAIL;
    }
}

void AXP20X_Class::clearIRQ(void)
{
    uint8_t val = 0xFF;
    switch (_chip_id) {
    case AXP192_CHIP_ID:
        for (int i = 0; i < 3; i++) {
            _writeByte(AXP192_INTSTS1 + i, 1, &val);
        }
        _writeByte(AXP192_INTSTS5, 1, &val);
        break;
    case AXP202_CHIP_ID:
        for (int i = 0; i < 5; i++) {
            _writeByte(AXP202_INTSTS1 + i, 1, &val);
        }
        break;
    default:
        break;
    }
    memset(_irq, 0, sizeof(_irq));
}


bool AXP20X_Class::isVBUSPlug(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    uint8_t reg;
    _readByte(AXP202_STATUS, 1, &reg);
    return IS_OPEN(reg, 5);
}

//IRQ1 REGISTER : AXP202:0x40H AXP192:0X44H
bool AXP20X_Class::isAcinOverVoltageIRQ(void)
{
    return (bool)(_irq[0] & _BV(7));
}

bool AXP20X_Class::isAcinPlugInIRQ(void)
{
    return (bool)(_irq[0] & _BV(6));
}

bool AXP20X_Class::isAcinRemoveIRQ(void)
{
    return (bool)(_irq[0] & _BV(5));
}

bool AXP20X_Class::isVbusOverVoltageIRQ(void)
{
    return (bool)(_irq[0] & _BV(4));
}

bool AXP20X_Class::isVbusPlugInIRQ(void)
{
    return (bool)(_irq[0] & _BV(3));
}

bool AXP20X_Class::isVbusRemoveIRQ(void)
{
    return (bool)(_irq[0] & _BV(2));
}

bool AXP20X_Class::isVbusLowVHOLDIRQ(void)
{
    return (bool)(_irq[0] & _BV(1));
}

//IRQ2 REGISTER : AXP202:0x41H AXP192:0X45H
bool AXP20X_Class::isBattPlugInIRQ(void)
{
    return (bool)(_irq[1] & _BV(7));
}

bool AXP20X_Class::isBattRemoveIRQ(void)
{
    return (bool)(_irq[1] & _BV(6));
}

bool AXP20X_Class::isBattEnterActivateIRQ(void)
{
    return (bool)(_irq[1] & _BV(5));
}

bool AXP20X_Class::isBattExitActivateIRQ(void)
{
    return (bool)(_irq[1] & _BV(4));
}

bool AXP20X_Class::isChargingIRQ(void)
{
    return (bool)(_irq[1] & _BV(3));
}

bool AXP20X_Class::isChargingDoneIRQ(void)
{
    return (bool)(_irq[1] & _BV(2));
}

bool AXP20X_Class::isBattTempHighIRQ(void)
{
    return (bool)(_irq[1] & _BV(1));
}

bool AXP20X_Class::isBattTempLowIRQ(void)
{
    return (bool)(_irq[1] & _BV(0));
}

//IRQ3 REGISTER : AXP202:0x42H AXP192:0X46H
bool AXP20X_Class::isChipOvertemperatureIRQ(void)
{
    return (bool)(_irq[2] & _BV(7));
}

bool AXP20X_Class::isChargingCurrentLessIRQ(void)
{
    return (bool)(_irq[2] & _BV(6));
}

// retention bit5

bool AXP20X_Class::isDC2VoltageLessIRQ(void)
{
    return (bool)(_irq[2] & _BV(4));
}

bool AXP20X_Class::isDC3VoltageLessIRQ(void)
{
    return (bool)(_irq[2] & _BV(3));
}

bool AXP20X_Class::isLDO3VoltageLessIRQ(void)
{
    return (bool)(_irq[2] & _BV(2));
}

bool AXP20X_Class::isPEKShortPressIRQ(void)
{
    return (bool)(_irq[2] & _BV(1));
}

bool AXP20X_Class::isPEKLongtPressIRQ(void)
{
    return (bool)(_irq[2] & _BV(0));
}

//IRQ4 REGISTER : AXP202:0x43H AXP192:0X47H
bool AXP20X_Class::isNOEPowerOnIRQ(void)
{
    return (bool)(_irq[3] & _BV(7));
}

bool AXP20X_Class::isNOEPowerDownIRQ(void)
{
    return (bool)(_irq[3] & _BV(6));
}

bool AXP20X_Class::isVBUSEffectiveIRQ(void)
{
    return (bool)(_irq[3] & _BV(5));
}

bool AXP20X_Class::isVBUSInvalidIRQ(void)
{
    return (bool)(_irq[3] & _BV(4));
}

bool AXP20X_Class::isVUBSSessionIRQ(void)
{
    return (bool)(_irq[3] & _BV(3));
}

bool AXP20X_Class::isVUBSSessionEndIRQ(void)
{
    return (bool)(_irq[3] & _BV(2));
}

bool AXP20X_Class::isLowVoltageLevel1IRQ(void)
{
    return (bool)(_irq[3] & _BV(1));
}

bool AXP20X_Class::isLowVoltageLevel2IRQ(void)
{
    return (bool)(_irq[3] & _BV(0));
}

//IRQ5 REGISTER : AXP202:0x44H AXP192:0X4DH
bool AXP20X_Class::isTimerTimeoutIRQ(void)
{
    return (bool)(_irq[4] & _BV(7));
}

bool AXP20X_Class::isPEKRisingEdgeIRQ(void)
{
    return (bool)(_irq[4] & _BV(6));
}

bool AXP20X_Class::isPEKFallingEdgeIRQ(void)
{
    return (bool)(_irq[4] & _BV(5));
}

// retention bit4

bool AXP20X_Class::isGPIO3InputEdgeTriggerIRQ(void)
{
    return (bool)(_irq[4] & _BV(3));
}

bool AXP20X_Class::isGPIO2InputEdgeTriggerIRQ(void)
{
    return (bool)(_irq[4] & _BV(2));
}

bool AXP20X_Class::isGPIO1InputEdgeTriggerIRQ(void)
{
    return (bool)(_irq[4] & _BV(1));
}

bool AXP20X_Class::isGPIO0InputEdgeTriggerIRQ(void)
{
    return (bool)(_irq[4] & _BV(0));
}


int AXP20X_Class::setDCDC2Voltage(uint16_t mv)
{
    if (!_init)
        return AXP_NOT_INIT;
    if (mv < 700) {
        AXP_DEBUG("DCDC2:Below settable voltage:700mV~2275mV");
        mv = 700;
    }
    if (mv > 2275) {
        AXP_DEBUG("DCDC2:Above settable voltage:700mV~2275mV");
        mv = 2275;
    }
    uint8_t val = (mv - 700) / 25;
    //! axp173/192/202 same register
    _writeByte(AXP202_DC2OUT_VOL, 1, &val);
    return AXP_PASS;
}

uint16_t AXP20X_Class::getDCDC2Voltage(void)
{
    uint8_t val = 0;
    //! axp173/192/202 same register
    _readByte(AXP202_DC2OUT_VOL, 1, &val);
    return val * 25 + 700;
}

uint16_t AXP20X_Class::getDCDC3Voltage(void)
{
    if (!_init)
        return 0;
    if (_chip_id == AXP173_CHIP_ID)return AXP_NOT_SUPPORT;
    uint8_t val = 0;
    _readByte(AXP202_DC3OUT_VOL, 1, &val);
    return val * 25 + 700;
}

int AXP20X_Class::setDCDC3Voltage(uint16_t mv)
{
    if (!_init)
        return AXP_NOT_INIT;
    if (_chip_id == AXP173_CHIP_ID)return AXP_NOT_SUPPORT;
    if (mv < 700) {
        AXP_DEBUG("DCDC3:Below settable voltage:700mV~3500mV");
        mv = 700;
    }
    if (mv > 3500) {
        AXP_DEBUG("DCDC3:Above settable voltage:700mV~3500mV");
        mv = 3500;
    }
    uint8_t val = (mv - 700) / 25;
    _writeByte(AXP202_DC3OUT_VOL, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::setLDO2Voltage(uint16_t mv)
{
    uint8_t rVal, wVal;
    if (!_init)
        return AXP_NOT_INIT;
    if (mv < 1800) {
        AXP_DEBUG("LDO2:Below settable voltage:1800mV~3300mV");
        mv = 1800;
    }
    if (mv > 3300) {
        AXP_DEBUG("LDO2:Above settable voltage:1800mV~3300mV");
        mv = 3300;
    }
    wVal = (mv - 1800) / 100;
    if (_chip_id == AXP202_CHIP_ID) {
        _readByte(AXP202_LDO24OUT_VOL, 1, &rVal);
        rVal &= 0x0F;
        rVal |= (wVal << 4);
        _writeByte(AXP202_LDO24OUT_VOL, 1, &rVal);
        return AXP_PASS;
    } else if (_chip_id == AXP192_CHIP_ID || _chip_id == AXP173_CHIP_ID) {
        _readByte(AXP192_LDO23OUT_VOL, 1, &rVal);
        rVal &= 0x0F;
        rVal |= (wVal << 4);
        _writeByte(AXP192_LDO23OUT_VOL, 1, &rVal);
        return AXP_PASS;
    }
    return AXP_FAIL;
}

uint16_t AXP20X_Class::getLDO2Voltage(void)
{
    uint8_t rVal;
    if (_chip_id == AXP202_CHIP_ID) {
        _readByte(AXP202_LDO24OUT_VOL, 1, &rVal);
        rVal &= 0xF0;
        rVal >>= 4;
        return rVal * 100 + 1800;
    } else if (_chip_id == AXP192_CHIP_ID || _chip_id == AXP173_CHIP_ID ) {
        _readByte(AXP192_LDO23OUT_VOL, 1, &rVal);
        AXP_DEBUG("get result:%x\n", rVal);
        rVal &= 0xF0;
        rVal >>= 4;
        return rVal * 100 + 1800;
    }
    return 0;
}

int AXP20X_Class::setLDO3Voltage(uint16_t mv)
{
    uint8_t rVal;
    if (!_init)
        return AXP_NOT_INIT;
    if (_chip_id == AXP202_CHIP_ID && mv < 700) {
        AXP_DEBUG("LDO3:Below settable voltage:700mV~3500mV");
        mv = 700;
    } else if (_chip_id == AXP192_CHIP_ID && mv < 1800) {
        AXP_DEBUG("LDO3:Below settable voltage:1800mV~3300mV");
        mv = 1800;
    }

    if (_chip_id == AXP202_CHIP_ID && mv > 3500) {
        AXP_DEBUG("LDO3:Above settable voltage:700mV~3500mV");
        mv = 3500;
    } else if (_chip_id == AXP192_CHIP_ID && mv > 3300) {
        AXP_DEBUG("LDO3:Above settable voltage:1800mV~3300mV");
        mv = 3300;
    }

    if (_chip_id == AXP202_CHIP_ID) {
        _readByte(AXP202_LDO3OUT_VOL, 1, &rVal);
        rVal &= 0x80;
        rVal |= ((mv - 700) / 25);
        _writeByte(AXP202_LDO3OUT_VOL, 1, &rVal);
        return AXP_PASS;
    } else if (_chip_id == AXP192_CHIP_ID || _chip_id == AXP173_CHIP_ID) {
        _readByte(AXP192_LDO23OUT_VOL, 1, &rVal);
        rVal &= 0xF0;
        rVal |= ((mv - 1800) / 100);
        _writeByte(AXP192_LDO23OUT_VOL, 1, &rVal);
        return AXP_PASS;
    }
    return AXP_FAIL;
}

uint16_t AXP20X_Class::getLDO3Voltage(void)
{
    uint8_t rVal;
    if (!_init)
        return AXP_NOT_INIT;

    if (_chip_id == AXP202_CHIP_ID) {
        _readByte(AXP202_LDO3OUT_VOL, 1, &rVal);
        if (rVal & 0x80) {
            //! According to the hardware N_VBUSEN Pin selection
            return getVbusVoltage() * 1000;
        } else {
            return (rVal & 0x7F) * 25 + 700;
        }
    } else if (_chip_id == AXP192_CHIP_ID || _chip_id == AXP173_CHIP_ID) {
        _readByte(AXP192_LDO23OUT_VOL, 1, &rVal);
        rVal &= 0x0F;
        return rVal * 100 + 1800;
    }
    return 0;
}

//! Only axp173 support
int AXP20X_Class::setLDO4Voltage(uint16_t mv)
{
    if (!_init)
        return AXP_NOT_INIT;
    if (_chip_id != AXP173_CHIP_ID)
        return AXP_FAIL;

    if (mv < 700) {
        AXP_DEBUG("LDO4:Below settable voltage:700mV~3500mV");
        mv = 700;
    }
    if (mv > 3500) {
        AXP_DEBUG("LDO4:Above settable voltage:700mV~3500mV");
        mv = 3500;
    }
    uint8_t val = (mv - 700) / 25;
    _writeByte(AXP173_LDO4_VLOTAGE, 1, &val);
    return AXP_PASS;
}

uint16_t AXP20X_Class::getLDO4Voltage(void)
{
    const uint16_t ldo4_table[] = {1250, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2500, 2700, 2800, 3000, 3100, 3200, 3300};
    if (!_init)
        return 0;
    uint8_t val = 0;
    switch (_chip_id) {
    case AXP173_CHIP_ID:
        _readByte(AXP173_LDO4_VLOTAGE, 1, &val);
        return val * 25 + 700;
    case AXP202_CHIP_ID:
        _readByte(AXP202_LDO24OUT_VOL, 1, &val);
        val &= 0xF;
        return ldo4_table[val];
        break;
    case AXP192_CHIP_ID:
    default:
        break;
    }
    return 0;
}


//! Only axp202 support
int AXP20X_Class::setLDO4Voltage(axp_ldo4_table_t param)
{
    if (!_init)
        return AXP_NOT_INIT;
    if (_chip_id == AXP202_CHIP_ID) {
        if (param >= AXP202_LDO4_MAX)
            return AXP_INVALID;
        uint8_t val;
        _readByte(AXP202_LDO24OUT_VOL, 1, &val);
        val &= 0xF0;
        val |= param;
        _writeByte(AXP202_LDO24OUT_VOL, 1, &val);
        return AXP_PASS;
    }
    return AXP_FAIL;
}

//! Only AXP202 support
int AXP20X_Class::setLDO3Mode(axp202_ldo3_mode_t mode)
{
    uint8_t val;
    if (_chip_id != AXP202_CHIP_ID)
        return AXP_FAIL;
    _readByte(AXP202_LDO3OUT_VOL, 1, &val);
    if (mode) {
        val |= _BV(7);
    } else {
        val &= (~_BV(7));
    }
    _writeByte(AXP202_LDO3OUT_VOL, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::setStartupTime(uint8_t param)
{
    uint8_t val;
    if (!_init)
        return AXP_NOT_INIT;
    if (param > sizeof(startupParams) / sizeof(startupParams[0]))
        return AXP_INVALID;
    _readByte(AXP202_POK_SET, 1, &val);
    val &= (~0b11000000);
    val |= startupParams[param];
    _writeByte(AXP202_POK_SET, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::setlongPressTime(uint8_t param)
{
    uint8_t val;
    if (!_init)
        return AXP_NOT_INIT;
    if (param > sizeof(longPressParams) / sizeof(longPressParams[0]))
        return AXP_INVALID;
    _readByte(AXP202_POK_SET, 1, &val);
    val &= (~0b00110000);
    val |= longPressParams[param];
    _writeByte(AXP202_POK_SET, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::setShutdownTime(uint8_t param)
{
    uint8_t val;
    if (!_init)
        return AXP_NOT_INIT;
    if (param > sizeof(shutdownParams) / sizeof(shutdownParams[0]))
        return AXP_INVALID;
    _readByte(AXP202_POK_SET, 1, &val);
    val &= (~0b00000011);
    val |= shutdownParams[param];
    _writeByte(AXP202_POK_SET, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::setTimeOutShutdown(bool en)
{
    uint8_t val;
    if (!_init)
        return AXP_NOT_INIT;
    _readByte(AXP202_POK_SET, 1, &val);
    if (en)
        val |= (1 << 3);
    else
        val &= (~(1 << 3));
    _writeByte(AXP202_POK_SET, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::shutdown(void)
{
    uint8_t val;
    if (!_init)
        return AXP_NOT_INIT;
    _readByte(AXP202_OFF_CTL, 1, &val);
    val |= (1 << 7);
    _writeByte(AXP202_OFF_CTL, 1, &val);
    return AXP_PASS;
}

float AXP20X_Class::getSettingChargeCurrent(void)
{
    uint8_t val;
    if (!_init)
        return AXP_NOT_INIT;
    _readByte(AXP202_CHARGE1, 1, &val);
    val &= 0b00000111;
    float cur = 300.0 + val * 100.0;
    AXP_DEBUG("Setting Charge current : %.2f mA\n", cur);
    return cur;
}

bool  AXP20X_Class::isChargeingEnable(void)
{
    return isChargingEnable();
}

bool AXP20X_Class::isChargingEnable(void)
{
    uint8_t val;
    if (!_init)
        return false;
    _readByte(AXP202_CHARGE1, 1, &val);
    if (val & (1 << 7)) {
        AXP_DEBUG("Charging enable is enable\n");
        val = true;
    } else {
        AXP_DEBUG("Charging enable is disable\n");
        val = false;
    }
    return val;
}

int AXP20X_Class::enableChargeing(bool en)
{
    return enableCharging(en);
}

int AXP20X_Class::enableCharging(bool en)
{
    uint8_t val;
    if (!_init)
        return AXP_NOT_INIT;
    _readByte(AXP202_CHARGE1, 1, &val);
    val = en ? (val | _BV(7)) : val & (~_BV(7));
    _writeByte(AXP202_CHARGE1, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::getChargingTargetVoltage(axp_chargeing_vol_t &chargeing_vol)
{
    uint8_t val;
    if (!_init)
        return AXP_NOT_INIT;
    _readByte(AXP202_CHARGE1, 1, &val);
    val &= (0b11 << 5);
    chargeing_vol = (axp_chargeing_vol_t) (val >> 5);
    return AXP_PASS;
}

int AXP20X_Class::setChargingTargetVoltage(axp_chargeing_vol_t param)
{
    uint8_t val;
    if (!_init)
        return AXP_NOT_INIT;
    if (param > sizeof(targetVolParams) / sizeof(targetVolParams[0]))
        return AXP_INVALID;
    _readByte(AXP202_CHARGE1, 1, &val);
    val &= ~(0b01100000);
    val |= targetVolParams[param];
    _writeByte(AXP202_CHARGE1, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::getBattPercentage(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    if (_chip_id != AXP202_CHIP_ID)
        return AXP_NOT_SUPPORT;
    uint8_t val;
    if (!isBatteryConnect())
        return 0;
    _readByte(AXP202_BATT_PERCENTAGE, 1, &val);
    if (!(val & _BV(7))) {
        return val & (~_BV(7));
    }
    return 0;
}

int AXP20X_Class::setMeteringSystem(bool en)
{
    if (!_init)
        return AXP_NOT_INIT;
    if (_chip_id != AXP202_CHIP_ID)
        return AXP_NOT_SUPPORT;
    uint8_t val = 0;
    _readByte(AXP202_BATT_PERCENTAGE, 1, &val);
    en ? (val |= _BV(7)) : (val &= (~_BV(7)));
    return AXP_PASS;
}


int AXP20X_Class::setChgLEDMode(axp_chgled_mode_t mode)
{
    uint8_t val;
    _readByte(AXP202_OFF_CTL, 1, &val);
    val &= 0b11001111;
    val |= _BV(3);
    switch (mode) {
    case AXP20X_LED_OFF:
        _writeByte(AXP202_OFF_CTL, 1, &val);
        break;
    case AXP20X_LED_BLINK_1HZ:
        val |= 0b00010000;
        _writeByte(AXP202_OFF_CTL, 1, &val);
        break;
    case AXP20X_LED_BLINK_4HZ:
        val |= 0b00100000;
        _writeByte(AXP202_OFF_CTL, 1, &val);
        break;
    case AXP20X_LED_LOW_LEVEL:
        val |= 0b00110000;
        _writeByte(AXP202_OFF_CTL, 1, &val);
        break;
    default:
        return AXP_FAIL;
    }
    return AXP_PASS;
}

int AXP20X_Class::debugCharging(void)
{
    uint8_t val;
    _readByte(AXP202_CHARGE1, 1, &val);
    AXP_DEBUG("SRC REG:0x%x\n", val);
    if (val & (1 << 7)) {
        AXP_DEBUG("Charging enable is enable\n");
    } else {
        AXP_DEBUG("Charging enable is disable\n");
    }
    AXP_DEBUG("Charging target-voltage : 0x%x\n", ((val & 0b01100000) >> 5) & 0b11);
    if (val & (1 << 4)) {
        AXP_DEBUG("end when the charge current is lower than 15%% of the set value\n");
    } else {
        AXP_DEBUG(" end when the charge current is lower than 10%% of the set value\n");
    }
    val &= 0b00000111;
    AXP_DEBUG("Charge current : %.2f mA\n", 300.0 + val * 100.0);
    return AXP_PASS;
}

int AXP20X_Class::debugStatus(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    uint8_t val, val1, val2;
    _readByte(AXP202_STATUS, 1, &val);
    _readByte(AXP202_MODE_CHGSTATUS, 1, &val1);
    _readByte(AXP202_IPS_SET, 1, &val2);
    AXP_DEBUG("AXP202_STATUS:   AXP202_MODE_CHGSTATUS   AXP202_IPS_SET\n");
    AXP_DEBUG("0x%x\t\t\t 0x%x\t\t\t 0x%x\n", val, val1, val2);
    return AXP_PASS;
}

int AXP20X_Class::limitingOff(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    uint8_t val;
    _readByte(AXP202_IPS_SET, 1, &val);
    if (_chip_id == AXP202_CHIP_ID) {
        val |= 0x03;
    } else {
        val &= ~(1 << 1);
    }
    _writeByte(AXP202_IPS_SET, 1, &val);
    return AXP_PASS;
}

// Only AXP129 chip and AXP173
int AXP20X_Class::setDCDC1Voltage(uint16_t mv)
{
    if (!_init)
        return AXP_NOT_INIT;
    if (_chip_id != AXP192_CHIP_ID && _chip_id != AXP173_CHIP_ID)
        return AXP_FAIL;
    if (mv < 700) {
        AXP_DEBUG("DCDC1:Below settable voltage:700mV~3500mV");
        mv = 700;
    }
    if (mv > 3500) {
        AXP_DEBUG("DCDC1:Above settable voltage:700mV~3500mV");
        mv = 3500;
    }
    uint8_t val = (mv - 700) / 25;
    //! axp192 and axp173 dc1 control register same
    _writeByte(AXP192_DC1_VLOTAGE, 1, &val);
    return AXP_PASS;
}

// Only AXP129 chip and AXP173
uint16_t AXP20X_Class::getDCDC1Voltage(void)
{
    if (_chip_id != AXP192_CHIP_ID && _chip_id != AXP173_CHIP_ID)
        return AXP_FAIL;
    uint8_t val = 0;
    //! axp192 and axp173 dc1 control register same
    _readByte(AXP192_DC1_VLOTAGE, 1, &val);
    return val * 25 + 700;
}


/***********************************************
 *              !!! TIMER FUNCTION !!!
 * *********************************************/

int AXP20X_Class::setTimer(uint8_t minutes)
{
    if (!_init)
        return AXP_NOT_INIT;
    if (_chip_id == AXP202_CHIP_ID || _chip_id == AXP192_CHIP_ID) {
        if (minutes > 63) {
            return AXP_ARG_INVALID;
        }
        minutes |= 0x80;    //Clear timer flag
        _writeByte(AXP202_TIMER_CTL, 1, &minutes);
        return AXP_PASS;
    }
    return AXP_NOT_SUPPORT;
}

int AXP20X_Class::offTimer(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    if (_chip_id == AXP202_CHIP_ID || _chip_id == AXP192_CHIP_ID) {
        uint8_t minutes = 0x80;
        _writeByte(AXP202_TIMER_CTL, 1, &minutes);
        return AXP_PASS;
    }
    return AXP_NOT_SUPPORT;
}

int AXP20X_Class::clearTimerStatus(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    if (_chip_id == AXP202_CHIP_ID || _chip_id == AXP192_CHIP_ID) {
        uint8_t val;
        _readByte(AXP202_TIMER_CTL, 1, &val);
        val |= 0x80;
        _writeByte(AXP202_TIMER_CTL, 1, &val);
        return AXP_PASS;
    }
    return AXP_NOT_SUPPORT;
}

bool AXP20X_Class::getTimerStatus(void)
{
    if (!_init)
        return AXP_NOT_INIT;
    if (_chip_id == AXP202_CHIP_ID || _chip_id == AXP192_CHIP_ID) {
        uint8_t val;
        _readByte(AXP202_TIMER_CTL, 1, &val);
        return ( val & 0x80 ) >> 7;
    }
    return AXP_NOT_SUPPORT;
}

/***********************************************
 *              !!! GPIO FUNCTION !!!
 * *********************************************/

int AXP20X_Class::_axp192_gpio_0_select( axp_gpio_mode_t mode)
{
    switch (mode) {
    case AXP_IO_OUTPUT_LOW_MODE:
        return 0b101;
    case AXP_IO_INPUT_MODE:
        return 0b001;
    case AXP_IO_LDO_MODE:
        return 0b010;
    case AXP_IO_ADC_MODE:
        return 0b100;
    case AXP_IO_FLOATING_MODE:
        return 0b111;
    case AXP_IO_OPEN_DRAIN_OUTPUT_MODE:
        return 0;
    case AXP_IO_OUTPUT_HIGH_MODE:
    case AXP_IO_PWM_OUTPUT_MODE:
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int AXP20X_Class::_axp192_gpio_1_select( axp_gpio_mode_t mode)
{
    switch (mode) {
    case AXP_IO_OUTPUT_LOW_MODE:
        return 0b101;
    case AXP_IO_INPUT_MODE:
        return 0b001;
    case AXP_IO_ADC_MODE:
        return 0b100;
    case AXP_IO_FLOATING_MODE:
        return 0b111;
    case AXP_IO_OPEN_DRAIN_OUTPUT_MODE:
        return 0;
    case AXP_IO_PWM_OUTPUT_MODE:
        return 0b010;
    case AXP_IO_OUTPUT_HIGH_MODE:
    case AXP_IO_LDO_MODE:
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}


int AXP20X_Class::_axp192_gpio_3_select( axp_gpio_mode_t mode)
{
    switch (mode) {
    case AXP_IO_EXTERN_CHARGING_CTRL_MODE:
        return 0;
    case AXP_IO_OPEN_DRAIN_OUTPUT_MODE:
        return 1;
    case AXP_IO_INPUT_MODE:
        return 2;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int AXP20X_Class::_axp192_gpio_4_select( axp_gpio_mode_t mode)
{
    switch (mode) {
    case AXP_IO_EXTERN_CHARGING_CTRL_MODE:
        return 0;
    case AXP_IO_OPEN_DRAIN_OUTPUT_MODE:
        return 1;
    case AXP_IO_INPUT_MODE:
        return 2;
    case AXP_IO_ADC_MODE:
        return 3;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}


int AXP20X_Class::_axp192_gpio_set(axp_gpio_t gpio, axp_gpio_mode_t mode)
{
    int rslt;
    uint8_t val;
    switch (gpio) {
    case AXP_GPIO_0: {
        rslt = _axp192_gpio_0_select(mode);
        if (rslt < 0)return rslt;
        _readByte(AXP192_GPIO0_CTL, 1, &val);
        val &= 0xF8;
        val |= (uint8_t)rslt;
        _writeByte(AXP192_GPIO0_CTL, 1, &val);
        return AXP_PASS;
    }
    case AXP_GPIO_1: {
        rslt = _axp192_gpio_1_select(mode);
        if (rslt < 0)return rslt;
        _readByte(AXP192_GPIO1_CTL, 1, &val);
        val &= 0xF8;
        val |= (uint8_t)rslt;
        _writeByte(AXP192_GPIO1_CTL, 1, &val);
        return AXP_PASS;
    }
    case AXP_GPIO_2: {
        rslt = _axp192_gpio_1_select(mode);
        if (rslt < 0)return rslt;
        _readByte(AXP192_GPIO2_CTL, 1, &val);
        val &= 0xF8;
        val |= (uint8_t)rslt;
        _writeByte(AXP192_GPIO2_CTL, 1, &val);
        return AXP_PASS;
    }
    case AXP_GPIO_3: {
        rslt = _axp192_gpio_3_select(mode);
        if (rslt < 0)return rslt;
        _readByte(AXP192_GPIO34_CTL, 1, &val);
        val &= 0xFC;
        val |= (uint8_t)rslt;
        _writeByte(AXP192_GPIO34_CTL, 1, &val);
        return AXP_PASS;
    }
    case AXP_GPIO_4: {
        rslt = _axp192_gpio_4_select(mode);
        if (rslt < 0)return rslt;
        _readByte(AXP192_GPIO34_CTL, 1, &val);
        val &= 0xF3;
        val |= (uint8_t)rslt;
        _writeByte(AXP192_GPIO34_CTL, 1, &val);
        return AXP_PASS;
    }
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int AXP20X_Class::_axp202_gpio_0_select( axp_gpio_mode_t mode)
{
    switch (mode) {
    case AXP_IO_OUTPUT_LOW_MODE:
        return 0;
    case AXP_IO_OUTPUT_HIGH_MODE:
        return 1;
    case AXP_IO_INPUT_MODE:
        return 2;
    case AXP_IO_LDO_MODE:
        return 3;
    case AXP_IO_ADC_MODE:
        return 4;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int AXP20X_Class::_axp202_gpio_1_select( axp_gpio_mode_t mode)
{
    switch (mode) {
    case AXP_IO_OUTPUT_LOW_MODE:
        return 0;
    case AXP_IO_OUTPUT_HIGH_MODE:
        return 1;
    case AXP_IO_INPUT_MODE:
        return 2;
    case AXP_IO_ADC_MODE:
        return 4;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int AXP20X_Class::_axp202_gpio_2_select( axp_gpio_mode_t mode)
{
    switch (mode) {
    case AXP_IO_OUTPUT_LOW_MODE:
        return 0;
    case AXP_IO_INPUT_MODE:
        return 2;
    case AXP_IO_FLOATING_MODE:
        return 1;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}


int AXP20X_Class::_axp202_gpio_3_select(axp_gpio_mode_t mode)
{
    switch (mode) {
    case AXP_IO_INPUT_MODE:
        return 1;
    case AXP_IO_OPEN_DRAIN_OUTPUT_MODE:
        return 0;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int AXP20X_Class::_axp202_gpio_set(axp_gpio_t gpio, axp_gpio_mode_t mode)
{
    uint8_t val;
    int rslt;
    switch (gpio) {
    case AXP_GPIO_0: {
        rslt = _axp202_gpio_0_select(mode);
        if (rslt < 0)return rslt;
        _readByte(AXP202_GPIO0_CTL, 1, &val);
        val &= 0b11111000;
        val |= (uint8_t)rslt;
        _writeByte(AXP202_GPIO0_CTL, 1, &val);
        return AXP_PASS;
    }
    case AXP_GPIO_1: {
        rslt = _axp202_gpio_1_select(mode);
        if (rslt < 0)return rslt;
        _readByte(AXP202_GPIO1_CTL, 1, &val);
        val &= 0b11111000;
        val |= (uint8_t)rslt;
        _writeByte(AXP202_GPIO1_CTL, 1, &val);
        return AXP_PASS;
    }
    case AXP_GPIO_2: {
        rslt = _axp202_gpio_2_select(mode);
        if (rslt < 0)return rslt;
        _readByte(AXP202_GPIO2_CTL, 1, &val);
        val &= 0b11111000;
        val |= (uint8_t)rslt;
        _writeByte(AXP202_GPIO2_CTL, 1, &val);
        return AXP_PASS;
    }
    case AXP_GPIO_3: {
        rslt = _axp202_gpio_3_select(mode);
        if (rslt < 0)return rslt;
        _readByte(AXP202_GPIO3_CTL, 1, &val);
        val = rslt ? (val | _BV(2)) : (val & (~_BV(2)));
        _writeByte(AXP202_GPIO3_CTL, 1, &val);
        return AXP_PASS;
    }
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}


int AXP20X_Class::setGPIOMode(axp_gpio_t gpio, axp_gpio_mode_t mode)
{
    if (!_init)
        return AXP_NOT_INIT;
    switch (_chip_id) {
    case AXP202_CHIP_ID:
        return _axp202_gpio_set(gpio, mode);
        break;
    case AXP192_CHIP_ID:
        return _axp192_gpio_set(gpio, mode);
        break;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}


int AXP20X_Class::_axp_irq_mask(axp_gpio_irq_t irq)
{
    switch (irq) {
    case AXP_IRQ_NONE:
        return 0;
    case AXP_IRQ_RISING:
        return _BV(7);
    case AXP_IRQ_FALLING:
        return _BV(6);
    case AXP_IRQ_DOUBLE_EDGE:
        return 0b1100000;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int AXP20X_Class::_axp202_gpio_irq_set(axp_gpio_t gpio, axp_gpio_irq_t irq)
{
    uint8_t reg;
    uint8_t val;
    int mask;
    mask = _axp_irq_mask(irq);

    if (mask < 0)return mask;
    switch (gpio) {
    case AXP_GPIO_0:
        reg = AXP202_GPIO0_CTL;
        break;
    case AXP_GPIO_1:
        reg = AXP202_GPIO1_CTL;
        break;
    case AXP_GPIO_2:
        reg = AXP202_GPIO2_CTL;
        break;
    case AXP_GPIO_3:
        reg = AXP202_GPIO3_CTL;
        break;
    default:
        return AXP_NOT_SUPPORT;
    }
    _readByte(reg, 1, &val);
    val = mask == 0 ? (val & 0b00111111) : (val | mask);
    _writeByte(reg, 1, &val);
    return AXP_PASS;
}


int AXP20X_Class::setGPIOIrq(axp_gpio_t gpio, axp_gpio_irq_t irq)
{
    if (!_init)
        return AXP_NOT_INIT;
    switch (_chip_id) {
    case AXP202_CHIP_ID:
        return _axp202_gpio_irq_set(gpio, irq);
    case AXP192_CHIP_ID:
    case AXP173_CHIP_ID:
        return AXP_NOT_SUPPORT;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int AXP20X_Class::setLDO5Voltage(axp_ldo5_table_t vol)
{
    const uint8_t params[] = {
        0b11111000, //1.8V
        0b11111001, //2.5V
        0b11111010, //2.8V
        0b11111011, //3.0V
        0b11111100, //3.1V
        0b11111101, //3.3V
        0b11111110, //3.4V
        0b11111111, //3.5V
    };
    if (!_init)
        return AXP_NOT_INIT;
    if (_chip_id != AXP202_CHIP_ID)
        return AXP_NOT_SUPPORT;
    if (vol > sizeof(params) / sizeof(params[0]))
        return AXP_ARG_INVALID;
    uint8_t val = 0;
    _readByte(AXP202_GPIO0_VOL, 1, &val);
    val &= 0b11111000;
    val |= params[vol];
    _writeByte(AXP202_GPIO0_VOL, 1, &val);
    return AXP_PASS;
}


int AXP20X_Class::_axp202_gpio_write(axp_gpio_t gpio, uint8_t val)
{
    uint8_t reg;
    uint8_t wVal = 0;
    switch (gpio) {
    case AXP_GPIO_0:
        reg = AXP202_GPIO0_CTL;
        break;
    case AXP_GPIO_1:
        reg = AXP202_GPIO1_CTL;
        break;
    case AXP_GPIO_2:
        reg = AXP202_GPIO2_CTL;
        if (val) {
            return AXP_NOT_SUPPORT;
        }
        break;
    case AXP_GPIO_3:
        if (val) {
            return AXP_NOT_SUPPORT;
        }
        _readByte(AXP202_GPIO3_CTL, 1, &wVal);
        wVal &= 0b11111101;
        _writeByte(AXP202_GPIO3_CTL, 1, &wVal);
        return AXP_PASS;
    default:
        return AXP_NOT_SUPPORT;
    }
    _readByte(reg, 1, &wVal);
    wVal = val ? (wVal | 1) : (wVal & 0b11111000);
    _writeByte(reg, 1, &wVal);
    return AXP_PASS;
}

int AXP20X_Class::_axp202_gpio_read(axp_gpio_t gpio)
{
    uint8_t val;
    uint8_t reg = AXP202_GPIO012_SIGNAL;
    uint8_t offset;
    switch (gpio) {
    case AXP_GPIO_0:
        offset = 4;
        break;
    case AXP_GPIO_1:
        offset = 5;
        break;
    case AXP_GPIO_2:
        offset = 6;
        break;
    case AXP_GPIO_3:
        reg = AXP202_GPIO3_CTL;
        offset = 0;
        break;
    default:
        return AXP_NOT_SUPPORT;
    }
    _readByte(reg, 1, &val);
    return val & _BV(offset) ? 1 : 0;
}

int AXP20X_Class::gpioWrite(axp_gpio_t gpio, uint8_t val)
{
    if (!_init)
        return AXP_NOT_INIT;
    switch (_chip_id) {
    case AXP202_CHIP_ID:
        return _axp202_gpio_write(gpio, val);
    case AXP192_CHIP_ID:
    case AXP173_CHIP_ID:
        return AXP_NOT_SUPPORT;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int AXP20X_Class::gpioRead(axp_gpio_t gpio)
{
    if (!_init)
        return AXP_NOT_INIT;
    switch (_chip_id) {
    case AXP202_CHIP_ID:
        return _axp202_gpio_read(gpio);
    case AXP192_CHIP_ID:
    case AXP173_CHIP_ID:
        return AXP_NOT_SUPPORT;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int AXP20X_Class::getChargeControlCur(void)
{
    int cur;
    uint8_t val;
    if (!_init)
        return AXP_NOT_INIT;
    switch (_chip_id) {
    case AXP202_CHIP_ID:
        _readByte(AXP202_CHARGE1, 1, &val);
        val &= 0x0F;
        cur =  val * 100 + 300;
        if (cur > 1800 || cur < 300)return 0;
        return cur;
    case AXP192_CHIP_ID:
    case AXP173_CHIP_ID:
        _readByte(AXP202_CHARGE1, 1, &val);
        return val & 0x0F;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int AXP20X_Class::setChargeControlCur(uint16_t mA)
{
    uint8_t val;
    if (!_init)
        return AXP_NOT_INIT;
    switch (_chip_id) {
    case AXP202_CHIP_ID:
        _readByte(AXP202_CHARGE1, 1, &val);
        val &= 0b11110000;
        mA -= 300;
        val |= (mA / 100);
        _writeByte(AXP202_CHARGE1, 1, &val);
        return AXP_PASS;
    case AXP192_CHIP_ID:
    case AXP173_CHIP_ID:
        _readByte(AXP202_CHARGE1, 1, &val);
        val &= 0b11110000;
        if (mA > AXP1XX_CHARGE_CUR_1320MA)
            mA = AXP1XX_CHARGE_CUR_1320MA;
        val |= mA;
        _writeByte(AXP202_CHARGE1, 1, &val);
        return AXP_PASS;
    default:
        break;
    }
    return AXP_NOT_SUPPORT;
}

int AXP20X_Class::setSleep()
{
    int ret;
    uint8_t val  = 0;
    ret = _readByte(AXP202_VOFF_SET, 1, &val);
    if (ret != 0)return AXP_FAIL;
    val |= _BV(3);
    ret = _writeByte(AXP202_VOFF_SET, 1, &val);
    if (ret != 0)return AXP_FAIL;
    ret = _readByte(AXP202_VOFF_SET, 1, &val);
    if (ret != 0)return AXP_FAIL;
    return (val & _BV(3)) ? AXP_PASS : AXP_FAIL;
}

// VOFF =[2.6+(Bit2-0)*0.1]V
int AXP20X_Class::setPowerDownVoltage(uint16_t mv)
{
    int ret;
    uint8_t val  = 0;
    ret = _readByte(AXP202_VOFF_SET, 1, &val);
    if (ret != 0)return AXP_FAIL;
    val &= ~(AXP202_VOFF_MASK);
    val |= ((mv - 2600) / 100);
    ret = _writeByte(AXP202_VOFF_SET, 1, &val);
    if (ret != 0)return AXP_FAIL;
    return AXP_PASS ;
}

uint16_t AXP20X_Class::getPowerDownVoltage(void)
{
    int ret = 0;
    uint8_t val  = 0;
    ret = _readByte(AXP202_VOFF_SET, 1, &val);
    if (ret != 0)return 0;
    val &= AXP202_VOFF_MASK;
    uint16_t voff = val * 100 + 2600;
    return voff;
}

int AXP20X_Class::setCurrentLimitControl(axp202_limit_setting_t opt)
{
    uint8_t val = 0;
    if (!_init)
        return AXP_NOT_INIT;
    if (_chip_id != AXP202_CHIP_ID) {
        return AXP_NOT_SUPPORT;
    }
    _readByte(AXP202_IPS_SET, 1, &val);
    val &= (~AXP202_LIMIT_MASK);
    val  |= opt;
    _writeByte(AXP202_IPS_SET, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::setCurrentLimitControl(axp192_limit_setting_t opt)
{
    uint8_t val = 0;
    if (!_init) {
        return AXP_NOT_INIT;
    }
    if (_chip_id != AXP192_CHIP_ID) {
        return AXP_NOT_SUPPORT;
    }
    _readByte(AXP202_IPS_SET, 1, &val);

    if (opt == AXP192_VBUS_LIMIT_OFF) {
        val &= (~AXP192_LIMIT_EN_MASK);
    } else {
        val &= (~AXP192_LIMIT_MASK);
        val  |= opt;
        val |= AXP192_LIMIT_EN_MASK;
    }
    _writeByte(AXP202_IPS_SET, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::setVWarningLevel1(uint16_t mv)
{
    ISCONNECETD(AXP_NOT_INIT);
    uint8_t val = (mv - 2867) / 5.6;
    AXP_DEBUG("setVWarningLevel1:0x%x\n", val);
    _writeByte(AXP202_APS_WARNING1, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::setVWarningLevel2(uint16_t mv)
{
    ISCONNECETD(AXP_NOT_INIT);
    uint8_t val = (mv - 2867) / 5.6;
    AXP_DEBUG("setVWarningLevel2:0x%x\n", val);
    _writeByte(AXP202_APS_WARNING2, 1, &val);
    return AXP_PASS;
}

uint16_t AXP20X_Class::getVWarningLevel1(void)
{
    ISCONNECETD(0);
    uint8_t val = 0;
    _readByte(AXP202_APS_WARNING1, 1, &val);
    AXP_DEBUG("TarageVoltage:%.2f HEX:0x%x\n", 2.8672 + 0.0014 * val * 4.0, val);
    return ( 2.8672 + 0.0014 * val * 4.0) * 1000;
}

uint16_t AXP20X_Class::getVWarningLevel2(void)
{
    ISCONNECETD(0);
    uint8_t val = 0;
    _readByte(AXP202_APS_WARNING2, 1, &val);
    AXP_DEBUG("TarageVoltage:%.2f HEX:0x%x\n", 2.8672 + 0.0014 * val * 4.0, val);
    return (2.8672 + 0.0014 * val * 4.0) * 1000;
}

int AXP20X_Class::setDCDCMode(axp202_dc_mode_t opt)
{
    uint8_t val = 0;
    _readByte(AXP202_DCDC_MODESET, 1, &val);
    val &= 0xF9;
    val |= opt;
    _writeByte(AXP202_DCDC_MODESET, 1, &val);
    return AXP_PASS;
}

axp202_dc_mode_t AXP20X_Class::getDCDCMode(void)
{
    uint8_t val = 0;
    _readByte(AXP202_DCDC_MODESET, 1, &val);
    val &= 0x6;
    return val ? AXP202_DCDC_AUTO_MODE : AXP202_DCDC_PWM_MODE;
}


int AXP20X_Class::enableLDO3VRC(bool en)
{
    ISCONNECETD(AXP_NOT_INIT);
    if (_chip_id != AXP202_CHIP_ID) {
        return AXP_NOT_SUPPORT;
    }
    uint8_t val = 0;
    _readByte(AXP202_LDO3_DC2_DVM, 1, &val);
    val &= (~_BV(3));
    val |= en;
    _writeByte(AXP202_LDO3_DC2_DVM, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::enableDC2VRC(bool en)
{
    ISCONNECETD(AXP_NOT_INIT);
    uint8_t val = 0;
    _readByte(AXP202_LDO3_DC2_DVM, 1, &val);
    val &= (~_BV(2));
    val |= en;
    _writeByte(AXP202_LDO3_DC2_DVM, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::setLDO3VRC(axp202_vrc_control_t opt)
{
    ISCONNECETD(AXP_NOT_INIT);
    if (_chip_id != AXP202_CHIP_ID) {
        return AXP_NOT_SUPPORT;
    }
    uint8_t val = 0;
    _readByte(AXP202_LDO3_DC2_DVM, 1, &val);
    val &= (~_BV(1));
    val |= opt;
    _writeByte(AXP202_LDO3_DC2_DVM, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::setDC2VRC(axp202_vrc_control_t opt)
{
    ISCONNECETD(AXP_NOT_INIT);
    uint8_t val = 0;
    _readByte(AXP202_LDO3_DC2_DVM, 1, &val);
    val &= (~_BV(0));
    val |= opt;
    _writeByte(AXP202_LDO3_DC2_DVM, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::setBackupChargeControl(bool en)
{
    ISCONNECETD(AXP_NOT_INIT);
    uint8_t val = 0;
    _readByte(AXP202_BACKUP_CHG, 1, &val);
    val &= (~_BV(7));
    val |= en;
    _writeByte(AXP202_BACKUP_CHG, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::setBackupChargeVoltage(axp202_backup_voltage_t opt)
{
    ISCONNECETD(AXP_NOT_INIT);
    uint8_t val = 0;
    _readByte(AXP202_BACKUP_CHG, 1, &val);
    val &= 0x9F;
    val |= opt;
    _writeByte(AXP202_BACKUP_CHG, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::setBackupChargeCurrent(axp202_backup_current_t opt)
{
    ISCONNECETD(AXP_NOT_INIT);
    uint8_t val = 0;
    _readByte(AXP202_BACKUP_CHG, 1, &val);
    val &= 0xFC;
    val |= opt;
    _writeByte(AXP202_BACKUP_CHG, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::setPrechargeTimeout(axp202_precharge_timeout_t opt)
{
    ISCONNECETD(AXP_NOT_INIT);
    uint8_t val = 0;
    _readByte(AXP202_CHARGE2, 1, &val);
    val &= 0x3F;
    val |= opt;
    _writeByte(AXP202_CHARGE2, 1, &val);
    return AXP_PASS;
}

int AXP20X_Class::setConstantCurrentTimeout(axp202_constant_current_t opt)
{
    ISCONNECETD(AXP_NOT_INIT);
    uint8_t val = 0;
    _readByte(AXP202_CHARGE2, 1, &val);
    val &= 0xFC;
    val |= opt;
    _writeByte(AXP202_CHARGE2, 1, &val);
    return AXP_PASS;
}







// Low-level I2C communication
uint16_t AXP20X_Class::_getRegistH8L5(uint8_t regh8, uint8_t regl5)
{
    uint8_t hv, lv;
    _readByte(regh8, 1, &hv);
    _readByte(regl5, 1, &lv);
    return (hv << 5) | (lv & 0x1F);
}

uint16_t AXP20X_Class::_getRegistResult(uint8_t regh8, uint8_t regl4)
{
    uint8_t hv, lv;
    _readByte(regh8, 1, &hv);
    _readByte(regl4, 1, &lv);
    return (hv << 4) | (lv & 0x0F);
}

int AXP20X_Class::_readByte(uint8_t reg, uint8_t nbytes, uint8_t *data)
{
    if (_read_cb != nullptr) {
        return _read_cb(_address, reg, data, nbytes);
    }
#ifdef ARDUINO
    if (nbytes == 0 || !data)
        return -1;
    _i2cPort->beginTransmission(_address);
    _i2cPort->write(reg);
    if (_i2cPort->endTransmission() != 0) {
        return -1;
    }
    _i2cPort->requestFrom(_address, nbytes);
    uint8_t index = 0;
    while (_i2cPort->available())
        data[index++] = _i2cPort->read();
#endif
    return 0;
}

int AXP20X_Class::_writeByte(uint8_t reg, uint8_t nbytes, uint8_t *data)
{
    if (_write_cb != nullptr) {
        return _write_cb(_address, reg, data, nbytes);
    }
#ifdef ARDUINO
    if (nbytes == 0 || !data)
        return -1;
    _i2cPort->beginTransmission(_address);
    _i2cPort->write(reg);
    for (uint8_t i = 0; i < nbytes; i++) {
        _i2cPort->write(data[i]);
    }
    return _i2cPort->endTransmission();
#endif
    return 0;
}
