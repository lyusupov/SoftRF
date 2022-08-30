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
 * @file      XPowersAXP192.tpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-05-07
 *
 */
#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <math.h>
#endif /*ARDUINO*/
#include "XPowersCommon.tpp"
#include "REG/AXP192Constants.h"
#include "XPowersLibInterface.hpp"

typedef enum {
    MONITOR_TS_PIN      = _BV(0),
    MONITOR_APS_VOLTAGE = _BV(1),
    MONITOR_USB_CURRENT = _BV(2),
    MONITOR_USB_VOLTAGE = _BV(3),
    MONITOR_AC_CURRENT  = _BV(4),
    MONITOR_AC_VOLTAGE  = _BV(5),
    MONITOR_BAT_CURRENT = _BV(6),
    MONITOR_BAT_VOLTAGE = _BV(7),
    MONITOR_ADC_IO3     = _BV(8),
    MONITOR_ADC_IO2     = _BV(9),
    MONITOR_ADC_IO1     = _BV(10),
    MONITOR_ADC_IO0     = _BV(11),
    MONITOR_TEMPERATURE = _BV(16),
} axp192_adc_func_t;


typedef enum {
    XPOWERS_AXP192_BOOT_TIME_128MS,
    XPOWERS_AXP192_BOOT_TIME_512MS,
    XPOWERS_AXP192_BOOT_TIME_1S,
    XPOWERS_AXP192_BOOT_TIME_2S,
} axp192_boot_time_t;

typedef enum {
    //! IRQ1 REG 40H
    XPOWERS_AXP192_VBUS_VHOLD_LOW_IRQ       = _BV(1),   //VBUS is available, but lower than V HOLD, IRQ enable
    XPOWERS_AXP192_VBUS_REMOVE_IRQ          = _BV(2),   //VBUS removed, IRQ enable
    XPOWERS_AXP192_VBUS_INSERT_IRQ          = _BV(3),   //VBUS connected, IRQ enable
    XPOWERS_AXP192_VBUS_OVER_VOL_IRQ        = _BV(4),   //VBUS over-voltage, IRQ enable
    XPOWERS_AXP192_ACIN_REMOVED_IRQ         = _BV(5),   //ACIN removed, IRQ enable
    XPOWERS_AXP192_ACIN_CONNECT_IRQ         = _BV(6),   //ACIN connected, IRQ enable
    XPOWERS_AXP192_ACIN_OVER_VOL_IRQ        = _BV(7),   //ACIN over-voltage, IRQ enable

    //! IRQ2 REG 41H
    XPOWERS_AXP192_BATT_LOW_TEMP_IRQ        = _BV(8),   //Battery low-temperature, IRQ enable
    XPOWERS_AXP192_BATT_OVER_TEMP_IRQ       = _BV(9),   //Battery over-temperature, IRQ enable
    XPOWERS_AXP192_BAT_CHG_DONE_IRQ         = _BV(10),  //Charge finished, IRQ enable
    XPOWERS_AXP192_BAT_CHG_START_IRQ        = _BV(11),  //Be charging, IRQ enable
    XPOWERS_AXP192_BATT_EXIT_ACTIVATE_IRQ   = _BV(12),  //Exit battery activate mode, IRQ enable
    XPOWERS_AXP192_BATT_ACTIVATE_IRQ        = _BV(13),  //Battery activate mode, IRQ enable
    XPOWERS_AXP192_BAT_REMOVE_IRQ           = _BV(14),  //Battery removed, IRQ enable
    XPOWERS_AXP192_BAT_INSERT_IRQ           = _BV(15),  //Battery connected, IRQ enable

    //! IRQ3 REG 42H
    XPOWERS_AXP192_PKEY_LONG_IRQ            = _BV(16),  //PEK long press, IRQ enable
    XPOWERS_AXP192_PKEY_SHORT_IRQ           = _BV(17),  //PEK short press, IRQ enable
    //**Reserved and unchangeable BIT 2
    XPOWERS_AXP192_DC3_LOW_VOL_IRQ          = _BV(19),  //DC-DC3output voltage is lower than the set value, IRQ enable
    XPOWERS_AXP192_DC2_LOW_VOL_IRQ          = _BV(20),  //DC-DC2 output voltage is lower than the set value, IRQ enable
    XPOWERS_AXP192_DC1_LOW_VOL_IRQ          = _BV(21),  //DC-DC1 output voltage is lower than the set value, IRQ enable
    XPOWERS_AXP192_CHARGE_LOW_CUR_IRQ       = _BV(22),  //Charge current is lower than the set current, IRQ enable
    XPOWERS_AXP192_CHIP_TEMP_HIGH_IRQ       = _BV(23),  //XPOWERS internal over-temperature, IRQ enable

    //! IRQ4 REG 43H
    XPOWERS_AXP192_APS_LOW_VOL_LEVEL_IRQ    = _BV(24),  //APS low-voltage, IRQ enable
    //**Reserved and unchangeable BIT 1
    XPOWERS_AXP192_VBUS_SESSION_END_IRQ     = _BV(26),  //VBUS Session End IRQ enable
    XPOWERS_AXP192_VBUS_SESSION_AB_IRQ      = _BV(27),  //VBUS Session A/B IRQ enable
    XPOWERS_AXP192_VBUS_INVALID_IRQ         = _BV(28),  //VBUS invalid, IRQ enable
    XPOWERS_AXP192_VBUS_VAILD_IRQ           = _BV(29),  //VBUS valid, IRQ enable
    XPOWERS_AXP192_NOE_OFF_IRQ              = _BV(30),  //N_OE shutdown, IRQ enable
    XPOWERS_AXP192_NOE_ON_IRQ               = _BV(31),  //N_OE startup, IRQ enable

    //! IRQ5 REG 4AH
    XPOWERS_AXP192_GPIO0_EDGE_TRIGGER_IRQ   = _BV(32),  //GPIO0 input edge trigger, IRQ enable
    XPOWERS_AXP192_GPIO1_EDGE_TRIGGER_IRQ   = _BV(33),  //GPIO1input edge trigger or ADC input, IRQ enable
    XPOWERS_AXP192_GPIO2_EDGE_TRIGGER_IRQ   = _BV(34),  //GPIO2input edge trigger, IRQ enable
    //**Reserved and unchangeable BIT 3
    //**Reserved and unchangeable BIT 4
    //**Reserved and unchangeable BIT 5
    //**Reserved and unchangeable BIT 6
    XPOWERS_AXP192_TIMER_TIMEOUT_IRQ        = _BV(39),  //Timer timeout, IRQ enable

    XPOWERS_AXP192_ALL_IRQ                  = (0xFFFFFFFFFFULL)
} xpowers_axp192_irq_t;

#define XPOWERS_AXP192_LDOIO_VOLTAGE_TABLE  {1800,1900,2000,2100,2200,2300,2400,2500,2600,2700,2800,2900,3000,3100,3200,3300}

typedef enum {
    PMU_GPIO0,
    PMU_GPIO1,
    PMU_GPIO2,
    PMU_GPIO3,
    PMU_GPIO4,
    PMU_GPIO5,
    PMU_TS_PIN
} xpowers_num_t;


typedef enum {
    XPOWERS_AXP192_CHG_CUR_100MA,
    XPOWERS_AXP192_CHG_CUR_190MA,
    XPOWERS_AXP192_CHG_CUR_280MA,
    XPOWERS_AXP192_CHG_CUR_360MA,
    XPOWERS_AXP192_CHG_CUR_450MA,
    XPOWERS_AXP192_CHG_CUR_550MA,
    XPOWERS_AXP192_CHG_CUR_630MA,
    XPOWERS_AXP192_CHG_CUR_700MA,
    XPOWERS_AXP192_CHG_CUR_780MA,
    XPOWERS_AXP192_CHG_CUR_880MA,
    XPOWERS_AXP192_CHG_CUR_960MA,
    XPOWERS_AXP192_CHG_CUR_1000MA,
    XPOWERS_AXP192_CHG_CUR_1080MA,
    XPOWERS_AXP192_CHG_CUR_1160MA,
    XPOWERS_AXP192_CHG_CUR_1240MA,
    XPOWERS_AXP192_CHG_CUR_1320MA,
} xpowers_charge_current_t;

typedef enum {
    XPOWERS_AXP192_CHG_ITERM_LESS_10_PERCENT,
    XPOWERS_AXP192_CHG_ITERM_LESS_15_PERCENT,
} xpowers_chg_iterm_t;

typedef enum {
    XPOWERS_AXP192_ICC_CHG_100MA,
    XPOWERS_AXP192_ICC_CHG_190MA,
    XPOWERS_AXP192_ICC_CHG_280MA,
    XPOWERS_AXP192_ICC_CHG_360MA,
    XPOWERS_AXP192_ICC_CHG_450MA,
    XPOWERS_AXP192_ICC_CHG_550MA,
    XPOWERS_AXP192_ICC_CHG_630MA,
    XPOWERS_AXP192_ICC_CHG_700MA,
    XPOWERS_AXP192_ICC_CHG_780MA,
    XPOWERS_AXP192_ICC_CHG_880MA,
    XPOWERS_AXP192_ICC_CHG_960MA,
    XPOWERS_AXP192_ICC_CHG_1000MA,
    XPOWERS_AXP192_ICC_CHG_1080MA,
    XPOWERS_AXP192_ICC_CHG_1160MA,
    XPOWERS_AXP192_ICC_CHG_1240MA,
    XPOWERS_AXP192_ICC_CHG_1320MA,
} xpowers_icc_chg_t;

typedef enum {
    XPOWERS_AXP192_POWERON_128MS,
    XPOWERS_AXP192_POWERON_512MS,
    XPOWERS_AXP192_POWERON_1S,
    XPOWERS_AXP192_POWERON_2S,
} xpowerss_pekey_poweron_arg_t;

typedef enum {
    XPOWERS_AXP192_LONGPRESS_1000MS,
    XPOWERS_AXP192_LONGPRESS_1500MS,
    XPOWERS_AXP192_LONGPRESS_2000MS,
    XPOWERS_AXP192_LONGPRESS_2500MS,
} xpowers_pekey_long_press_t;

typedef enum {
    XPOWERS_AXP192_POWEROFF_4S,
    XPOWERS_AXP192_POWEROFF_65,
    XPOWERS_AXP192_POWEROFF_8S,
    XPOWERS_AXP192_POWEROFF_16S,
} xpowers_pekey_poweroff_arg_t;


typedef enum {
    XPOWERS_AXP192_CHG_LED_DISABLE,
    XPOWERS_AXP192_CHG_LED_FRE_1HZ,
    XPOWERS_AXP192_CHG_LED_FRE_4HZ,
    XPOWERS_AXP192_CHG_LED_LEVEL_LOW,
    XPOWERS_AXP192_CHG_LED_CTRL_CHG,    // The charging indicator is controlled by the charger
} xpowers_chgled_t;

typedef enum {
    XPOWERS_AXP192_VBUS_VOL_LIM_4V,
    XPOWERS_AXP192_VBUS_VOL_LIM_4V1,
    XPOWERS_AXP192_VBUS_VOL_LIM_4V2,
    XPOWERS_AXP192_VBUS_VOL_LIM_4V3,
    XPOWERS_AXP192_VBUS_VOL_LIM_4V4,
    XPOWERS_AXP192_VBUS_VOL_LIM_4V5,
    XPOWERS_AXP192_VBUS_VOL_LIM_4V6,
    XPOWERS_AXP192_VBUS_VOL_LIM_4V7,
} xpowers_axp192_vbus_vol_limit_t;

typedef enum {
    XPOWERS_AXP192_VBUS_CUR_LIM_500MA,
    XPOWERS_AXP192_VBUS_CUR_LIM_100MA,
} xpowers_axp192_vbus_cur_limit_t;

typedef enum {
    XPOWER_CHGLED_CTRL_CHGER,        //Controlled by PMU internal charger
    XPOWER_CHGLED_CTRL_MANUAL,       //Controlled by setChargingLedFreq
} xpowers_axp192_chgled_func_t;

typedef enum {
    XPOWERS_AXP192_CHG_VOL_4V1,
    XPOWERS_AXP192_CHG_VOL_4V15,
    XPOWERS_AXP192_CHG_VOL_4V2,
    XPOWERS_AXP192_CHG_VOL_4V36,
} xpowers_axp192_chg_vol_t;

typedef enum {
    XPOWERS_AXP192_CHG_CONS_TIMEOUT_7H,
    XPOWERS_AXP192_CHG_CONS_TIMEOUT_8H,
    XPOWERS_AXP192_CHG_CONS_TIMEOUT_9H,
    XPOWERS_AXP192_CHG_CONS_TIMEOUT_10H,
} xpowers_chg_cons_to_t;


typedef enum {
    XPOWERS_AXP192_BACKUP_BAT_VOL_3V1,
    XPOWERS_AXP192_BACKUP_BAT_VOL_3V,
    XPOWERS_AXP192_BACKUP_BAT_VOL_3V0, //!NEED FIX,DATASHEET ERROR!
    XPOWERS_AXP192_BACKUP_BAT_VOL_2V5,
} xpowers_backup_batt_vol_t;

typedef enum {
    XPOWERS_AXP192_BACKUP_BAT_CUR_50UA,
    XPOWERS_AXP192_BACKUP_BAT_CUR_100UA,
    XPOWERS_AXP192_BACKUP_BAT_CUR_200UA,
    XPOWERS_AXP192_BACKUP_BAT_CUR_400UA,
} xpowers_backup_batt_curr_t;

typedef struct {
    uint8_t mode;
} xpowers_gpio_t;


class XPowersAXP192 :
    public XPowersCommon<XPowersAXP192>, public XPowersLibInterface
{
    friend class XPowersCommon<XPowersAXP192>;
public:


#if defined(ARDUINO)
    XPowersAXP192(TwoWire &w, int sda = SDA, int scl = SCL, uint8_t addr = AXP192_SLAVE_ADDRESS)
    {
        __wire = &w;
        __sda = sda;
        __scl = scl;
        __addr = addr;
    }

    XPowersAXP192()
    {
        __wire = &Wire;
        __sda = SDA;
        __scl = SCL;
        __addr = AXP192_SLAVE_ADDRESS;
    }
#endif


    bool isAcinVbusStart()
    {
        return getRegisterBit(XPOWERS_AXP192_STATUS, 0);
    }

    bool isDischarge()
    {
        return !getRegisterBit(XPOWERS_AXP192_STATUS, 2);
    }

    bool isVbusIn(void)
    {
        return getRegisterBit(XPOWERS_AXP192_STATUS, 5);
    }

    bool isAcinEfficient()
    {
        return getRegisterBit(XPOWERS_AXP192_STATUS, 6);
    }

    bool isAcinIn()
    {
        return getRegisterBit(XPOWERS_AXP192_STATUS, 7);
    }

    bool isOverTemperature()
    {
        return getRegisterBit(XPOWERS_AXP192_MODE_CHGSTATUS, 7);
    }

    bool isCharging(void)
    {
        return getRegisterBit(XPOWERS_AXP192_MODE_CHGSTATUS, 6);
    }

    bool isBatteryConnect(void)
    {
        return getRegisterBit(XPOWERS_AXP192_MODE_CHGSTATUS, 5);
    }

    bool isBattInActiveMode()
    {
        return getRegisterBit(XPOWERS_AXP192_MODE_CHGSTATUS, 3);
    }

    bool isChargeCurrLessPreset()
    {
        return getRegisterBit(XPOWERS_AXP192_MODE_CHGSTATUS, 2);
    }

    void enableVbusVoltageLimit()
    {
        setRegisterBit(XPOWERS_AXP192_IPS_SET, 6);
    }

    void disableVbusVoltageLimit()
    {
        clrRegisterBit(XPOWERS_AXP192_IPS_SET, 6);
    }

    void setVbusVoltageLimit(xpowers_axp192_vbus_vol_limit_t opt)
    {
        int val = readRegister(XPOWERS_AXP192_IPS_SET);
        if (val == -1)return;
        val &= 0xC7;
        writeRegister(XPOWERS_AXP192_IPS_SET, val | (opt << 3));
    }

    void enableVbusCurrLimit()
    {
        setRegisterBit(XPOWERS_AXP192_IPS_SET, 1);
    }

    void disableVbusCurrLimit()
    {
        clrRegisterBit(XPOWERS_AXP192_IPS_SET, 1);
    }

    void setVbusCurrentLimit(xpowers_axp192_vbus_cur_limit_t opt)
    {
        int val = readRegister(XPOWERS_AXP192_IPS_SET);
        if (val == -1)return;
        switch (opt) {
        case XPOWERS_AXP192_VBUS_CUR_LIM_500MA:
            clrRegisterBit(XPOWERS_AXP192_IPS_SET, 0);
            break;
        case XPOWERS_AXP192_VBUS_CUR_LIM_100MA:
            setRegisterBit(XPOWERS_AXP192_IPS_SET, 0);
            break;
        default:
            break;
        }
    }

    // System shutdown voltage range: 2600~3300mV
    void setMinSystemVoltage(uint16_t millivolt)
    {
        millivolt =  constrain(millivolt, XPOWER_VOFF_VOL_MIN, XPOWER_VOFF_VOL_MAX);
        int val = readRegister(XPOWERS_AXP192_VOFF_SET);
        if (val == -1)return;
        val &= 0xF8;
        val |= (millivolt - XPOWER_VOFF_VOL_MIN) / XPOWER_VOFF_VOL_STEP;
        writeRegister(XPOWERS_AXP192_VOFF_SET, val);
    }

    uint16_t getMinSystemVoltage()
    {
        int val = readRegister(XPOWERS_AXP192_VOFF_SET);
        if (val == -1)return 0;
        val &= 0x07;
        return (val * XPOWER_VOFF_VOL_STEP) + XPOWER_VOFF_VOL_MIN;
    }



    void shutdown()
    {
        setRegisterBit(XPOWERS_AXP192_OFF_CTL, 7);
    }


    /*
    * Charge setting
    */
    void enableCharge()
    {
        setRegisterBit(XPOWERS_AXP192_CHARGE1, 7);
    }

    void disableCharge()
    {
        clrRegisterBit(XPOWERS_AXP192_CHARGE1, 7);
    }

    bool setChargerVoltageLimit(xpowers_axp192_chg_vol_t opt)
    {
        int val = readRegister(XPOWERS_AXP192_CHARGE1);
        if (val == -1) return false;
        val &= 0x9F;
        return 0 == writeRegister(XPOWERS_AXP192_CHARGE1, val | (opt << 5));
    }

    uint8_t getChargerVoltageLimit()
    {
        int val = readRegister(XPOWERS_AXP192_CHARGE1);
        if (val == -1) return 0;
        return (val & 0x60) >> 5;
    }

    void setChargeCurrent(xpowers_charge_current_t opt)
    {
        int val = readRegister(XPOWERS_AXP192_CHARGE1);
        if (val == -1) {
            log_e("setChargeCurrent failed");
        }
        val &= 0xF0;
        writeRegister(XPOWERS_AXP192_CHARGE1, val | opt);
    }

    xpowers_charge_current_t getChargeCurrent(void)
    {
        int val = readRegister(XPOWERS_AXP192_CHARGE1) & 0x0F;
        if (val == -1)return XPOWERS_AXP192_CHG_CUR_780MA;
        return (xpowers_charge_current_t)val;
    }

    void setChargerTerminationCurr(xpowers_chg_iterm_t opt)
    {
        switch (opt) {
        case XPOWERS_AXP192_CHG_ITERM_LESS_10_PERCENT:
            clrRegisterBit(XPOWERS_AXP192_CHARGE1, 0);
            break;
        case XPOWERS_AXP192_CHG_ITERM_LESS_15_PERCENT:
            setRegisterBit(XPOWERS_AXP192_CHARGE1, 0);
            break;
        default:
            break;
        }
    }


    uint8_t getChargerTerminationCurr()
    {
        return getRegisterBit(XPOWERS_AXP192_CHARGE1, 4);
    }

    void setChargerConstantCurr(xpowers_icc_chg_t opt)
    {
        int val = readRegister(XPOWERS_AXP192_CHARGE1);
        if (val == -1)return;
        val &= 0xF8;
        writeRegister(XPOWERS_AXP192_CHARGE1, val | opt);
    }

    uint8_t getChargerConstantCurr()
    {
        int val = readRegister(XPOWERS_AXP192_CHARGE1);
        if (val == -1)return 0;
        return val & 0x07;
    }
    typedef enum {
        XPOWERS_AXP192_PRECHG_TIMEOUT_30MIN,
        XPOWERS_AXP192_PRECHG_TIMEOUT_40MIN,
        XPOWERS_AXP192_PRECHG_TIMEOUT_50MIN,
        XPOWERS_AXP192_PRECHG_TIMEOUT_60MIN,
    } xpoers_prechg_to_t;

    bool setPrechargeTimeout(xpoers_prechg_to_t opt)
    {
        int val = readRegister(XPOWERS_AXP192_CHARGE2);
        if (val == -1)return false;
        val &= 0x3F;
        return 0 == writeRegister(XPOWERS_AXP192_CHARGE2, val | (opt << 6));
    }

    // External channel charge current setting,Range:300~1000mA
    bool setChargerExternChannelCurr(uint16_t milliampere)
    {
        milliampere =  constrain(milliampere, XPOWERS_AXP192_CHG_EXT_CURR_MIN, XPOWERS_AXP192_CHG_EXT_CURR_MAX);
        int val = readRegister(XPOWERS_AXP192_CHARGE2);
        if (val == -1)return false;
        val &= 0xC7;
        val |= ((milliampere - XPOWERS_AXP192_CHG_EXT_CURR_MIN ) / XPOWERS_AXP192_CHG_EXT_CURR_STEP);
        return 0 == writeRegister(XPOWERS_AXP192_CHARGE2, val);
    }

    bool enableChargerExternChannel()
    {
        return setRegisterBit(XPOWERS_AXP192_CHARGE2, 2);
    }

    bool disableChargerExternChannel()
    {
        return clrRegisterBit(XPOWERS_AXP192_CHARGE2, 2);
    }

    // Timeout setting in constant current mode
    bool setChargerConstantTimeout(xpowers_chg_cons_to_t opt)
    {
        int val = readRegister(XPOWERS_AXP192_CHARGE2);
        if (val == -1)return false;
        val &= 0xFC;
        return 0 == writeRegister(XPOWERS_AXP192_CHARGE2, val | opt);
    }

    bool enableBackupBattCharger()
    {
        return setRegisterBit(XPOWERS_AXP192_BACKUP_CHG, 7);
    }

    bool disableBackupBattCharger()
    {
        return clrRegisterBit(XPOWERS_AXP192_BACKUP_CHG, 7);
    }

    bool setBackupBattChargerVoltage(xpowers_backup_batt_vol_t opt)
    {
        int val = readRegister(XPOWERS_AXP192_BACKUP_CHG);
        if (val == -1)return false;
        val &= 0x9F;
        return 0 == writeRegister(XPOWERS_AXP192_BACKUP_CHG, val | (opt << 5));
    }

    bool setBackupBattChargerCurr(xpowers_backup_batt_curr_t opt)
    {
        int val = readRegister(XPOWERS_AXP192_BACKUP_CHG);
        if (val == -1)return false;
        val &= 0xFC;
        return 0 == writeRegister(XPOWERS_AXP192_BACKUP_CHG, val | opt);
    }

    /*
    * Temperature
    */
    float getTemperature()
    {
        return readRegisterH8L4(XPOWERS_AXP192_INTERNAL_TEMP_H8, XPOWERS_AXP192_INTERNAL_TEMP_L4)
               * XPOWERS_AXP192_INTERNAL_TEMP_STEP - XPOWERS_AXP192_INERNAL_TEMP_OFFSET;
    }

    bool enableTemperatureMeasure()
    {
        return  setRegisterBit(XPOWERS_AXP192_ADC_EN2, 7);
    }

    bool disableTemperatureMeasure()
    {
        return  clrRegisterBit(XPOWERS_AXP192_ADC_EN2, 7);
    }

    /*
    * Power control LDOio functions
    */
    bool isEnableLDOio(void)
    {
        int val = readRegister(XPOWERS_AXP192_GPIO0_CTL);
        return (val & 0x02);
    }

    bool enableLDOio(void)
    {
        int val = readRegister(XPOWERS_AXP192_GPIO0_CTL) & 0xF8;
        return 0 == writeRegister(XPOWERS_AXP192_GPIO0_CTL, val | 0x02);
    }

    bool disableLDOio(void)
    {
        int val = readRegister(XPOWERS_AXP192_GPIO0_CTL) & 0xF8;
        return 0 == writeRegister(XPOWERS_AXP192_GPIO0_CTL, val);
    }

    bool setLDOioVoltage(uint16_t millivolt)
    {
        millivolt = constrain(millivolt, XPOWER_LDOIO_VOL_MIN, XPOWER_LDOIO_VOL_MAX);
        int val = readRegister(XPOWERS_AXP192_GPIO0_VOL);
        if (val == -1)return false;
        val |=  (((millivolt - XPOWER_LDOIO_VOL_MIN) / XPOWER_LDOIO_VOL_STEP) << 4);
        return 0 == writeRegister(XPOWERS_AXP192_GPIO0_VOL, val);
    }

    uint16_t getLDOioVoltage(void)
    {
        int val = readRegister(XPOWERS_AXP192_GPIO0_VOL);
        if (val == -1)return 0;
        val >>= 4;
        val *= XPOWER_LDOIO_VOL_STEP;
        val += XPOWER_LDOIO_VOL_MIN;
        return val;
    }

    /*
    * Power control LDO2 functions
    */
    bool isEnableLDO2(void)
    {
        return getRegisterBit(XPOWERS_AXP192_LDO23_DC123_EXT_CTL, 2);
    }

    bool enableLDO2(void)
    {
        return setRegisterBit(XPOWERS_AXP192_LDO23_DC123_EXT_CTL, 2);
    }

    bool disableLDO2(void)
    {
        return clrRegisterBit(XPOWERS_AXP192_LDO23_DC123_EXT_CTL, 2);
    }

    bool setLDO2Voltage(uint16_t millivolt)
    {
        if (millivolt % XPOWERS_AXP192_LDO2_VOL_STEPS) {
            log_e("Mistake !  Voltage range,The steps is must %umV", XPOWERS_AXP192_LDO2_VOL_STEPS);
            return false;
        }
        millivolt =  constrain(millivolt, XPOWERS_AXP192_LDO2_VOL_MIN, XPOWERS_AXP192_LDO2_VOL_MAX);
        int val = readRegister(XPOWERS_AXP192_LDO23OUT_VOL);
        if (val == -1) return false;
        val  &= 0x0F;
        return 0 == writeRegister(XPOWERS_AXP192_LDO23OUT_VOL, val | (((millivolt - XPOWERS_AXP192_LDO2_VOL_MIN) / XPOWERS_AXP192_LDO2_VOL_STEPS) << XPOWERS_AXP192_LDO2_VOL_BIT_MASK));
    }

    uint16_t getLDO2Voltage(void)
    {
        int val = readRegister(XPOWERS_AXP192_LDO23OUT_VOL) & 0xF0;
        return (val >> XPOWERS_AXP192_LDO2_VOL_BIT_MASK) * XPOWERS_AXP192_LDO2_VOL_STEPS + XPOWERS_AXP192_LDO2_VOL_MIN;
    }

    /*
     * Power control LDO3 functions
     */
    bool isEnableLDO3(void)
    {
        return getRegisterBit(XPOWERS_AXP192_LDO23_DC123_EXT_CTL, 3);
    }

    bool enableLDO3(void)
    {
        return setRegisterBit(XPOWERS_AXP192_LDO23_DC123_EXT_CTL, 3);
    }

    bool disableLDO3(void)
    {
        return clrRegisterBit(XPOWERS_AXP192_LDO23_DC123_EXT_CTL, 3);
    }


    bool setLDO3Voltage(uint16_t millivolt)
    {
        if (millivolt % XPOWERS_AXP192_LDO3_VOL_STEPS) {
            log_e("Mistake !  Voltage range,The steps is must %umV", XPOWERS_AXP192_LDO3_VOL_STEPS);
            return false;
        }
        millivolt =  constrain(millivolt, XPOWERS_AXP192_LDO3_VOL_MIN, XPOWERS_AXP192_LDO3_VOL_MAX);
        int val = readRegister(XPOWERS_AXP192_LDO23OUT_VOL) & 0xF0;
        return 0 == writeRegister(XPOWERS_AXP192_LDO23OUT_VOL, val | ((millivolt - XPOWERS_AXP192_LDO3_VOL_MIN) / XPOWERS_AXP192_LDO3_VOL_STEPS));
    }

    uint16_t getLDO3Voltage(void)
    {
        int val = readRegister(XPOWERS_AXP192_LDO23OUT_VOL);
        if (val == -1)return 0;
        val &= 0x0F;
        return (val * XPOWERS_AXP192_LDO3_VOL_STEPS) + XPOWERS_AXP192_LDO3_VOL_MIN;
    }

    /*
     * Power control DCDC1 functions
     */
    void setDC1PwmMode(void)
    {
        int val = readRegister(XPOWERS_AXP192_DCDC_MODESET) & 0xF7;
        writeRegister(XPOWERS_AXP192_DCDC_MODESET, val | 0x08);
    }

    void setDC1AutoMode(void)
    {
        int val = readRegister(XPOWERS_AXP192_DCDC_MODESET) & 0xF7;
        writeRegister(XPOWERS_AXP192_DCDC_MODESET, val);
    }

    bool isEnableDC1(void)
    {
        return getRegisterBit(XPOWERS_AXP192_LDO23_DC123_EXT_CTL, 0);
    }

    bool enableDC1(void)
    {
        return setRegisterBit(XPOWERS_AXP192_LDO23_DC123_EXT_CTL, 0);
    }

    bool disableDC1(void)
    {
        return clrRegisterBit(XPOWERS_AXP192_LDO23_DC123_EXT_CTL, 0);
    }

    bool setDC1Voltage(uint16_t millivolt)
    {
        if (millivolt % XPOWERS_AXP192_DC1_VOL_STEPS) {
            log_e("Mistake !  Voltage range,The steps is must %umV", XPOWERS_AXP192_DC1_VOL_STEPS);
            return false;
        }
        millivolt =  constrain(millivolt, XPOWERS_AXP192_DC1_VOL_MIN, XPOWERS_AXP192_DC1_VOL_MAX);
        int val = readRegister(XPOWERS_AXP192_DC1_VLOTAGE);
        if (val == -1)return false;
        val &= 0x80;
        val |= (millivolt - XPOWERS_AXP192_DC1_VOL_MIN) / XPOWERS_AXP192_DC1_VOL_STEPS;
        return 0 == writeRegister(XPOWERS_AXP192_DC1_VLOTAGE, val);
    }

    uint16_t getDC1Voltage(void)
    {
        int val = readRegister(XPOWERS_AXP192_DC1_VLOTAGE) & 0x7F;
        return val * XPOWERS_AXP192_DC1_VOL_STEPS + XPOWERS_AXP192_DC1_VOL_MIN;
    }

    /*
     * Power control DCDC2 functions
     */
    void setDC2PwmMode(void)
    {
        int val = readRegister(XPOWERS_AXP192_DCDC_MODESET) & 0xFB;
        writeRegister(XPOWERS_AXP192_DCDC_MODESET, val | 0x04);
    }

    void setDC2AutoMode(void)
    {
        int val = readRegister(XPOWERS_AXP192_DCDC_MODESET) & 0xFB;
        writeRegister(XPOWERS_AXP192_DCDC_MODESET, val);
    }

    void enableDC2VRC(void)
    {
        int val =  readRegister(XPOWERS_AXP192_DC2_DVM);
        writeRegister(XPOWERS_AXP192_DC2_DVM, val | 0x04);
    }

    void disableDC2VRC(void)
    {
        int val =  readRegister(XPOWERS_AXP192_DC2_DVM);
        writeRegister(XPOWERS_AXP192_DC2_DVM, val & 0xFB);
    }

    bool setDC2VRC(uint8_t opts)
    {
        if (opts > 1) {
            return false;
        }
        int val =  readRegister(XPOWERS_AXP192_DC2_DVM) & 0xFE;
        writeRegister(XPOWERS_AXP192_DC2_DVM, val | opts);
    }

    bool isEanbleDC2VRC(void)
    {
        return (readRegister(XPOWERS_AXP192_DC2_DVM) & 0x04) == 0x04;
    }

    bool isEnableDC2(void)
    {
        return getRegisterBit(XPOWERS_AXP192_LDO23_DC123_EXT_CTL, 4);
    }

    bool enableDC2(void)
    {
        return setRegisterBit(XPOWERS_AXP192_LDO23_DC123_EXT_CTL, 4);
    }

    bool disableDC2(void)
    {
        return clrRegisterBit(XPOWERS_AXP192_LDO23_DC123_EXT_CTL, 4);
    }

    bool setDC2Voltage(uint16_t millivolt)
    {
        if (millivolt % XPOWERS_AXP192_DC2_VOL_STEPS) {
            log_e("Mistake !  Voltage range,The steps is must %umV", XPOWERS_AXP192_DC2_VOL_STEPS);
            return false;
        }
        millivolt =  constrain(millivolt, XPOWERS_AXP192_DC2_VOL_MIN, XPOWERS_AXP192_DC2_VOL_MAX);
        int val = readRegister(XPOWERS_AXP192_DC2OUT_VOL);
        if (val == -1)return false;
        val  &= 0x80;
        val |= (millivolt - XPOWERS_AXP192_DC2_VOL_MIN) / XPOWERS_AXP192_DC2_VOL_STEPS;
        return 0 == writeRegister(XPOWERS_AXP192_DC2OUT_VOL, val);
    }

    uint16_t getDC2Voltage(void)
    {
        int val = readRegister(XPOWERS_AXP192_DC2OUT_VOL);
        if (val == -1)return 0;
        return (val * XPOWERS_AXP192_DC2_VOL_STEPS) + XPOWERS_AXP192_DC2_VOL_MIN;
    }

    /*
     * Power control DCDC3 functions
     */
    void setDC3PwmMode(void)
    {
        int val = readRegister(XPOWERS_AXP192_DCDC_MODESET) & 0xFD;
        writeRegister(XPOWERS_AXP192_DCDC_MODESET, val | 0x02);
    }

    void setDC3AutoMode(void)
    {
        int val = readRegister(XPOWERS_AXP192_DCDC_MODESET) & 0xFD;
        writeRegister(XPOWERS_AXP192_DCDC_MODESET, val);
    }

    bool isEnableDC3(void)
    {
        return getRegisterBit(XPOWERS_AXP192_LDO23_DC123_EXT_CTL, 1);
    }

    bool enableDC3(void)
    {
        return setRegisterBit(XPOWERS_AXP192_LDO23_DC123_EXT_CTL, 1);
    }

    bool disableDC3(void)
    {
        return clrRegisterBit(XPOWERS_AXP192_LDO23_DC123_EXT_CTL, 1);
    }

    bool setDC3Voltage(uint16_t millivolt)
    {
        if (millivolt % XPOWERS_AXP192_DC3_VOL_STEPS) {
            log_e("Mistake !  Voltage range,The steps is must %umV", XPOWERS_AXP192_DC3_VOL_STEPS);
            return false;
        }
        millivolt =  constrain(millivolt, XPOWERS_AXP192_DC3_VOL_MIN, XPOWERS_AXP192_DC3_VOL_MAX);
        return 0 == writeRegister(XPOWERS_AXP192_DC3OUT_VOL, (millivolt - XPOWERS_AXP192_DC3_VOL_MIN) / XPOWERS_AXP192_DC3_VOL_STEPS);
    }

    uint16_t getDC3Voltage(void)
    {
        int val = readRegister(XPOWERS_AXP192_DC3OUT_VOL);
        if (val == -1)return 0;
        return (val * XPOWERS_AXP192_DC3_VOL_STEPS) + XPOWERS_AXP192_DC3_VOL_MIN;
    }

    /*
     * Power control EXTEN functions
     */
    bool enableExternalPin(void)
    {
        return setRegisterBit(XPOWERS_AXP192_LDO23_DC123_EXT_CTL, 6);
    }

    bool disableExternalPin(void)
    {
        return clrRegisterBit(XPOWERS_AXP192_LDO23_DC123_EXT_CTL, 6);
    }

    bool isEnableExternalPin(void)
    {
        return getRegisterBit(XPOWERS_AXP192_LDO23_DC123_EXT_CTL, 6);
    }

    /*
    * Interrupt status functions
    */
    uint64_t getIrqStatus(void)
    {
        statusRegister[0] = readRegister(XPOWERS_AXP192_INTSTS1);
        statusRegister[1] = readRegister(XPOWERS_AXP192_INTSTS2);
        statusRegister[2] = readRegister(XPOWERS_AXP192_INTSTS3);
        statusRegister[3] = readRegister(XPOWERS_AXP192_INTSTS4);
        statusRegister[4] = readRegister(XPOWERS_AXP192_INTSTS5);
        return ((uint64_t)statusRegister[4]) << 32 |
               ((uint64_t)statusRegister[3]) << 24 |
               ((uint64_t)statusRegister[2]) << 16 |
               ((uint64_t)statusRegister[1]) << 8  |
               ((uint64_t)statusRegister[0]);
    }

    void clearIrqStatus(void)
    {
        for (int i = 0; i < 4; i++) {
            writeRegister(XPOWERS_AXP192_INTSTS1 + i, 0xFF);
        }
        writeRegister(XPOWERS_AXP192_INTSTS5, 0xFF);
    }

    bool enableIRQ(uint64_t opt)
    {
        return setInterrupt(opt, true);
    }

    bool disableIRQ(uint64_t opt)
    {
        return setInterrupt(opt, false);
    }

    bool isAcinOverVoltageIrq(void)
    {
        return (bool)(statusRegister[0] & _BV(7));
    }

    bool isAcinInserIrq(void)
    {
        return (bool)(statusRegister[0] & _BV(6));
    }

    bool isAcinRemoveIrq(void)
    {
        return (bool)(statusRegister[0] & _BV(5));
    }

    bool isVbusOverVoltageIrq(void)
    {
        return (bool)(statusRegister[0] & _BV(4));
    }

    bool isVbusInsertIrq(void)
    {
        return (bool)(statusRegister[0] & _BV(3));
    }

    bool isVbusRemoveIrq(void)
    {
        return (bool)(statusRegister[0] & _BV(2));
    }

    bool isVbusLowVholdIrq(void)
    {
        return (bool)(statusRegister[0] & _BV(1));
    }

    bool isBatInsertIrq(void)
    {
        return (bool)(statusRegister[1] & _BV(7));
    }

    bool isBatRemoveIrq(void)
    {
        return (bool)(statusRegister[1] & _BV(6));
    }

    bool isBattEnterActivateIrq(void)
    {
        return (bool)(statusRegister[1] & _BV(5));
    }

    bool isBattExitActivateIrq(void)
    {
        return (bool)(statusRegister[1] & _BV(4));
    }

    bool isBatChagerStartIrq(void)
    {
        return (bool)(statusRegister[1] & _BV(3));
    }

    bool isBatChagerDoneIrq(void)
    {
        return (bool)(statusRegister[1] & _BV(2));
    }

    bool isBattTempHighIrq(void)
    {
        return (bool)(statusRegister[1] & _BV(1));
    }

    bool isBattTempLowIrq(void)
    {
        return (bool)(statusRegister[1] & _BV(0));
    }

    bool isChipOverTemperatureIrq(void)
    {
        return (bool)(statusRegister[2] & _BV(7));
    }

    bool isChargingCurrentLessIrq(void)
    {
        return (bool)(statusRegister[2] & _BV(6));
    }

    bool isDC1VoltageLessIrq(void)
    {
        return (bool)(statusRegister[2] & _BV(5));
    }

    bool isDC2VoltageLessIrq(void)
    {
        return (bool)(statusRegister[2] & _BV(4));
    }

    bool isDC3VoltageLessIrq(void)
    {
        return (bool)(statusRegister[2] & _BV(3));
    }

    bool isPekeyShortPressIrq(void)
    {
        return (bool)(statusRegister[2] & _BV(1));
    }

    bool isPekeyLongPressIrq(void)
    {
        return (bool)(statusRegister[2] & _BV(0));
    }

    bool isNOEPowerOnIrq(void)
    {
        return (bool)(statusRegister[3] & _BV(7));
    }

    bool isNOEPowerDownIrq(void)
    {
        return (bool)(statusRegister[3] & _BV(6));
    }

    bool isVbusEffectiveIrq(void)
    {
        return (bool)(statusRegister[3] & _BV(5));
    }

    bool isVbusInvalidIrq(void)
    {
        return (bool)(statusRegister[3] & _BV(4));
    }

    bool isVbusSessionIrq(void)
    {
        return (bool)(statusRegister[3] & _BV(3));
    }

    bool isVbusSessionEndIrq(void)
    {
        return (bool)(statusRegister[3] & _BV(2));
    }

    bool isLowVoltageLevel2Irq(void)
    {
        return (bool)(statusRegister[3] & _BV(0));
    }

    //IRQ5 REGISTER :
    bool isWdtExpireIrq(void)
    {
        return (bool)(statusRegister[4] & _BV(7));
    }

    bool isGpio2EdgeTriggerIrq(void)
    {
        return (bool)(statusRegister[4] & _BV(2));
    }

    bool isGpio1EdgeTriggerIrq(void)
    {
        return (bool)(statusRegister[4] & _BV(1));
    }

    bool isGpio0EdgeTriggerIrq(void)
    {
        return (bool)(statusRegister[4] & _BV(0));
    }

    /*
     *   ADC Functions
     */

    bool enableBattDetection()
    {
        return setRegisterBit(XPOWERS_AXP192_OFF_CTL, 6);
    }

    bool disableBattDetection()
    {
        return clrRegisterBit(XPOWERS_AXP192_OFF_CTL, 6);
    }

    bool enableVbusVoltageMeasure()
    {
        return setSignalCaptureImpl(MONITOR_USB_CURRENT | MONITOR_USB_VOLTAGE, true);
    }

    bool disableVbusVoltageMeasure()
    {
        return setSignalCaptureImpl(MONITOR_USB_CURRENT | MONITOR_USB_VOLTAGE, false);
    }

    bool enableBattVoltageMeasure()
    {
        return setSignalCaptureImpl(MONITOR_BAT_CURRENT | MONITOR_BAT_VOLTAGE, true);
    }

    bool disableBattVoltageMeasure()
    {
        return setSignalCaptureImpl(MONITOR_BAT_CURRENT | MONITOR_BAT_VOLTAGE, false);
    }

    bool enableSystemVoltageMeasure()
    {
        return setSignalCaptureImpl(MONITOR_APS_VOLTAGE, true);
    }

    bool disableSystemVoltageMeasure()
    {
        return setSignalCaptureImpl(MONITOR_APS_VOLTAGE, false);
    }

    bool enableTSPinMeasure()
    {
        return setSignalCaptureImpl(MONITOR_TS_PIN, true);
    }

    bool disableTSPinMeasure()
    {
        return setSignalCaptureImpl(MONITOR_TS_PIN, false);
    }

    bool enableAdcChannel(uint32_t opts)
    {
        return setSignalCaptureImpl(opts, true);
    }

    bool disableAdcChannel(uint32_t opts)
    {
        return setSignalCaptureImpl(opts, false);
    }

    uint16_t getVbusVoltage()
    {
        if (!isVbusIn()) {
            return 0;
        }
        return readRegisterH8L4(XPOWERS_AXP192_VBUS_VOL_H8,
                                XPOWERS_AXP192_VBUS_VOL_L4
                               ) * XPOWERS_AXP192_VBUS_VOLTAGE_STEP;
    }

    float getVbusCurrent()
    {
        if (!isVbusIn()) {
            return 0;
        }
        return readRegisterH8L4(XPOWERS_AXP192_VBUS_CUR_H8,
                                XPOWERS_AXP192_VBUS_CUR_L4
                               ) * XPOWERS_AXP192_VBUS_CUR_STEP;
    }

    uint16_t getBattVoltage()
    {
        if (!isBatteryConnect()) {
            return 0;
        }
        return readRegisterH8L4(XPOWERS_AXP192_BAT_AVERVOL_H8,
                                XPOWERS_AXP192_BAT_AVERVOL_L4
                               ) * XPOWERS_AXP192_BATT_VOLTAGE_STEP;
    }

    float getBattDischargeCurrent()
    {
        if (!isBatteryConnect()) {
            return 0;
        }
        return readRegisterH8L5(XPOWERS_AXP192_BAT_AVERDISCHGCUR_H8,
                                XPOWERS_AXP192_BAT_AVERDISCHGCUR_L5) * XPOWERS_AXP192_BATT_DISCHARGE_CUR_STEP;
    }

    uint16_t getAcinVoltage()
    {
        if (!isAcinIn()) {
            return 0;
        }
        return readRegisterH8L4(XPOWERS_AXP192_ACIN_VOL_H8, XPOWERS_AXP192_ACIN_VOL_L4) * XPOWERS_AXP192_ACIN_VOLTAGE_STEP;
    }

    float getAcinCurrent()
    {
        if (!isAcinIn()) {
            return 0;
        }
        return readRegisterH8L4(XPOWERS_AXP192_ACIN_CUR_H8, XPOWERS_AXP192_ACIN_CUR_L4) * XPOWERS_AXP192_ACIN_CUR_STEP;
    }

    uint16_t getSystemVoltage()
    {
        return readRegisterH8L4(XPOWERS_AXP192_APS_AVERVOL_H8, XPOWERS_AXP192_APS_AVERVOL_L4) * XPOWERS_AXP192_APS_VOLTAGE_STEP;
    }

    /*
    * Timer Control
    */
    void setTimerout(uint8_t minute)
    {
        writeRegister(XPOWERS_AXP192_TIMER_CTL, 0x80 | minute);
    }

    void disableTimer()
    {
        writeRegister(XPOWERS_AXP192_TIMER_CTL, 0x80);
    }

    void clearTimerFlag()
    {
        setRegisterBit(XPOWERS_AXP192_TIMER_CTL, 7);
    }

    /*
    * Data Buffer
    */
    bool writeDataBuffer(uint8_t *data, uint8_t size)
    {
        if (size > XPOWERS_AXP192_DATA_BUFFER_SIZE)return false;
        for (int i = 0; i < size; ++i) {
            writeRegister(XPOWERS_AXP192_DATA_BUFFER1 + i, data[i]);
        }
        return true;
    }

    bool readDataBuffer(uint8_t *data, uint8_t size)
    {
        if (size > XPOWERS_AXP192_DATA_BUFFER_SIZE)return false;
        for (int i = 0; i < size; ++i) {
            data[i] = readRegister(XPOWERS_AXP192_DATA_BUFFER1 + i);
        }
        return true;
    }

    /*
     * Charge led functions
     */
    void setChargerLedFunction(xpowers_axp192_chgled_func_t opt)
    {
        switch (opt) {
        case XPOWER_CHGLED_CTRL_CHGER:
            clrRegisterBit(XPOWERS_AXP192_OFF_CTL, 3);
            break;
        case XPOWER_CHGLED_CTRL_MANUAL:
            setRegisterBit(XPOWERS_AXP192_OFF_CTL, 3);
            break;
        default:
            break;
        }
    }

    void setChargingLedFreq(xpowers_chgled_t mode)
    {
        int val = readRegister(XPOWERS_AXP192_OFF_CTL);
        if (val == -1)return;
        writeRegister(XPOWERS_AXP192_OFF_CTL, (val & 0xCF) | (mode << 4));
    }



    /*
     * Coulomb counter control
     */
    void enableCoulomb()
    {
        setRegisterBit(XPOWERS_AXP192_COULOMB_CTL, 7);
    }

    void disableCoulomb()
    {
        clrRegisterBit(XPOWERS_AXP192_COULOMB_CTL, 7);
    }

    void stopCoulomb()
    {
        setRegisterBit(XPOWERS_AXP192_COULOMB_CTL, 6);
    }

    void clearCoulomb()
    {
        setRegisterBit(XPOWERS_AXP192_COULOMB_CTL, 5);
    }

    uint32_t getBattChargeCoulomb()
    {
        int data[4];
        data[0] = readRegister(XPOWERS_AXP192_BAT_CHGCOULOMB3);
        data[1] = readRegister(XPOWERS_AXP192_BAT_CHGCOULOMB2);
        data[2] = readRegister(XPOWERS_AXP192_BAT_CHGCOULOMB1);
        data[3] = readRegister(XPOWERS_AXP192_BAT_CHGCOULOMB0);
        for (int i = 0; i < 4; ++i) {
            if (data[i] == -1)return 0;
        }
        return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | (uint32_t)data[3];
    }

    uint32_t getBattDischargeCoulomb()
    {
        int data[4];
        data[0] = readRegister(XPOWERS_AXP192_BAT_DISCHGCOULOMB3);
        data[1] = readRegister(XPOWERS_AXP192_BAT_DISCHGCOULOMB2);
        data[2] = readRegister(XPOWERS_AXP192_BAT_DISCHGCOULOMB1);
        data[3] = readRegister(XPOWERS_AXP192_BAT_DISCHGCOULOMB0);
        for (int i = 0; i < 4; ++i) {
            if (data[i] == -1)return 0;
        }
        return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | (uint32_t)data[3];
    }

    uint8_t getAdcSamplingRate(void)
    {
        int val = readRegister(XPOWERS_AXP192_ADC_SPEED);
        if (val == -1)return 0;
        return 25 * (int)pow(2, (val & 0xC0) >> 6);
    }

    float getCoulombData(void)
    {
        uint32_t charge = getBattChargeCoulomb(), discharge = getBattDischargeCoulomb();
        uint8_t rate = getAdcSamplingRate();
        float result = 65536.0 * 0.5 * ((float)charge - (float)discharge) / 3600.0 / rate;
        return result;
    }

    /*
     * GPIO control functions
     */
    float getBatteryChargeCurrent(void)
    {
        return readRegisterH8L5(
                   XPOWERS_AXP192_BAT_AVERCHGCUR_H8,
                   XPOWERS_AXP192_BAT_AVERCHGCUR_L5
               ) * XPOWERS_AXP192_BATT_CHARGE_CUR_STEP;
    }

    uint16_t getGpio0Voltage(uint8_t opts)
    {
        return readRegisterH8L4(XPOWERS_AXP192_GPIO0_VOL_ADC_H8, XPOWERS_AXP192_GPIO0_VOL_ADC_L4) * XPOWERS_AXP192_GPIO0_STEP * 1000;
    }

    uint16_t getGpio1Voltage(uint8_t opts)
    {
        return readRegisterH8L4(XPOWERS_AXP192_GPIO1_VOL_ADC_H8, XPOWERS_AXP192_GPIO1_VOL_ADC_L4) * XPOWERS_AXP192_GPIO1_STEP * 1000;
    }

    int8_t pwmSetup(uint8_t channel, uint8_t freq, uint16_t duty)
    {
        // PWM输出频率 = 2.25MHz / (X+1) / Y1
        // PWM输出占空比 = Y2 / Y1
        switch (channel) {
        case 0:
            writeRegister(XPOWERS_AXP192_PWM1_FREQ_SET,  freq);
            writeRegister(XPOWERS_AXP192_PWM1_DUTY_SET1, duty >> 8);
            writeRegister(XPOWERS_AXP192_PWM1_DUTY_SET2, duty & 0xFF);
            break;
        case 1:
            writeRegister(XPOWERS_AXP192_PWM2_FREQ_SET,  freq);
            writeRegister(XPOWERS_AXP192_PWM2_DUTY_SET1, duty >> 8);
            writeRegister(XPOWERS_AXP192_PWM2_DUTY_SET2, duty & 0xFF);
            break;
        default:
            return -1;
            break;
        }
        return 0;
    }

    int8_t pwmEnable(uint8_t channel)
    {
        int val = 0;
        switch (channel) {
        case 0:
            val = readRegister(XPOWERS_AXP192_GPIO1_CTL) & 0xF8;
            writeRegister(XPOWERS_AXP192_GPIO1_CTL, val | 0x02);
            return 0;
        case 1:
            val = readRegister(XPOWERS_AXP192_GPIO2_CTL) & 0xF8;
            writeRegister(XPOWERS_AXP192_GPIO2_CTL, val | 0x02);
            return 0;
        default:
            break;
        }
        return -1;
    }

    int getBatteryPercent(void)
    {
        if (!isBatteryConnect()) {
            return -1;
        }
        const static int table[11] = {
            3000, 3650, 3700, 3740, 3760, 3795,
            3840, 3910, 3980, 4070, 4150
        };
        uint16_t voltage = getBattVoltage();
        if (voltage < table[0])
            return 0;
        for (int i = 0; i < 11; i++) {
            if (voltage < table[i])
                return i * 10 - (10UL * (int)(table[i] - voltage)) /
                       (int)(table[i] - table[i - 1]);;
        }
        return 100;
    }

    uint8_t getChipID(void)
    {
        return readRegister(XPOWERS_AXP192_IC_TYPE);
    }





    /*
    * GPIO setting
    */
    int8_t pinMode(uint8_t pin, uint8_t mode)
    {
        int val = 0;
        switch (pin) {
        case PMU_GPIO0:
            /*
            * 000: NMOS open-drain output
            * 001: Universal input function
            * 010: Low noise LDO
            * 011: reserved
            * 100: ADC input
            * 101: Low output
            * 11X: Floating
            * * */
            if (mode == INPUT || mode == INPUT_PULLDOWN) {
                if (gpio[pin].mode != INPUT) {
                    gpio[pin].mode = INPUT;
                }
                val = readRegister(XPOWERS_AXP192_GPIO0_CTL) & 0xF8;
                writeRegister(XPOWERS_AXP192_GPIO0_CTL, val | 0x01);
                //Set pull-down mode
                val = readRegister(XPOWERS_AXP192_GPIO012_PULLDOWN) & 0xFE;
                if (mode == INPUT_PULLDOWN ) {
                    writeRegister(XPOWERS_AXP192_GPIO012_PULLDOWN, val | 0x01);
                } else {
                    writeRegister(XPOWERS_AXP192_GPIO012_PULLDOWN, val);
                }
            }
            break;

        case PMU_GPIO1:
            /*
            * 000: NMOS open-drain output
            * 001: Universal input function
            * 010: PWM1 output, high level is VINT, not Can add less than 100K pull-down resistance
            * 011: reserved
            * 100: ADC input
            * 101: Low output
            * 11X: Floating
            * * */
            if (mode == INPUT || mode == INPUT_PULLDOWN) {
                if (gpio[pin].mode != INPUT) {
                    gpio[pin].mode = INPUT;
                }
                val = readRegister(XPOWERS_AXP192_GPIO1_CTL) & 0xF8;
                writeRegister(XPOWERS_AXP192_GPIO1_CTL, val | 0x01);

                //Set pull-down mode
                val = readRegister(XPOWERS_AXP192_GPIO012_PULLDOWN) & 0xFD;
                if (mode == INPUT_PULLDOWN ) {
                    writeRegister(XPOWERS_AXP192_GPIO012_PULLDOWN, val | 0x02);
                } else {
                    writeRegister(XPOWERS_AXP192_GPIO012_PULLDOWN, val);
                }
            }
            break;

        case PMU_GPIO2:
            /*
            * 000: NMOS open-drain output
            * 001: Universal input function
            * 010: PWM2 output, high level is VINT, not Can add less than 100K pull-down resistance
            * 011: reserved
            * 100: ADC input
            * 101: Low output
            * 11X: Floating
            * */
            if (mode == INPUT || mode == INPUT_PULLDOWN) {
                if (gpio[pin].mode != INPUT) {
                    gpio[pin].mode = INPUT;
                }
                val = readRegister(XPOWERS_AXP192_GPIO2_CTL) & 0xF8;
                writeRegister(XPOWERS_AXP192_GPIO2_CTL, val | 0x01);

                //Set pull-down mode
                val = readRegister(XPOWERS_AXP192_GPIO012_PULLDOWN) & 0xFB;
                if (mode == INPUT_PULLDOWN ) {
                    writeRegister(XPOWERS_AXP192_GPIO012_PULLDOWN, val | 0x04);
                } else {
                    writeRegister(XPOWERS_AXP192_GPIO012_PULLDOWN, val);
                }
            }
            break;

        case PMU_GPIO3:
            /*
            * 00: External charging control
            * 01: NMOS open-drain output port 3
            * 10: Universal input port 3
            * 11: ADC input
            * * */
            if (mode == INPUT) {
                if (gpio[pin].mode != INPUT) {
                    gpio[pin].mode = INPUT;
                }
                val = readRegister(XPOWERS_AXP192_GPIO34_CTL) & 0xFC;
                writeRegister(XPOWERS_AXP192_GPIO34_CTL, val | 0x82 );
            }
            break;

        case PMU_GPIO4:
            /*
            * 00: External charging control
            * 01: NMOS open-drain output port 4
            * 10: Universal input port 4
            * 11: undefined
            * * */
            if (mode == INPUT) {
                if (gpio[pin].mode != INPUT) {
                    gpio[pin].mode = INPUT;
                }
                val = readRegister(XPOWERS_AXP192_GPIO34_CTL) & 0xF3;
                writeRegister(XPOWERS_AXP192_GPIO34_CTL, val | 0x88 );
            }
            break;
        case PMU_GPIO5:
            if (mode == INPUT) {
                if (gpio[pin].mode != INPUT) {
                    gpio[pin].mode = INPUT;
                }
                val = readRegister(XPOWERS_AXP192_GPIO5_CTL) & 0xBF;
                writeRegister(XPOWERS_AXP192_GPIO5_CTL, val | 0x40);
            }
            break;
        default:
            break;
        }
        return 0;
    }

    uint8_t digitalRead(uint8_t pin)
    {
        switch (pin) {
        case PMU_GPIO0:
            return getRegisterBit(XPOWERS_AXP192_GPIO012_SIGNAL, 4);
        case PMU_GPIO1:
            return getRegisterBit(XPOWERS_AXP192_GPIO012_SIGNAL, 5);
        case PMU_GPIO2:
            return getRegisterBit(XPOWERS_AXP192_GPIO012_SIGNAL, 6);
        case PMU_GPIO3:
            return getRegisterBit(XPOWERS_AXP192_GPIO34_SIGNAL, 4);
        case PMU_GPIO4:
            return getRegisterBit(XPOWERS_AXP192_GPIO34_SIGNAL, 5);
        case PMU_GPIO5:
            return getRegisterBit(XPOWERS_AXP192_GPIO5_CTL, 4);
        default:
            break;
        }
        return 0;
    }

    void digitalWrite(uint8_t pin, uint8_t val)
    {
        int reg = 0;
        switch (pin) {
        case PMU_GPIO0:
            if (gpio[pin].mode != OUTPUT) {
                gpio[pin].mode = OUTPUT;
            }
            reg = readRegister(XPOWERS_AXP192_GPIO0_CTL) & 0xFE;
            writeRegister(XPOWERS_AXP192_GPIO0_CTL,  val ? (reg | 0x01) : reg);
            break;
        case PMU_GPIO1:
            if (gpio[pin].mode != OUTPUT) {
                gpio[pin].mode = OUTPUT;
            }
            reg = readRegister(XPOWERS_AXP192_GPIO1_CTL) & 0xFD;
            writeRegister(XPOWERS_AXP192_GPIO1_CTL,  val ? (reg | 0x01) : reg);
            break;
        case PMU_GPIO2:
            if (gpio[pin].mode != OUTPUT) {
                gpio[pin].mode = OUTPUT;
            }
            reg = readRegister(XPOWERS_AXP192_GPIO2_CTL) & 0xFB;
            writeRegister(XPOWERS_AXP192_GPIO2_CTL,  val ? (reg | 0x01) : reg);
            break;
        case PMU_GPIO3:
            if (gpio[pin].mode != OUTPUT) {
                gpio[pin].mode = OUTPUT;
                reg = readRegister(XPOWERS_AXP192_GPIO34_CTL) & 0xFC;
                writeRegister(XPOWERS_AXP192_GPIO34_CTL,   reg | 0x01);
            }
            reg = readRegister(XPOWERS_AXP192_GPIO34_SIGNAL) & 0xF7;
            writeRegister(XPOWERS_AXP192_GPIO34_SIGNAL,   val ? (val | 0x08) : reg);
            break;
        case PMU_GPIO4:
            if (gpio[pin].mode != OUTPUT) {
                gpio[pin].mode = OUTPUT;
                reg = readRegister(XPOWERS_AXP192_GPIO34_CTL) & 0xF3;
                writeRegister(XPOWERS_AXP192_GPIO34_CTL,  reg | 0x04);
            }
            reg = readRegister(XPOWERS_AXP192_GPIO34_SIGNAL) & 0xEF;
            writeRegister(XPOWERS_AXP192_GPIO34_SIGNAL,   val ? (val | 0x10) : reg);
            break;
        case PMU_GPIO5:
            if (gpio[pin].mode != OUTPUT) {
                gpio[pin].mode = OUTPUT;
                reg = readRegister(XPOWERS_AXP192_GPIO5_CTL) & 0xBF;
                writeRegister(XPOWERS_AXP192_GPIO5_CTL,  reg);
            }
            reg = readRegister(XPOWERS_AXP192_GPIO5_CTL) & 0xDF;
            writeRegister(XPOWERS_AXP192_GPIO5_CTL, val ? (reg | 0x20) : reg);
            break;
        default:
            break;
        }
    }

#if 0
    // ! Need FIX
    uint16_t analogRead(uint8_t pin)
    {
        switch (pin) {
        case PMU_GPIO0:
            if (gpio[pin].mode != ANALOG) {
                // Enable GPIO ADC Function
                uint8_t val = readRegister(XPOWERS_AXP192_GPIO0_CTL) & 0xF8;
                writeRegister(XPOWERS_AXP192_GPIO0_CTL, val | 0x04);

                //Enable ADC2 / GPIO0
                // val = readRegister(XPOWERS_AXP192_ADC_EN2) | 0x08;
                // writeRegister(XPOWERS_AXP192_ADC_EN2, val );
                setRegisterBit(XPOWERS_AXP192_ADC_EN2, 3);

                // Set adc input range 0~2.0475V
                val = readRegister(XPOWERS_AXP192_ADC_INPUTRANGE) & 0xFE;
                writeRegister(XPOWERS_AXP192_ADC_INPUTRANGE, val);
                gpio[pin].mode = ANALOG;
            }
            return readRegisterH8L4(XPOWERS_AXP192_GPIO0_VOL_ADC_H8, XPOWERS_AXP192_GPIO0_VOL_ADC_L4);

        case PMU_GPIO1:
            if (gpio[pin].mode != ANALOG) {
                // Enable GPIO ADC Function
                uint8_t val = readRegister(XPOWERS_AXP192_GPIO1_CTL) & 0xF8;
                writeRegister(XPOWERS_AXP192_GPIO1_CTL, val | 0x04);

                //Enable ADC2 / GPIO1
                // val = readRegister(XPOWERS_AXP192_ADC_EN2) | 0x04;
                // writeRegister(XPOWERS_AXP192_ADC_EN2, val );
                setRegisterBit(XPOWERS_AXP192_ADC_EN2, 2);


                // Set adc input range 0~2.0475V
                val = readRegister(XPOWERS_AXP192_ADC_INPUTRANGE) & 0xFD;
                writeRegister(XPOWERS_AXP192_ADC_INPUTRANGE, val);
                gpio[pin].mode = ANALOG;
            }
            return readRegisterH8L4(XPOWERS_AXP192_GPIO1_VOL_ADC_H8, XPOWERS_AXP192_GPIO1_VOL_ADC_L4);

        case PMU_GPIO2:
            if (gpio[pin].mode != ANALOG) {
                // Enable GPIO ADC Function
                uint8_t val = readRegister(XPOWERS_AXP192_GPIO1_CTL) & 0xF8;
                writeRegister(XPOWERS_AXP192_GPIO1_CTL, val | 0x04);
                //Enable ADC2 / GPIO1
                // val = readRegister(XPOWERS_AXP192_ADC_EN2) | 0x02;
                // writeRegister(XPOWERS_AXP192_ADC_EN2, val );
                setRegisterBit(XPOWERS_AXP192_ADC_EN2, 1);

                // Set adc input range 0~2.0475V
                val = readRegister(XPOWERS_AXP192_ADC_INPUTRANGE) & 0xFB;
                writeRegister(XPOWERS_AXP192_ADC_INPUTRANGE, val);
                gpio[pin].mode = ANALOG;
            }
            return readRegisterH8L4(XPOWERS_AXP192_GPIO2_VOL_ADC_H8, XPOWERS_AXP192_GPIO2_VOL_ADC_L4);


        case PMU_GPIO3:
            if (gpio[pin].mode != ANALOG) {
                // Enable GPIO ADC Function
                uint8_t val = readRegister(XPOWERS_AXP192_GPIO1_CTL) & 0xF8;
                writeRegister(XPOWERS_AXP192_GPIO1_CTL, val | 0x04);

                //Enable ADC2 / GPIO1
                // val = readRegister(XPOWERS_AXP192_ADC_EN2) | 0X01;
                // writeRegister(XPOWERS_AXP192_ADC_EN2, val );
                setRegisterBit(XPOWERS_AXP192_ADC_EN2, 0);

                // Set adc input range 0~2.0475V
                val = readRegister(XPOWERS_AXP192_ADC_INPUTRANGE) & 0xF7;
                writeRegister(XPOWERS_AXP192_ADC_INPUTRANGE, val);
                gpio[pin].mode = ANALOG;
            }
            return readRegisterH8L4(XPOWERS_AXP192_GPIO3_VOL_ADC_H8, XPOWERS_AXP192_GPIO3_VOL_ADC_L4);

        case PMU_TS_PIN:
            if (gpio[pin].mode != ANALOG) {
                // Enable TS PIN ADC Function
                setRegisterBit(XPOWERS_AXP192_ADC_SPEED, 2);
                // uint8_t val = readRegister(XPOWERS_AXP192_ADC_SPEED) & 0xFB;
                // writeRegister(XPOWERS_AXP192_ADC_SPEED, val | 0x04);
                gpio[pin].mode = ANALOG;
            }
            return readRegisterH8L4(XPOWERS_AXP192_TS_IN_H8, XPOWERS_AXP192_TS_IN_L4);
            break;
        default:
            break;
        }
        return 0;
    }
#endif


    /**
     * Sleep function
     */
    bool enableSleep()
    {
        return setRegisterBit(XPOWERS_AXP192_VOFF_SET, 3);
    }

    /*
     * Pekey function
     */
    void setPowerKeyPressOnTime(xpowerss_pekey_poweron_arg_t opt)
    {
        int val =  readRegister(XPOWERS_AXP192_POK_SET);
        if (val == -1)return;
        writeRegister(XPOWERS_AXP192_POK_SET, (val & 0x3F) | (opt << 6));
    }

    uint8_t getPowerKeyPressOnTime()
    {
        int val =  readRegister(XPOWERS_AXP192_POK_SET);
        if (val == -1)return 0;
        return (val & 0xC0) >> 6;
    }

    void setPowerKeyPressOffTime(xpowers_pekey_poweroff_arg_t opt)
    {
        int val =  readRegister(XPOWERS_AXP192_POK_SET);
        if (val == -1)return;
        writeRegister(XPOWERS_AXP192_POK_SET, (val & 0xFC)  | opt);
    }

    uint8_t getPowerKeyPressOffTime()
    {
        int val =  readRegister(XPOWERS_AXP192_POK_SET);
        if (val == -1)return 0;
        return (val & 0x03);
    }


    void setPowerKeyLongPressOnTime(xpowers_pekey_long_press_t opt)
    {
        int val =  readRegister(XPOWERS_AXP192_POK_SET);
        if (val == -1)return;
        writeRegister(XPOWERS_AXP192_POK_SET, (val & 0xCF) | (opt << 4));
    }

    void enablePowerKeyLongPressPowerOff()
    {
        setRegisterBit(XPOWERS_AXP192_POK_SET, 3);
    }

    void disablePowerKeyLongPressPowerOff()
    {
        clrRegisterBit(XPOWERS_AXP192_POK_SET, 3);
    }

protected:

    bool inline enablePowerOutput(uint8_t val)
    {
        return 0 == writeRegister(XPOWERS_AXP192_LDO23_DC123_EXT_CTL,
                                  val | readRegister(XPOWERS_AXP192_LDO23_DC123_EXT_CTL));
    }

    bool inline disablePowerOutput(uint8_t val)
    {
        return 0 == writeRegister(XPOWERS_AXP192_LDO23_DC123_EXT_CTL,
                                  val & readRegister(XPOWERS_AXP192_LDO23_DC123_EXT_CTL));
    }

    bool inline getPowerEnable(uint8_t val)
    {
        return (bool)(readRegister(XPOWERS_AXP192_LDO23_DC123_EXT_CTL) & val);
    }

    bool initImpl()
    {
        return getChipID() == XPOWERS_AXP192_CHIP_ID;
    }


    /*
     * Interrupt control functions
     */
    bool setInterrupt(uint64_t opts, bool enable)
    {
        int res = 0;
        int data = 0, value = 0;

        log_d("%s %s - 0x%llx\n", __func__, enable ? "ENABLE" : "DISABLE", opts);

        if (opts & 0xFF) {
            value = opts & 0xFF;
            data = readRegister(XPOWERS_AXP192_INTEN1);
            res |= writeRegister(XPOWERS_AXP192_INTEN1, enable ? (data | value) : (data & (~value)));
        }

        if (opts & 0xFF00) {
            value = opts >> 8;
            data = readRegister(XPOWERS_AXP192_INTEN2);
            res |= writeRegister(XPOWERS_AXP192_INTEN2, enable ? (data | value) : (data & (~value)));
        }

        if (opts & 0xFF0000) {
            value = opts >> 16;
            data = readRegister(XPOWERS_AXP192_INTEN3);
            res |= writeRegister(XPOWERS_AXP192_INTEN3, enable ? (data | value) : (data & (~value)));
        }

        if (opts & 0xFF000000) {
            value = opts >> 24;
            data = readRegister(XPOWERS_AXP192_INTEN4);
            res |= writeRegister(XPOWERS_AXP192_INTEN4, enable ? (data | value) : (data & (~value)));
        }

        if (opts & 0xFF00000000) {
            value = opts >> 32;
            data = readRegister(XPOWERS_AXP192_INTEN5);
            res |= writeRegister(XPOWERS_AXP192_INTEN5, enable ? (data | value) : (data & (~value)));
        }
        return res == 0;
    }

    /*
     * Signal Capture control functions
     */
    bool setSignalCaptureImpl(uint32_t opts, bool enable)
    {
        int value = 0;
        if (opts & 0xFF) {
            value = readRegister(XPOWERS_AXP192_ADC_EN1);
            writeRegister(XPOWERS_AXP192_ADC_EN1, enable ? (value | opts) : (value & (~opts)));
        }
        if (opts & 0xFF00) {
            opts >>= 8;
            value = readRegister(XPOWERS_AXP192_ADC_EN2);
            writeRegister(XPOWERS_AXP192_ADC_EN2, enable ? (value | opts) : (value & (~opts)));
        }
        return true;
    }

    const char  *getChipNameImpl(void)
    {
        return "AXP192";
    }

private:
    uint8_t statusRegister[5];
    xpowers_gpio_t gpio[6];
};



