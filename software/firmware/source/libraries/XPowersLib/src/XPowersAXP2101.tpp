/**
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
 * @file      XPowersAXP2101.tpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-05-07
 *
 */

#include <Arduino.h>
#include "XPowersCommon.tpp"
#include "REG/AXP2101Constants.h"


typedef enum {
    //! IRQ1 REG 40H
    XPOWERS_BAT_NOR_UNDER_TEMP_IRQ   = _BV(0),   // Battery Under Temperature in Work
    XPOWERS_BAT_NOR_OVER_TEMP_IRQ    = _BV(1),   // Battery Over Temperature in Work mode
    XPOWERS_BAT_CHG_UNDER_TEMP_IRQ   = _BV(2),   // Battery Under Temperature in Charge mode IRQ(bcut_irq)
    XPOWERS_BAT_CHG_OVER_TEMP_IRQ    = _BV(3),   // Battery Over Temperature in Charge mode IRQ(bcot_irq) enable
    XPOWERS_GAUGE_NEW_SOC_IRQ        = _BV(4),   // Gauge New SOC IRQ(lowsoc_irq) enable ???
    XPOWERS_WDT_TIMEOUT_IRQ          = _BV(5),   // Gauge Watchdog Timeout IRQ(gwdt_irq) enable
    XPOWERS_WARNING_LEVEL1_IRQ       = _BV(6),   // SOC drop to Warning Level1 IRQ(socwl1_irq) enable
    XPOWERS_WARNING_LEVEL2_IRQ       = _BV(7),   // SOC drop to Warning Level2 IRQ(socwl2_irq) enable

    //! IRQ2 REG 41H
    XPOWERS_PKEY_POSITIVE_IRQ        = _BV(8),   // POWERON Positive Edge IRQ(ponpe_irq_en) enable
    XPOWERS_PKEY_NEGATIVE_IRQ        = _BV(9),   // POWERON Negative Edge IRQ(ponne_irq_en) enable
    XPOWERS_PKEY_LONG_IRQ            = _BV(10),  // POWERON Long PRESS IRQ(ponlp_irq) enable
    XPOWERS_PKEY_SHORT_IRQ           = _BV(11),  // POWERON Short PRESS IRQ(ponsp_irq_en) enable
    XPOWERS_BAT_REMOVE_IRQ           = _BV(12),  // Battery Remove IRQ(bremove_irq) enable
    XPOWERS_BAT_INSERT_IRQ           = _BV(13),  // Battery Insert IRQ(binsert_irq) enabl
    XPOWERS_VBUS_REMOVE_IRQ          = _BV(14),  // VBUS Remove IRQ(vremove_irq) enabl
    XPOWERS_VBUS_INSERT_IRQ          = _BV(15),  // VBUS Insert IRQ(vinsert_irq) enable

    //! IRQ3 REG 42H
    XPOWERS_BAT_OVER_VOL_IRQ         = _BV(16),  // Battery Over Voltage Protection IRQ(bovp_irq) enable
    XPOWERS_CHAGER_TIMER_IRQ         = _BV(17),  // Charger Safety Timer1/2 expire IRQ(chgte_irq) enable
    XPOWERS_DIE_OVER_TEMP_IRQ        = _BV(18),  // DIE Over Temperature level1 IRQ(dotl1_irq) enable
    XPOWERS_BAT_CHG_START_IRQ        = _BV(19),  // Charger start IRQ(chgst_irq) enable
    XPOWERS_BAT_CHG_DONE_IRQ         = _BV(20),  // Battery charge done IRQ(chgdn_irq) enable
    XPOWERS_BATFET_OVER_CURR_IRQ     = _BV(21),  // BATFET Over Current Protection IRQ(bocp_irq) enable
    XPOWERS_LDO_OVER_CURR_IRQ        = _BV(22),  // LDO Over Current IRQ(ldooc_irq) enable
    XPOWERS_WDT_EXPIRE_IRQ           = _BV(23),  // Watchdog Expire IRQ(wdexp_irq) enable

    XPOWERS_ALL_IRQ                  = (0xFFFFFFFFUL)

} xpowers_irq_t;


typedef enum {
    XPOWERS_IRQ_TIME_1S,
    XPOWERS_IRQ_TIME_1S5,
    XPOWERS_IRQ_TIME_2S,
    XPOWERS_PRESSOFF_2S5,
} xpowers_irq_time_t;

typedef enum {
    XPOWERS_POWEROFF_4S,
    XPOWERS_POWEROFF_6S,
    XPOWERS_POWEROFF_8S,
    XPOWERS_POWEROFF_10S,
} xpowers_press_off_time_t;

typedef enum {
    XPOWERS_POWERON_128MS,
    XPOWERS_POWERON_512MS,
    XPOWERS_POWERON_1S,
    XPOWERS_POWERON_2S,
} xpowers_press_on_time_t;


typedef enum {
    XPOWERS_CHG_LED_FRE_0HZ,
    XPOWERS_CHG_LED_FRE_1HZ,
    XPOWERS_CHG_LED_FRE_4HZ,
    XPOWERS_CHG_LED_DISABLE,
} xpowers_chgled_t;

typedef enum {
    XPOWERS_PRECHARGE_0MA,
    XPOWERS_PRECHARGE_25MA,
    XPOWERS_PRECHARGE_50MA,
    XPOWERS_PRECHARGE_75MA,
    XPOWERS_PRECHARGE_100MA,
    XPOWERS_PRECHARGE_125MA,
    XPOWERS_PRECHARGE_150MA,
    XPOWERS_PRECHARGE_175MA,
    XPOWERS_PRECHARGE_200MA,
} xpowers_prechg_t;

typedef enum {
    XPOWERS_ICC_CHG_0MA,
    XPOWERS_ICC_CHG_100MA = 4,
    XPOWERS_ICC_CHG_125MA,
    XPOWERS_ICC_CHG_150MA,
    XPOWERS_ICC_CHG_175MA,
    XPOWERS_ICC_CHG_200MA,
    XPOWERS_ICC_CHG_300MA,
    XPOWERS_ICC_CHG_400MA,
    XPOWERS_ICC_CHG_500MA,
    XPOWERS_ICC_CHG_600MA,
    XPOWERS_ICC_CHG_700MA,
    XPOWERS_ICC_CHG_800MA,
    XPOWERS_ICC_CHG_900MA,
    XPOWERS_ICC_CHG_1000MA,
} xpowers_icc_chg_t;

typedef enum {
    XPOWERS_CHG_ITERM_0MA,
    XPOWERS_CHG_ITERM_25MA,
    XPOWERS_CHG_ITERM_50MA,
    XPOWERS_CHG_ITERM_75MA,
    XPOWERS_CHG_ITERM_100MA,
    XPOWERS_CHG_ITERM_125MA,
    XPOWERS_CHG_ITERM_150MA,
    XPOWERS_CHG_ITERM_175MA,
    XPOWERS_CHG_ITERM_200MA,
} xpowers_chg_iterm_t;

typedef enum {
    XPOWERS_CHG_VOL_4V,
    XPOWERS_CHG_VOL_4V1,
    XPOWERS_CHG_VOL_4V2,
    XPOWERS_CHG_VOL_4V35,
    XPOWERS_CHG_VOL_4V4
} xpowers_chg_vol_t;

typedef enum {
    XPOWERS_THREMAL_60DEG,
    XPOWERS_THREMAL_80DEG,
    XPOWERS_THREMAL_100DEG,
    XPOWERS_THREMAL_120DEG,
} xpowers_thermal_t;

typedef enum {
    XPOWERS_CHG_TRI_STATE,   //tri_charge
    XPOWERS_CHG_PRE_STATE,   //pre_charge
    XPOWERS_CHG_CC_STATE,    //constant charge
    XPOWERS_CHG_CV_STATE,    //constant voltage
    XPOWERS_CHG_DONE_STATE,  //charge done
    XPOWERS_CHG_STOP_STATE,  //not chargin
} xpowers_chg_status_t;

typedef enum {
    XPOWERS_WAKEUP_IRQ_PIN_TO_LOW = _BV(4),
    XPOWERS_WAKEUP_PWROK_TO_LOW   = _BV(3),
    XPOWERS_WAKEUP_DC_DLO_SELECT  = _BV(2),
} xpowers_wakeup_t;

typedef enum {
    XPOWERS_FAST_DCDC1,
    XPOWERS_FAST_DCDC2,
    XPOWERS_FAST_DCDC3,
    XPOWERS_FAST_DCDC4,
    XPOWERS_FAST_DCDC5,
    XPOWERS_FAST_ALDO1,
    XPOWERS_FAST_ALDO2,
    XPOWERS_FAST_ALDO3,
    XPOWERS_FAST_ALDO4,
    XPOWERS_FAST_BLDO1,
    XPOWERS_FAST_BLDO2,
    XPOWERS_FAST_CPUSLDO,
    XPOWERS_FAST_DLDO1,
    XPOWERS_FAST_DLDO2,
} xpowers_fast_on_opt_t;


typedef enum {
    XPOWERS_SEQUENCE_LEVEL_0,
    XPOWERS_SEQUENCE_LEVEL_1,
    XPOWERS_SEQUENCE_LEVEL_2,
    XPOWERS_SEQUENCE_DISABLE,
} xpower_start_sequence_t;

typedef enum {
    XPOWERS_WDT_IRQ_TO_PIN,             //Just interrupt to pin
    XPOWERS_WDT_IRQ_AND_RSET,           //IRQ to pin and reset pmu system
    XPOWERS_WDT_IRQ_AND_RSET_PD_PWROK,  //IRQ to pin and reset pmu system,pull down pwrok
    XPOWERS_WDT_IRQ_AND_RSET_ALL_OFF,   //IRQ to pin and reset pmu system,turn off dcdc & ldo ,pull down pwrok
} xpowers_wdt_config_t;

typedef enum {
    XPOWERS_WDT_TIMEOUT_1S,
    XPOWERS_WDT_TIMEOUT_2S,
    XPOWERS_WDT_TIMEOUT_4S,
    XPOWERS_WDT_TIMEOUT_8S,
    XPOWERS_WDT_TIMEOUT_16S,
    XPOWERS_WDT_TIMEOUT_32S,
    XPOWERS_WDT_TIMEOUT_64S,
    XPOWERS_WDT_TIMEOUT_128S,
} xpowers_wdt_timeout_t;

typedef enum {
    XPOWER_CHGLED_TYPEA,        //See datasheet 6.7.5 page 27, Table 6-4 CHGLED Function Control
    XPOWER_CHGLED_TYPEB,        //See datasheet 6.7.5 page 27, Table 6-4 CHGLED Function Control
    XPOWER_CHGLED_MANUAL,       //The charging indicator is controlled by setChargingLedFreq to control the output frequency
} xpowers_chgled_func_t;

typedef enum {
    XPOWERS_VBUS_VOL_LIM_3V88,
    XPOWERS_VBUS_VOL_LIM_3V96,
    XPOWERS_VBUS_VOL_LIM_4V04,
    XPOWERS_VBUS_VOL_LIM_4V12,
    XPOWERS_VBUS_VOL_LIM_4V20,
    XPOWERS_VBUS_VOL_LIM_4V28,
    XPOWERS_VBUS_VOL_LIM_4V36,
    XPOWERS_VBUS_VOL_LIM_4V44,
    XPOWERS_VBUS_VOL_LIM_4V52,
    XPOWERS_VBUS_VOL_LIM_4V60,
    XPOWERS_VBUS_VOL_LIM_4V68,
    XPOWERS_VBUS_VOL_LIM_4V76,
    XPOWERS_VBUS_VOL_LIM_4V84,
    XPOWERS_VBUS_VOL_LIM_4V92,
    XPOWERS_VBUS_VOL_LIM_5V,
    XPOWERS_VBUS_VOL_LIM_5V08,
} xpower_vbus_vol_limit_t;

typedef enum {
    XPOWERS_VBUS_CUR_LIM_100MA,
    XPOWERS_VBUS_CUR_LIM_500MA,
    XPOWERS_VBUS_CUR_LIM_900MA,
    XPOWERS_VBUS_CUR_LIM_1000MA,
    XPOWERS_VBUS_CUR_LIM_1500MA,
    XPOWERS_VBUS_CUR_LIM_2000MA,
} xpower_vbus_cur_limit_t;

typedef enum {
    XPOWERS_VSYS_VOL_4V1,
    XPOWERS_VSYS_VOL_4V2,
    XPOWERS_VSYS_VOL_4V3,
    XPOWERS_VSYS_VOL_4V4,
    XPOWERS_VSYS_VOL_4V5,
    XPOWERS_VSYS_VOL_4V6,
    XPOWERS_VSYS_VOL_4V7,
    XPOWERS_VSYS_VOL_4V8,
} xpower_vsys_vol_t;

typedef enum {
    XPOWER_POWERON_SRC_POWERON_LOW,                     //POWERON low for on level when POWERON Mode as POWERON Source
    XPOWER_POWERON_SRC_IRQ_LOW,                         //IRQ PIN Pull-down as POWERON Source
    XPOWER_POWERON_SRC_VBUS_INSERT,                     //Vbus Insert and Good as POWERON Source
    XPOWER_POWERON_SRC_BAT_CHARGE,                      //Vbus Insert and Good as POWERON Source
    XPOWER_POWERON_SRC_BAT_INSERT,                      //Battery Insert and Good as POWERON Source
    XPOWER_POWERON_SRC_ENMODE,                          //POWERON always high when EN Mode as POWERON Source
    XPOWER_POWERON_SRC_UNKONW,                          //Unkonw
} xpower_power_on_source_t;

typedef enum {
    XPOWER_POWEROFF_SRC_PWEKEY_PULLDOWN,            //POWERON Pull down for off level when POWERON Mode as POWEROFF Source
    XPOWER_POWEROFF_SRC_SOFT_OFF,                   //Software configuration as POWEROFF Source
    XPOWER_POWEROFF_SRC_PWEKEY_LOW,                 //POWERON always low when EN Mode as POWEROFF Source
    XPOWER_POWEROFF_SRC_UNDER_VSYS,                 //Vsys Under Voltage as POWEROFF Source
    XPOWER_POWEROFF_SRC_OVER_VBUS,                  //VBUS Over Voltage as POWEROFF Source
    XPOWER_POWEROFF_SRC_UNDER_VOL,                  //DCDC Under Voltage as POWEROFF Source
    XPOWER_POWEROFF_SRC_OVER_VOL,                   //DCDC Over Voltage as POWEROFF Source
    XPOWER_POWEROFF_SRC_OVER_TEMP,                  //Die Over Temperature as POWEROFF Source
    XPOWER_POWEROFF_SRC_UNKONW,                     //Unkonw
} xpower_power_off_source_t;

typedef enum {
    XPOWER_PWROK_DELAY_8MS,
    XPOWER_PWROK_DELAY_16MS,
    XPOWER_PWROK_DELAY_32MS,
    XPOWER_PWROK_DELAY_64MS,
} xpower_pwrok_delay_t;

class XPowersAXP2101 :
    public XPowersCommon<XPowersAXP2101>
{
    friend class XPowersCommon<XPowersAXP2101>;

public:

    /*
     * PMU status functions
     */
    bool isVbusGood(void)
    {
        return  getRegisterBit(XPOWERS_STATUS1, 5);
    }

    bool getBatfetState(void)
    {
        return  getRegisterBit(XPOWERS_STATUS1, 4);
    }

    // getBatPresentState
    bool isBatteryConnect(void)
    {
        return  getRegisterBit(XPOWERS_STATUS1, 3);
    }

    bool isBatInActiveModeState(void)
    {
        return  getRegisterBit(XPOWERS_STATUS1, 3);
    }

    bool getThermalRegulationStatus(void)
    {
        return  getRegisterBit(XPOWERS_STATUS1, 2);
    }

    bool getCurrnetLimitStatus(void)
    {
        return getRegisterBit(XPOWERS_STATUS1, 1);
    }

    bool isCharging(void)
    {
        return (readRegister(XPOWERS_STATUS2) >> 5) == 0x01;
    }

    bool isDischarge(void)
    {
        return (readRegister(XPOWERS_STATUS2) >> 5) == 0x02;
    }

    bool isStandby(void)
    {
        return (readRegister(XPOWERS_STATUS2) >> 5) == 0x00;
    }

    bool isPowerOn(void)
    {
        return getRegisterBit(XPOWERS_STATUS2, 4);
    }

    bool isPowerOff(void)
    {
        return getRegisterBit(XPOWERS_STATUS2, 4);
    }

    bool isVbusIn(void)
    {
        return getRegisterBit(XPOWERS_STATUS2, 3) == 0;
    }

    xpowers_chg_status_t getChargerStatus(void)
    {
        uint8_t val = readRegister(XPOWERS_STATUS2) & 0x07;
        return (xpowers_chg_status_t)val;
    }

    /*
     * Data Buffer
     */

    bool writeDataBuffer(uint8_t *data, uint8_t size)
    {
        if (size > XPOWERS_DATA_BUFFER_SIZE)return false;
        return writeRegister(XPOWERS_DATA_BUFFER1, data, size);
    }

    bool readDataBuffer(uint8_t *data, uint8_t size)
    {
        if (size > XPOWERS_DATA_BUFFER_SIZE)return false;
        return readRegister(XPOWERS_DATA_BUFFER1, data, size);
    }

    /*
     * PMU common configuration
     */

    /**
     * @brief   Internal off-discharge enable for DCDC & LDO & SWITCH
     */

    void enableInternalDischarge(void)
    {
        setRegisterBit(XPOWERS_COMMON_CONFIG, 5);
    }

    void disableInternalDischarge(void)
    {
        clrRegisterBit(XPOWERS_COMMON_CONFIG, 5);
    }


    /**
     * @brief   PWROK PIN pull low to Restart
     */
    void enablePwrOkPinPullLow(void)
    {
        setRegisterBit(XPOWERS_COMMON_CONFIG, 3);
    }

    void disablePwrOkPinPullLow(void)
    {
        clrRegisterBit(XPOWERS_COMMON_CONFIG, 3);
    }

    void enablePwronShutPMIC(void)
    {
        setRegisterBit(XPOWERS_COMMON_CONFIG, 2);
    }

    void disablePwronShutPMIC(void)
    {
        clrRegisterBit(XPOWERS_COMMON_CONFIG, 2);
    }

    void resetPmuSocSystem(void)
    {
        setRegisterBit(XPOWERS_COMMON_CONFIG, 1);
    }

    void softPowerOff(void)
    {
        setRegisterBit(XPOWERS_COMMON_CONFIG, 0);
    }

    /**
     * @brief  BATFET control / REG 12H
     * @note   DIE Over Temperature Protection Level1 Configuration
     * @param  opt: 0:115 , 1:125 , 2:135
     * @retval None
     */
    void setBatfetDieOverTempLevel1(uint8_t opt)
    {
        uint8_t val = (readRegister(XPOWERS_BATFET_CTRL) & 0xF9);
        writeRegister(XPOWERS_BATFET_CTRL, val | (opt << 1));
    }

    uint8_t getBatfetDieOverTempLevel1(void)
    {
        return (readRegister(XPOWERS_BATFET_CTRL) & 0x06);
    }

    void enableBatfetDieOverTempDetect(void)
    {
        setRegisterBit(XPOWERS_BATFET_CTRL, 0);
    }

    void disableBatfetDieOverTempDetect(void)
    {
        setRegisterBit(XPOWERS_BATFET_CTRL, 0);
    }

    /**
     * @param  opt: 0:115 , 1:125 , 2:135
     */
    void setDieOverTempLevel1(uint8_t opt)
    {
        uint8_t val = (readRegister(XPOWERS_DIE_TEMP_CTRL) & 0xF9);
        writeRegister(XPOWERS_DIE_TEMP_CTRL, val | (opt << 1));
    }

    uint8_t getDieOverTempLevel1(void)
    {
        return (readRegister(XPOWERS_DIE_TEMP_CTRL) & 0x06);
    }

    void enableDieOverTempDetect(void)
    {
        setRegisterBit(XPOWERS_DIE_TEMP_CTRL, 0);
    }

    void disableDieOverTempDetect(void)
    {
        setRegisterBit(XPOWERS_DIE_TEMP_CTRL, 0);
    }

    // Set the minimum system operating voltage inside the PMU,
    // below this value will shut down the PMU
    void setMinSystemVoltage(xpower_vsys_vol_t opt)
    {
        uint8_t val = (readRegister(XPOWERS_MIN_SYS_VOL_CTRL) & 0x8F);
        writeRegister(XPOWERS_MIN_SYS_VOL_CTRL, val | (opt << 4));
    }

    uint8_t getMinSystemVoltage(void)
    {
        return (readRegister(XPOWERS_MIN_SYS_VOL_CTRL) & 0x70) >> 4;
    }

    // Set the minimum common working voltage of the PMU VBUS input,
    // below this value will turn off the PMU
    void setVbusVoltageLimit(xpower_vbus_vol_limit_t opt)
    {
        int val = readRegister(XPOWERS_INPUT_VOL_LIMIT_CTRL);
        if (val == -1)return;
        val &= 0xF0;
        writeRegister(XPOWERS_INPUT_VOL_LIMIT_CTRL, val | (opt & 0x0F));
    }

    uint8_t getVbusVoltageLimit(void)
    {
        return (readRegister(XPOWERS_INPUT_VOL_LIMIT_CTRL) & 0x0F);
    }

    // Set the maximum current of the PMU VBUS input,
    // higher than this value will turn off the PMU
    void setVbusCurrentLimit(xpower_vbus_cur_limit_t opt)
    {
        int val = readRegister(XPOWERS_INPUT_CUR_LIMIT_CTRL);
        if (val == -1)return;
        val &= 0xF8;
        writeRegister(XPOWERS_INPUT_CUR_LIMIT_CTRL, val | (opt & 0x07));
    }

    uint8_t getVinCurrentLimit(void)
    {
        return (readRegister(XPOWERS_INPUT_CUR_LIMIT_CTRL) & 0x07);
    }

    /**
     * @brief  Reset the fuel gauge
     */
    void resetGauge(void)
    {
        setRegisterBit(XPOWERS_RESET_FUEL_GAUGE, 3);
    }

    /**
     * @brief   reset the gauge besides reset
     */
    void resetGaugeBesides(void)
    {
        setRegisterBit(XPOWERS_RESET_FUEL_GAUGE, 2);
    }


    /**
     * @brief Gauge Module
     */
    void enableGauge(void)
    {
        setRegisterBit(XPOWERS_CHARGE_GAUGE_WDT_CTRL, 3);
    }

    void disableGauge(void)
    {
        clrRegisterBit(XPOWERS_CHARGE_GAUGE_WDT_CTRL, 3);
    }

    /**
     * @brief  Button Battery charge
     */
    void enableButtonBatteryCharge(void)
    {
        setRegisterBit(XPOWERS_CHARGE_GAUGE_WDT_CTRL, 2);
    }

    void disableButtonBatteryCharge(void)
    {
        clrRegisterBit(XPOWERS_CHARGE_GAUGE_WDT_CTRL, 2);
    }

    /**
     * @brief Cell Battery charge
     */
    void enableCellbatteryCharge(void)
    {
        setRegisterBit(XPOWERS_CHARGE_GAUGE_WDT_CTRL, 1);
    }

    void disableCellbatteryCharge(void)
    {
        clrRegisterBit(XPOWERS_CHARGE_GAUGE_WDT_CTRL, 1);
    }

    /**
     * @brief  Watchdog Module
     */
    void enableWatchdog(void)
    {
        setRegisterBit(XPOWERS_CHARGE_GAUGE_WDT_CTRL, 0);
        enableIRQ(XPOWERS_WDT_EXPIRE_IRQ);
    }

    void disableWatchdog(void)
    {
        disableIRQ(XPOWERS_WDT_EXPIRE_IRQ);
        clrRegisterBit(XPOWERS_CHARGE_GAUGE_WDT_CTRL, 0);
    }

    /**
     * @brief Watchdog Config
     * @note
     * @param  opt: 0: IRQ Only 1: IRQ and System reset  2: IRQ, System Reset and Pull down PWROK 1s  3: IRQ, System Reset, DCDC/LDO PWROFF & PWRON
     * @retval None
     */
    void setWatchdogConfig(xpowers_wdt_config_t opt)
    {
        uint8_t val = readRegister(XPOWERS_WDT_CTRL) & 0xCF;
        writeRegister(XPOWERS_WDT_CTRL, val | (opt << 4));
    }

    uint8_t getWatchConfig(void)
    {
        return (readRegister(XPOWERS_WDT_CTRL) & 0x30) >> 4;
    }

    void clrWatchdog(void)
    {
        setRegisterBit(XPOWERS_WDT_CTRL, 3);
    }


    void setWatchdogTimeout(xpowers_wdt_timeout_t opt)
    {
        uint8_t val = readRegister(XPOWERS_WDT_CTRL) & 0xF8;
        writeRegister(XPOWERS_WDT_CTRL, val | opt);
    }

    uint8_t getWatchdogTimerout(void)
    {
        return readRegister(XPOWERS_WDT_CTRL) & 0x07;
    }

    /**
     * @brief Low battery warning threshold 5-20%, 1% per step
     */
    void setLowBatWarnThreshold(uint8_t opt)
    {
        uint8_t val = readRegister(XPOWERS_LOW_BAT_WARN_SET) & 0x0F;
        writeRegister(XPOWERS_LOW_BAT_WARN_SET, val | (opt << 4));
    }

    uint8_t getLowBatWarnThreshold(void)
    {
        return (readRegister(XPOWERS_LOW_BAT_WARN_SET) & 0xF0) >> 4;
    }

    /**
     * @brief Low battery shutdown threshold 0-15%, 1% per step
     */

    void setLowBatShutdownThreshold(uint8_t opt)
    {
        uint8_t val = readRegister(XPOWERS_LOW_BAT_WARN_SET) & 0xF0;
        writeRegister(XPOWERS_LOW_BAT_WARN_SET, val | opt);
    }

    uint8_t getLowBatShutdownThreshold(void)
    {
        return (readRegister(XPOWERS_LOW_BAT_WARN_SET) & 0x0F);
    }

    //!  PWRON statu  20
    // POWERON always high when EN Mode as POWERON Source
    bool isPoweronAlwaysHighSource()
    {
        return getRegisterBit(XPOWERS_PWRON_STATUS, 5);
    }

    // Battery Insert and Good as POWERON Source
    bool isBattInsertOnSource()
    {
        return getRegisterBit(XPOWERS_PWRON_STATUS, 4);
    }

    // Battery Voltage > 3.3V when Charged as Source
    bool isBattNormalOnSource()
    {
        return getRegisterBit(XPOWERS_PWRON_STATUS, 3);
    }

    // Vbus Insert and Good as POWERON Source
    bool isVbusInsertOnSource()
    {
        return getRegisterBit(XPOWERS_PWRON_STATUS, 2);
    }

    // IRQ PIN Pull-down as POWERON Source
    bool isIrqLowOnSource()
    {
        return getRegisterBit(XPOWERS_PWRON_STATUS, 1);
    }

    // POWERON low for on level when POWERON Mode as POWERON Source
    bool isPwronLowOnSource()
    {
        return getRegisterBit(XPOWERS_PWRON_STATUS, 0);
    }

    xpower_power_on_source_t getPowerOnSource()
    {
        int val = readRegister(XPOWERS_PWRON_STATUS);
        if (val == -1) return XPOWER_POWERON_SRC_UNKONW;
        return (xpower_power_on_source_t)val;
    }

    //!  PWROFF status  21
    // Die Over Temperature as POWEROFF Source
    bool isOverTemperatureOffSource()
    {
        return getRegisterBit(XPOWERS_PWROFF_STATUS, 7);
    }

    // DCDC Over Voltage as POWEROFF Source
    bool isDcOverVoltageOffSource()
    {
        return getRegisterBit(XPOWERS_PWROFF_STATUS, 6);
    }

    // DCDC Under Voltage as POWEROFF Source
    bool isDcUnderVoltageOffSource()
    {
        return getRegisterBit(XPOWERS_PWROFF_STATUS, 5);
    }

    // VBUS Over Voltage as POWEROFF Source
    bool isVbusOverVoltageOffSource()
    {
        return getRegisterBit(XPOWERS_PWROFF_STATUS, 4);
    }

    // Vsys Under Voltage as POWEROFF Source
    bool isVsysUnderVoltageOffSource()
    {
        return getRegisterBit(XPOWERS_PWROFF_STATUS, 3);
    }

    // POWERON always low when EN Mode as POWEROFF Source
    bool isPwronAlwaysLowOffSource()
    {
        return getRegisterBit(XPOWERS_PWROFF_STATUS, 2);
    }

    // Software configuration as POWEROFF Source
    bool isSwConfigOffSource()
    {
        return getRegisterBit(XPOWERS_PWROFF_STATUS, 1);
    }

    // POWERON Pull down for off level when POWERON Mode as POWEROFF Source
    bool isPwrSourcePullDown()
    {
        return getRegisterBit(XPOWERS_PWROFF_STATUS, 0);
    }

    xpower_power_off_source_t getPowerOffSource()
    {
        int val = readRegister(XPOWERS_PWRON_STATUS);
        if (val == -1) return XPOWER_POWEROFF_SRC_UNKONW;
        return (xpower_power_off_source_t)val;
    }

    //!REG 22H
    void enableOverTemperatureLevel2PowerOff()
    {
        setRegisterBit(XPOWERS_PWROFF_EN, 2);
    }

    void disableOverTemperaturePowerOff()
    {
        clrRegisterBit(XPOWERS_PWROFF_EN, 2);
    }

    void enablePwrOnOverVolOffLevelPowerOff()
    {
        setRegisterBit(XPOWERS_PWROFF_EN, 1);
    }

    void disablePwrOnOverVolOffLevelPowerOff()
    {
        clrRegisterBit(XPOWERS_PWROFF_EN, 1);
    }

    void enablePwrOffSelectFunction()
    {
        setRegisterBit(XPOWERS_PWROFF_EN, 0);
    }

    void disablePwrOffSelectFunction()
    {
        clrRegisterBit(XPOWERS_PWROFF_EN, 0);
    }

    //!REG 23H
    // DCDC 120%(130%) high voltage turn off PMIC function
    void enableDCHighVoltageTurnOff()
    {
        setRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 5);
    }

    void disableDCHighVoltageTurnOff()
    {
        clrRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 5);
    }

    // DCDC5 85% low voltage turn Off PMIC function
    void enableDC5LowVoltageTurnOff()
    {
        setRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 4);
    }

    void disableDC5LowVoltageTurnOff()
    {
        clrRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 4);
    }

    // DCDC4 85% low voltage turn Off PMIC function
    void enableDC4LowVoltageTurnOff()
    {
        setRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 3);
    }

    void disableDC4LowVoltageTurnOff()
    {
        clrRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 3);
    }

    // DCDC3 85% low voltage turn Off PMIC function
    void enableDC3LowVoltageTurnOff()
    {
        setRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 2);
    }

    void disableDC3LowVoltageTurnOff()
    {
        clrRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 2);
    }

    // DCDC2 85% low voltage turn Off PMIC function
    void enableDC2LowVoltageTurnOff()
    {
        setRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 1);
    }

    void disableDC2LowVoltageTurnOff()
    {
        clrRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 1);
    }

    // DCDC1 85% low voltage turn Off PMIC function
    void enableDC1LowVoltageTurnOff()
    {
        setRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 0);
    }

    void disableDC1LowVoltageTurnOff()
    {
        clrRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 0);
    }


    //  Vsys voltage for PWROFF threshold setting 24
    void setVsysPowerOffThreshold(uint8_t opt)
    {
        uint8_t val = readRegister(XPOWERS_VOFF_SET) & 0xF8;
        writeRegister(XPOWERS_VOFF_SET, val | (opt));
    }

    uint8_t getVsysPowerOffThreshold(void)
    {
        return readRegister(XPOWERS_VOFF_SET) & 0x07;
    }

    //  PWROK setting and PWROFF sequence control 25.
    // Check the PWROK Pin enable after all dcdc/ldo output valid 128ms
    void enablePwrOk()
    {
        setRegisterBit(XPOWERS_PWROK_SEQU_CTRL, 4);
    }

    void disablePwrOk()
    {
        clrRegisterBit(XPOWERS_PWROK_SEQU_CTRL, 4);
    }

    // POWEROFF Delay 4ms after PWROK enable
    void eanblePowerOffDelay()
    {
        setRegisterBit(XPOWERS_PWROK_SEQU_CTRL, 3);
    }

    // POWEROFF Delay 4ms after PWROK disable
    void disablePowerOffDelay()
    {
        clrRegisterBit(XPOWERS_PWROK_SEQU_CTRL, 3);
    }

    // POWEROFF Sequence Control the reverse of the Startup
    void eanblePowerSequence()
    {
        setRegisterBit(XPOWERS_PWROK_SEQU_CTRL, 2);
    }

    // POWEROFF Sequence Control at the same time
    void disablePowerSequence()
    {
        clrRegisterBit(XPOWERS_PWROK_SEQU_CTRL, 2);
    }

    // Delay of PWROK after all power output good
    bool setPwrOkDelay(xpower_pwrok_delay_t opt)
    {
        int val = readRegister(XPOWERS_PWROK_SEQU_CTRL);
        if (val == -1)return false;
        val &= 0xFC;
        return 0 == writeRegister(XPOWERS_PWROK_SEQU_CTRL, val | opt);
    }

    xpower_pwrok_delay_t getPwrOkDelay()
    {
        int val = readRegister(XPOWERS_PWROK_SEQU_CTRL);
        if (val == -1)return XPOWER_PWROK_DELAY_8MS;
        return (xpower_pwrok_delay_t)(val & 0x03);
    }

    //  Sleep and 26
    void wakeupControl(xpowers_wakeup_t opt, bool enable)
    {
        uint8_t val = readRegister(XPOWERS_SLEEP_WAKEUP_CTRL) ;
        enable ? (val | opt) : (val & (~opt));
        writeRegister(XPOWERS_SLEEP_WAKEUP_CTRL, val | opt);
    }

    void enableWakeup(void)
    {
        setRegisterBit(XPOWERS_SLEEP_WAKEUP_CTRL, 1);
    }

    void disableWakeup(void)
    {
        clrRegisterBit(XPOWERS_SLEEP_WAKEUP_CTRL, 1);
    }

    void enableSleep(void)
    {
        setRegisterBit(XPOWERS_SLEEP_WAKEUP_CTRL, 0);
    }

    void disableSleep(void)
    {
        clrRegisterBit(XPOWERS_SLEEP_WAKEUP_CTRL, 0);
    }


    //  RQLEVEL/OFFLEVEL/ONLEVEL setting 27
    /**
     * @brief  IRQLEVEL configur
     * @param  opt: 0:1s  1:1.5s  2:2s 3:2.5s
     */
    void setIrqLevel(uint8_t opt)
    {
        uint8_t val = readRegister(XPOWERS_IRQ_OFF_ON_LEVEL_CTRL);
        writeRegister(XPOWERS_IRQ_OFF_ON_LEVEL_CTRL, val | (opt << 4));
    }

    /**
     * @brief  OFFLEVEL configuration
     * @param  opt:  0:4s 1:6s 2:8s 3:10s
     */
    void setOffLevel(uint8_t opt)
    {
        uint8_t val = readRegister(XPOWERS_IRQ_OFF_ON_LEVEL_CTRL);
        writeRegister(XPOWERS_IRQ_OFF_ON_LEVEL_CTRL, val | (opt << 2));
    }

    /**
     * @brief  ONLEVEL configuration
     * @param  opt: 0:128ms 1:512ms 2:1s  3:2s
     */
    void setOnLevel(uint8_t opt)
    {
        uint8_t val = readRegister(XPOWERS_IRQ_OFF_ON_LEVEL_CTRL);
        writeRegister(XPOWERS_IRQ_OFF_ON_LEVEL_CTRL, val | opt);
    }

    // Fast pwron setting 0  28
    // Fast Power On Start Sequence
    void setDc4FastStartSequence(xpower_start_sequence_t opt)
    {
        int val = readRegister(XPOWERS_FAST_PWRON_SET0);
        if (val == -1)return;
        writeRegister(XPOWERS_FAST_PWRON_SET0, val | ((opt & 0x3) << 6));
    }

    void setDc3FastStartSequence(xpower_start_sequence_t opt)
    {
        int val = readRegister(XPOWERS_FAST_PWRON_SET0);
        if (val == -1)return;
        writeRegister(XPOWERS_FAST_PWRON_SET0, val | ((opt & 0x3) << 4));
    }
    void setDc2FastStartSequence(xpower_start_sequence_t opt)
    {
        int val = readRegister(XPOWERS_FAST_PWRON_SET0);
        if (val == -1)return;
        writeRegister(XPOWERS_FAST_PWRON_SET0, val | ((opt & 0x3) << 2));
    }
    void setDc1FastStartSequence(xpower_start_sequence_t opt)
    {
        int val = readRegister(XPOWERS_FAST_PWRON_SET0);
        if (val == -1)return;
        writeRegister(XPOWERS_FAST_PWRON_SET0, val | (opt & 0x3));
    }

    //  Fast pwron setting 1  29
    void setAldo3FastStartSequence(xpower_start_sequence_t opt)
    {
        int val = readRegister(XPOWERS_FAST_PWRON_SET1);
        if (val == -1)return;
        writeRegister(XPOWERS_FAST_PWRON_SET1, val | ((opt & 0x3) << 6));
    }
    void setAldo2FastStartSequence(xpower_start_sequence_t opt)
    {
        int val = readRegister(XPOWERS_FAST_PWRON_SET1);
        if (val == -1)return;
        writeRegister(XPOWERS_FAST_PWRON_SET1, val | ((opt & 0x3) << 4));
    }
    void setAldo1FastStartSequence(xpower_start_sequence_t opt)
    {
        int val = readRegister(XPOWERS_FAST_PWRON_SET1);
        if (val == -1)return;
        writeRegister(XPOWERS_FAST_PWRON_SET1, val | ((opt & 0x3) << 2));
    }

    void setDc5FastStartSequence(xpower_start_sequence_t opt)
    {
        int val = readRegister(XPOWERS_FAST_PWRON_SET1);
        if (val == -1)return;
        writeRegister(XPOWERS_FAST_PWRON_SET1, val | (opt & 0x3));
    }

    //  Fast pwron setting 2  2A
    void setCpuldoFastStartSequence(xpower_start_sequence_t opt)
    {
        int val = readRegister(XPOWERS_FAST_PWRON_SET2);
        if (val == -1)return;
        writeRegister(XPOWERS_FAST_PWRON_SET2, val | ((opt & 0x3) << 6));
    }

    void setBldo2FastStartSequence(xpower_start_sequence_t opt)
    {
        int val = readRegister(XPOWERS_FAST_PWRON_SET2);
        if (val == -1)return;
        writeRegister(XPOWERS_FAST_PWRON_SET2, val | ((opt & 0x3) << 4));
    }

    void setBldo1FastStartSequence(xpower_start_sequence_t opt)
    {
        int val = readRegister(XPOWERS_FAST_PWRON_SET2);
        if (val == -1)return;
        writeRegister(XPOWERS_FAST_PWRON_SET2, val | ((opt & 0x3) << 2));
    }

    void setAldo4FastStartSequence(xpower_start_sequence_t opt)
    {
        int val = readRegister(XPOWERS_FAST_PWRON_SET2);
        if (val == -1)return;
        writeRegister(XPOWERS_FAST_PWRON_SET2, val | (opt & 0x3));
    }

    //  Fast pwron setting 3  2B
    void setDldo2FastStartSequence(xpower_start_sequence_t opt)
    {
        int val = readRegister(XPOWERS_FAST_PWRON_CTRL);
        if (val == -1)return;
        writeRegister(XPOWERS_FAST_PWRON_CTRL, val | ((opt & 0x3) << 2));
    }

    void setDldo1FastStartSequence(xpower_start_sequence_t opt)
    {
        int val = readRegister(XPOWERS_FAST_PWRON_CTRL);
        if (val == -1)return;
        writeRegister(XPOWERS_FAST_PWRON_CTRL, val | (opt & 0x3));
    }

    /**
     * @brief   Setting Fast Power On Start Sequence
     */
    void setFastPowerOnLevel(xpowers_fast_on_opt_t opt, xpower_start_sequence_t seq_level)
    {
        uint8_t val = 0;
        switch (opt) {
        case XPOWERS_FAST_DCDC1:
            val = readRegister(XPOWERS_FAST_PWRON_SET0);
            writeRegister(XPOWERS_FAST_PWRON_SET0, val | seq_level);
            break;
        case XPOWERS_FAST_DCDC2:
            val = readRegister(XPOWERS_FAST_PWRON_SET0);
            writeRegister(XPOWERS_FAST_PWRON_SET0, val | (seq_level << 2));
            break;
        case XPOWERS_FAST_DCDC3:
            val = readRegister(XPOWERS_FAST_PWRON_SET0);
            writeRegister(XPOWERS_FAST_PWRON_SET0, val | (seq_level << 4));
            break;
        case XPOWERS_FAST_DCDC4:
            val = readRegister(XPOWERS_FAST_PWRON_SET0);
            writeRegister(XPOWERS_FAST_PWRON_SET0, val | (seq_level << 6));
            break;
        case XPOWERS_FAST_DCDC5:
            val = readRegister(XPOWERS_FAST_PWRON_SET1);
            writeRegister(XPOWERS_FAST_PWRON_SET1, val | seq_level);
            break;
        case XPOWERS_FAST_ALDO1:
            val = readRegister(XPOWERS_FAST_PWRON_SET1);
            writeRegister(XPOWERS_FAST_PWRON_SET1, val | (seq_level << 2));
            break;
        case XPOWERS_FAST_ALDO2:
            val = readRegister(XPOWERS_FAST_PWRON_SET1);
            writeRegister(XPOWERS_FAST_PWRON_SET1, val | (seq_level << 4));
            break;
        case XPOWERS_FAST_ALDO3:
            val = readRegister(XPOWERS_FAST_PWRON_SET1);
            writeRegister(XPOWERS_FAST_PWRON_SET1, val | (seq_level << 6));
            break;
        case XPOWERS_FAST_ALDO4:
            val = readRegister(XPOWERS_FAST_PWRON_SET2);
            writeRegister(XPOWERS_FAST_PWRON_SET2, val | seq_level);
            break;
        case XPOWERS_FAST_BLDO1:
            val = readRegister(XPOWERS_FAST_PWRON_SET2);
            writeRegister(XPOWERS_FAST_PWRON_SET2, val | (seq_level << 2));
            break;
        case XPOWERS_FAST_BLDO2:
            val = readRegister(XPOWERS_FAST_PWRON_SET2);
            writeRegister(XPOWERS_FAST_PWRON_SET2, val | (seq_level << 4));
            break;
        case XPOWERS_FAST_CPUSLDO:
            val = readRegister(XPOWERS_FAST_PWRON_SET2);
            writeRegister(XPOWERS_FAST_PWRON_SET2, val | (seq_level << 6));
            break;
        case XPOWERS_FAST_DLDO1:
            val = readRegister(XPOWERS_FAST_PWRON_CTRL);
            writeRegister(XPOWERS_FAST_PWRON_CTRL, val | seq_level);
            break;
        case XPOWERS_FAST_DLDO2:
            val = readRegister(XPOWERS_FAST_PWRON_CTRL);
            writeRegister(XPOWERS_FAST_PWRON_CTRL, val | (seq_level << 2));
            break;
        default:
            break;
        }
    }

    void disableFastPowerOn(xpowers_fast_on_opt_t opt)
    {
        uint8_t val = 0;
        switch (opt) {
        case XPOWERS_FAST_DCDC1:
            val = readRegister(XPOWERS_FAST_PWRON_SET0);
            writeRegister(XPOWERS_FAST_PWRON_SET0, val & 0xFC);
            break;
        case XPOWERS_FAST_DCDC2:
            val = readRegister(XPOWERS_FAST_PWRON_SET0);
            writeRegister(XPOWERS_FAST_PWRON_SET0, val & 0xF3);
            break;
        case XPOWERS_FAST_DCDC3:
            val = readRegister(XPOWERS_FAST_PWRON_SET0);
            writeRegister(XPOWERS_FAST_PWRON_SET0, val & 0xCF);
            break;
        case XPOWERS_FAST_DCDC4:
            val = readRegister(XPOWERS_FAST_PWRON_SET0);
            writeRegister(XPOWERS_FAST_PWRON_SET0, val & 0x3F);
            break;
        case XPOWERS_FAST_DCDC5:
            val = readRegister(XPOWERS_FAST_PWRON_SET1);
            writeRegister(XPOWERS_FAST_PWRON_SET1, val & 0xFC);
            break;
        case XPOWERS_FAST_ALDO1:
            val = readRegister(XPOWERS_FAST_PWRON_SET1);
            writeRegister(XPOWERS_FAST_PWRON_SET1, val & 0xF3);
            break;
        case XPOWERS_FAST_ALDO2:
            val = readRegister(XPOWERS_FAST_PWRON_SET1);
            writeRegister(XPOWERS_FAST_PWRON_SET1, val & 0xCF);
            break;
        case XPOWERS_FAST_ALDO3:
            val = readRegister(XPOWERS_FAST_PWRON_SET1);
            writeRegister(XPOWERS_FAST_PWRON_SET1, val & 0x3F);
            break;
        case XPOWERS_FAST_ALDO4:
            val = readRegister(XPOWERS_FAST_PWRON_SET2);
            writeRegister(XPOWERS_FAST_PWRON_SET2, val & 0xFC);
            break;
        case XPOWERS_FAST_BLDO1:
            val = readRegister(XPOWERS_FAST_PWRON_SET2);
            writeRegister(XPOWERS_FAST_PWRON_SET2, val & 0xF3);
            break;
        case XPOWERS_FAST_BLDO2:
            val = readRegister(XPOWERS_FAST_PWRON_SET2);
            writeRegister(XPOWERS_FAST_PWRON_SET2, val & 0xCF);
            break;
        case XPOWERS_FAST_CPUSLDO:
            val = readRegister(XPOWERS_FAST_PWRON_SET2);
            writeRegister(XPOWERS_FAST_PWRON_SET2, val & 0x3F);
            break;
        case XPOWERS_FAST_DLDO1:
            val = readRegister(XPOWERS_FAST_PWRON_CTRL);
            writeRegister(XPOWERS_FAST_PWRON_CTRL, val & 0xFC);
            break;
        case XPOWERS_FAST_DLDO2:
            val = readRegister(XPOWERS_FAST_PWRON_CTRL);
            writeRegister(XPOWERS_FAST_PWRON_CTRL, val & 0xF3);
            break;
        default:
            break;
        }
    }

    void enableFastPowerOn(void)
    {
        setRegisterBit(XPOWERS_FAST_PWRON_CTRL, 7);
    }

    void disableFastPowerOn(void)
    {
        clrRegisterBit(XPOWERS_FAST_PWRON_CTRL, 7);
    }

    void enableFastWakeup(void)
    {
        setRegisterBit(XPOWERS_FAST_PWRON_CTRL, 6);
    }

    void disableFastWakeup(void)
    {
        clrRegisterBit(XPOWERS_FAST_PWRON_CTRL, 6);
    }

    // DCDC 120%(130%) high voltage turn off PMIC function
    void setDCHighVoltagePowerDowm(bool en)
    {
        en ? setRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 5) : clrRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 5);
    }

    bool getDCHighVoltagePowerDowmEn()
    {
        return getRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 5);
    }

    /*
     * Power control DCDC1 functions
     */
    bool isEnableDC1(void)
    {
        return getRegisterBit(XPOWERS_DC_ONOFF_DVM_CTRL, 0);
    }

    bool enableDC1(void)
    {
        return setRegisterBit(XPOWERS_DC_ONOFF_DVM_CTRL, 0);
    }

    bool disableDC1(void)
    {
        return clrRegisterBit(XPOWERS_DC_ONOFF_DVM_CTRL, 0);
    }

    bool setDC1Voltage(uint16_t millivolt)
    {
        if (millivolt % XPOWERS_DCDC1_VOL_STEPS) {
            log_e("Mistake ! The steps is must %u mV", XPOWERS_DCDC1_VOL_STEPS);
            return false;
        }
        if (millivolt < XPOWERS_DCDC1_VOL_MIN) {
            log_e("Mistake ! DC1 minimum voltage is %u mV", XPOWERS_DCDC1_VOL_MIN);
            return false;
        } else if (millivolt > XPOWERS_DCDC5_VOL_MAX) {
            log_e("Mistake ! DC1 maximum voltage is %u mV", XPOWERS_DCDC1_VOL_MAX);
            return false;
        }
        return 0 == writeRegister(XPOWERS_DC_VOL0_CTRL, (millivolt - XPOWERS_DCDC1_VOL_MIN) / XPOWERS_DCDC1_VOL_STEPS);
    }

    uint16_t getDC1Voltage(void)
    {
        return (readRegister(XPOWERS_DC_VOL0_CTRL) & 0x1F) * XPOWERS_DCDC1_VOL_STEPS + XPOWERS_DCDC1_VOL_MIN;
    }

    uint8_t getDC1WorkMode(void)
    {
        // return (readRegister(XPOWERS_DCDC_MODESET) & _BV(0))  == _BV(0);
        return 0;
    }

    // DCDC1 85% low voltage turn off PMIC function
    void setDC1LowVoltagePowerDowm(bool en)
    {
        en ? setRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 0) : clrRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 0);
    }

    bool getDC1LowVoltagePowerDowmEn()
    {
        return getRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 0);
    }

    /*
     * Power control DCDC2 functions
     */
    bool isEnableDC2(void)
    {
        return getRegisterBit(XPOWERS_DC_ONOFF_DVM_CTRL, 1);
    }

    bool enableDC2(void)
    {
        return setRegisterBit(XPOWERS_DC_ONOFF_DVM_CTRL, 1);
    }

    bool disableDC2(void)
    {
        return clrRegisterBit(XPOWERS_DC_ONOFF_DVM_CTRL, 1);
    }

    bool setDC2Voltage(uint16_t millivolt)
    {
        uint8_t val = readRegister(XPOWERS_DC_VOL1_CTRL);
        if (val == -1)return 0;
        val &= 0x80;
        if (millivolt >= XPOWERS_DCDC2_VOL1_MIN && millivolt <= XPOWERS_DCDC2_VOL1_MAX) {
            if (millivolt % XPOWERS_DCDC2_VOL_STEPS1) {
                log_e("Mistake !  The steps is must %umV", XPOWERS_DCDC2_VOL_STEPS1);
                return false;
            }
            return  0 == writeRegister(XPOWERS_DC_VOL1_CTRL, val | (millivolt - XPOWERS_DCDC2_VOL1_MIN) / XPOWERS_DCDC2_VOL_STEPS1);
        } else if (millivolt >= XPOWERS_DCDC2_VOL2_MIN && millivolt <= XPOWERS_DCDC2_VOL2_MAX) {
            if (millivolt % XPOWERS_DCDC2_VOL_STEPS2) {
                log_e("Mistake !  The steps is must %umV", XPOWERS_DCDC2_VOL_STEPS2);
                return false;
            }
            val |= (((millivolt - XPOWERS_DCDC2_VOL2_MIN) / XPOWERS_DCDC2_VOL_STEPS2) + XPOWERS_DCDC2_VOL_STEPS2_BASE);
            return  0 == writeRegister(XPOWERS_DC_VOL1_CTRL, val);
        }
        return false;
    }

    uint16_t getDC2Voltage(void)
    {
        int val = readRegister(XPOWERS_DC_VOL1_CTRL);
        if (val ==  -1)return 0;
        val &= 0x7F;
        if (val < XPOWERS_DCDC2_VOL_STEPS2_BASE) {
            return (val  * XPOWERS_DCDC2_VOL_STEPS1) +  XPOWERS_DCDC2_VOL1_MIN;
        } else  {
            return (val  * XPOWERS_DCDC2_VOL_STEPS2) - 200;
        }
        return 0;
    }

    uint8_t getDC2WorkMode(void)
    {
        return getRegisterBit(XPOWERS_DCDC2_VOL_STEPS2, 7);
    }

    void setDC2LowVoltagePowerDowm(bool en)
    {
        en ? setRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 1) : clrRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 1);
    }

    bool getDC2LowVoltagePowerDowmEn()
    {
        return getRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 1);
    }

    /*
     * Power control DCDC3 functions
     */
    bool isEnableDC3(void)
    {
        return getRegisterBit(XPOWERS_DC_ONOFF_DVM_CTRL, 2);
    }

    bool enableDC3(void)
    {
        return setRegisterBit(XPOWERS_DC_ONOFF_DVM_CTRL, 2);
    }

    bool disableDC3(void)
    {
        return clrRegisterBit(XPOWERS_DC_ONOFF_DVM_CTRL, 2);
    }

    /**
        0.5~1.2V,10mV/step,71steps
        1.22~1.54V,20mV/step,17steps
        1.6~3.4V,100mV/step,19steps
     */
    bool setDC3Voltage(uint16_t millivolt)
    {
        int val = readRegister(XPOWERS_DC_VOL2_CTRL);
        if (val == -1)return false;
        val &= 0x80;
        if (millivolt >= XPOWERS_DCDC3_VOL1_MIN && millivolt <= XPOWERS_DCDC3_VOL1_MAX) {
            if (millivolt % XPOWERS_DCDC3_VOL_STEPS1) {
                log_e("Mistake ! The steps is must %umV", XPOWERS_DCDC3_VOL_STEPS1);
                return false;
            }
            return  0 == writeRegister(XPOWERS_DC_VOL2_CTRL, val | (millivolt - XPOWERS_DCDC3_VOL_MIN) / XPOWERS_DCDC3_VOL_STEPS1);
        } else if (millivolt >= XPOWERS_DCDC3_VOL2_MIN && millivolt <= XPOWERS_DCDC3_VOL2_MAX) {
            if (millivolt % XPOWERS_DCDC3_VOL_STEPS2) {
                log_e("Mistake ! The steps is must %umV", XPOWERS_DCDC3_VOL_STEPS2);
                return false;
            }
            val |= (((millivolt - XPOWERS_DCDC3_VOL2_MIN) / XPOWERS_DCDC3_VOL_STEPS2) + XPOWERS_DCDC3_VOL_STEPS2_BASE);
            return  0 == writeRegister(XPOWERS_DC_VOL2_CTRL, val);
        } else if (millivolt >= XPOWERS_DCDC3_VOL3_MIN && millivolt <= XPOWERS_DCDC3_VOL3_MAX) {
            if (millivolt % XPOWERS_DCDC3_VOL_STEPS3) {
                log_e("Mistake ! The steps is must %umV", XPOWERS_DCDC3_VOL_STEPS3);
                return false;
            }
            val |= (((millivolt - XPOWERS_DCDC3_VOL3_MIN) / XPOWERS_DCDC3_VOL_STEPS3) + XPOWERS_DCDC3_VOL_STEPS3_BASE);
            return  0 == writeRegister(XPOWERS_DC_VOL2_CTRL, val);
        }
        return false;
    }


    uint16_t getDC3Voltage(void)
    {
        int val = readRegister(XPOWERS_DC_VOL2_CTRL) & 0x7F;
        if (val < XPOWERS_DCDC3_VOL_STEPS2_BASE) {
            return (val  * XPOWERS_DCDC3_VOL_STEPS1) +  XPOWERS_DCDC3_VOL_MIN;
        } else if (val >= XPOWERS_DCDC3_VOL_STEPS2_BASE && val < XPOWERS_DCDC3_VOL_STEPS3_BASE) {
            return (val  * XPOWERS_DCDC3_VOL_STEPS2) - 200;
        } else  {
            return (val  * XPOWERS_DCDC3_VOL_STEPS3)  - 7200;
        }
        return 0;
    }

    uint8_t getDC3WorkMode(void)
    {
        return getRegisterBit(XPOWERS_DC_VOL2_CTRL, 7);
    }

    // DCDC3 85% low voltage turn off PMIC function
    void setDC3LowVoltagePowerDowm(bool en)
    {
        en ? setRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 2) : clrRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 2);
    }

    bool getDC3LowVoltagePowerDowmEn()
    {
        return getRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 2);
    }


    /*
    * Power control DCDC4 functions
    */

    /**
        0.5~1.2V,10mV/step,71steps
        1.22~1.84V,20mV/step,32steps
     */
    bool isEnableDC4(void)
    {
        return getRegisterBit(XPOWERS_DC_ONOFF_DVM_CTRL, 3);
    }

    bool enableDC4(void)
    {
        return setRegisterBit(XPOWERS_DC_ONOFF_DVM_CTRL, 3);
    }

    bool disableDC4(void)
    {
        return clrRegisterBit(XPOWERS_DC_ONOFF_DVM_CTRL, 3);
    }

    bool setDC4Voltage(uint16_t millivolt)
    {
        int val = readRegister(XPOWERS_DC_VOL3_CTRL);
        if (val == -1)return false;
        val &= 0x80;
        if (millivolt >= XPOWERS_DCDC4_VOL1_MIN && millivolt <= XPOWERS_DCDC4_VOL1_MAX) {
            if (millivolt % XPOWERS_DCDC4_VOL_STEPS1) {
                log_e("Mistake ! The steps is must %umV", XPOWERS_DCDC4_VOL_STEPS1);
                return false;
            }
            return  0 == writeRegister(XPOWERS_DC_VOL3_CTRL, val | (millivolt - XPOWERS_DCDC4_VOL1_MIN) / XPOWERS_DCDC4_VOL_STEPS1);

        } else if (millivolt >= XPOWERS_DCDC4_VOL2_MIN && millivolt <= XPOWERS_DCDC4_VOL2_MAX) {
            if (millivolt % XPOWERS_DCDC4_VOL_STEPS2) {
                log_e("Mistake ! The steps is must %umV", XPOWERS_DCDC4_VOL_STEPS2);
                return false;
            }
            val |= (((millivolt - XPOWERS_DCDC4_VOL2_MIN) / XPOWERS_DCDC4_VOL_STEPS2) + XPOWERS_DCDC4_VOL_STEPS2_BASE);
            return  0 == writeRegister(XPOWERS_DC_VOL3_CTRL, val);

        }
        return false;
    }

    uint16_t getDC4Voltage(void)
    {
        int val = readRegister(XPOWERS_DC_VOL3_CTRL);
        if (val == -1)return 0;
        val &= 0x7F;
        if (val < XPOWERS_DCDC4_VOL_STEPS2_BASE) {
            return (val  * XPOWERS_DCDC4_VOL_STEPS1) +  XPOWERS_DCDC4_VOL1_MIN;
        } else  {
            return (val  * XPOWERS_DCDC4_VOL_STEPS2) - 200;
        }
        return 0;
    }

    // DCDC4 85% low voltage turn off PMIC function
    void setDC4LowVoltagePowerDowm(bool en)
    {
        en ? setRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 3) : clrRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 3);
    }

    bool getDC4LowVoltagePowerDowmEn()
    {
        return getRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 3);
    }

    /*
    * Power control DCDC5 functions,Output to gpio pin
    */
    bool isEnableDC5(void)
    {
        return getRegisterBit(XPOWERS_DC_ONOFF_DVM_CTRL, 4);
    }

    bool enableDC5(void)
    {
        return setRegisterBit(XPOWERS_DC_ONOFF_DVM_CTRL, 4);
    }

    bool disableDC5(void)
    {
        return clrRegisterBit(XPOWERS_DC_ONOFF_DVM_CTRL, 4);
    }

    bool setDC5Voltage(uint16_t millivolt)
    {
        if (millivolt % XPOWERS_DCDC5_VOL_STEPS) {
            log_e("Mistake ! The steps is must %u mV", XPOWERS_DCDC5_VOL_STEPS);
            return false;
        }
        if (millivolt != XPOWERS_DCDC5_VOL_1200MV && millivolt < XPOWERS_DCDC5_VOL_MIN) {
            log_e("Mistake ! DC5 minimum voltage is %umV ,%umV", XPOWERS_DCDC5_VOL_1200MV, XPOWERS_DCDC5_VOL_MIN);
            return false;
        } else if (millivolt > XPOWERS_DCDC5_VOL_MAX) {
            log_e("Mistake ! DC5 maximum voltage is %umV", XPOWERS_DCDC5_VOL_MAX);
            return false;
        }

        int val =  readRegister(XPOWERS_DC_VOL4_CTRL);
        if (val == -1)return false;
        val &= 0xE0;
        if (millivolt == XPOWERS_DCDC5_VOL_1200MV) {
            return 0 == writeRegister(XPOWERS_DC_VOL4_CTRL, val | XPOWERS_DCDC5_VOL_VAL);
        }
        val |= (millivolt - XPOWERS_DCDC5_VOL_MIN) / XPOWERS_DCDC5_VOL_STEPS;
        return 0 == writeRegister(XPOWERS_DC_VOL4_CTRL, val);
    }

    uint16_t getDC5Voltage(void)
    {
        int val = readRegister(XPOWERS_DC_VOL4_CTRL) ;
        if (val == -1)return 0;
        val &= 0x1F;
        if (val == XPOWERS_DCDC5_VOL_VAL)return XPOWERS_DCDC5_VOL_1200MV;
        return  (val * XPOWERS_DCDC5_VOL_STEPS) + XPOWERS_DCDC5_VOL_MIN;
    }

    bool isDC5FreqCompensationEn(void)
    {
        return getRegisterBit(XPOWERS_DC_VOL4_CTRL, 5);
    }

    void enableDC5FreqCompensation()
    {
        setRegisterBit(XPOWERS_DC_VOL4_CTRL, 5);
    }

    void disableFreqCompensation()
    {
        clrRegisterBit(XPOWERS_DC_VOL4_CTRL, 5);
    }

    // DCDC4 85% low voltage turn off PMIC function
    void setDC5LowVoltagePowerDowm(bool en)
    {
        en ? setRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 4) : clrRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 4);
    }

    bool getDC5LowVoltagePowerDowmEn()
    {
        return getRegisterBit(XPOWERS_DC_OVP_UVP_CTRL, 4);
    }

    /*
    * Power control ALDO1 functions
    */
    bool isEnableALDO1(void)
    {
        return getRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 0);
    }

    bool enableALDO1(void)
    {
        return setRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 0);
    }

    bool disableALDO1(void)
    {
        return clrRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 0);
    }

    bool setALDO1Voltage(uint16_t millivolt)
    {
        if (millivolt % XPOWERS_ALDO1_VOL_STEPS) {
            log_e("Mistake ! The steps is must %u mV", XPOWERS_ALDO1_VOL_STEPS);
            return false;
        }
        if (millivolt < XPOWERS_ALDO1_VOL_MIN) {
            log_e("Mistake ! ALDO1 minimum output voltage is  %umV", XPOWERS_ALDO1_VOL_MIN);
            return false;
        } else if (millivolt > XPOWERS_ALDO1_VOL_MAX) {
            log_e("Mistake ! ALDO1 maximum output voltage is  %umV", XPOWERS_ALDO1_VOL_MAX);
            return false;
        }
        uint16_t val =  readRegister(XPOWERS_LDO_VOL0_CTRL) & 0xE0;
        val |= (millivolt - XPOWERS_ALDO1_VOL_MIN) / XPOWERS_ALDO1_VOL_STEPS;
        return 0 == writeRegister(XPOWERS_LDO_VOL0_CTRL, val);
    }

    uint16_t getALDO1Voltage(void)
    {
        uint16_t val =  readRegister(XPOWERS_LDO_VOL0_CTRL) & 0x1F;
        return val * XPOWERS_ALDO1_VOL_STEPS + XPOWERS_ALDO1_VOL_MIN;
    }

    /*
    * Power control ALDO2 functions
    */
    bool isEnableALDO2(void)
    {
        return getRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 1);
    }

    bool enableALDO2(void)
    {
        return setRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 1);
    }

    bool disableALDO2(void)
    {
        return clrRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 1);
    }

    bool setALDO2Voltage(uint16_t millivolt)
    {
        if (millivolt % XPOWERS_ALDO2_VOL_STEPS) {
            log_e("Mistake ! The steps is must %u mV", XPOWERS_ALDO2_VOL_STEPS);
            return false;
        }
        if (millivolt < XPOWERS_ALDO2_VOL_MIN) {
            log_e("Mistake ! ALDO2 minimum output voltage is  %umV", XPOWERS_ALDO2_VOL_MIN);
            return false;
        } else if (millivolt > XPOWERS_ALDO2_VOL_MAX) {
            log_e("Mistake ! ALDO2 maximum output voltage is  %umV", XPOWERS_ALDO2_VOL_MAX);
            return false;
        }
        uint16_t val =  readRegister(XPOWERS_LDO_VOL1_CTRL) & 0xE0;
        val |= (millivolt - XPOWERS_ALDO2_VOL_MIN) / XPOWERS_ALDO2_VOL_STEPS;
        return 0 == writeRegister(XPOWERS_LDO_VOL1_CTRL, val);
    }

    uint16_t getALDO2Voltage(void)
    {
        uint16_t val =  readRegister(XPOWERS_LDO_VOL1_CTRL) & 0x1F;
        return val * XPOWERS_ALDO2_VOL_STEPS + XPOWERS_ALDO2_VOL_MIN;
    }

    /*
     * Power control ALDO3 functions
     */
    bool isEnableALDO3(void)
    {
        return getRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 2);
    }

    bool enableALDO3(void)
    {
        return setRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 2);
    }

    bool disableALDO3(void)
    {
        return clrRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 2);
    }

    bool setALDO3Voltage(uint16_t millivolt)
    {
        if (millivolt % XPOWERS_ALDO3_VOL_STEPS) {
            log_e("Mistake ! The steps is must %u mV", XPOWERS_ALDO3_VOL_STEPS);
            return false;
        }
        if (millivolt < XPOWERS_ALDO3_VOL_MIN) {
            log_e("Mistake ! ALDO3 minimum output voltage is  %umV", XPOWERS_ALDO3_VOL_MIN);
            return false;
        } else if (millivolt > XPOWERS_ALDO3_VOL_MAX) {
            log_e("Mistake ! ALDO3 maximum output voltage is  %umV", XPOWERS_ALDO3_VOL_MAX);
            return false;
        }
        uint16_t val =  readRegister(XPOWERS_LDO_VOL2_CTRL) & 0xE0;
        val |= (millivolt - XPOWERS_ALDO3_VOL_MIN) / XPOWERS_ALDO3_VOL_STEPS;
        return 0 == writeRegister(XPOWERS_LDO_VOL2_CTRL, val);
    }

    uint16_t getALDO3Voltage(void)
    {
        uint16_t val =  readRegister(XPOWERS_LDO_VOL2_CTRL) & 0x1F;
        return val * XPOWERS_ALDO3_VOL_STEPS + XPOWERS_ALDO3_VOL_MIN;
    }

    /*
     * Power control ALDO4 functions
     */
    bool isEnableALDO4(void)
    {
        return getRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 3);
    }

    bool enableALDO4(void)
    {
        return setRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 3);
    }

    bool disableALDO4(void)
    {
        return clrRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 3);
    }

    bool setALDO4Voltage(uint16_t millivolt)
    {
        if (millivolt % XPOWERS_ALDO4_VOL_STEPS) {
            log_e("Mistake ! The steps is must %u mV", XPOWERS_ALDO4_VOL_STEPS);
            return false;
        }
        if (millivolt < XPOWERS_ALDO4_VOL_MIN) {
            log_e("Mistake ! ALDO4 minimum output voltage is  %umV", XPOWERS_ALDO4_VOL_MIN);
            return false;
        } else if (millivolt > XPOWERS_ALDO4_VOL_MAX) {
            log_e("Mistake ! ALDO4 maximum output voltage is  %umV", XPOWERS_ALDO4_VOL_MAX);
            return false;
        }
        uint16_t val =  readRegister(XPOWERS_LDO_VOL3_CTRL) & 0xE0;
        val |= (millivolt - XPOWERS_ALDO4_VOL_MIN) / XPOWERS_ALDO4_VOL_STEPS;
        return 0 == writeRegister(XPOWERS_LDO_VOL3_CTRL, val);
    }

    uint16_t getALDO4Voltage(void)
    {
        uint16_t val =  readRegister(XPOWERS_LDO_VOL3_CTRL) & 0x1F;
        return val * XPOWERS_ALDO4_VOL_STEPS + XPOWERS_ALDO4_VOL_MIN;
    }

    /*
    * Power control BLDO1 functions
    */
    bool isEnableBLDO1(void)
    {
        return getRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 4);
    }

    bool enableBLDO1(void)
    {
        return setRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 4);
    }

    bool disableBLDO1(void)
    {
        return clrRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 4);
    }

    bool setBLDO1Voltage(uint16_t millivolt)
    {
        if (millivolt % XPOWERS_BLDO1_VOL_STEPS) {
            log_e("Mistake ! The steps is must %u mV", XPOWERS_BLDO1_VOL_STEPS);
            return false;
        }
        if (millivolt < XPOWERS_BLDO1_VOL_MIN) {
            log_e("Mistake ! BLDO1 minimum output voltage is  %umV", XPOWERS_BLDO1_VOL_MIN);
            return false;
        } else if (millivolt > XPOWERS_BLDO1_VOL_MAX) {
            log_e("Mistake ! BLDO1 maximum output voltage is  %umV", XPOWERS_BLDO1_VOL_MAX);
            return false;
        }
        int val =  readRegister(XPOWERS_LDO_VOL4_CTRL);
        if (val == -1)return  false;
        val &= 0xE0;
        val |= (millivolt - XPOWERS_BLDO1_VOL_MIN) / XPOWERS_BLDO1_VOL_STEPS;
        return 0 == writeRegister(XPOWERS_LDO_VOL4_CTRL, val);
    }

    uint16_t getBLDO1Voltage(void)
    {
        int val =  readRegister(XPOWERS_LDO_VOL4_CTRL);
        if (val == -1)return 0;
        val &= 0x1F;
        return val * XPOWERS_BLDO1_VOL_STEPS + XPOWERS_BLDO1_VOL_MIN;
    }

    /*
    * Power control BLDO2 functions
    */
    bool isEnableBLDO2(void)
    {
        return getRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 5);
    }

    bool enableBLDO2(void)
    {
        return setRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 5);
    }

    bool disableBLDO2(void)
    {
        return clrRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 5);
    }

    bool setBLDO2Voltage(uint16_t millivolt)
    {
        if (millivolt % XPOWERS_BLDO2_VOL_STEPS) {
            log_e("Mistake ! The steps is must %u mV", XPOWERS_BLDO2_VOL_STEPS);
            return false;
        }
        if (millivolt < XPOWERS_BLDO2_VOL_MIN) {
            log_e("Mistake ! BLDO2 minimum output voltage is  %umV", XPOWERS_BLDO2_VOL_MIN);
            return false;
        } else if (millivolt > XPOWERS_BLDO2_VOL_MAX) {
            log_e("Mistake ! BLDO2 maximum output voltage is  %umV", XPOWERS_BLDO2_VOL_MAX);
            return false;
        }
        uint16_t val =  readRegister(XPOWERS_LDO_VOL5_CTRL) & 0xE0;
        val |= (millivolt - XPOWERS_BLDO2_VOL_MIN) / XPOWERS_BLDO2_VOL_STEPS;
        return 0 == writeRegister(XPOWERS_LDO_VOL5_CTRL, val);
    }

    uint16_t getBLDO2Voltage(void)
    {
        int val =  readRegister(XPOWERS_LDO_VOL5_CTRL);
        if (val == -1)return 0;
        val &= 0x1F;
        return val * XPOWERS_BLDO2_VOL_STEPS + XPOWERS_BLDO2_VOL_MIN;
    }

    /*
    * Power control CPUSLDO functions
    */
    bool isEnableCPUSLDO(void)
    {
        return getRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 6);
    }

    bool enableCPUSLDO(void)
    {
        return setRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 6);
    }

    bool disableCPUSLDO(void)
    {
        return clrRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 6);
    }

    bool setCPUSLDOVoltage(uint16_t millivolt)
    {
        if (millivolt % XPOWERS_CPUSLDO_VOL_STEPS) {
            log_e("Mistake ! The steps is must %u mV", XPOWERS_CPUSLDO_VOL_STEPS);
            return false;
        }
        if (millivolt < XPOWERS_CPUSLDO_VOL_MIN) {
            log_e("Mistake ! CPULDO minimum output voltage is  %umV", XPOWERS_CPUSLDO_VOL_MIN);
            return false;
        } else if (millivolt > XPOWERS_CPUSLDO_VOL_MAX) {
            log_e("Mistake ! CPULDO maximum output voltage is  %umV", XPOWERS_CPUSLDO_VOL_MAX);
            return false;
        }
        uint16_t val =  readRegister(XPOWERS_LDO_VOL6_CTRL) & 0xE0;
        val |= (millivolt - XPOWERS_CPUSLDO_VOL_MIN) / XPOWERS_CPUSLDO_VOL_STEPS;
        return 0 == writeRegister(XPOWERS_LDO_VOL6_CTRL, val);
    }

    uint16_t getCPUSLDOVoltage(void)
    {
        int val =  readRegister(XPOWERS_LDO_VOL6_CTRL);
        if (val == -1)return 0;
        val &= 0x1F;
        return val * XPOWERS_CPUSLDO_VOL_STEPS + XPOWERS_CPUSLDO_VOL_MIN;
    }


    /*
    * Power control DLDO1 functions
    */
    bool isEnableDLDO1(void)
    {
        return getRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 7);
    }

    bool enableDLDO1(void)
    {
        return setRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 7);
    }

    bool disableDLDO1(void)
    {
        return clrRegisterBit(XPOWERS_LDO_ONOFF_CTRL0, 7);
    }

    bool setDLDO1Voltage(uint16_t millivolt)
    {
        if (millivolt % XPOWERS_DLDO1_VOL_STEPS) {
            log_e("Mistake ! The steps is must %u mV", XPOWERS_DLDO1_VOL_STEPS);
            return false;
        }
        if (millivolt < XPOWERS_DLDO1_VOL_MIN) {
            log_e("Mistake ! DLDO1 minimum output voltage is  %umV", XPOWERS_DLDO1_VOL_MIN);
            return false;
        } else if (millivolt > XPOWERS_DLDO1_VOL_MAX) {
            log_e("Mistake ! DLDO1 maximum output voltage is  %umV", XPOWERS_DLDO1_VOL_MAX);
            return false;
        }
        uint16_t val =  readRegister(XPOWERS_LDO_VOL7_CTRL) & 0xE0;
        val |= (millivolt - XPOWERS_DLDO1_VOL_MIN) / XPOWERS_DLDO1_VOL_STEPS;
        return 0 == writeRegister(XPOWERS_LDO_VOL7_CTRL, val);
    }

    uint16_t getDLDO1Voltage(void)
    {
        int val =  readRegister(XPOWERS_LDO_VOL7_CTRL);
        if (val == -1)return 0;
        val &= 0x1F;
        return val * XPOWERS_DLDO1_VOL_STEPS + XPOWERS_DLDO1_VOL_MIN;
    }

    /*
    * Power control DLDO2 functions
    */
    bool isEnableDLDO2(void)
    {
        return getRegisterBit(XPOWERS_LDO_ONOFF_CTRL1, 0);
    }

    bool enableDLDO2(void)
    {
        return setRegisterBit(XPOWERS_LDO_ONOFF_CTRL1, 0);
    }

    bool disableDLDO2(void)
    {
        return clrRegisterBit(XPOWERS_LDO_ONOFF_CTRL1, 0);
    }

    bool setDLDO2Voltage(uint16_t millivolt)
    {
        if (millivolt % XPOWERS_DLDO2_VOL_STEPS) {
            log_e("Mistake ! The steps is must %u mV", XPOWERS_DLDO2_VOL_STEPS);
            return false;
        }
        if (millivolt < XPOWERS_DLDO2_VOL_MIN) {
            log_e("Mistake ! DLDO2 minimum output voltage is  %umV", XPOWERS_DLDO2_VOL_MIN);
            return false;
        } else if (millivolt > XPOWERS_DLDO2_VOL_MAX) {
            log_e("Mistake ! DLDO2 maximum output voltage is  %umV", XPOWERS_DLDO2_VOL_MAX);
            return false;
        }
        uint16_t val =  readRegister(XPOWERS_LDO_VOL8_CTRL) & 0xE0;
        val |= (millivolt - XPOWERS_DLDO2_VOL_MIN) / XPOWERS_DLDO2_VOL_STEPS;
        return 0 == writeRegister(XPOWERS_LDO_VOL8_CTRL, val);
    }

    uint16_t getDLDO2Voltage(void)
    {
        int val =  readRegister(XPOWERS_LDO_VOL8_CTRL);
        if (val == -1)return 0;
        val &= 0x1F;
        return val * XPOWERS_DLDO2_VOL_STEPS + XPOWERS_DLDO2_VOL_MIN;
    }


    /*
     * Power ON OFF IRQ TIMMING Control method
     */

    void setIrqLevelTime(xpowers_irq_time_t opt)
    {
        uint8_t val = readRegister(XPOWERS_IRQ_OFF_ON_LEVEL_CTRL) & 0xCF;
        writeRegister(XPOWERS_IRQ_OFF_ON_LEVEL_CTRL, val | (opt << 4));
    }

    xpowers_irq_time_t getIrqLevelTime(void)
    {
        return (xpowers_irq_time_t)((readRegister(XPOWERS_IRQ_OFF_ON_LEVEL_CTRL) & 0x30) >> 4);
    }

    void setPowerKeyPressOffTime(xpowers_press_off_time_t opt)
    {
        uint8_t val = readRegister(XPOWERS_IRQ_OFF_ON_LEVEL_CTRL) & 0xF3;
        writeRegister(XPOWERS_IRQ_OFF_ON_LEVEL_CTRL, val | (opt << 2));
    }

    xpowers_press_off_time_t getPowerKeyPressOffTime(void)
    {
        return (xpowers_press_off_time_t)((readRegister(XPOWERS_IRQ_OFF_ON_LEVEL_CTRL) & 0x0C) >> 2);
    }

    void setPowerKeyPressOnTime(xpowers_press_on_time_t opt)
    {
        uint8_t val = readRegister(XPOWERS_IRQ_OFF_ON_LEVEL_CTRL) & 0xFC;
        writeRegister(XPOWERS_IRQ_OFF_ON_LEVEL_CTRL, val | opt);

    }
    xpowers_press_on_time_t getPowerKeyPressOnTime(void)
    {
        return (xpowers_press_on_time_t)(readRegister(XPOWERS_IRQ_OFF_ON_LEVEL_CTRL) & 0x03);
    }

    /*
     * ADC Control method
     */
    void enableGeneralAdcChannel(void)
    {
        setRegisterBit(XPOWERS_ADC_CHANNEL_CTRL, 5);
    }

    void disableGeneralAdcChannel(void)
    {
        clrRegisterBit(XPOWERS_ADC_CHANNEL_CTRL, 5);
    }

    void enableTemperatureMeasure(void)
    {
        setRegisterBit(XPOWERS_ADC_CHANNEL_CTRL, 4);
    }

    void disableTemperatureMeasure(void)
    {
        clrRegisterBit(XPOWERS_ADC_CHANNEL_CTRL, 4);
    }

    uint16_t getTemperature(void)
    {
        //!FIXME
        return readRegisterH6L8(XPOWERS_ADC_DATA_RELUST8, XPOWERS_ADC_DATA_RELUST9);
    }

    void enableSystemVoltageMeasure(void)
    {
        setRegisterBit(XPOWERS_ADC_CHANNEL_CTRL, 3);
    }

    void disableSystemVoltageMeasure(void)
    {
        clrRegisterBit(XPOWERS_ADC_CHANNEL_CTRL, 3);
    }

    uint16_t getSystemVoltage(void)
    {
        return readRegisterH6L8(XPOWERS_ADC_DATA_RELUST6, XPOWERS_ADC_DATA_RELUST7);
    }

    void enableVbusVoltageMeasure(void)
    {
        setRegisterBit(XPOWERS_ADC_CHANNEL_CTRL, 2);
    }

    void disableVbusVoltageMeasure(void)
    {
        clrRegisterBit(XPOWERS_ADC_CHANNEL_CTRL, 2);
    }

    uint16_t getVbusVoltage(void)
    {
        return readRegisterH6L8(XPOWERS_ADC_DATA_RELUST4, XPOWERS_ADC_DATA_RELUST5);
    }

    void enableTSPinMeasure(void)
    {
        setRegisterBit(XPOWERS_ADC_CHANNEL_CTRL, 1);
    }

    void disableTSPinMeasure(void)
    {
        clrRegisterBit(XPOWERS_ADC_CHANNEL_CTRL, 1);
    }

    void enableTSPinLowFreqSample(void)
    {
        setRegisterBit(XPOWERS_ADC_CHANNEL_CTRL, 7);
    }

    void disableTSPinLowFreqSample(void)
    {
        clrRegisterBit(XPOWERS_ADC_DATA_RELUST2, 7);
    }

    uint16_t getTsTemperature(void)
    {
        return readRegisterH6L8(XPOWERS_ADC_DATA_RELUST2, XPOWERS_ADC_DATA_RELUST3);
    }

    void enableBattVoltageMeasure(void)
    {
        setRegisterBit(XPOWERS_ADC_CHANNEL_CTRL, 0);
    }

    void disableBattVoltageMeasure(void)
    {
        clrRegisterBit(XPOWERS_ADC_CHANNEL_CTRL, 0);
    }

    void enableBattDetection(void)
    {
        setRegisterBit(XPOWERS_BAT_DET_CTRL, 0);
    }

    void disableBattDetection(void)
    {
        clrRegisterBit(XPOWERS_BAT_DET_CTRL, 0);
    }

    uint16_t getBattVoltage(void)
    {
        return readRegisterH5L8(XPOWERS_ADC_DATA_RELUST0, XPOWERS_ADC_DATA_RELUST1);
    }

    uint8_t getBatteryPercent(void)
    {
        return readRegister(XPOWERS_BAT_PERCENT_DATA);
    }

    /*
    * CHG LED setting and control
    */
    void enableChargingLed(void)
    {
        setRegisterBit(XPOWERS_CHGLED_SET_CTRL, 0);
    }

    void disableChargingLed(void)
    {
        clrRegisterBit(XPOWERS_CHGLED_SET_CTRL, 0);
    }

    void setChargingLedFreq(xpowers_chgled_t opt)
    {
        int val = readRegister(XPOWERS_CHGLED_SET_CTRL);
        if (val == -1)return;
        val &= 0xCF;
        val |= (opt << 4);
        writeRegister(XPOWERS_CHGLED_SET_CTRL, val);
    }

    xpowers_chgled_t getChargingLedFreq(void)
    {
        return (xpowers_chgled_t)((readRegister(XPOWERS_CHGLED_SET_CTRL) & 0x30) >> 4);
    }

    void setChargerLedFunction(xpowers_chgled_func_t opt)
    {
        uint8_t val = readRegister(XPOWERS_CHGLED_SET_CTRL) & 0xF9;
        writeRegister(XPOWERS_CHGLED_SET_CTRL, val | (opt << 1));
    }

    //Button battery charge termination voltage setting
    bool setButtonBatteryChargeVoltage(uint16_t millivolt)
    {
        if (millivolt % XPOWERS_BTN_VOL_STEPS) {
            log_e("Mistake ! Button battery charging step voltage is %u mV", XPOWERS_BTN_VOL_STEPS);
            return false;
        }
        if (millivolt < XPOWERS_BTN_VOL_MIN) {
            log_e("Mistake ! The minimum charge termination voltage of the coin cell battery is %u mV", XPOWERS_BTN_VOL_MIN);
            return false;
        } else if (millivolt > XPOWERS_BTN_VOL_MAX) {
            log_e("Mistake ! The minimum charge termination voltage of the coin cell battery is %u mV", XPOWERS_BTN_VOL_MAX);
            return false;
        }

        millivolt =  constrain(millivolt, XPOWERS_BTN_VOL_MIN, XPOWERS_BTN_VOL_MAX);
        uint16_t val =  readRegister(XPOWERS_BTN_BAT_CHG_VOL_SET) & 0xFC;
        val |= (millivolt - XPOWERS_BTN_VOL_MIN) / XPOWERS_BTN_VOL_STEPS;
        return 0 == writeRegister(XPOWERS_BTN_BAT_CHG_VOL_SET, val);
    }

    /**
     * @brief 预充电充电电流限制
     * @note  Precharge current limit 25*N mA
     * @param  opt: 25 * opt
     * @retval None
     */
    void setPrechargeCurr(xpowers_prechg_t opt)
    {
        uint8_t val = readRegister(XPOWERS_IPRECHG_SET) & 0xFC ;
        writeRegister(XPOWERS_IPRECHG_SET, val | opt);
    }

    xpowers_prechg_t getPrechargeCurr(void)
    {
        return (xpowers_prechg_t)(readRegister(XPOWERS_IPRECHG_SET) & 0x03);
    }

    /**
     * @brief  恒流充电电流限制
     * @note   constant current charge current limit
     * @param  opt: 25*N mA if N<=8 200+100*(N-8) mA if N>8
     * @retval None
     */
    void setChargerConstantCurr(xpowers_icc_chg_t opt)
    {
        uint8_t val = readRegister(XPOWERS_ICC_CHG_SET) & 0xE0 ;
        writeRegister(XPOWERS_ICC_CHG_SET, val | opt);
    }

    xpowers_icc_chg_t getChargerConstantCurr(void)
    {
        return (xpowers_icc_chg_t)(readRegister(XPOWERS_ICC_CHG_SET) & 0x1F);
    }

    /**
     * @brief  充电终止电流限制
     * @note   Charging termination of current limit
     * @retval
     */
    void setChargerTerminationCurr(xpowers_chg_iterm_t opt)
    {
        uint8_t val = readRegister(XPOWERS_ITERM_CHG_SET_CTRL) & 0xF0;
        writeRegister(XPOWERS_ICC_CHG_SET, val | opt);
    }

    xpowers_chg_iterm_t getChargerTerminationCurr(void)
    {
        return (xpowers_chg_iterm_t)(readRegister(XPOWERS_ITERM_CHG_SET_CTRL) & 0x0F);
    }

    void enableChargerTerminationLimit(void)
    {
        uint8_t val = readRegister(XPOWERS_ITERM_CHG_SET_CTRL);
        writeRegister(XPOWERS_ITERM_CHG_SET_CTRL, val | 0x10);
    }

    void disableChargerTerminationLimit(void)
    {
        uint8_t val = readRegister(XPOWERS_ITERM_CHG_SET_CTRL);
        writeRegister(XPOWERS_ITERM_CHG_SET_CTRL, val & 0xEF);
    }

    bool isChargerTerminationLimit(void)
    {
        return getRegisterBit(XPOWERS_ITERM_CHG_SET_CTRL, 4);
    }


    /**
     * @brief  设置充电电压
     * @note   Charger voltage limit
     */
    void setChargerVoltageLimit(xpowers_chg_vol_t opt)
    {
        uint8_t val = readRegister(XPOWERS_CV_CHG_VOL_SET) & 0xFC;
        writeRegister(XPOWERS_ITERM_CHG_SET_CTRL, val | opt);
    }

    xpowers_chg_vol_t getChargerVoltageLimit(void)
    {
        return (xpowers_chg_vol_t)(readRegister(XPOWERS_CV_CHG_VOL_SET) & 0x03);
    }


    /**
     * @brief  设定热阈值
     * @note   Thermal regulation threshold setting
     */
    void setThermaThreshold(xpowers_thermal_t opt)
    {
        uint8_t val = readRegister(XPOWERS_THE_REGU_THRES_SET) & 0xFC;
        writeRegister(XPOWERS_THE_REGU_THRES_SET, val | opt);
    }

    xpowers_thermal_t getThermaThreshold(void)
    {
        return (xpowers_thermal_t)(readRegister(XPOWERS_THE_REGU_THRES_SET) & 0x03);
    }

    uint8_t getBatteryParameter()
    {
        return  readRegister(XPOWERS_BAT_PARAME);
    }

    /*
     * Interrupt status/control functions
     */
    uint32_t getIrqStatus(void)
    {
        statusRegister[0] = readRegister(XPOWERS_INTSTS1);
        statusRegister[1] = readRegister(XPOWERS_INTSTS2);
        statusRegister[2] = readRegister(XPOWERS_INTSTS3);
        return (uint32_t)(statusRegister[0] << 16) | (uint32_t)(statusRegister[1] << 8) | (uint32_t)(statusRegister[2]);
    }

    void clearIrqStatus(void)
    {
        for (int i = 0; i < XPOWERS_INTSTS_CNT; i++) {
            writeRegister(XPOWERS_INTSTS1 + i, 0xFF);
            statusRegister[i] = 0;
        }
    }

    void enableIRQ(uint32_t opt)
    {
        setInterruptImpl(opt, true);
    }

    void disableIRQ(uint32_t opt)
    {
        setInterruptImpl(opt, false);
    }

    //IRQ STATUS 0
    bool isDropWarningLevel2Irq(void)
    {
        return IS_BIT_SET(statusRegister[0], XPOWERS_WARNING_LEVEL2_IRQ);
    }

    bool isDropWarningLevel1Irq(void)
    {
        return IS_BIT_SET(statusRegister[0], XPOWERS_WARNING_LEVEL1_IRQ);
    }

    bool isGaugeWdtTimeoutIrq()
    {
        return IS_BIT_SET(statusRegister[0], XPOWERS_WDT_TIMEOUT_IRQ);
    }

    bool isBatChargerOverTemperatureIrq(void)
    {
        return IS_BIT_SET(statusRegister[0], XPOWERS_BAT_CHG_OVER_TEMP_IRQ);
    }

    bool isBatChargerUnderTemperatureIrq(void)
    {
        return IS_BIT_SET(statusRegister[0], XPOWERS_BAT_CHG_UNDER_TEMP_IRQ);
    }

    bool isBatWorkOverTemperatureIrq(void)
    {
        return IS_BIT_SET(statusRegister[0], XPOWERS_BAT_NOR_OVER_TEMP_IRQ);
    }

    bool isBatWorkUnderTemperatureIrq(void)
    {
        return IS_BIT_SET(statusRegister[0], XPOWERS_BAT_NOR_UNDER_TEMP_IRQ);
    }

    //IRQ STATUS 1
    bool isVbusInsertIrq(void)
    {
        return IS_BIT_SET(statusRegister[1], XPOWERS_VBUS_INSERT_IRQ >> 8);
    }

    bool isVbusRemoveIrq(void)
    {
        return IS_BIT_SET(statusRegister[1], XPOWERS_VBUS_REMOVE_IRQ >> 8);
    }

    bool isBatInsertIrq(void)
    {
        return IS_BIT_SET(statusRegister[1], XPOWERS_BAT_INSERT_IRQ >> 8);
    }

    bool isBatRemoveIrq(void)
    {
        return IS_BIT_SET(statusRegister[1], XPOWERS_BAT_REMOVE_IRQ >> 8);
    }

    bool isPekeyShortPressIrq(void)
    {
        return IS_BIT_SET(statusRegister[1], XPOWERS_PKEY_SHORT_IRQ >> 8);
    }

    bool isPekeyLongPressIrq(void)
    {
        return IS_BIT_SET(statusRegister[1], XPOWERS_PKEY_LONG_IRQ >> 8);
    }

    bool isPekeyNegativeIrq(void)
    {
        return IS_BIT_SET(statusRegister[1], XPOWERS_PKEY_NEGATIVE_IRQ >> 8);
    }

    bool isPekeyPositiveIrq(void)
    {
        return IS_BIT_SET(statusRegister[1], XPOWERS_PKEY_POSITIVE_IRQ >> 8);
    }

    //IRQ STATUS 2
    bool isWdtExpireIrq(void)
    {
        return IS_BIT_SET(statusRegister[2], XPOWERS_WDT_EXPIRE_IRQ >> 16);
    }

    bool isLdoOverCurrentIrq(void)
    {
        return IS_BIT_SET(statusRegister[2], XPOWERS_LDO_OVER_CURR_IRQ >> 16);
    }

    bool isBatfetOverCurrentIrq(void)
    {
        return IS_BIT_SET(statusRegister[2], XPOWERS_BATFET_OVER_CURR_IRQ >> 16);
    }

    bool isBatChagerDoneIrq(void)
    {
        return IS_BIT_SET(statusRegister[2], XPOWERS_BAT_CHG_DONE_IRQ >> 16);
    }

    bool isBatChagerStartIrq(void)
    {
        return IS_BIT_SET(statusRegister[2], XPOWERS_BAT_CHG_START_IRQ >> 16);
    }

    bool isBatDieOverTemperatureIrq(void)
    {
        return IS_BIT_SET(statusRegister[2], XPOWERS_DIE_OVER_TEMP_IRQ >> 16);
    }

    bool isChagerOverTimeoutIrq(void)
    {
        return IS_BIT_SET(statusRegister[2], XPOWERS_CHAGER_TIMER_IRQ >> 16);
    }

    bool isBatOverVoltageIrq(void)
    {
        return IS_BIT_SET(statusRegister[2], XPOWERS_BAT_OVER_VOL_IRQ  >> 16);
    }


    uint8_t getChipID(void)
    {
        return readRegister(XPOWERS_IC_TYPE);
    }

protected:

    bool initImpl()
    {
        return getChipID() == XPOWERS_CHIP_ID;
    }

    /*
     * Interrupt control functions
     */
    bool setInterruptImpl(uint32_t opts, bool enable)
    {
        int res = 0;
        uint8_t data = 0, value = 0;
        log_i("%s - HEX:0x%lx BIN:", enable ? "ENABLE" : "DISABLE", opts);
        if (opts & 0x0000FF) {
            value = opts & 0xFF;
            data = readRegister(XPOWERS_INTEN1);
            res |= writeRegister(XPOWERS_INTEN1, enable ? (data | value) : (data & (~value)));
        }
        if (opts & 0x00FF00) {
            value = opts >> 8;
            data = readRegister(XPOWERS_INTEN2);
            res |= writeRegister(XPOWERS_INTEN2, enable ? (data | value) : (data & (~value)));
        }
        if (opts & 0xFF0000) {
            value = opts >> 16;
            data = readRegister(XPOWERS_INTEN3);
            res |= writeRegister(XPOWERS_INTEN3, enable ? (data | value) : (data & (~value)));
        }
        return res == 0;
    }

    const char  *getChipNameImpl(void)
    {
        return "AXP2101";
    }

private:
    uint8_t statusRegister[XPOWERS_INTSTS_CNT];
};



