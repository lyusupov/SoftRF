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
 * @file      PowersSY6970.tpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-07-20
 *
 */
#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <math.h>
#endif /*ARDUINO*/
#include "XPowersCommon.tpp"
#include "REG/SY6970Constants.h"
// #include "XPowersLibInterface.hpp"

enum PowersSY6970BusStatus {
    POWERS_SY_NOINPUT,
    POWERS_SY_USB_SDP,
    POWERS_SY_USB_CDP,
    POWERS_SY_USB_DCP,
    POWERS_SY_HVDCP,
    POWERS_SY_UNKONW_ADAPTER,
    POWERS_SY_NO_STANDARD_ADAPTER,
    POWERS_SY_OTG
} ;

enum PowersSY6970ChargeStatus {
    POWERS_SY_NO_CHARGE,
    POWERS_SY_PRE_CHARGE,
    POWERS_SY_FAST_CHARGE,
    POWERS_SY_CHARGE_DONE,
    POWERS_SY_CHARGE_UNKOWN,
} ;

enum PowersSY6970NTCStatus {
    POWERS_SY_BUCK_NTC_NORMAL,
    POWERS_SY_BUCK_NTC_WARM,
    POWERS_SY_BUCK_NTC_COOL,
    POWERS_SY_BUCK_NTC_COLD,
    POWERS_SY_BUCK_NTC_HOT,
    POWERS_SY_BOOST_NTC_NORMAL,
    POWERS_SY_BOOST_NTC_COLD,
    POWERS_SY_BOOST_NTC_HOT,
};

enum SY6970_WDT_Timeout {
    SY6970_WDT_TIMEROUT_40SEC,      //40 Second
    SY6970_WDT_TIMEROUT_80SEC,      //80 Second
    SY6970_WDT_TIMEROUT_160SEC,     //160 Second
} ;

enum ADCMeasure {
    SY6970_ADC_ONE_SHORT,
    SY6970_ADC_CONTINUOUS,
};

enum BoostFreq {
    SY6970_BOOST_FREQ_1500KHZ,
    SY6970_BOOST_FREQ_500KHZ,
};

enum RequestRange {
    RANGE_0_9V,
    RANGE_1_12V,
};

enum FastChargeTimer {
    FAST_CHARGE_TIMER_5H,
    FAST_CHARGE_TIMER_8H,
    FAST_CHARGE_TIMER_12H,
    FAST_CHARGE_TIMER_20H,
};

enum BoostCurrentLimit {
    BOOST_CUR_LIMIT_500MA,
    BOOST_CUR_LIMIT_750MA,
    BOOST_CUR_LIMIT_1200MA,
    BOOST_CUR_LIMIT_1400MA,
    BOOST_CUR_LIMIT_1650MA,
    BOOST_CUR_LIMIT_1875MA,
    BOOST_CUR_LIMIT_2150MA,
    BOOST_CUR_LIMIT_2450MA,
} ;

class PowersSY6970 :
    public XPowersCommon<PowersSY6970> //, public XPowersLibInterface
{
    friend class XPowersCommon<PowersSY6970>;

public:


#if defined(ARDUINO)
    PowersSY6970(TwoWire &w, int sda = SDA, int scl = SCL, uint8_t addr = SY6970_SLAVE_ADDRESS)
    {
        __wire = &w;
        __sda = sda;
        __scl = scl;
        __addr = addr;
    }
#endif

    PowersSY6970(uint8_t addr, iic_fptr_t readRegCallback, iic_fptr_t writeRegCallback)
    {
        thisReadRegCallback = readRegCallback;
        thisWriteRegCallback = writeRegCallback;
        __addr = addr;
    }

    PowersSY6970()
    {
#if defined(ARDUINO)
        __wire = &Wire;
        __sda = SDA;
        __scl = SCL;
#endif
        __addr = SY6970_SLAVE_ADDRESS;
    }

    ~PowersSY6970()
    {
        log_i("~PowersSY6970");
        deinit();
    }

#if defined(ARDUINO)
    bool init(TwoWire &w, int sda = SDA, int scl = SCL, uint8_t addr = SY6970_SLAVE_ADDRESS)
    {
        __wire = &w;
        __sda = sda;
        __scl = scl;
        __addr = addr;
        __irq_mask = 0;
        return begin();
    }
#endif

    uint8_t getChipID()
    {
        int res = readRegister(POWERS_SY6970_REG_14H);
        return (res & 0x03);
    }

    bool init()
    {
        return begin();
    }

    void deinit()
    {
        end();
    }

    ///REG0B
    bool isVbusIn()
    {
        return getBusStatus() != POWERS_SY_NOINPUT;
    }

    bool isOTG()
    {
        return getBusStatus() == POWERS_SY_OTG;
    }

    bool isCharging(void)
    {
        return chargeStatus() != POWERS_SY_NO_CHARGE;
    }

    bool isChargeDone()
    {
        return chargeStatus() != POWERS_SY_CHARGE_DONE;
    }

    bool isBatteryConnect(void) __attribute__((error("Not implemented")))
    {
        //TODO:
        return false;
    }

    bool isPowerGood()
    {
        return getRegisterBit(POWERS_SY6970_REG_0BH, 2);
    }

    bool isEnableCharge()
    {
        return getRegisterBit(POWERS_SY6970_REG_03H, 4);
    }

    void disableCharge()
    {
        __user_disable_charge = true;
        clrRegisterBit(POWERS_SY6970_REG_03H, 4);
    }

    void enableCharge()
    {
        __user_disable_charge = false;
        setRegisterBit(POWERS_SY6970_REG_03H, 4);
    }

    bool isEnableOTG()
    {
        return getRegisterBit(POWERS_SY6970_REG_03H, 5);
    }

    void disableOTG()
    {
        clrRegisterBit(POWERS_SY6970_REG_03H, 5);
        /*
        * After turning on the OTG function, the charging function will
        * be automatically disabled. If the user does not disable the charging
        * function, the charging function will be automatically enabled after
        * turning off the OTG output.
        * */
        if (!__user_disable_charge) {
            setRegisterBit(POWERS_SY6970_REG_03H, 4);
        }
    }

    bool enableOTG()
    {
        if (isVbusIn())
            return false;
        return setRegisterBit(POWERS_SY6970_REG_03H, 5);
    }

    void feedWatchdog()
    {
        setRegisterBit(POWERS_SY6970_REG_03H, 6);
    }

    bool setSysPowerDownVoltage(uint16_t millivolt)
    {
        if (millivolt % POWERS_SY6970_SYS_VOL_STEPS) {
            log_e("Mistake ! The steps is must %u mV", POWERS_SY6970_SYS_VOL_STEPS);
            return false;
        }
        if (millivolt < POWERS_SY6970_SYS_VOFF_VOL_MIN) {
            log_e("Mistake ! SYS minimum output voltage is  %umV", POWERS_SY6970_SYS_VOFF_VOL_MIN);
            return false;
        } else if (millivolt > POWERS_SY6970_SYS_VOFF_VOL_MAX) {
            log_e("Mistake ! SYS maximum output voltage is  %umV", POWERS_SY6970_SYS_VOFF_VOL_MAX);
            return false;
        }
        int val = readRegister(POWERS_SY6970_REG_03H);
        if (val == -1)return false;
        val &= 0xF1;
        val |= (millivolt - POWERS_SY6970_SYS_VOFF_VOL_MIN) / POWERS_SY6970_SYS_VOL_STEPS;
        val <<= 1;
        return 0 ==  writeRegister(POWERS_SY6970_REG_03H, val);

    }

    uint16_t getSysPowerDownVoltage()
    {
        int val = readRegister(POWERS_SY6970_REG_03H);
        if (val == -1)return 0;
        val &= 0x0E;
        val >>= 1;
        return (val * POWERS_SY6970_SYS_VOL_STEPS) + POWERS_SY6970_SYS_VOFF_VOL_MIN;
    }

    // Charging Termination Enable
    void enableChargingTermination()
    {
        setRegisterBit(POWERS_SY6970_REG_07H, 7);
    }

    // Charging Termination Enable
    void disableChargingTermination()
    {
        clrRegisterBit(POWERS_SY6970_REG_07H, 7);
    }

    // Charging Termination Enable
    bool isEnableChargingTermination()
    {
        return getRegisterBit(POWERS_SY6970_REG_07H, 7);
    }

    void disableStatLed()
    {
        setRegisterBit(POWERS_SY6970_REG_07H, 6);
    }

    void enableStatLed()
    {
        clrRegisterBit(POWERS_SY6970_REG_07H, 6);
    }

    bool isEnableStatLed()
    {
        return getRegisterBit(POWERS_SY6970_REG_07H, 6) == false;
    }

    void disableWatchdog()
    {
        int regVal = readRegister(POWERS_SY6970_REG_07H);
        regVal  &= 0xCF;
        writeRegister(POWERS_SY6970_REG_07H, regVal);
    }

    void enableWatchdog(enum SY6970_WDT_Timeout val = SY6970_WDT_TIMEROUT_40SEC )
    {
        int regVal = readRegister(POWERS_SY6970_REG_07H);
        regVal  &= 0xCF;
        switch (val) {
        case SY6970_WDT_TIMEROUT_40SEC:
            writeRegister(POWERS_SY6970_REG_07H, regVal | 0x10);
            break;
        case SY6970_WDT_TIMEROUT_80SEC:
            writeRegister(POWERS_SY6970_REG_07H, regVal | 0x20);
            break;
        case SY6970_WDT_TIMEROUT_160SEC:
            writeRegister(POWERS_SY6970_REG_07H, regVal | 0x30);
            break;
        default:
            break;
        }
    }


    void disableChargingSafetyTimer()
    {
        clrRegisterBit(POWERS_SY6970_REG_07H, 3);
    }

    void enableChargingSafetyTimer()
    {
        setRegisterBit(POWERS_SY6970_REG_07H, 3);
    }

    bool isEnableChargingSafetyTimer()
    {
        return getRegisterBit(POWERS_SY6970_REG_07H, 3);
    }

    void setFastChargeTimer(FastChargeTimer timer)
    {
        int val;
        switch (timer) {
        case FAST_CHARGE_TIMER_5H:
        case FAST_CHARGE_TIMER_8H:
        case FAST_CHARGE_TIMER_12H:
        case FAST_CHARGE_TIMER_20H:
            val = readRegister(POWERS_SY6970_REG_07H);
            if (val == -1)
                return;
            val &= 0xF1;
            val |= (timer << 1);
            writeRegister(POWERS_SY6970_REG_07H, val);
            break;
        default:
            break;
        }
    }

    FastChargeTimer getFastChargeTimer()
    {
        int  val = readRegister(POWERS_SY6970_REG_07H);
        return static_cast<FastChargeTimer>((val & 0x0E) >> 1);
    }

    // Return  Battery Load status
    bool isEnableBatLoad()
    {
        return getRegisterBit(POWERS_SY6970_REG_03H, 7);
    }

    // Battery Load (10mA) Disable
    void disableBatLoad()
    {
        clrRegisterBit(POWERS_SY6970_REG_03H, 7);
    }

    // Battery Load (10mA) Enable
    void enableBatLoad()
    {
        setRegisterBit(POWERS_SY6970_REG_03H, 7);
    }

    PowersSY6970BusStatus getBusStatus()
    {
        int val =  readRegister(POWERS_SY6970_REG_0BH);
        return (PowersSY6970BusStatus)((val >> 5) & 0x07);
    }

    const char *getBusStatusString()
    {
        PowersSY6970BusStatus status = getBusStatus();
        switch (status) {
        case POWERS_SY_NOINPUT:
            return "No input";
        case POWERS_SY_USB_SDP:
            return "USB Host SDP";
        case POWERS_SY_USB_CDP:
            return "USB CDP";
        case POWERS_SY_USB_DCP:
            return "USB DCP";
        case POWERS_SY_HVDCP:
            return "HVDCP";
        case POWERS_SY_UNKONW_ADAPTER:
            return "Unknown Adapter";
        case POWERS_SY_NO_STANDARD_ADAPTER:
            return "Non-Standard Adapter";
        case POWERS_SY_OTG:
            return "OTG";
        default:
            return "Unknown";
        }
    }

    PowersSY6970ChargeStatus chargeStatus()
    {
        int val =  readRegister(POWERS_SY6970_REG_0BH);
        if (val == -1)return POWERS_SY_CHARGE_UNKOWN;
        return (PowersSY6970ChargeStatus)((val >> 3) & 0x03);
    }

    const char *getChargeStatusString()
    {
        PowersSY6970ChargeStatus status = chargeStatus();
        switch (status) {
        case POWERS_SY_NO_CHARGE:
            return "Not Charging";
        case POWERS_SY_PRE_CHARGE:
            return "Pre-charge";
        case POWERS_SY_FAST_CHARGE:
            return "Fast Charging";
        case POWERS_SY_CHARGE_DONE:
            return "Charge Termination Done";
        default:
            return "Unknown";
        }
    }

    PowersSY6970NTCStatus getNTCStatus()
    {
        int val =  readRegister(POWERS_SY6970_REG_0CH);
        return (PowersSY6970NTCStatus)(val & 0x07);
    }

    const char *getNTCStatusString()
    {
        PowersSY6970NTCStatus status = getNTCStatus();
        switch (status) {
        case POWERS_SY_BUCK_NTC_NORMAL:
            return "Buck mode NTC normal";
        case POWERS_SY_BUCK_NTC_WARM:
            return "Buck mode NTC warm";
        case POWERS_SY_BUCK_NTC_COOL:
            return "Buck mode NTC cool";
        case POWERS_SY_BUCK_NTC_COLD:
            return "Buck mode NTC cold";
        case POWERS_SY_BUCK_NTC_HOT:
            return "Buck mode NTC hot";
        case POWERS_SY_BOOST_NTC_NORMAL:
            return "Boost mode NTC hot";
        case POWERS_SY_BOOST_NTC_COLD:
            return "Boost mode NTC cold";
        case POWERS_SY_BOOST_NTC_HOT:
            return "Boost mode NTC hot";
        default:
            return "Unknown";
        }
    }

    bool isWatchdogNormal()
    {
        return getRegisterBit(POWERS_SY6970_REG_0CH, 7) == false;
    }

    bool isBoostNormal()
    {
        return getRegisterBit(POWERS_SY6970_REG_0CH, 6) == false;
    }

    bool isChargeNormal()
    {
        int val = readRegister(POWERS_SY6970_REG_0CH);
        return ((val & 0x30) >> 3) == 0;
    }

    bool isBatteryNormal()
    {
        return getRegisterBit(POWERS_SY6970_REG_0CH, 3) == false;
    }

    bool isNtcNormal()
    {
        int val = readRegister(POWERS_SY6970_REG_0CH);
        return (val & 0x7) == 0;
    }

    bool enableADCMeasure(ADCMeasure mode = SY6970_ADC_CONTINUOUS)
    {
        int val = readRegister(POWERS_SY6970_REG_02H);
        switch (mode) {
        case SY6970_ADC_CONTINUOUS:
            val |= _BV(6);
            break;
        case SY6970_ADC_ONE_SHORT:
        default:
            break;
        }
        val |= _BV(7);
        return writeRegister(POWERS_SY6970_REG_02H, val) != -1;
    }

    bool disableADCMeasure()
    {
        int val = readRegister(POWERS_SY6970_REG_02H);
        if (val == -1) {
            return false;
        }
        val &= (~_BV(7));
        return writeRegister(POWERS_SY6970_REG_02H, val) != 1;
    }

    bool setBoostFreq(BoostFreq freq)
    {
        switch (freq) {
        case SY6970_BOOST_FREQ_500KHZ:
            return setRegisterBit(POWERS_SY6970_REG_02H, 5);
        case SY6970_BOOST_FREQ_1500KHZ:
            return clrRegisterBit(POWERS_SY6970_REG_02H, 5);
        default:
            break;
        }
        return false;
    }

    BoostFreq getBoostFreq()
    {
        return getRegisterBit(POWERS_SY6970_REG_02H, 5) ? SY6970_BOOST_FREQ_500KHZ : SY6970_BOOST_FREQ_1500KHZ;
    }

    void enableInputCurrentLimit()
    {
        setRegisterBit(POWERS_SY6970_REG_02H, 4);
    }

    void disableInputCurrentLimit()
    {
        clrRegisterBit(POWERS_SY6970_REG_02H, 4);
    }

    void enableHVDCP()
    {
        setRegisterBit(POWERS_SY6970_REG_02H, 3);
    }

    void disableHVDCP()
    {
        clrRegisterBit(POWERS_SY6970_REG_02H, 3);
    }

    bool isEnableHVDCP()
    {
        return getRegisterBit(POWERS_SY6970_REG_02H, 3);
    }

    void setHighVoltageRequestedRange(RequestRange range)
    {
        switch (range) {
        case RANGE_0_9V:
            clrRegisterBit(POWERS_SY6970_REG_02H, 2);
            break;
        case RANGE_1_12V:
            setRegisterBit(POWERS_SY6970_REG_02H, 2);
            break;
        default:
            break;
        }
    }

    RequestRange getHighVoltageRequestedRange()
    {
        return getRegisterBit(POWERS_SY6970_REG_02H, 2) ? RANGE_1_12V : RANGE_0_9V;
    }

    // Enable Force DP/DM detection
    void enableDetectionDPDM()
    {
        setRegisterBit(POWERS_SY6970_REG_02H, 1);
    }

    // Disable Force DP/DM detection
    void disableDetectionDPDM()
    {
        clrRegisterBit(POWERS_SY6970_REG_02H, 1);
    }

    // Get Force DP/DM detection
    bool isEnableDetectionDPDM()
    {
        return getRegisterBit(POWERS_SY6970_REG_02H, 1);
    }

    // Enable DPDM detection when BUS is plugged-in.
    void enableAutoDetectionDPDM()
    {
        setRegisterBit(POWERS_SY6970_REG_02H, 0);
    }

    // Disable DPDM detection when BUS is plugged-in.
    void disableAutoDetectionDPDM()
    {
        clrRegisterBit(POWERS_SY6970_REG_02H, 0);
    }

    // Get DPDM detection when BUS is plugged-in.
    bool isEnableAutoDetectionDPDM()
    {
        return getRegisterBit(POWERS_SY6970_REG_02H, 0);
    }

    bool setInputCurrentLimit(uint16_t milliampere)
    {
        if (milliampere % POWERS_SY6970_IN_CURRENT_STEP) {
            log_e("Mistake ! The steps is must %u mA", POWERS_SY6970_IN_CURRENT_STEP);
            return false;
        }
        if (milliampere < POWERS_SY6970_IN_CURRENT_MIN) {
            milliampere = POWERS_SY6970_IN_CURRENT_MIN;
        }
        if (milliampere > POWERS_SY6970_IN_CURRENT_MAX) {
            milliampere = POWERS_SY6970_IN_CURRENT_MAX;
        }
        int val = readRegister(POWERS_SY6970_REG_00H);
        if (val == -1)
            return false;
        val &= 0xC0;
        milliampere = ((milliampere - POWERS_SY6970_IN_CURRENT_MIN) / POWERS_SY6970_IN_CURRENT_STEP);
        val |=  milliampere;
        return writeRegister(POWERS_SY6970_REG_00H, val) != -1;
    }

    uint32_t getInputCurrentLimit()
    {
        int val = readRegister(POWERS_SY6970_REG_00H);
        if (val == -1)
            return false;
        val &= 0x3F;
        return (val * POWERS_SY6970_IN_CURRENT_STEP) + POWERS_SY6970_IN_CURRENT_MIN;
    }

    // USB input path is disabled and can only be reset by disconnecting
    // the power supply, otherwise the power cannot be turned on
    void enableHIZ()
    {
        setRegisterBit(POWERS_SY6970_REG_00H, 7);
    }

    void disableHIZ()
    {
        clrRegisterBit(POWERS_SY6970_REG_00H, 7);
    }

    bool isHIZ()
    {
        return getRegisterBit(POWERS_SY6970_REG_00H, 7);
    }

    void enableCurrentLimitPin()
    {
        setRegisterBit(POWERS_SY6970_REG_00H, 6);
    }

    void disableCurrentLimitPin()
    {
        clrRegisterBit(POWERS_SY6970_REG_00H, 6);
    }

    bool isEnableCurrentLimitPin()
    {
        return getRegisterBit(POWERS_SY6970_REG_00H, 6);
    }

    uint16_t getVbusVoltage()
    {
        if (!isVbusIn()) {
            return 0;
        }
        int val = readRegister(POWERS_SY6970_REG_11H);
        return (POWERS_SY6970_VBUS_MASK_VAL(val) * POWERS_SY6970_VBUS_VOL_STEP) + POWERS_SY6970_VBUS_BASE_VAL;
    }

    uint16_t getBattVoltage()
    {
        int val = readRegister(POWERS_SY6970_REG_0EH);
        val = POWERS_SY6970_VBAT_MASK_VAL(val);
        if (val == 0)return 0;
        return (val * POWERS_SY6970_VBAT_VOL_STEP) + POWERS_SY6970_VBAT_BASE_VAL;
    }

    uint16_t getSystemVoltage()
    {
        int val = readRegister(POWERS_SY6970_REG_0FH);
        return (POWERS_SY6970_VSYS_MASK_VAL(val) * POWERS_SY6970_VSYS_VOL_STEP) + POWERS_SY6970_VSYS_BASE_VAL;
    }

    float getNTCPercentage()
    {
        int val = readRegister(POWERS_SY6970_REG_10H);
        return (POWERS_SY6970_NTC_MASK_VAL(val) * POWERS_SY6970_NTC_VOL_STEP) + POWERS_SY6970_NTC_BASE_VAL;
    }

    uint16_t getChargeCurrent()
    {
        int val = readRegister(POWERS_SY6970_REG_12H);
        if (val == 0)return 0;
        val = (val & 0x7F);
        return (val * POWERS_SY6970_CHG_STEP_VAL) ;

    }

    // Range: 64mA ~ 1024mA ,step:64mA
    bool setPrechargeCurr(uint16_t milliampere)
    {
        if (milliampere % POWERS_SY6970_PRE_CHG_CUR_STEP) {
            log_e("Mistake ! The steps is must %u mA", POWERS_SY6970_PRE_CHG_CUR_STEP);
            return false;
        }
        if (milliampere < POWERS_PRE_CHG_CURRENT_MIN) {
            milliampere = POWERS_PRE_CHG_CURRENT_MIN;
        }
        if (milliampere > POWERS_PRE_CHG_CURRENT_MAX) {
            milliampere = POWERS_PRE_CHG_CURRENT_MAX;
        }
        int val = readRegister(POWERS_SY6970_REG_05H);
        val &= 0x0F;
        milliampere = ((milliampere - POWERS_SY6970_PRE_CHG_CUR_BASE) / POWERS_SY6970_PRE_CHG_CUR_STEP);
        val |=  milliampere << 4;
        return writeRegister(POWERS_SY6970_REG_05H, val) != -1;
    }

    uint16_t getPrechargeCurr(void)
    {
        int val = readRegister(POWERS_SY6970_REG_05H);
        val &= 0xF0;
        val >>= 4;
        return POWERS_SY6970_PRE_CHG_CUR_STEP + (val * POWERS_SY6970_PRE_CHG_CUR_STEP);
    }

    uint16_t getChargerConstantCurr()
    {
        int val = readRegister(POWERS_SY6970_REG_04H);
        val &= 0x7F;
        return val * POWERS_SY6970_FAST_CHG_CUR_STEP;
    }

    // Range:0~5056mA ,step:64mA
    bool setChargerConstantCurr(uint16_t milliampere)
    {
        if (milliampere % POWERS_SY6970_FAST_CHG_CUR_STEP) {
            log_e("Mistake ! The steps is must %u mA", POWERS_SY6970_FAST_CHG_CUR_STEP);
            return false;
        }
        if (milliampere > POWERS_FAST_CHG_CURRENT_MAX) {
            milliampere = POWERS_FAST_CHG_CURRENT_MAX;
        }

        int val = readRegister(POWERS_SY6970_REG_04H);
        val &= 0x80;
        val |= (milliampere / POWERS_SY6970_FAST_CHG_CUR_STEP);
        return  writeRegister(POWERS_SY6970_REG_04H, val) != -1;
    }

    uint16_t getChargeTargetVoltage()
    {
        int val = readRegister(POWERS_SY6970_REG_06H);
        val = (val & 0xFC) >> 2;
        if (val > 0x30) {
            return POWERS_FAST_CHG_VOL_MAX;
        }
        return val * POWERS_SY6970_CHG_VOL_STEP + POWERS_SY6970_CHG_VOL_BASE;
    }

    // Range:3840 ~ 4608mV ,step:16 mV
    bool setChargeTargetVoltage(uint16_t millivolt)
    {
        if (millivolt % POWERS_SY6970_CHG_VOL_STEP) {
            log_e("Mistake ! The steps is must %u mV", POWERS_SY6970_CHG_VOL_STEP);
            return false;
        }
        if (millivolt < POWERS_FAST_CHG_VOL_MIN) {
            millivolt = POWERS_FAST_CHG_VOL_MIN;
        }
        if (millivolt > POWERS_FAST_CHG_VOL_MAX) {
            millivolt = POWERS_FAST_CHG_VOL_MAX;
        }
        int val = readRegister(POWERS_SY6970_REG_06H);
        val &= 0x03;
        val |= (((millivolt - POWERS_SY6970_CHG_VOL_BASE) / POWERS_SY6970_CHG_VOL_STEP) << 2);
        return  writeRegister(POWERS_SY6970_REG_06H, val) != -1;
    }

    // Turn off the battery power supply path. It can only be turned off when the
    // battery is powered. It cannot be turned off when USB is connected.
    void shutdown()
    {
        disableBATFET();
    }

    // Close battery power path
    void disableBATFET()
    {
        setRegisterBit(POWERS_SY6970_REG_09H, 5);       //Force BATFET Off : BATFET_DIS
    }

    // Enable battery power path
    void enableBATFET()
    {
        clrRegisterBit(POWERS_SY6970_REG_09H, 5);       //Force BATFET Off : BATFET_DIS
    }

    // Boost Mode Voltage Regulation: 4550mV ~ 5510mV
    bool setBoostVoltage(uint16_t millivolt)
    {
        if (millivolt % POWERS_SY6970_BOOTS_VOL_STEP) {
            log_e("Mistake ! The steps is must %u mV", POWERS_SY6970_BOOTS_VOL_STEP);
            return false;
        }
        if (millivolt < POWERS_SY6970_BOOST_VOL_MIN) {
            millivolt = POWERS_SY6970_BOOST_VOL_MIN;
        }
        if (millivolt > POWERS_SY6970_BOOST_VOL_MAX) {
            millivolt = POWERS_SY6970_BOOST_VOL_MAX;
        }
        int val = readRegister(POWERS_SY6970_REG_0AH);
        val &= 0x03;
        val |= (((millivolt - POWERS_SY6970_BOOTS_VOL_BASE) / POWERS_SY6970_BOOTS_VOL_STEP) << 2);
        return  writeRegister(POWERS_SY6970_REG_0AH, val) != -1;
    }

    // Boost Current Limit: 500mA ~2450mA
    bool setBoostCurrentLimit(BoostCurrentLimit milliampere)
    {
        if (milliampere > BOOST_CUR_LIMIT_2450MA) {
            return false;
        }
        int val = readRegister(POWERS_SY6970_REG_0AH);
        val &= 0x03;
        val |= milliampere;
        return  writeRegister(POWERS_SY6970_REG_0AH, val) != -1;
    }

    uint64_t getIrqStatus(void)
    {
        int val = readRegister(POWERS_SY6970_REG_0CH);
        if (val == -1) {
            return 0;
        }
        __irq_mask = val;

        val = readRegister(POWERS_SY6970_REG_0BH);
        if (val == -1) {
            return 0;
        }
        __irq_mask |= ((val >> 7) & 0x1) << 8;
        return __irq_mask;
    }

    void getReadOnlyRegisterValue()
    {
#ifdef ARDUINO //debug ..
        static uint8_t last_val[8] = {0};
        const uint8_t regis[] = {
            POWERS_SY6970_REG_0BH,
            POWERS_SY6970_REG_0CH,
            // POWERS_SY6970_REG_0EH, //BATTERY VOLTAGE
            // POWERS_SY6970_REG_0FH, //SYSTEM VOLTAGE
            // POWERS_SY6970_REG_10H, //NTC PERCENTAGE
            // POWERS_SY6970_REG_11H, //VBUS VOLTAGE
            POWERS_SY6970_REG_12H,
            POWERS_SY6970_REG_13H
        };
        Serial.println();
        Serial.println("-------------------------");
        for (int i = 0; i < sizeof(regis) / sizeof(regis[0]); ++i) {
            int val = readRegister(regis[i]);
            if (val == -1) {
                continue;
            }
            if (last_val[i] != val) {
                Serial.printf("\t---> REG%02X Prev:0x%02X ", regis[i], last_val[i]);
                Serial.print(" BIN:"); Serial.print(last_val[i], BIN);
                Serial.printf(" Curr: 0x%02X", val);
                Serial.print(" BIN:"); Serial.println(val, BIN);
                last_val[i] = val;
            }
            Serial.printf("\tREG%02XH:0x%X BIN:0b", regis[i], val);
            Serial.println(val, BIN);
        }
        Serial.println("-------------------------");
#endif
    }


    bool isWatchdogFault()
    {
        return POWERS_SY6970_IRQ_WTD_FAULT(__irq_mask);
    }

    bool isBoostFault()
    {
        return POWERS_SY6970_IRQ_BOOST_FAULT(__irq_mask);
    }

    bool isChargeFault()
    {
        return POWERS_SY6970_IRQ_CHG_FAULT(__irq_mask);
    }

    bool isBatteryFault()
    {
        return POWERS_SY6970_IRQ_BAT_FAULT(__irq_mask);
    }

    bool isNTCFault()
    {
        return POWERS_SY6970_IRQ_NTC_FAULT(__irq_mask);
    }

    // True: In VSYSMIN regulation (BAT<VSYSMIN)
    // False: Not in VSYSMIN regulation (BAT>VSYSMIN)
    bool isVsysLowVoltageWarning()
    {
        uint8_t tmp = __irq_mask >> 8;
        return (bool)(tmp & 0x01);
    }

    bool setVinDpmThreshold(uint16_t millivolt)
    {
        if (millivolt % POWERS_SY6970_VINDPM_VOL_STEPS) {
            log_e("Mistake ! The steps is must %u mV", POWERS_SY6970_VINDPM_VOL_STEPS);
            return false;
        }
        if (millivolt < POWERS_SY6970_VINDPM_VOL_MIN) {
            millivolt = POWERS_SY6970_VINDPM_VOL_MIN;
        }
        if (millivolt > POWERS_SY6970_VINDPM_VOL_MAX) {
            millivolt = POWERS_SY6970_VINDPM_VOL_MAX;
        }
        int val = readRegister(POWERS_SY6970_REG_0DH);
        val &= 0x80;
        val |= (((millivolt - POWERS_SY6970_VINDPM_VOL_BASE) / POWERS_SY6970_VINDPM_VOL_STEPS));
        return  writeRegister(POWERS_SY6970_REG_0DH, val) != -1;
    }


private:

    bool initImpl()
    {
        __user_disable_charge = false;

        uint8_t rev = getChipID();
        if (rev != SY6970_DEV_REV && rev != BQ25896_DEV_REV) {
            return false;
        }
        // Set the minimum operating voltage. Below this voltage, the PMU will protect
        setSysPowerDownVoltage(3300);

        //Default disable Watchdog
        disableWatchdog();

        return true;
    }

    bool __user_disable_charge;
    uint32_t __irq_mask;
};



typedef PowersSY6970 XPowersPPM;
