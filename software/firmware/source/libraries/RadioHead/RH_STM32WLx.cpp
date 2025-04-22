// RH_STM32WLx.cpp
//
// Copyright (C) 2023 Mike McCauley
// $Id: RH_STM32WLx.cpp,v 1.27 2020/07/05 08:52:21 mikem Exp $
//

#include <RH_STM32WLx.h>
#include <RHSUBGHZSPI.h>

// Are we building for a suitable STM processor
#if defined(SUBGHZSPI_BASE)

// On this chip, the radio is connected by a dedicated, internal SPI interface  called SubGhz,
// and accessed via our RHSUBGHZSPI
static RHSUBGHZSPI subghzspi;

// Set up radio management pins for various modes for your specific radio wiring
// YOU MUST DO THIS FOR YOUR PARTICULAR RADIO. FAILURE TO SET
// THIS UP PROPERLY WILL CERTAINLY PREVENT TRANSMISSION AT FULL
// POWER AND POSSIBLY RECEPTION TOO
#if (RH_PLATFORM == RH_PLATFORM_ARDUINO && defined(ARDUINO_LORA_E5_MINI))
    // EG: Seeed WiO-E5 mini
    // These LoRa-E5 modules have an RF switch but only use the high power PA
    // Only 2 control pins are used

    static RH_SX126x::RadioPinConfig radioPinConfig = {
	.pinNumber = {PA4, PA5, RH_INVALID_PIN},
	.configState = {
	    {RH_SX126x::RadioPinConfigMode_IDLE,          LOW,  LOW},
	    {RH_SX126x::RadioPinConfigMode_RX,            HIGH, LOW},
	    {RH_SX126x::RadioPinConfigMode_TX_HIGH_POWER, LOW,  HIGH}, // low power not supported on this module
	    {RH_SX126x::RadioPinConfigMode_EOT},
	}
    };
#elif (RH_PLATFORM == RH_PLATFORM_ARDUINO && defined(ARDUINO_NUCLEO_WL55JC1))
    // These devices have an RF switch and low and high power PAs
    // 3 control pins are used
	.pinNumber = {PC3, PC4, PC5},
	.configState = {
	    {RH_SX126x::RadioPinConfigMode_IDLE,          LOW,  LOW,   LOW},
	    {RH_SX126x::RadioPinConfigMode_RX,            HIGH, HIGH,  LOW},
	    {RH_SX126x::RadioPinConfigMode_TX_LOW_POWER,  HIGH, HIGH,  HIGH},
	    {RH_SX126x::RadioPinConfigMode_TX_HIGH_POWER, HIGH, LOW,   HIGH},
	    {RH_SX126x::RadioPinConfigMode_EOT},
	}
    };
#else
#warning RH_STM32WLx Do not know how to set radio control pins for this processor
#endif

RH_STM32WLx::RH_STM32WLx()
    :
    RH_SX126x(RH_INVALID_PIN, RH_INVALID_PIN, RH_INVALID_PIN, RH_INVALID_PIN, subghzspi, &radioPinConfig)
{
    SubGhz.setResetActive(true);
    delay(2);
    SubGhz.setResetActive(false);
    delay(200); // After reset: else can get runt transmission during startup FIXME: wait until ready? No doesnt work
}

// Override the init() function becaue we need to adjust some things afterwards to suit this radio module
bool RH_STM32WLx::init()
{
    bool ret = RH_SX126x::init(); // In STDBY mode after this
    
    setDIO2AsRfSwitchCtrl(false); // Dont use DIO2 as RF control for Wio-E5, which uses separate pins  FIXME: not correct for NICERF
    setTCXO(1.7, 5000); // MUST do this (in standby mode) else get no output. volts, us
    return ret;
}

bool RH_STM32WLx::setupInterruptHandler()
{
    // It is necessary to use the SUBGHZ SPI radio interface and its built-in interrupt support
    // BUT: only the first one can be used if RHSUBGHZSPI is in use
    // This is is a modern lambda to attach a C++ function as an ordinary C callback
    // Hopefuly this will compile successfully everywhere. If not let us know and we will
    // change it to something more backwards compatible
    SubGhz.attachInterrupt([this]() {
	handleInterrupt();
    });

    return true;
}


#endif
