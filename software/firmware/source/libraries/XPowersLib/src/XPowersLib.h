/**
 * @file      XPowersLib.h
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2022
 * @date      2022-05-07
 *
 */

#ifndef __XPOWERSLIB_H__
#define __XPOWERSLIB_H__


#if defined(XPOWERS_CHIP_AXP192)
#include "XPowersAXP192.tpp"
typedef XPowersAXP192 XPowersPMU;
#elif defined(XPOWERS_CHIP_AXP202)
// #include "XPowersAXP202.tpp"
// typedef XPowersAXP202 XPowersPMU;
#error "Not currently supported!"
#elif defined(XPOWERS_CHIP_AXP2102)
#include "XPowersAXP2101.tpp"
typedef XPowersAXP2101 XPowersPMU;
#else
#error "Please define power management chip model"
#endif







#endif /*__XPOWERSLIB_H__*/