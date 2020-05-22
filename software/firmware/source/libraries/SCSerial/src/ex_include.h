#ifndef EX_INCLUDE_H
#define EX_INCLUDE_H


#undef DEVICE_FAMILY_PATH
#ifdef DEVICE_FAMILY
    #define DEVICE_FAMILY_PATH(x) <ti/devices/DEVICE_FAMILY/x>
#else
    #define DEVICE_FAMILY_PATH(x) <x>
#endif


#include DEVICE_FAMILY_PATH(inc/hw_types.h)
#include DEVICE_FAMILY_PATH(inc/hw_memmap.h)
#include DEVICE_FAMILY_PATH(driverlib/gpio.h)
#include DEVICE_FAMILY_PATH(driverlib/cpu.h)
#include DEVICE_FAMILY_PATH(driverlib/ioc.h)
#include DEVICE_FAMILY_PATH(driverlib/prcm.h)
#include DEVICE_FAMILY_PATH(driverlib/uart.h)
#include DEVICE_FAMILY_PATH(driverlib/interrupt.h)
#include DEVICE_FAMILY_PATH(driverlib/aon_rtc.h)
#include DEVICE_FAMILY_PATH(driverlib/sys_ctrl.h)
#include DEVICE_FAMILY_PATH(driverlib/aux_adc.h)
#include DEVICE_FAMILY_PATH(inc/hw_nvic.h)
#include DEVICE_FAMILY_PATH(inc/hw_aux_sce.h)
#include DEVICE_FAMILY_PATH(inc/hw_ioc.h)
#include DEVICE_FAMILY_PATH(inc/hw_gpio.h)
#include DEVICE_FAMILY_PATH(inc/hw_uart.h)


#endif
