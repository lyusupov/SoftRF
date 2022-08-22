/**
  ******************************************************************************
  * @file    LowPower.c
  * @author  Frederic Pillon
  * @brief   Provides a Low Power interface
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2021, STMicroelectronics
  * All rights reserved.
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#include "Arduino.h"
#include "low_power.h"
#include "stm32yyxx_ll_pwr.h"

#if defined(STM32_CORE_VERSION) && (STM32_CORE_VERSION  > 0x01090000) \
 && defined(HAL_PWR_MODULE_ENABLED) && !defined(HAL_PWR_MODULE_ONLY)

#if defined(HAL_UART_MODULE_ENABLED) && !defined(HAL_UART_MODULE_ONLY) \
  && (defined(UART_IT_WUF) || defined(LPUART1_BASE))
  #define UART_WKUP_SUPPORT
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if defined(UART_WKUP_SUPPORT)
/* Save UART handler for callback */
static UART_HandleTypeDef *WakeUpUart = NULL;
#endif
/* Save callback pointer */
static void (*WakeUpUartCb)(void) = NULL;

#if defined(PWR_FLAG_WUF)
#define PWR_FLAG_WU PWR_FLAG_WUF
#elif defined(PWR_WAKEUP_ALL_FLAG)
#define PWR_FLAG_WU PWR_WAKEUP_ALL_FLAG
#endif
#if defined(PWR_FLAG_SBF)
#define PWR_FLAG_SB PWR_FLAG_SBF
#endif

/**
  * @brief  Initialize low power mode
  * @param  None
  * @retval None
  */
void LowPower_init()
{
#if defined(__HAL_RCC_PWR_CLK_ENABLE)
  /* Enable Power Clock */
  __HAL_RCC_PWR_CLK_ENABLE();
#endif
  /* Allow access to Backup domain */
  HAL_PWR_EnableBkUpAccess();

#ifdef __HAL_RCC_WAKEUPSTOP_CLK_CONFIG
  /* Ensure that HSI is wake-up system clock */
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);
#endif
  /* Check if the system was resumed from StandBy mode */
  if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET) {
    /* Clear Standby flag */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
  }

  /* Clear all related wakeup flags */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
}

/**
  * @brief  Configure a pin as wakeup source if compatible.
  * @param  pin: pin to configure
  * @param  mode: pin mode (edge or state). The configuration have to be compatible.
  * @retval None
  */
void LowPower_EnableWakeUpPin(uint32_t pin, uint32_t mode)
{
#if !defined(PWR_WAKEUP_PIN1_HIGH)
  UNUSED(mode);
#endif
  uint32_t wkup_pin = 0;
  PinName p = digitalPinToPinName(pin);
  if (p != NC) {
#ifdef PWR_WAKEUP_PIN1
    if ((p == SYS_WKUP1)
#ifdef PWR_WAKEUP_PIN1_1
        || (p == SYS_WKUP1_1)
#endif
#ifdef PWR_WAKEUP_PIN1_2
        || (p == SYS_WKUP1_2)
#endif
       ) {
      wkup_pin = PWR_WAKEUP_PIN1;
#ifdef PWR_WAKEUP_PIN1_HIGH
      if (mode != RISING) {
        wkup_pin = PWR_WAKEUP_PIN1_LOW;
      }
#endif
    }
#endif /* PWR_WAKEUP_PIN1 */
#ifdef PWR_WAKEUP_PIN2
    if ((p == SYS_WKUP2)
#ifdef PWR_WAKEUP_PIN2_1
        || (p == SYS_WKUP2_1)
#endif
#ifdef PWR_WAKEUP_PIN2_2
        || (p == SYS_WKUP2_2)
#endif
       ) {
      wkup_pin = PWR_WAKEUP_PIN2;
#ifdef PWR_WAKEUP_PIN2_HIGH
      if (mode != RISING) {
        wkup_pin = PWR_WAKEUP_PIN2_LOW;
      }
#endif
    }
#endif /* PWR_WAKEUP_PIN2 */
#ifdef PWR_WAKEUP_PIN3
    if ((p == SYS_WKUP3)
#ifdef PWR_WAKEUP_PIN3_1
        || (p == SYS_WKUP3_1)
#endif
#ifdef PWR_WAKEUP_PIN3_2
        || (p == SYS_WKUP3_2)
#endif
       ) {
      wkup_pin = PWR_WAKEUP_PIN3;
#ifdef PWR_WAKEUP_PIN3_HIGH
      if (mode != RISING) {
        wkup_pin = PWR_WAKEUP_PIN3_LOW;
      }
#endif
    }
#endif /* PWR_WAKEUP_PIN3 */
#ifdef PWR_WAKEUP_PIN4
    if ((p == SYS_WKUP4)
#ifdef PWR_WAKEUP_PIN4_1
        || (p == SYS_WKUP4_1)
#endif
#ifdef PWR_WAKEUP_PIN4_2
        || (p == SYS_WKUP4_2)
#endif
       ) {
      wkup_pin = PWR_WAKEUP_PIN4;
#ifdef PWR_WAKEUP_PIN4_HIGH
      if (mode != RISING) {
        wkup_pin = PWR_WAKEUP_PIN4_LOW;
      }
#endif
    }
#endif /* PWR_WAKEUP_PIN4 */
#ifdef PWR_WAKEUP_PIN5
    if ((p == SYS_WKUP5)
#ifdef PWR_WAKEUP_PIN5_1
        || (p == SYS_WKUP5_1)
#endif
#ifdef PWR_WAKEUP_PIN5_2
        || (p == SYS_WKUP5_2)
#endif
       ) {
      wkup_pin = PWR_WAKEUP_PIN5;
#ifdef PWR_WAKEUP_PIN5_HIGH
      if (mode != RISING) {
        wkup_pin = PWR_WAKEUP_PIN5_LOW;
      }
#endif
    }
#endif /* PWR_WAKEUP_PIN5 */
#ifdef PWR_WAKEUP_PIN6
    if ((p == SYS_WKUP6)
#ifdef PWR_WAKEUP_PIN6_1
        || (p == SYS_WKUP6_1)
#endif
#ifdef PWR_WAKEUP_PIN6_2
        || (p == SYS_WKUP6_2)
#endif
       ) {
      wkup_pin = PWR_WAKEUP_PIN6;
#ifdef PWR_WAKEUP_PIN6_HIGH
      if (mode != RISING) {
        wkup_pin = PWR_WAKEUP_PIN6_LOW;
      }
#endif
    }
#endif /* PWR_WAKEUP_PIN6 */
#ifdef PWR_WAKEUP_PIN7
    if ((p == SYS_WKUP7)
#ifdef PWR_WAKEUP_PIN7_1
        || (p == SYS_WKUP7_1)
#endif
#ifdef PWR_WAKEUP_PIN7_2
        || (p == SYS_WKUP7_2)
#endif
       ) {
      wkup_pin = PWR_WAKEUP_PIN7;
    }
#endif /* PWR_WAKEUP_PIN7 */
#ifdef PWR_WAKEUP_PIN8
    if ((p == SYS_WKUP8)
#ifdef PWR_WAKEUP_PIN8_1
        || (p == SYS_WKUP8_1)
#endif
#ifdef PWR_WAKEUP_PIN8_2
        || (p == SYS_WKUP8_2)
#endif
       ) {
      wkup_pin = PWR_WAKEUP_PIN8;
    }
#endif /* PWR_WAKEUP_PIN8 */
    if (IS_PWR_WAKEUP_PIN(wkup_pin)) {
      HAL_PWR_EnableWakeUpPin(wkup_pin);
    }
  }
}

/**
  * @brief  Enable the sleep mode.
  * @param  None
  * @retval None
  */
void LowPower_sleep(uint32_t regulator)
{
  /*
   * Suspend Tick increment to prevent wakeup by Systick interrupt.
   * Otherwise the Systick interrupt will wake up the device within
   * 1ms (HAL time base)
   */
  HAL_SuspendTick();

  /* Enter Sleep Mode , wake up is done once User push-button is pressed */
  HAL_PWR_EnterSLEEPMode(regulator, PWR_SLEEPENTRY_WFI);

  /* Resume Tick interrupt if disabled prior to SLEEP mode entry */
  HAL_ResumeTick();

  if (WakeUpUartCb != NULL) {
    WakeUpUartCb();
  }
}

/**
  * @brief  Enable the stop mode.
  * @param  obj : pointer to serial_t structure
  * @retval None
  */
void LowPower_stop(serial_t *obj)
{
  __disable_irq();

#if defined(UART_WKUP_SUPPORT)
  if (WakeUpUart != NULL) {
    HAL_UARTEx_EnableStopMode(WakeUpUart);
  }
#endif

#if defined(PWR_CR_ULP)
  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();
#endif
#if defined(PWR_CR1_ULPMEN) || defined(PWR_CR3_ULPMEN)
  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPowerMode();
#endif
#if defined(PWR_CR_FWU)
  /* Enable the fast wake up from Ultra low power mode */
  HAL_PWREx_EnableFastWakeUp();
#endif
#ifdef __HAL_RCC_WAKEUPSTOP_CLK_CONFIG
  /* Select MSI or HSI as system clock source after Wake Up from Stop mode */
#ifdef RCC_STOP_WAKEUPCLOCK_MSI
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);
#else
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);
#endif
#endif

  /* Enter Stop mode */
#if defined(UART_WKUP_SUPPORT) && (defined(PWR_CPUCR_RETDS_CD) \
 || defined(PWR_CR1_LPMS_STOP2) || defined(PWR_LOWPOWERMODE_STOP2) \
 || defined(LL_PWR_STOP2_MODE))
  if ((WakeUpUart == NULL)
      || (WakeUpUart->Instance == (USART_TypeDef *)LPUART1_BASE)
#ifdef LPUART2_BASE
      || (WakeUpUart->Instance == (USART_TypeDef *)LPUART2_BASE)
#endif
     ) {
#if defined(PWR_CR1_RRSTP)
    // STM32L4+ must keep SRAM3 content when entering STOP2 lowpower mode
    HAL_PWREx_EnableSRAM3ContentRetention();
#endif /* PWR_CR1_RRSTP */
    // STM32L4xx supports STOP2 mode which halves consumption
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
  } else
#endif
  {
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  }

  /* Exit Stop mode reset clocks */
  SystemClock_ConfigFromStop();
#if defined(UART_WKUP_SUPPORT)
  if (WakeUpUart != NULL) {
    /* In case of WakeUp from UART, reset its clock source to HSI */
    uart_config_lowpower(obj);
    HAL_UARTEx_DisableStopMode(WakeUpUart);
  }
#else
  UNUSED(obj);
#endif
  __enable_irq();

  HAL_Delay(10);

  if (WakeUpUartCb != NULL) {
    WakeUpUartCb();
  }
}

/**
  * @brief  Enable the standby mode. The board reset when leaves this mode.
  * @param  None
  * @retval None
  */
void LowPower_standby()
{
  __disable_irq();

#if defined(PWR_CR_ULP)
  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();
#endif
#if defined(PWR_CR_FWU)
  /* Enable the fast wake up from Ultra low power mode */
  HAL_PWREx_EnableFastWakeUp();
#endif

  HAL_PWR_EnterSTANDBYMode();
}

/**
  * @brief  Enable the shutdown mode.The board reset when leaves this mode.
  *         If shutdown mode not available, use standby mode instead.
  * @param  None
  * @retval None
  */
void LowPower_shutdown()
{
  __disable_irq();
#if defined(PWR_CR1_LPMS)
  /* LSE must be on to use shutdown mode */
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == SET) {
    HAL_PWREx_EnterSHUTDOWNMode();
  } else
#endif
  {
    LowPower_standby();
  }
}

/**
  * @brief  Configure the UART as a wakeup source. A callback can be called when
  *         the chip leaves the low power mode. See board datasheet to check
  *         with which low power mode the UART is compatible.
  * Warning This function will change UART clock source to HSI
  * @param  serial: pointer to serial
  * @param  FuncPtr: pointer to callback
  * @retval None
  */
void LowPower_EnableWakeUpUart(serial_t *serial, void (*FuncPtr)(void))
{
#if defined(UART_WKUP_SUPPORT)
#ifdef IS_UART_WAKEUP_SELECTION
  UART_WakeUpTypeDef WakeUpSelection;
  if (serial == NULL) {
    return;
  }
  /* Save Uart handler and Serial object */
  WakeUpUart = &(serial->handle);

  /* make sure that no UART transfer is on-going */
  while (__HAL_UART_GET_FLAG(WakeUpUart, USART_ISR_BUSY) == SET);
  /* make sure that UART is ready to receive
   * (test carried out again later in HAL_UARTEx_StopModeWakeUpSourceConfig) */
  while (__HAL_UART_GET_FLAG(WakeUpUart, USART_ISR_REACK) == RESET);

  /* set the wake-up event:
   * specify wake-up on RXNE flag
   */
  WakeUpSelection.WakeUpEvent = UART_WAKEUP_ON_READDATA_NONEMPTY;
  HAL_UARTEx_StopModeWakeUpSourceConfig(WakeUpUart, WakeUpSelection);
#endif
#if defined(UART_IT_WUF)
  /* Enable the UART Wake UP from STOPx mode Interrupt */
  __HAL_UART_ENABLE_IT(WakeUpUart, UART_IT_WUF);
#endif
#else
  UNUSED(serial);
#endif
  /* Save callback */
  WakeUpUartCb = FuncPtr;
}

/**
  * @brief  Configures system clock and system IP clocks after wake-up from STOP
  * @note   Weaked function which can be redefined by user at the sketch level.
  *         By default, call 'SystemClock_Config()'.
  * @param  None
  * @retval None
  */
WEAK void SystemClock_ConfigFromStop(void)
{
  configIPClock();
  SystemClock_Config();
}

#ifdef __cplusplus
}
#endif

#endif /* HAL_PWR_MODULE_ENABLED  && !HAL_PWR_MODULE_ONLY */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
