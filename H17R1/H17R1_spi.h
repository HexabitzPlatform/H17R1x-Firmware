/** 
  ******************************************************************************
  * @file    x_nucleo_ihm03a1_stm32l0xx.h
  * @author  IPC Rennes
  * @version V1.6.0
  * @date    January 25th, 2016
  * @brief   Header for BSP driver for x-nucleo-ihm03a1 Nucleo extension board 
  *  (based on powerSTEP01)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef G0_MCU_HARDWARE_H
#define G0_MCU_HARDWARE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

   
/******************************************************************************/
/* USE_STM32G0XX                                                     */
/******************************************************************************/
#define FLAG_Pin GPIO_PIN_11
#define FLAG_GPIO_Port GPIOA
#define FLAG_EXTI_IRQn EXTI4_15_IRQn
#define BUSY_Pin GPIO_PIN_12
#define BUSY_GPIO_Port GPIOA
#define BUSY_EXTI_IRQn EXTI4_15_IRQn
#define RESET_Pin GPIO_PIN_6
#define RESET_GPIO_Port GPIOB
#define STCK_Pin GPIO_PIN_7
#define STCK_GPIO_Port GPIOB
void MX_SPI1_Init(void);


#ifdef __cplusplus
}
#endif

#endif /* X_NUCLEO_IHM03A1_STM32L0XX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
