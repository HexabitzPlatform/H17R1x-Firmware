/*
 BitzOS (BOS) V0.3.5 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H17R1_spi.c
 Description   : BSP driver for x-nucleo-ihm03a1 Nucleo extension board
  *  (based on powerSTEP01)
  *
 */

/* Includes ------------------------------------------------------------------*/

#include "H17R1_spi.h"


/// Timer Prescaler
#define TIMER_PRESCALER (1024)

/// SPI Maximum Timeout values for flags waiting loops
#define SPIx_TIMEOUT_MAX                      ((uint32_t)0x1000)

/**
  * @}
  */

SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim4;

/** @defgroup G0_MCU_Hardware_Private_Function_Prototypes Board Private Function Prototypes
  * @{
  */
extern void StartMilliDelay(uint16_t Delay);
void Powerstep01_Board_Delay(uint32_t delay);         //Delay of the requested number of milliseconds
void Powerstep01_Board_EnableIrq(void);               //Disable Irq
void Powerstep01_Board_DisableIrq(void);              //Enable Irq
void Powerstep01_Board_StartStepClock(uint16_t newFreq); //Start the step clock by using the given frequency
void Powerstep01_Board_StopStepClock(void);              //Stop the PWM uses for the step clock
void Powerstep01_Board_ReleaseReset(void);               //Reset the powerSTEP01 reset pin
void Powerstep01_Board_Reset(void);                      //Set the powerSTEP01 reset pin
uint8_t Powerstep01_Board_SpiWriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte, uint8_t nbDevices); //Write bytes to the powerSTEP01s via SPI
uint32_t Powerstep01_Board_BUSY_PIN_GetState(void); //Returns the BUSY pin state
uint32_t Powerstep01_Board_FLAG_PIN_GetState(void); //Returns the FLAG pin state

/**
  * @}
  */


/** @defgroup  IHM03A1_Board_Private_Functions Board Private Functions
  * @{
  */

/******************************************************//**
 * @brief This function provides an accurate delay in milliseconds
 * @param[in] delay  time length in milliseconds
  * @retval None
 **********************************************************/
void Powerstep01_Board_Delay(uint32_t delay)
{

   StartMilliDelay(1);

}

/******************************************************//**
 * @brief This function disable the interruptions
 * @retval None
 **********************************************************/
void Powerstep01_Board_DisableIrq(void)
{
  __disable_irq();
}

/******************************************************//**
 * @brief This function enable the interruptions
 * @retval None
 **********************************************************/
void Powerstep01_Board_EnableIrq(void)
{
  __enable_irq();
}

/******************************************************//**
 * @brief  Start the step clock by using the given frequency
 * @param[in] newFreq in Hz of the step clock
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void Powerstep01_Board_StartStepClock(uint16_t newFreq)
{
  uint32_t sysFreq = HAL_RCC_GetSysClockFreq();
  uint32_t period = (sysFreq/ (TIMER_PRESCALER * newFreq)) - 1;
  __HAL_TIM_SetAutoreload(&htim4, period);
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, period >> 1);
  HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_2);
}

/******************************************************//**
 * @brief  Stops the PWM uses for the step clock
 * @retval None
 **********************************************************/
void Powerstep01_Board_StopStepClock(void)
{
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
}

/******************************************************//**
 * @brief  Releases the powerSTEP01 reset (pin set to High) of all devices
 * @retval None
 **********************************************************/
void Powerstep01_Board_ReleaseReset(void)
{
  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
}

/******************************************************//**
 * @brief  Resets the powerSTEP01 (reset pin set to low) of all devices
 * @retval None
 **********************************************************/
void Powerstep01_Board_Reset(void)
{
  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);
}

/******************************************************//**
 * @brief  Write and read SPI byte to the powerSTEP01
 * @param[in] pByteToTransmit pointer to the byte to transmit
 * @param[in] pReceivedByte pointer to the received byte
 * @param[in] nbDevices Number of device in the SPI chain
 * @retval HAL_OK if SPI transaction is OK, HAL_KO else
 **********************************************************/
uint8_t Powerstep01_Board_SpiWriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte, uint8_t nbDevices)
{
  HAL_StatusTypeDef status;
  uint32_t i;
//  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
  for (i = 0; i < nbDevices; i++)
  {
    status = HAL_SPI_TransmitReceive(&hspi1, pByteToTransmit, pReceivedByte, 1, SPIx_TIMEOUT_MAX);
    if (status != HAL_OK)
    {
      break;
    }
    pByteToTransmit++;
    pReceivedByte++;
  }
//  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  return (uint8_t) status;
}

/******************************************************//**
 * @brief  Returns the BUSY pin state.
 * @retval The BUSY pin value.
 **********************************************************/
uint32_t Powerstep01_Board_BUSY_PIN_GetState(void)
{
  return HAL_GPIO_ReadPin(BUSY_GPIO_Port, BUSY_Pin);
}

/******************************************************//**
 * @brief  Returns the FLAG pin state.
 * @retval The FLAG pin value.
 **********************************************************/
uint32_t Powerstep01_Board_FLAG_PIN_GetState(void)
{
  return HAL_GPIO_ReadPin(FLAG_GPIO_Port, FLAG_Pin);
}

void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
		Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}
/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA15     ------> SPI1_NSS
    PB3     ------> SPI1_SCK
    PB4     ------> SPI1_MISO
    PB5     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }

}
/**
* @brief SPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA15     ------> SPI1_NSS
    PB3     ------> SPI1_SCK
    PB4     ------> SPI1_MISO
    PB5     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }

}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
