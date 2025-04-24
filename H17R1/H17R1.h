/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved
 
 File Name     : H17R1.h
 Description   : Header file for module H17R1.
 (Description_of_module)

 (Description of Special module peripheral configuration):
 >>
 >>
 >>
 */

/* Define to prevent recursive inclusion ***********************************/
#ifndef H17R1_H
#define H17R1_H

/* Includes ****************************************************************/
#include "BOS.h"
#include "H17R1_MemoryMap.h"
#include "H17R1_uart.h"
#include "H17R1_spi.h"
#include "H17R1_gpio.h"
#include "H17R1_dma.h"
#include "H17R1_inputs.h"
#include "H17R1_eeprom.h"
#include "powerstep01.h"

/* Exported Macros *********************************************************/
#define	MODULE_PN		_H17R1

/* Port-related Definitions */
#define	NUM_OF_PORTS	4
#define P_PROG 			P2		/* ST factory bootloader UART */

/* Define available ports */
#define _P1
#define _P2
#define _P3
#define _P4

/* Define available USARTs */
#define _USART2
#define _USART3
#define _USART5
#define _USART6

/* Port-UART mapping */
#define UART_P1 &huart6
#define UART_P2 &huart2
#define UART_P3 &huart3
#define UART_P4 &huart5

/* Module-specific Hardware Definitions ************************************/
/* Port Definitions */
#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT		GPIOA
#define	USART2_RX_PORT		GPIOA
#define	USART2_AF			GPIO_AF1_USART2

#define	USART3_TX_PIN		GPIO_PIN_10
#define	USART3_RX_PIN		GPIO_PIN_11
#define	USART3_TX_PORT		GPIOB
#define	USART3_RX_PORT		GPIOB
#define	USART3_AF			GPIO_AF4_USART3

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT		GPIOA
#define	USART4_RX_PORT		GPIOA
#define	USART4_AF			GPIO_AF4_USART4

#define	USART5_TX_PIN		GPIO_PIN_3
#define	USART5_RX_PIN		GPIO_PIN_2
#define	USART5_TX_PORT		GPIOD
#define	USART5_RX_PORT		GPIOD
#define	USART5_AF			GPIO_AF3_USART5

#define	USART6_TX_PIN		GPIO_PIN_8
#define	USART6_RX_PIN		GPIO_PIN_9
#define	USART6_TX_PORT		GPIOB
#define	USART6_RX_PORT		GPIOB
#define	USART6_AF			GPIO_AF8_USART6

/* SPI Pin Definitions */
#define SPI_SCK_PIN
#define SPI_MOSI_PIN
#define SPI_MISO_PIN
#define SPI_PORT

#define SPI_NSS_PIN
#define SPI_NSS_PORT

#define SPI_HANDLER

/* Motor A/B Timer Definitions */
#define MOTOR_PWM_PIN
#define MOTOR_PWM_PORT
#define MOTOR_TIM_HANDLE
#define MOTOR_TIM_CH
#define MOTOR_ARR
#define MOTOR_CCR

/* Stepper GPIO Pin Definitions */
#define FLAG_Pin          GPIO_PIN_11
#define FLAG_GPIO_Port    GPIOA
#define FLAG_EXTI_IRQn    EXTI4_15_IRQn
#define BUSY_Pin          GPIO_PIN_12
#define BUSY_GPIO_Port    GPIOA
#define BUSY_EXTI_IRQn    EXTI4_15_IRQn
#define RESET_Pin         GPIO_PIN_6
#define RESET_GPIO_Port   GPIOB
#define STCK_Pin          GPIO_PIN_7
#define STCK_GPIO_Port    GPIOB

/* Indicator LED */
#define _IND_LED_PORT			GPIOB
#define _IND_LED_PIN			GPIO_PIN_14

/* Module-specific Macro Definitions ***************************************/
#define NUM_MODULE_PARAMS		  1
#define TIMER_PRESCALER           (1024)
#define SPIx_TIMEOUT_MAX          ((uint32_t)0x1000) /* SPI Maximum Timeout values for flags waiting loops */

/* Module-specific Type Definition *****************************************/
/* Module-status Type Definition */
typedef enum {
	H17R1_OK = 0,
	H17R1_ERR_UnknownMessage,
	H17R1_ERR_WrongParams,
	H17R1_ERROR = 255
} Module_Status;

/* */
typedef enum {
	SoftStop = 0, HardStop, Clock
} StoppingMethod;

/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);
extern void SystemClock_Config(void);

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
Module_Status StepperStop(StoppingMethod mode);
Module_Status StepperRun(motorDir_t direction, uint32_t speed);
Module_Status StepperMove(motorDir_t direction, uint32_t n_step);
Module_Status StepperIcInit(int8_t steppermode, float Accelaration, float Declaration, float MaxSpeed, float Overcurrent);

#endif /* H17R1_H */

/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
