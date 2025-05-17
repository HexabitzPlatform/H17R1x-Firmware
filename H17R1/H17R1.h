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
//#include "motor.h"

/* Motors ******************************************************************/
//#define MOTOR_23HS8240
////#define MOTOR_17HS4401
////#define MOTOR_SY42STH38_1684A
////#define MOTOR_23HS45_4204S
////#define MOTOR_KL23H256_21_8B
////#define MOTOR_34HS59_5008D


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
#define SPI_SCK_PIN         GPIO_PIN_3
#define SPI_MOSI_PIN        GPIO_PIN_5
#define SPI_MISO_PIN        GPIO_PIN_4
#define SPI_PORT            GPIOB

#define SPI_NSS_PIN         GPIO_PIN_15
#define SPI_NSS_PORT        GPIOA

#define SPI_HANDLER         &hspi1

/* Motor A/B Timer Definitions */
#define MOTOR_PWM_PIN       GPIO_PIN_7
#define MOTOR_PWM_PORT      GPIOB
#define MOTOR_TIM_HANDLE    &htim4
#define MOTOR_TIM_CH        TIM_CHANNEL_2
#define TIMER_PRESCALER     (1024)

/* Stepper GPIO Pin Definitions */
#define FLAG_PIN            GPIO_PIN_11
#define FLAG_GPIO_PORT      GPIOA
#define FLAG_EXTI_IRQN      EXTI4_15_IRQn
#define BUSY_PIN            GPIO_PIN_12
#define BUSY_GPIO_PORT      GPIOA
#define BUSY_EXTI_IRQN      EXTI4_15_IRQn
#define RESET_PIN           GPIO_PIN_6
#define RESET_GPIO_PORT     GPIOB

/* Indicator LED */
#define _IND_LED_PORT	    GPIOB
#define _IND_LED_PIN	    GPIO_PIN_14

/* Module-specific Macro Definitions ***************************************/
#define NUM_MODULE_PARAMS	    1
#define TIMEOUT_MAX             ((uint32_t)0x1000)

#define MOTOR_MAX_SPEED         15610   /* step/tick */
#define MOTOR_MIN_SPEED         15.25   /* step/tick */

#define MOTOR_MAX_ACC_DEC_V_C   59590
#define MOTOR_MIN_ACC_DEC_V_C   14.55

/* Initialization parameters for current mode */
#define ACCELERATION_CURRENT    5000
#define DECLARATION_CURRENT     1000
#define	MAX_SPEED_CURRENT       15610
#define	OVERCURRENT_CURRENT     48

/* Initialization parameters for voltage mode */
#define ACCELERATION_VOLTAGE    582
#define DECLARATION_VOLTAGE     582
#define MAX_SPEED_VOLTAGE       488
#define OVERCURRENT_VOLTAGE     281.25

/* Module-specific Type Definition *****************************************/
/* Module-status Type Definition */
typedef enum {
	H17R1_OK = 0,
	H17R1_ERR_UNKNOWNMESSAGE,
	H17R1_ERR_WRONGPARAMS,
	H17R1_ERROR = 255
} Module_Status;

/* Motor Driving Method */
typedef enum {
	CURRENT_MODE = 0,
	VOLTAGE_MODE = 1
}DrivingMethod;

/* Motor Stopping Method */
typedef enum {
	SOFT_STOP = 0,
	HARD_STOP,
	CLOCK
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
Module_Status StepperIcInit(DrivingMethod, float Accelaration, float Declaration, float MaxSpeed, float Overcurrent);

#endif /* H17R1_H */

/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
