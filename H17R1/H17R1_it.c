/*
 BitzOS (BOS) V0.3.4 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H17R1_it.c
 Description   :Interrupt Service Routines.

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "powerstep01.h"
uint8_t temp_length[NumOfPorts] = {0};
uint8_t temp_index[NumOfPorts] = {0};
uint8_t* error_restart_message = "Restarting...\r\n";


/* External variables --------------------------------------------------------*/
extern uint8_t UARTRxBuf[NumOfPorts][MSG_RX_BUF_SIZE];
extern uint8_t UARTRxBufIndex[NumOfPorts];
extern TIM_HandleTypeDef htim4;
extern void Error_Handler(void);
 void MyBusyInterruptHandler(void);
 void MyFlagInterruptHandler(void);
 void MyErrorHandler(uint16_t error);
/* External function prototypes ----------------------------------------------*/

extern TaskHandle_t xCommandConsoleTaskHandle; // CLI Task handler.


/******************************************************************************/
/*            Cortex-M0 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void){
	
	HAL_IncTick();
	osSystickHandler();
	
}

/**
 * @brief This function handles Hard Fault error callback.
 */
void HardFault_Handler(void){
	/* Loop here */
	uint8_t* error_message = "HardFault Error\r\n";
	writePxMutex(PcPort, (char*) error_message, 17, 0xff, 0xff);
	writePxMutex(PcPort, (char*) error_restart_message, 15, 0xff, 0xff);
	NVIC_SystemReset();
	for(;;){
	};
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
 */
void USART1_IRQHandler(void){
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
#if defined (_Usart1)		
	HAL_UART_IRQHandler(&huart1);
#endif
	
	/* If lHigherPriorityTaskWoken is now equal to pdTRUE, then a context
	 switch should be performed before the interrupt exists.  That ensures the
	 unblocked (higher priority) task is returned to immediately. */
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/*-----------------------------------------------------------*/

/**
 * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
 */
void USART2_LPUART2_IRQHandler(void){
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
#if defined (_Usart2)	
	HAL_UART_IRQHandler(&huart2);
#endif
	
	/* If lHigherPriorityTaskWoken is now equal to pdTRUE, then a context
	 switch should be performed before the interrupt exists.  That ensures the
	 unblocked (higher priority) task is returned to immediately. */
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/*-----------------------------------------------------------*/

/**
 * @brief This function handles USART3 to USART8 global interrupts / USART3 wake-up interrupt through EXTI line 28.
 */

void USART3_4_5_6_LPUART1_IRQHandler(void){
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
#if defined (_Usart3)
	HAL_UART_IRQHandler(&huart3);
#endif
#if defined (_Usart4)
	HAL_UART_IRQHandler(&huart4);
#endif
#if defined (_Usart5)
	HAL_UART_IRQHandler(&huart5);
#endif
#if defined (_Usart6)
	HAL_UART_IRQHandler(&huart6);
#endif
	
	/* If lHigherPriorityTaskWoken is now equal to pdTRUE, then a context
	 switch should be performed before the interrupt exists.  That ensures the
	 unblocked (higher priority) task is returned to immediately. */
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
/*-----------------------------------------------------------*/
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(FLAG_Pin);
  HAL_GPIO_EXTI_IRQHandler(BUSY_Pin);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}
/*-----------------------------------------------------------*/
/**
  * @brief This function handles TIM3, TIM4 global Interrupt.
  */
void TIM3_TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_TIM4_IRQn 0 */

  /* USER CODE END TIM3_TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM3_TIM4_IRQn 1 */

  /* USER CODE END TIM3_TIM4_IRQn 1 */
}
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
void MyFlagInterruptHandler(void)
{
  /* Get the value of the status register via the command GET_STATUS */
	  // HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	uint16_t statusRegister = Powerstep01_CmdGetStatus(0);

  /* Check HIZ flag: if set, power brigdes are disabled */
  if ((statusRegister & POWERSTEP01_STATUS_HIZ) == POWERSTEP01_STATUS_HIZ)
  {
    // HIZ state
  }

  /* Check BUSY flag: if not set, a command is under execution */
  if ((statusRegister & POWERSTEP01_STATUS_BUSY) == 0)
  {
    // BUSY
  }

  /* Check SW_F flag: if not set, the SW input is opened */
  if ((statusRegister & POWERSTEP01_STATUS_SW_F ) == 0)
  {
     // SW OPEN
  }
  else
  {
    // SW CLOSED
  }
  /* Check SW_EN bit */
  if ((statusRegister & POWERSTEP01_STATUS_SW_EVN) == POWERSTEP01_STATUS_SW_EVN)
  {
    // switch turn_on event
  }
  /* Check direction bit */
  if ((statusRegister & POWERSTEP01_STATUS_DIR) == 0)
  {
    // BACKWARD
  }
  else
  {
    // FORWARD
  }
  if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS) == POWERSTEP01_STATUS_MOT_STATUS_STOPPED )
  {
       // MOTOR STOPPED
  }
  else  if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS) == POWERSTEP01_STATUS_MOT_STATUS_ACCELERATION )
  {
           // MOTOR ACCELERATION
  }
  else  if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS) == POWERSTEP01_STATUS_MOT_STATUS_DECELERATION )
  {
           // MOTOR DECELERATION
  }
  else  if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS) == POWERSTEP01_STATUS_MOT_STATUS_CONST_SPD )
  {
       // MOTOR RUNNING AT CONSTANT SPEED
  }

  /* Check Command Error flag: if set, the command received by SPI can't be */
  /* performed. This occurs for instance when a move command is sent to the */
  /* Powerstep01 while it is already running */
  if ((statusRegister & POWERSTEP01_STATUS_CMD_ERROR) == POWERSTEP01_STATUS_CMD_ERROR)
  {
       // Command Error
  }

  /* Check Step mode clock flag: if set, the device is working in step clock mode */
  if ((statusRegister & POWERSTEP01_STATUS_STCK_MOD) == POWERSTEP01_STATUS_STCK_MOD)
  {
     //Step clock mode enabled
  }

  /* Check UVLO flag: if not set, there is an undervoltage lock-out */
  if ((statusRegister & POWERSTEP01_STATUS_UVLO) == 0)
  {
     //undervoltage lock-out
  }

  /* Check UVLO ADC flag: if not set, there is an ADC undervoltage lock-out */
  if ((statusRegister & POWERSTEP01_STATUS_UVLO_ADC) == 0)
  {
     //ADC undervoltage lock-out
  }

  /* Check thermal STATUS flags: if  set, the thermal status is not normal */
  if ((statusRegister & POWERSTEP01_STATUS_TH_STATUS) != 0)
  {
    //thermal status: 1: Warning, 2: Bridge shutdown, 3: Device shutdown
  }

  /* Check OCD  flag: if not set, there is an overcurrent detection */
  if ((statusRegister & POWERSTEP01_STATUS_OCD) == 0)
  {
    //overcurrent detection
  }

  /* Check STALL_A flag: if not set, there is a Stall condition on bridge A */
  if ((statusRegister & POWERSTEP01_STATUS_STALL_A) == 0)
  {
    //overcurrent detection
  }

  /* Check STALL_B flag: if not set, there is a Stall condition on bridge B */
  if ((statusRegister & POWERSTEP01_STATUS_STALL_B) == 0)
  {
    //overcurrent detection
  }

}

/**
  * @brief  This function is the User handler for the busy interrupt
  * @param  None
  * @retval None
  */
void MyBusyInterruptHandler(void)
{

   if (Powerstep01_CheckBusyHw())
   {
      /* Busy pin is low, so at list one Powerstep01 chip is busy */
     /* To be customized (for example Switch on a LED) */
   }
   else
   {
     /* To be customized (for example Switch off a LED) */
   }
}
void MyErrorHandler(uint16_t error)
{

  Error_Handler();
}
/**
  * @brief External Line Callback
  * @param[in] GPIO_Pin pin number
  * @retval None
  */

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	 if (GPIO_Pin == BUSY_Pin)
		    {
			  Powerstep01_BusyInterruptHandler();
		}
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	 if (GPIO_Pin == BUSY_Pin)
	    {
		  Powerstep01_BusyInterruptHandler();
	}

	  if (GPIO_Pin == FLAG_Pin)
	  {
		  Powerstep01_FlagInterruptHandler();
	  }
}
/**
 * @brief This function handles DMA1 channel 1 interrupt (Uplink DMA 1).
 */
void DMA1_Ch1_IRQHandler(void){
	/* Streaming or messaging DMA on P1 */
	DMA_IRQHandler(P1);
	
}

/*-----------------------------------------------------------*/

/**
 * @brief This function handles DMA1 channel 2 to 3 and DMA2 channel 1 to 2 interrupts.
 */
void DMA1_Ch2_3_DMA2_Ch1_2_IRQHandler(void){
	/* Streaming or messaging DMA on P5 */
	if(HAL_DMA_GET_IT_SOURCE(DMA2,DMA_ISR_GIF2) == SET){
		DMA_IRQHandler(P5);
		/* Streaming or messaging DMA on P2 */
	}
	else if(HAL_DMA_GET_IT_SOURCE(DMA1,DMA_ISR_GIF3) == SET){
		DMA_IRQHandler(P2);
		/* TX messaging DMA 0 */
	}
	else if(HAL_DMA_GET_IT_SOURCE(DMA1,DMA_ISR_GIF2) == SET){
		HAL_DMA_IRQHandler(&msgTxDMA[0]);
	}
}

/*-----------------------------------------------------------*/

/**
 * @brief This function handles DMA1 channel 4 to 7 and DMA2 channel 3 to 5 interrupts.
 */
void DMA1_Ch4_7_DMA2_Ch3_5_IRQHandler(void){
	/* Streaming or messaging DMA on P3 */
	if(HAL_DMA_GET_IT_SOURCE(DMA1,DMA_ISR_GIF5) == SET){
		DMA_IRQHandler(P3);
		/* Streaming or messaging DMA on P4 */
	}
	else if(HAL_DMA_GET_IT_SOURCE(DMA1,DMA_ISR_GIF6) == SET){
		DMA_IRQHandler(P4);
		/* Streaming or messaging DMA on P6 */
	}
	else if(HAL_DMA_GET_IT_SOURCE(DMA2,DMA_ISR_GIF3) == SET){
		DMA_IRQHandler(P6);
		/* TX messaging DMA 1 */
	}
	else if(HAL_DMA_GET_IT_SOURCE(DMA1,DMA_ISR_GIF4) == SET){
		HAL_DMA_IRQHandler(&msgTxDMA[1]);
		/* TX messaging DMA 2 */
	}
	else if(HAL_DMA_GET_IT_SOURCE(DMA1,DMA_ISR_GIF7) == SET){
		HAL_DMA_IRQHandler(&msgTxDMA[2]);
	}
}

/*-----------------------------------------------------------*/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	/* TX DMAs are shared so unsetup them here to be reused */
	if(huart->hdmatx != NULL)
		DMA_MSG_TX_UnSetup(huart);
	
	/* Give back the mutex. */
	xSemaphoreGiveFromISR(PxTxSemaphoreHandle[GetPort(huart)],&(xHigherPriorityTaskWoken));
}

/*-----------------------------------------------------------*/

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	/* Loop here */
	//for(;;) {};
	/* Set the UART state ready to be able to start the process again */
	huart->gState =HAL_UART_STATE_READY;
	
	/* Resume streaming DMA for this UART port */
	uint8_t port =GetPort(huart);
	if(portStatus[port] == STREAM){
		HAL_UART_Receive_DMA(huart,(uint8_t* )(&(dmaStreamDst[port - 1]->Instance->TDR)),huart->hdmarx->Instance->CNDTR);
		/* Or parse the circular buffer and restart messaging DMA for this port */
	}
	else{
		index_input[port - 1] = 0;
		index_process[port - 1] = 0;
		memset((uint8_t* )&UARTRxBuf[port - 1], 0, MSG_RX_BUF_SIZE);
		HAL_UART_Receive_DMA(huart,(uint8_t* )&UARTRxBuf[port - 1] ,MSG_RX_BUF_SIZE);
		MsgDMAStopped[port - 1] = true;		// Set a flag here and let the backend task restart DMA after parsing the buffer	
	}
}

/*-----------------------------------------------------------*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	uint8_t port_number = GetPort(huart);
//	uint8_t port_index = port_number - 1;
//	if(Rx_Data[port_index] == 0x0D && portStatus[port_number] == FREE)
//	{
//		for(int i=0;i<=NumOfPorts;i++) // Free previous CLI port
//		{
//			if(portStatus[i] == CLI)
//			{
//				portStatus[i] = FREE;
//			}
//		}
//		portStatus[port_number] =CLI; // Continue the CLI session on this port
//		PcPort = port_number;
//		xTaskNotifyGive(xCommandConsoleTaskHandle);
//
//		if(Activate_CLI_For_First_Time_Flag == 1) Read_In_CLI_Task_Flag = 1;
//		Activate_CLI_For_First_Time_Flag = 1;
//
//	}
//	else if(portStatus[port_number] == CLI)
//	{
//		Read_In_CLI_Task_Flag = 1;
//	}
//
//	else if(Rx_Data[port_index] == 'H' && portStatus[port_number] == FREE)
//	{
//		portStatus[port_number] =H_Status; // H  Character was received, waiting for Z character.
//	}
//
//	else if(Rx_Data[port_index] == 'Z' && portStatus[port_number] == H_Status)
//	{
//		portStatus[port_number] =Z_Status; // Z  Character was received, waiting for length byte.
//	}
//
//	else if(Rx_Data[port_index] != 'Z' && portStatus[port_number] == H_Status)
//	{
//		portStatus[port_number] =FREE; // Z  Character was not received, so there is no message to receive.
//	}
//
//	else if(portStatus[port_number] == Z_Status)
//	{
//		portStatus[port_number] =MSG; // Receive length byte.
//		MSG_Buffer[port_index][MSG_Buffer_Index_End[port_index]][2] = Rx_Data[port_index];
//		temp_index[port_index] = 3;
//		temp_length[port_index] = Rx_Data[port_index] + 1;
//	}
//
//	else if(portStatus[port_number] == MSG)
//	{
//		if(temp_length[port_index] > 1)
//		{
//			MSG_Buffer[port_index][MSG_Buffer_Index_End[port_index]][temp_index[port_index]] = Rx_Data[port_index];
//			temp_index[port_index]++;
//			temp_length[port_index]--;
//		}
//		else
//		{
//			MSG_Buffer[port_index][MSG_Buffer_Index_End[port_index]][temp_index[port_index]] = Rx_Data[port_index];
//			temp_index[port_index]++;
//			temp_length[port_index]--;
//			MSG_Buffer_Index_End[port_index]++;
//			if(MSG_Buffer_Index_End[port_index] == MSG_COUNT) MSG_Buffer_Index_End[port_index] = 0;
//
//
//			Process_Message_Buffer[Process_Message_Buffer_Index_End] = port_number;
//			Process_Message_Buffer_Index_End++;
//			if(Process_Message_Buffer_Index_End == MSG_COUNT) Process_Message_Buffer_Index_End = 0;
//			portStatus[port_number] =FREE; // End of receiving message.
//		}
//	}
//
////		HAL_UART_Receive_DMA(huart,(uint8_t* )&Rx_Data[GetPort(huart) - 1] , 1);
//	HAL_UART_Receive_IT(huart,(uint8_t* )&Rx_Data[GetPort(huart) - 1] , 1);
}

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/* Run time stack overflow checking is performed if
 configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
 function is called if a stack overflow is detected. */
void vApplicationStackOverflowHook( xTaskHandle pxTask,signed char *pcTaskName){
	(void )pcTaskName;
	(void )pxTask;
	uint8_t* error_message = "Stack Overflow\r\n";
	writePxMutex(PcPort, (char*) error_message, 16, 0xff, 0xff);
	writePxMutex(PcPort, (char*) error_restart_message, 15, 0xff, 0xff);
	NVIC_SystemReset();
//	taskDISABLE_INTERRUPTS();
	for(;;);
}
/*-----------------------------------------------------------*/

/* vApplicationMallocFailedHook() will only be called if
 configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
 function that will get called if a call to pvPortMalloc() fails.
 pvPortMalloc() is called internally by the kernel whenever a task, queue,
 timer or semaphore is created.  It is also called by various parts of the
 demo application.  If heap_1.c or heap_2.c are used, then the size of the
 heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
 FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
 to query the size of free heap space that remains (although it does not
 provide information on how the remaining heap might be fragmented). */
void vApplicationMallocFailedHook(void){
	uint8_t* error_message = "Heap size exceeded\r\n";
	writePxMutex(PcPort, (char*) error_message, 20, 0xff, 0xff);
	writePxMutex(PcPort, (char*) error_restart_message, 15, 0xff, 0xff);
	NVIC_SystemReset();
//	taskDISABLE_INTERRUPTS();
	for(;;);
}
/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
