/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H17R1.c
 Description   : Source code for module H17R1.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>> Current mode is better when high speed is needed
>> Default parameters is Current mode parameters
>>
 */

/* Includes ****************************************************************/
#include "BOS.h"
#include "H17R1_inputs.h"
#include "H17R1.h"

/* Exported Typedef ******************************************************/
/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

extern TIM_HandleTypeDef htim4;

union powerstep01_Init_u initDeviceParameters;

/* Private Variables *******************************************************/


/* Module Parameters */
ModuleParam_t ModuleParam[NUM_MODULE_PARAMS] ={0};

/* Private function prototypes *********************************************/
uint8_t ClearROtopology(void);
void MX_TIM4_Init(void);
void Error_Handler(void);
void Module_Peripheral_Init(void);
void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void RemoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift);

/* Local function prototypes ***********************************************/
void MyBusyInterruptHandler(void);
void MyFlagInterruptHandler(void);
void MyErrorHandler(uint16_t error);
void Powerstep01_Board_EnableIrq(void);
void Powerstep01_Board_DisableIrq(void);
void Powerstep01_Board_Delay(uint32_t delay);
void Powerstep01_Board_StartStepClock(uint16_t newFreq);
void Powerstep01_Board_StopStepClock(void);
void Powerstep01_Board_ReleaseReset(uint8_t deviceId);
void Powerstep01_Board_Reset(uint8_t deviceId);

uint8_t Powerstep01_Board_SpiWriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte, uint8_t nbDevices);
uint32_t Powerstep01_Board_BUSY_PIN_GetState(void);
uint32_t Powerstep01_Board_FLAG_PIN_GetState(void);

/* Create CLI commands *****************************************************/
portBASE_TYPE CLI_StepperIcInitCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_StepperMoveCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_StepperRunCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_StepperStopCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* CLI command structure ***************************************************/
/* CLI command structure : StepperIcInitCommand */
const CLI_Command_Definition_t CLI_StepperIcInitCommandDefinition = {
	( const int8_t * ) "steppericinit", /* The command string to type. */
	( const int8_t * ) "steppericinit:\r\nParameters required to execute a motor_mode:0 for current mode,1 for voltage mode , Acceleration ,Declaration ,max speed ,over current(try from 0 and increase it until rotate the motor(be careful) \r\n\r\n",
	CLI_StepperIcInitCommand, /* The function to run. */
	5 /* five parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : StepperMoveCommand */
const CLI_Command_Definition_t CLI_StepperMoveCommandDefinition = {
	( const int8_t * ) "steppermove", /* The command string to type. */
	( const int8_t * ) "steppermove:\r\nParameters required to execute a motor_Direction:0,1  , Number of Steps \r\n\r\n",
	CLI_StepperMoveCommand, /* The function to run. */
	2 /* two parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : StepperRunCommand */
const CLI_Command_Definition_t CLI_StepperRunCommandDefinition = {
	( const int8_t * ) "stepperrun", /* The command string to type. */
	( const int8_t * ) "stepperrun:\r\nParameters required to execute a motor_Direction:0,1  , speed \r\n\r\n",
	CLI_StepperRunCommand, /* The function to run. */
	2 /* two parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : StepperStopCommand */
const CLI_Command_Definition_t CLI_StepperStopCommandDefinition = {
	( const int8_t * ) "stepperstop", /* The command string to type. */
	( const int8_t * ) "stepperstop:\r\nParameters required to execute a stopMode:0,1,2   \r\n\r\n",
	CLI_StepperStopCommand, /* The function to run. */
	1 /* one parameters are expected. */
};

/***************************************************************************/
/************************ Private function Definitions *********************/
/***************************************************************************/
/* @brief  System CLOCK Configuration
 *         This function configures the system clock as follows:
 *            - System CLOCK source            = PLL (HSE)
 *            - SYSCLK(Hz)                     = 64000000
 *            - HCLK(Hz)                       = 64000000
 *            - AHB Prescaler                  = 1
 *            - APB1 Prescaler                 = 1
 *            - HSE Frequency(Hz)              = 8000000
 *            - PLLM                           = 1
 *            - PLLN                           = 16
 *            - PLLP                           = 2
 *            - Flash Latency(WS)              = 2
 *            - CLOCK Source for UART1,UART2,UART3 = 16MHz (HSI)
 */
void SystemClock_Config(void){
	RCC_OscInitTypeDef RCC_OscInitStruct ={0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct ={0};

	/** Configure the main internal regulator output voltage */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE; // Enable both HSI and HSE oscillators
	RCC_OscInitStruct.HSEState = RCC_HSE_ON; // Enable HSE (External High-Speed Oscillator)
	RCC_OscInitStruct.HSIState = RCC_HSI_ON; // Enable HSI (Internal High-Speed Oscillator)
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1; // No division on HSI
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Default calibration value for HSI
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; // Enable PLL
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; // Set PLL source to HSE
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1; // Prescaler for PLL input
	RCC_OscInitStruct.PLL.PLLN =16; // Multiplication factor for PLL
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // PLLP division factor
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; // PLLQ division factor
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // PLLR division factor
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/** Initializes the CPU, AHB and APB buses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Select PLL as the system clock source
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB Prescaler set to 1
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; // APB1 Prescaler set to 1

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_2); // Configure system clocks with flash latency of 2 WS
}

/***************************************************************************/
/* enable stop mode regarding only UART1 , UART2 , and UART3 */
BOS_Status EnableStopModebyUARTx(uint8_t port){

	UART_WakeUpTypeDef WakeUpSelection;
	UART_HandleTypeDef *huart =GetUart(port);

	if((huart->Instance == USART1) || (huart->Instance == USART2) || (huart->Instance == USART3)){

		/* make sure that no UART transfer is on-going */
		while(__HAL_UART_GET_FLAG(huart, USART_ISR_BUSY) == SET);

		/* make sure that UART is ready to receive */
		while(__HAL_UART_GET_FLAG(huart, USART_ISR_REACK) == RESET);

		/* set the wake-up event:
		 * specify wake-up on start-bit detection */
		WakeUpSelection.WakeUpEvent = UART_WAKEUP_ON_STARTBIT;
		HAL_UARTEx_StopModeWakeUpSourceConfig(huart,WakeUpSelection);

		/* Enable the UART Wake UP from stop mode Interrupt */
		__HAL_UART_ENABLE_IT(huart,UART_IT_WUF);

		/* enable MCU wake-up by LPUART */
		HAL_UARTEx_EnableStopMode(huart);

		/* enter STOP mode */
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
	}
	else
		return BOS_ERROR;

}

/***************************************************************************/
/* Enable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status EnableStandbyModebyWakeupPinx(WakeupPins_t wakeupPins){

	/* Clear the WUF FLAG */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);

	/* Enable the WAKEUP PIN */
	switch(wakeupPins){

		case PA0_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
			break;

		case PA2_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
			break;

		case PB5_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
			break;

		case PC13_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
			break;

		case NRST_PIN:
			/* do no thing*/
			break;
	}

	/* Enable SRAM content retention in Standby mode */
	HAL_PWREx_EnableSRAMRetention();

	/* Finally enter the standby mode */
	HAL_PWR_EnterSTANDBYMode();

	return BOS_OK;
}

/***************************************************************************/
/* Disable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status DisableStandbyModeWakeupPinx(WakeupPins_t wakeupPins){

	/* The standby wake-up is same as a system RESET:
	 * The entire code runs from the beginning just as if it was a RESET.
	 * The only difference between a reset and a STANDBY wake-up is that, when the MCU wakes-up,
	 * The SBF status flag in the PWR power control/status register (PWR_CSR) is set */
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET){
		/* clear the flag */
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

		/* Disable  Wake-up Pinx */
		switch(wakeupPins){

			case PA0_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
				break;

			case PA2_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
				break;

			case PB5_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
				break;

			case PC13_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
				break;

			case NRST_PIN:
				/* do no thing*/
				break;
		}

		IND_blink(1000);

	}
	else
		return BOS_OK;

}

/***************************************************************************/
/* Save Command Topology in Flash RO */
uint8_t SaveTopologyToRO(void){

	HAL_StatusTypeDef flashStatus =HAL_OK;

	/* flashAdd is initialized with 8 because the first memory room in topology page
	 * is reserved for module's ID */
	uint16_t flashAdd =8;
	uint16_t temp =0;

	/* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();

	/* Erase Topology page */
	FLASH_PageErase(FLASH_BANK_2,TOPOLOGY_PAGE_NUM);

	/* Wait for an Erase operation to complete */
	flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(flashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}

	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save module's ID and topology */
	if(myID){

		/* Save module's ID */
		temp =(uint16_t )(N << 8) + myID;

		/* Save module's ID in Flash memory */
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS,temp);

		/* Wait for a Write operation to complete */
		flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

		if(flashStatus != HAL_OK){
			/* return FLASH error code */
			return pFlash.ErrorCode;
		}

		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}

		/* Save topology */
		for(uint8_t row =1; row <= N; row++){
			for(uint8_t column =0; column <= MAX_NUM_OF_PORTS; column++){
				/* Check the module serial number
				 * Note: there isn't a module has serial number 0
				 */
				if(Array[row - 1][0]){
					/* Save each element in topology Array in Flash memory */
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS + flashAdd,Array[row - 1][column]);
					/* Wait for a Write operation to complete */
					flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(flashStatus != HAL_OK){
						/* return FLASH error code */
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
						/* update new flash memory address */
						flashAdd +=8;
					}
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Save Command Snippets in Flash RO */
uint8_t SaveSnippetsToRO(void){
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint8_t snipBuffer[sizeof(Snippet_t) + 1] ={0};

	/* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();
	/* Erase Snippets page */
	FLASH_PageErase(FLASH_BANK_2,SNIPPETS_PAGE_NUM);
	/* Wait for an Erase operation to complete */
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(FlashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save Command Snippets */
	int currentAdd = SNIPPETS_START_ADDRESS;
	for(uint8_t index =0; index < NumOfRecordedSnippets; index++){
		/* Check if Snippet condition is true or false */
		if(Snippets[index].Condition.ConditionType){
			/* A marker to separate Snippets */
			snipBuffer[0] =0xFE;
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&Snippets[index],sizeof(Snippet_t));
			/* Copy the snippet struct buffer (20 x NumOfRecordedSnippets). Note this is assuming sizeof(Snippet_t) is even */
			for(uint8_t j =0; j < (sizeof(Snippet_t) / 4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j * 8]);
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
			/* Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped */
			for(uint8_t j =0; j < ((strlen(Snippets[index].CMD) + 1) / 4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(Snippets[index].CMD + j * 4));
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Clear Array topology in SRAM and Flash RO */
uint8_t ClearROtopology(void){
	/* Clear the Array */
	memset(Array,0,sizeof(Array));
	N =1;
	myID =0;

	return SaveTopologyToRO();
}

/***************************************************************************/
/* Trigger ST factory bootloader update for a remote module */
void RemoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport){

	uint8_t myOutport =0, lastModule =0;
	int8_t *pcOutputString;

	/* 1. Get Route to destination module */
	myOutport =FindRoute(myID,dst);
	if(outport && dst == myID){ /* This is a 'via port' update and I'm the last module */
		myOutport =outport;
		lastModule =myID;
	}
	else if(outport == 0){ /* This is a remote update */
		if(NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = Route[NumberOfHops(dst)-1]; /* previous module = Route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if(src == myID){
		/* Obtain the address of the output buffer.  Note there is no mutual
		 * exclusion on this buffer as it is assumed only one command console
		 * interface will be used at any one time. */
		pcOutputString =FreeRTOS_CLIGetOutputBuffer();

		if(outport == 0)		// This is a remote module update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateMessage,dst);
		else
			// This is a 'via port' remote update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateViaPortMessage,dst,outport);

		strcat((char* )pcOutputString,pcRemoteBootloaderUpdateWarningMessage);
		writePxITMutex(inport,(char* )pcOutputString,strlen((char* )pcOutputString),cmd50ms);
		Delay_ms(100);
	}

	/* 3. Setup my inport and outport for bootloader update */
	SetupPortForRemoteBootloaderUpdate(inport);
	SetupPortForRemoteBootloaderUpdate(myOutport);

	/* 5. Build a DMA stream between my inport and outport */
	StartScastDMAStream(inport,myID,myOutport,myID,BIDIRECTIONAL,0xFFFFFFFF,0xFFFFFFFF,false);
}

/***************************************************************************/
/* Setup a port for remote ST factory bootloader update:
 * Set baudrate to 57600
 * Enable even parity
 * Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port){

	UART_HandleTypeDef *huart =GetUart(port);
	HAL_UART_DeInit(huart);
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);

}

/***************************************************************************/
/* H17R1 module initialization */
void Module_Peripheral_Init(void) {

	/* Array ports */
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();

	StepperGPIOInit();
	MX_SPI_Init();
	MX_TIM4_Init();

	/* default stepper mode is current mode */
	StepperIcInit(CURRENT_MODE, ACCELERATION_CURRENT, DECLARATION_CURRENT, MAX_SPEED_CURRENT, OVERCURRENT_CURRENT);

	/* Circulating DMA Channels ON All Module */
	for (int i = 1; i <= NUM_OF_PORTS; i++) {
		if (GetUart(i) == &huart1) {
			dmaIndex[i - 1] = &(DMA1_Channel1->CNDTR);
		} else if (GetUart(i) == &huart2) {
			dmaIndex[i - 1] = &(DMA1_Channel2->CNDTR);
		} else if (GetUart(i) == &huart3) {
			dmaIndex[i - 1] = &(DMA1_Channel3->CNDTR);
		} else if (GetUart(i) == &huart4) {
			dmaIndex[i - 1] = &(DMA1_Channel4->CNDTR);
		} else if (GetUart(i) == &huart5) {
			dmaIndex[i - 1] = &(DMA1_Channel5->CNDTR);
		} else if (GetUart(i) == &huart6) {
			dmaIndex[i - 1] = &(DMA1_Channel6->CNDTR);
		}
	}
}

/***************************************************************************/
/* H17R1 message processing task */
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift) {
	Module_Status result = H17R1_OK;
	StoppingMethod mode = 0;
	motorDir_t Direction = 0;

	uint32_t Steps = 0;
	uint32_t Speed = 0;
	uint32_t Accelaration = 0;
	uint32_t Declaration = 0;
	uint32_t MaxSpeed = 0;
	uint32_t Overcurrent = 0;
	int8_t steppermode = 0;

	switch (code) {

	case CODE_H17R1_StepperIcInit:
		steppermode = cMessage[port - 1][shift];
		Accelaration = ((uint32_t) cMessage[port - 1][1 + shift])
				+ ((uint32_t) cMessage[port - 1][2 + shift] << 8)
				+ ((uint32_t) cMessage[port - 1][3 + shift] << 16)
				+ ((uint32_t) cMessage[port - 1][4 + shift] << 24);
		Declaration = ((uint32_t) cMessage[port - 1][5 + shift])
				+ ((uint32_t) cMessage[port - 1][6 + shift] << 8)
				+ ((uint32_t) cMessage[port - 1][7 + shift] << 16)
				+ ((uint32_t) cMessage[port - 1][8 + shift] << 24);
		MaxSpeed = ((uint32_t) cMessage[port - 1][9 + shift])
				+ ((uint32_t) cMessage[port - 1][10 + shift] << 8)
				+ ((uint32_t) cMessage[port - 1][11 + shift] << 16)
				+ ((uint32_t) cMessage[port - 1][12 + shift] << 24);
		Overcurrent = ((uint32_t) cMessage[port - 1][13 + shift])
				+ ((uint32_t) cMessage[port - 1][14 + shift] << 8)
				+ ((uint32_t) cMessage[port - 1][15 + shift] << 16)
				+ ((uint32_t) cMessage[port - 1][16 + shift] << 24);
		StepperIcInit(steppermode, Accelaration, Declaration, MaxSpeed, Overcurrent);
		break;

	case CODE_H17R1_STEPPER_MOVE:
		Direction = cMessage[port - 1][shift];
		Steps = ((uint32_t) cMessage[port - 1][1 + shift])
				+ ((uint32_t) cMessage[port - 1][2 + shift] << 8)
				+ ((uint32_t) cMessage[port - 1][3 + shift] << 16)
				+ ((uint32_t) cMessage[port - 1][4 + shift] << 24);
		StepperMove(Direction, Steps);
		break;

	case CODE_H17R1_StepperRun:
		Direction = cMessage[port - 1][shift];
		Speed = ((uint32_t) cMessage[port - 1][1 + shift])
				+ ((uint32_t) cMessage[port - 1][2 + shift] << 8)
				+ ((uint32_t) cMessage[port - 1][3 + shift] << 16)
				+ ((uint32_t) cMessage[port - 1][4 + shift] << 24);
		StepperRun(Direction, Speed);
		break;

	case CODE_H17R1_StepperStop:
		mode = cMessage[port - 1][shift];
		StepperStop(mode);
		break;

	default:
		result = H17R1_ERR_UNKNOWNMESSAGE;
		break;

	}

	return result;
}

/***************************************************************************/
/* Get the port for a given UART */
uint8_t GetPort(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART2)
		return P2;
	else if (huart->Instance == USART3)
		return P3;
	else if (huart->Instance == USART5)
		return P4;
	else if (huart->Instance == USART6)
		return P1;

	return 0;
}

/***************************************************************************/
/* Register this module CLI Commands */
void RegisterModuleCLICommands(void) {
	FreeRTOS_CLIRegisterCommand(&CLI_StepperIcInitCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_StepperMoveCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_StepperRunCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_StepperStopCommandDefinition);
}

/***************************************************************************/
/* This functions is useful only for input (sensors) modules.
 * Samples a module parameter value based on parameter index.
 * paramIndex: Index of the parameter (1-based index).
 * value: Pointer to store the sampled float value.
 */
Module_Status GetModuleParameter(uint8_t paramIndex, float *value) {
	Module_Status status = BOS_OK;

	switch (paramIndex) {

	/* Invalid parameter index */
	default:
		status = BOS_ERR_WrongParam;
		break;
	}

	return status;
}

/***************************************************************************/
/****************************** Local Functions ****************************/
/***************************************************************************/
/* This function provides an accurate delay in milliseconds */
void Powerstep01_Board_Delay(uint32_t delay) {

	StartMilliDelay(1);

}

/***************************************************************************/
/* This function disable the interruptions */
void Powerstep01_Board_DisableIrq(void) {
	__disable_irq();
}

/***************************************************************************/
/* This function enable the interruptions */
void Powerstep01_Board_EnableIrq(void) {
	__enable_irq();
}

/***************************************************************************/
/* Start the step clock by using the given frequency
 * The frequency is directly the current speed of the device
 * newFreq: in Hz of the step clock
 */
void Powerstep01_Board_StartStepClock(uint16_t newFreq) {
	uint32_t sysFreq = HAL_RCC_GetSysClockFreq();
	uint32_t period = (sysFreq / (TIMER_PRESCALER * newFreq)) - 1;

	__HAL_TIM_SetAutoreload(MOTOR_TIM_HANDLE, period);
	__HAL_TIM_SetCompare(MOTOR_TIM_HANDLE, MOTOR_TIM_CH, period >> 1);

	HAL_TIM_PWM_Start_IT(MOTOR_TIM_HANDLE, MOTOR_TIM_CH);
}

/***************************************************************************/
/* Stops the PWM uses for the step clock */
void Powerstep01_Board_StopStepClock(void) {
	HAL_TIM_PWM_Stop(MOTOR_TIM_HANDLE, MOTOR_TIM_CH);
}

/***************************************************************************/
/* Releases the powerSTEP01 reset (pin set to High) of all devices */
void Powerstep01_Board_ReleaseReset(uint8_t deviceId) {
	HAL_GPIO_WritePin(RESET_GPIO_PORT, RESET_PIN, GPIO_PIN_SET);
}

/***************************************************************************/
/* Resets the powerSTEP01 (reset pin set to low) of all devices */
void Powerstep01_Board_Reset(uint8_t deviceId) {
	HAL_GPIO_WritePin(RESET_GPIO_PORT, RESET_PIN, GPIO_PIN_RESET);
}

/***************************************************************************/
/* Write and read SPI byte to the powerSTEP01
 * pByteToTransmit: pointer to the byte to transmit
 * pReceivedByte: pointer to the received byte
 * nbDevices: Number of device in the SPI chain
 */
uint8_t Powerstep01_Board_SpiWriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte, uint8_t nbDevices) {
	HAL_StatusTypeDef status;

	for (uint8_t i = 0; i < nbDevices; i++) {
		status = HAL_SPI_TransmitReceive(SPI_HANDLER, pByteToTransmit, pReceivedByte, 1, TIMEOUT_MAX);
		if (status != HAL_OK)
			break;

		pByteToTransmit++;
		pReceivedByte++;
	}

	return (uint8_t) status;
}

/***************************************************************************/
/* Returns the BUSY pin state */
uint32_t Powerstep01_Board_BUSY_PIN_GetState(void) {
	return HAL_GPIO_ReadPin(BUSY_GPIO_PORT, BUSY_PIN);
}

/***************************************************************************/
/* Returns the FLAG pin state */
uint32_t Powerstep01_Board_FLAG_PIN_GetState(void) {
	return HAL_GPIO_ReadPin(FLAG_GPIO_PORT, FLAG_PIN);
}

/***************************************************************************/
/*  This function is the User handler for the busy interrupt */
void MyBusyInterruptHandler(void) {

	if (Powerstep01_CheckBusyHw()) {
		/* Busy pin is low, so at list one Powerstep01 chip is busy */
		/* To be customized (for example Switch on a LED) */
	} else {
		/* To be customized (for example Switch off a LED) */
	}
}

/***************************************************************************/
/* Get the value of the status register via the command GET_STATUS */
void MyFlagInterruptHandler(void) {
	uint16_t statusRegister = Powerstep01_CmdGetStatus(0);

	/* Check HIZ flag: if set, power brigdes are disabled */
	if ((statusRegister & POWERSTEP01_STATUS_HIZ) == POWERSTEP01_STATUS_HIZ) {
		// HIZ state
	}

	/* Check BUSY flag: if not set, a command is under execution */
	if ((statusRegister & POWERSTEP01_STATUS_BUSY) == 0) {
		// BUSY
	}

	/* Check SW_F flag: if not set, the SW input is opened */
	if ((statusRegister & POWERSTEP01_STATUS_SW_F) == 0) {
		// SW OPEN
	} else {
		// SW CLOSED
	}

	/* Check SW_EN bit */
	if ((statusRegister & POWERSTEP01_STATUS_SW_EVN) == POWERSTEP01_STATUS_SW_EVN) {
		// switch turn_on event
	}

	/* Check direction bit */
	if ((statusRegister & POWERSTEP01_STATUS_DIR) == 0) {
		// BACKWARD
	} else {
		// FORWARD
	}

	if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS) == POWERSTEP01_STATUS_MOT_STATUS_STOPPED) {
		// MOTOR STOPPED
	} else if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS) == POWERSTEP01_STATUS_MOT_STATUS_ACCELERATION) {
		// MOTOR ACCELERATION
	} else if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS) == POWERSTEP01_STATUS_MOT_STATUS_DECELERATION) {
		// MOTOR DECELERATION
	} else if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS) == POWERSTEP01_STATUS_MOT_STATUS_CONST_SPD) {
		// MOTOR RUNNING AT CONSTANT SPEED
	}

	/* Check Command Error flag: if set, the command received by SPI can't be */
	/* performed. This occurs for instance when a move command is sent to the */
	/* Powerstep01 while it is already running */
	if ((statusRegister & POWERSTEP01_STATUS_CMD_ERROR) == POWERSTEP01_STATUS_CMD_ERROR) {
		// Command Error
	}

	/* Check Step mode clock flag: if set, the device is working in step clock mode */
	if ((statusRegister & POWERSTEP01_STATUS_STCK_MOD) == POWERSTEP01_STATUS_STCK_MOD) {
		//Step clock mode enabled
	}

	/* Check UVLO flag: if not set, there is an undervoltage lock-out */
	if ((statusRegister & POWERSTEP01_STATUS_UVLO) == 0) {
		//undervoltage lock-out
	}

	/* Check UVLO ADC flag: if not set, there is an ADC undervoltage lock-out */
	if ((statusRegister & POWERSTEP01_STATUS_UVLO_ADC) == 0) {
		//ADC undervoltage lock-out
	}

	/* Check thermal STATUS flags: if  set, the thermal status is not normal */
	if ((statusRegister & POWERSTEP01_STATUS_TH_STATUS) != 0) {
		//thermal status: 1: Warning, 2: Bridge shutdown, 3: Device shutdown
	}

	/* Check OCD  flag: if not set, there is an overcurrent detection */
	if ((statusRegister & POWERSTEP01_STATUS_OCD) == 0) {
		//overcurrent detection
	}

	/* Check STALL_A flag: if not set, there is a Stall condition on bridge A */
	if ((statusRegister & POWERSTEP01_STATUS_STALL_A) == 0) {
		//overcurrent detection
	}

	/* Check STALL_B flag: if not set, there is a Stall condition on bridge B */
	if ((statusRegister & POWERSTEP01_STATUS_STALL_B) == 0) {
		//overcurrent detection
	}
}

/***************************************************************************/
void MyErrorHandler(uint16_t error) {

	Error_Handler();
}

/***************************************************************************/
union powerstep01_Init_u StepperIcInit_current_mode(float Accelaration_current,
		float Declaration_current, float MaxSpeed_current, float Overcurrent_current) {

	union powerstep01_Init_u initDeviceParameters = {
	/* common parameters */
	.cm.cp.cmVmSelection = POWERSTEP01_CM_VM_CURRENT, // enum powerstep01_CmVm_t
			Accelaration_current, // Acceleration rate in step/s2, range 14.55 to 59590 steps/s^2
			Declaration_current, // Deceleration rate in step/s2, range 14.55 to 59590 steps/s^2
			MaxSpeed_current, // Maximum speed in step/s, range 15.25 to 15610 steps/s
			0, // Minimum speed in step/s, range 0 to 976.3 steps/s
			POWERSTEP01_LSPD_OPT_OFF, // Low speed optimization bit, enum powerstep01_LspdOpt_t
			2000, // Full step speed in step/s, range 7.63 to 15625 steps/s
			POWERSTEP01_BOOST_MODE_OFF, // Boost of the amplitude square wave, enum powerstep01_BoostMode_t
			Overcurrent_current, // Overcurrent threshold settings via enum powerstep01_OcdTh_t
			STEP_MODE_1_16, // Step mode settings via enum motorStepMode_t
			POWERSTEP01_SYNC_SEL_DISABLED, // Synch. Mode settings via enum powerstep01_SyncSel_t

			(POWERSTEP01_ALARM_EN_THERMAL_SHUTDOWN | POWERSTEP01_ALARM_EN_THERMAL_WARNING | POWERSTEP01_ALARM_EN_UVLO
					| POWERSTEP01_ALARM_EN_STALL_DETECTION | POWERSTEP01_ALARM_EN_SW_TURN_ON | POWERSTEP01_ALARM_EN_WRONG_NPERF_CMD), // Alarm settings via bitmap enum powerstep01_AlarmEn_t

			POWERSTEP01_IGATE_64mA, // Gate sink/source current via enum powerstep01_Igate_t
			POWERSTEP01_TBOOST_500ns, // Duration of the overboost phase during gate turn-off via enum powerstep01_Tboost_t
			POWERSTEP01_TCC_500ns, // Controlled current time via enum powerstep01_Tcc_t
			POWERSTEP01_WD_EN_DISABLE, // External clock watchdog, enum powerstep01_WdEn_t
			POWERSTEP01_TBLANK_375ns, // Duration of the blanking time via enum powerstep01_TBlank_t
			POWERSTEP01_TDT_125ns, // Duration of the dead time via enum powerstep01_Tdt_t

			/* current mode parameters */
			7.8, // Hold torque in mV, range from 7.8mV to 1000 mV
			32, // Running torque in mV, range from 7.8mV to 1000 mV
			32, // Acceleration torque in mV, range from 7.8mV to 1000 mV
			32, // Deceleration torque in mV, range from 7.8mV to 1000 mV
			POWERSTEP01_TOFF_FAST_8us, //Maximum fast decay time , enum powerstep01_ToffFast_t
			POWERSTEP01_FAST_STEP_12us, //Maximum fall step time , enum powerstep01_FastStep_t
			3.0, // Minimum on-time in us, range 0.5us to 64us
			21.0, // Minimum off-time in us, range 0.5us to 64us
			POWERSTEP01_CONFIG_INT_16MHZ, // CLOCK setting , enum powerstep01_ConfigOscMgmt_t
			POWERSTEP01_CONFIG_SW_USER, // External switch hard stop interrupt mode, enum powerstep01_ConfigSwMode_t
			POWERSTEP01_CONFIG_TQ_REG_TVAL_USED, // External torque regulation enabling , enum powerstep01_ConfigEnTqReg_t
			POWERSTEP01_CONFIG_VS_COMP_DISABLE, // Motor Supply Voltage Compensation enabling , enum powerstep01_ConfigEnVscomp_t
			POWERSTEP01_CONFIG_OC_SD_ENABLE, // Over current shutwdown enabling, enum powerstep01_ConfigOcSd_t
			POWERSTEP01_CONFIG_UVLOVAL_LOW, // UVLO Threshold via powerstep01_ConfigUvLoVal_t
			POWERSTEP01_CONFIG_VCCVAL_7_5V, // VCC Val, enum powerstep01_ConfigVccVal_t
			POWERSTEP01_CONFIG_TSW_048us, // Switching period, enum powerstep01_ConfigTsw_t
			POWERSTEP01_CONFIG_PRED_DISABLE, // Predictive current enabling , enum powerstep01_ConfigPredEn_t
			};

	return initDeviceParameters;
}

/***************************************************************************/
union powerstep01_Init_u StepperIcInit_voltage_mode(float Accelaration_voltage,
		float Declaration_voltage, float MaxSpeed_voltage, float Overcurrent_voltage) {

	union powerstep01_Init_u initDeviceParameters = {
	/* common parameters */
	.vm.cp.cmVmSelection = POWERSTEP01_CM_VM_VOLTAGE, // enum powerstep01_CmVm_t
			Accelaration_voltage, // Acceleration rate in step/s2, range 14.55 to 59590 steps/s^2
			Declaration_voltage, // Deceleration rate in step/s2, range 14.55 to 59590 steps/s^2
			MaxSpeed_voltage, // Maximum speed in step/s, range 15.25 to 15610 steps/s
			0, // Minimum speed in step/s, range 0 to 976.3 steps/s
			POWERSTEP01_LSPD_OPT_OFF, // Low speed optimization bit, enum powerstep01_LspdOpt_t
			244.16, // Full step speed in step/s, range 7.63 to 15625 steps/s
			POWERSTEP01_BOOST_MODE_OFF, // Boost of the amplitude square wave, enum powerstep01_BoostMode_t
			Overcurrent_voltage, // Overcurrent threshold settings via enum powerstep01_OcdTh_t
			STEP_MODE_1_128, // Step mode settings via enum motorStepMode_t
			POWERSTEP01_SYNC_SEL_DISABLED, // Synch. Mode settings via enum powerstep01_SyncSel_t

			(POWERSTEP01_ALARM_EN_OVERCURRENT | POWERSTEP01_ALARM_EN_THERMAL_SHUTDOWN | POWERSTEP01_ALARM_EN_THERMAL_WARNING
					| POWERSTEP01_ALARM_EN_UVLO | POWERSTEP01_ALARM_EN_STALL_DETECTION
					| POWERSTEP01_ALARM_EN_SW_TURN_ON | POWERSTEP01_ALARM_EN_WRONG_NPERF_CMD), // Alarm settings via bitmap enum powerstep01_AlarmEn_t

			POWERSTEP01_IGATE_64mA, // Gate sink/source current via enum powerstep01_Igate_t
			POWERSTEP01_TBOOST_0ns, // Duration of the overboost phase during gate turn-off via enum powerstep01_Tboost_t
			POWERSTEP01_TCC_500ns, // Controlled current time via enum powerstep01_Tcc_t
			POWERSTEP01_WD_EN_DISABLE, // External clock watchdog, enum powerstep01_WdEn_t
			POWERSTEP01_TBLANK_375ns, // Duration of the blanking time via enum powerstep01_TBlank_t
			POWERSTEP01_TDT_125ns, // Duration of the dead time via enum powerstep01_Tdt_t

			/* voltage mode parameters */
			16.02, // Hold duty cycle (torque) in %, range 0 to 99.6%
			16.02, // Run duty cycle (torque) in %, range 0 to 99.6%
			16.02, // Acceleration duty cycle (torque) in %, range 0 to 99.6%
			16.02, // Deceleration duty cycle (torque) in %, range 0 to 99.6%
			61.512, // Intersect speed settings for BEMF compensation in steps/s, range 0 to 3906 steps/s
			0.03815, // BEMF start slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step
			0.06256, // BEMF final acc slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step
			0.06256, // BEMF final dec slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step
			1, // Thermal compensation param, range 1 to 1.46875
			531.25, // Stall threshold settings in mV, range 31.25mV to 1000mV
			POWERSTEP01_CONFIG_INT_16MHZ_OSCOUT_2MHZ, // CLOCK setting , enum powerstep01_ConfigOscMgmt_t
			POWERSTEP01_CONFIG_SW_HARD_STOP, // External switch hard stop interrupt mode, enum powerstep01_ConfigSwMode_t
			POWERSTEP01_CONFIG_VS_COMP_DISABLE, // Motor Supply Voltage Compensation enabling , enum powerstep01_ConfigEnVscomp_t
			POWERSTEP01_CONFIG_OC_SD_DISABLE, // Over current shutwdown enabling, enum powerstep01_ConfigOcSd_t
			POWERSTEP01_CONFIG_UVLOVAL_LOW, // UVLO Threshold via powerstep01_ConfigUvLoVal_t
			POWERSTEP01_CONFIG_VCCVAL_15V, // VCC Val, enum powerstep01_ConfigVccVal_t
			POWERSTEP01_CONFIG_PWM_DIV_1, // PWM Frequency Integer division, enum powerstep01_ConfigFPwmInt_t
			POWERSTEP01_CONFIG_PWM_MUL_0_75, // PWM Frequency Integer Multiplier, enum powerstep01_ConfigFPwmDec_t
			};
	return initDeviceParameters;
}

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
Module_Status StepperIcInit(DrivingMethod, float Accelaration, float Declaration, float MaxSpeed, float Overcurrent) {
	Module_Status status = H17R1_OK;

	if (CURRENT_MODE != 0 && VOLTAGE_MODE != 1)
		return H17R1_ERR_WRONGPARAMS;

	if (Accelaration < MOTOR_MIN_ACC_DEC_V_C || Accelaration > MOTOR_MAX_ACC_DEC_V_C)
		return H17R1_ERR_WRONGPARAMS;

	if (Declaration < MOTOR_MIN_ACC_DEC_V_C || Declaration > MOTOR_MAX_ACC_DEC_V_C)
		return H17R1_ERR_WRONGPARAMS;

	if (MaxSpeed < MOTOR_MIN_SPEED || MaxSpeed > MOTOR_MAX_SPEED)
		return H17R1_ERR_WRONGPARAMS;

//	/* Default Mode is current mode */
//	if (CURRENT_MODE == 0)
//		initDeviceParameters = StepperIcInit_current_mode(Accelaration, Declaration, MaxSpeed, Overcurrent);
//	 else
//		initDeviceParameters = StepperIcInit_voltage_mode(Accelaration, Declaration, MaxSpeed, Overcurrent);

	/* Set the Powerstep01 library to use 1 device */
	Powerstep01_SetNbDevices(1);

	/* device with the union declared in the the H17R1.h file and comment the    */
	/* subsequent call having the NULL pointer                                  */
	Powerstep01_Init(NULL/*&initDeviceParameters*/);

	/* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
	Powerstep01_AttachFlagInterrupt(MyFlagInterruptHandler);
	/* Attach the function MyBusyInterruptHandler (defined below) to the busy interrupt */
	Powerstep01_AttachBusyInterrupt(MyBusyInterruptHandler);
	/* Attach the function Error_Handler (defined below) to the error Handler*/
	Powerstep01_AttachErrorHandler(MyErrorHandler);

	return status;
}

/***************************************************************************/
/* motor will move depending on the number of steps */
Module_Status StepperMove(motorDir_t direction, uint32_t n_step) {
	Module_Status status = H17R1_OK;

	if (direction != 0 && direction != 1)
		return H17R1_ERR_WRONGPARAMS;

	Powerstep01_CmdMove(0, direction, n_step);
	Powerstep01_WaitWhileActive(0);

	return status;
}

/***************************************************************************/
/* motor will run with the given speed unti it is stopped using StepperStop function
 * speed in 2^-28 step/tick
 */
Module_Status StepperRun(motorDir_t direction, uint32_t speed) {
	Module_Status status = H17R1_OK;

	if (direction != Backward && direction != Forward)
		return H17R1_ERR_WRONGPARAMS;

	if (speed > MOTOR_MAX_SPEED)
		return H17R1_ERR_WRONGPARAMS;

	Powerstep01_CmdRun(0, direction, speed);

	return status;
}

/***************************************************************************/
Module_Status StepperStop(StoppingMethod mode) {
	Module_Status status = H17R1_OK;

	if (mode != SOFT_STOP && mode != HARD_STOP && mode != CLOCK)
		return H17R1_ERR_WRONGPARAMS;

	switch (mode) {
	case SOFT_STOP:
		Powerstep01_CmdSoftStop(0);
		break;

	case HARD_STOP:
		Powerstep01_CmdHardStop(0);
		break;

	case CLOCK:
		Powerstep01_StopStepClock();
		break;

	default:
		break;
	}

	return status;
}

/***************************************************************************/
/********************************* Commands ********************************/
/***************************************************************************/
portBASE_TYPE CLI_StepperIcInitCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H17R1_OK;

	int8_t steppermode;
	static int8_t *pcParameterString1;
	static int8_t *pcParameterString2;
	static int8_t *pcParameterString3;
	static int8_t *pcParameterString4;
	static int8_t *pcParameterString5;

	static const int8_t *pcOKMessage=(int8_t* )"stepper is moving:\r\n %d  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";
	static const int8_t *pcWrongRangeMessage =(int8_t* )"Direction is not true!\n\r";

	float Accelaration ,Declaration,MaxSpeed ,Overcurrent;

	portBASE_TYPE xParameterStringLength1 =0;
	portBASE_TYPE xParameterStringLength2 =0;
	portBASE_TYPE xParameterStringLength3 =0;
	portBASE_TYPE xParameterStringLength4 =0;
	portBASE_TYPE xParameterStringLength5 =0;

	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
	steppermode =(uint8_t )atol((char* )pcParameterString1);

	pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength2 );
	Accelaration =(float )atol((char* )pcParameterString2);
	pcParameterString3 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 3, &xParameterStringLength3 );
	Declaration =(float )atol((char* )pcParameterString3);

	pcParameterString4 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 4, &xParameterStringLength4 );
	MaxSpeed =(float )atol((char* )pcParameterString4);

	pcParameterString5 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 5, &xParameterStringLength5 );
	Overcurrent =(float )atol((char* )pcParameterString5);

	status=StepperIcInit( steppermode, Accelaration, Declaration,  MaxSpeed,Overcurrent );

	if(status == H17R1_OK)
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,steppermode);

	else if(status == H17R1_ERR_WRONGPARAMS)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);

	return pdFALSE;
}

/***************************************************************************/
portBASE_TYPE CLI_StepperMoveCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H17R1_OK;

	uint8_t direction;
	static int8_t *pcParameterString1;
	static int8_t *pcParameterString2;

	static const int8_t *pcOKMessage=(int8_t* )"stepper is moving:\r\n %d  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";
	static const int8_t *pcWrongRangeMessage =(int8_t* )"Direction is not true!\n\r";

	uint32_t steps;

	portBASE_TYPE xParameterStringLength1 =0;
	portBASE_TYPE xParameterStringLength2 =0;

	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
	direction =(uint8_t )atol((char* )pcParameterString1);

	pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength2 );
	steps =(uint32_t )atol((char* )pcParameterString2);
    status=StepperMove(direction,steps);

	if(status == H17R1_OK)
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,steps);

	else if(status == H17R1_ERR_WRONGPARAMS)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);

	return pdFALSE;
}

/***************************************************************************/
portBASE_TYPE CLI_StepperRunCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H17R1_OK;

	uint8_t direction;
	static int8_t *pcParameterString1;
	static int8_t *pcParameterString2;

	static const int8_t *pcOKMessage=(int8_t* )"stepper is moving:\r\n %d  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";
	static const int8_t *pcWrongRangeMessage =(int8_t* )"Direction is not true!\n\r";

	uint32_t speed;

	portBASE_TYPE xParameterStringLength1 =0;
	portBASE_TYPE xParameterStringLength2 =0;

	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
	direction =(uint8_t )atol((char* )pcParameterString1);

	pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength2 );
	speed =(uint32_t )atol((char* )pcParameterString2);
    status=StepperRun(direction,speed);

	if(status == H17R1_OK)
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,speed);

	else if(status == H17R1_ERR_WRONGPARAMS)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);

	return pdFALSE;
}

/***************************************************************************/
portBASE_TYPE CLI_StepperStopCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H17R1_OK;

    uint8_t stopmode;
	static int8_t *pcParameterString1;
	static const int8_t *pcOKMessage=(int8_t* )"stepper is stop:\r\n %d  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";
	static const int8_t *pcWrongRangeMessage =(int8_t* )"Direction is not true!\n\r";

	portBASE_TYPE xParameterStringLength1 =0;

	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
	stopmode =(uint8_t )atol((char* )pcParameterString1);

	 status=StepperStop(stopmode);

	if(status == H17R1_OK)
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,stopmode);

	else if(status == H17R1_ERR_WRONGPARAMS)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);

	return pdFALSE;
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
