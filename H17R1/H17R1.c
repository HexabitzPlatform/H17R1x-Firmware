/*
 BitzOS (BOS) V0.3.3 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H17R1.c
 Description   : Source code for module H17R1.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H17R1_inputs.h"
#include "H17R1.h"


/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;
//the current mode is better when high speed is needed
//int8_t steppermode = 0;// Default Mode is current mode
//default parameters is Current mode parameters
/* Initialization parameters for current mode */
uint32_t Accelaration_current=5000 , Declaration_current=1000 , MaxSpeed_current=15610 ,Overcurrent_current=48;
/* Initialization parameters for voltage mode */
float Accelaration_voltage=582 , Declaration_voltage=582 , MaxSpeed_voltage=488 ,Overcurrent_voltage= 281.25;
void MX_GPIO_Init(void);
extern void MX_SPI1_Init(void);
extern void MX_TIM4_Init(void);
/* Module exported parameters ------------------------------------------------*/
module_param_t modParam[NUM_MODULE_PARAMS] ={{.paramPtr = NULL, .paramFormat =FMT_FLOAT, .paramName =""}};

/* Private variables ---------------------------------------------------------*/
extern void MyBusyInterruptHandler(void);
extern void MyFlagInterruptHandler(void);
extern void MyErrorHandler(uint16_t error);
union powerstep01_Init_u initDeviceParameters;

/* Private function prototypes -----------------------------------------------*/
void ExecuteMonitor(void);

/* Create CLI commands --------------------------------------------------------*/

/*-----------------------------------------------------------*/
/* Create CLI commands --------------------------------------------------------*/
portBASE_TYPE CLI_StepperIcInitCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_StepperMoveCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_StepperRunCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE CLI_StepperStopCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/*-----------------------------------------------------------*/

/* CLI command structure : StepperIcInitCommand */
const CLI_Command_Definition_t CLI_StepperIcInitCommandDefinition =
{
	( const int8_t * ) "steppericinit", /* The command string to type. */
	( const int8_t * ) "steppericinit:\r\nParameters required to execute a motor_mode:0 for current mode,1 for voltage mode , Acceleration ,Declaration ,max speed ,over current(try from 0 and increase it until rotate the motor(be careful) \r\n\r\n",
	CLI_StepperIcInitCommand, /* The function to run. */
	5 /* two parameters are expected. */
};

/* CLI command structure : StepperMoveCommand */
const CLI_Command_Definition_t CLI_StepperMoveCommandDefinition =
{
	( const int8_t * ) "steppermove", /* The command string to type. */
	( const int8_t * ) "steppermove:\r\nParameters required to execute a motor_Direction:0,1  , Number of Steps \r\n\r\n",
	CLI_StepperMoveCommand, /* The function to run. */
	2 /* two parameters are expected. */
};

/* CLI command structure : StepperRunCommand */
const CLI_Command_Definition_t CLI_StepperRunCommandDefinition =
{
	( const int8_t * ) "stepperrun", /* The command string to type. */
	( const int8_t * ) "stepperrun:\r\nParameters required to execute a motor_Direction:0,1  , speed \r\n\r\n",
	CLI_StepperRunCommand, /* The function to run. */
	2 /* two parameters are expected. */
};

/* CLI command structure : StepperStopCommand */
const CLI_Command_Definition_t CLI_StepperStopCommandDefinition =
{
	( const int8_t * ) "stepperstop", /* The command string to type. */
	( const int8_t * ) "stepperstop:\r\nParameters required to execute a stopMode:0,1,2   \r\n\r\n",
	CLI_StepperStopCommand, /* The function to run. */
	1 /* two parameters are expected. */
};





//***********************************************************************
/* -----------------------------------------------------------------------
 |												 Private Functions	 														|
 ----------------------------------------------------------------------- 
 */

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow : 
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 48000000
 *            HCLK(Hz)                       = 48000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 8000000
 *            PREDIV                         = 1
 *            PLLMUL                         = 6
 *            Flash Latency(WS)              = 1
 * @param  None
 * @retval None
 */
void SystemClock_Config(void){
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	  /** Configure the main internal regulator output voltage
	  */
	  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	  RCC_OscInitStruct.PLL.PLLN = 12;
	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	  HAL_RCC_OscConfig(&RCC_OscInitStruct);

	  /** Initializes the CPU, AHB and APB buses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	 HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

	  /** Initializes the peripherals clocks
	  */
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2;
	  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	  HAL_NVIC_SetPriority(SysTick_IRQn,0,0);
	
}

/*-----------------------------------------------------------*/


/* --- Save array topology and Command Snippets in Flash RO --- 
 */
uint8_t SaveToRO(void){
	BOS_Status result =BOS_OK;
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint16_t add =8;
    uint16_t temp =0;
	uint8_t snipBuffer[sizeof(snippet_t) + 1] ={0};
	
	HAL_FLASH_Unlock();
	/* Erase RO area */
	FLASH_PageErase(FLASH_BANK_1,RO_START_ADDRESS);
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
	FLASH_PageErase(FLASH_BANK_1,RO_MID_ADDRESS);
	//TOBECHECKED
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
	if(FlashStatus != HAL_OK){
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}
	
	/* Save number of modules and myID */
	if(myID){
		temp =(uint16_t )(N << 8) + myID;
		//HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,RO_START_ADDRESS,temp);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,RO_START_ADDRESS,temp);
		//TOBECHECKED
		FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
		if(FlashStatus != HAL_OK){
			return pFlash.ErrorCode;
		}
		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}
		
		/* Save topology */
		for(uint8_t i =1; i <= N; i++){
			for(uint8_t j =0; j <= MaxNumOfPorts; j++){
				if(array[i - 1][0]){

          	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,RO_START_ADDRESS + add,array[i - 1][j]);
				 //HALFWORD 	//TOBECHECKED
					FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(FlashStatus != HAL_OK){
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
						add +=8;
					}
				}
			}
		}
	}
	
	// Save Command Snippets
	int currentAdd = RO_MID_ADDRESS;
	for(uint8_t s =0; s < numOfRecordedSnippets; s++){
		if(snippets[s].cond.conditionType){
			snipBuffer[0] =0xFE;		// A marker to separate Snippets
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&snippets[s],sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for(uint8_t j =0; j < (sizeof(snippet_t)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j*8]);
				//HALFWORD
				//TOBECHECKED
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
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for(uint8_t j =0; j < ((strlen(snippets[s].cmd) + 1)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(snippets[s].cmd + j*4 ));
				//HALFWORD
				//TOBECHECKED
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
	
	HAL_FLASH_Lock();
	
	return result;
}

/* --- Clear array topology in SRAM and Flash RO --- 
 */
uint8_t ClearROtopology(void){
	// Clear the array 
	memset(array,0,sizeof(array));
	N =1;
	myID =0;
	
	return SaveToRO();
}
/*-----------------------------------------------------------*/

/* --- Trigger ST factory bootloader update for a remote module.
 */
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport){

	uint8_t myOutport =0, lastModule =0;
	int8_t *pcOutputString;

	/* 1. Get route to destination module */
	myOutport =FindRoute(myID,dst);
	if(outport && dst == myID){ /* This is a 'via port' update and I'm the last module */
		myOutport =outport;
		lastModule =myID;
	}
	else if(outport == 0){ /* This is a remote update */
		if(NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = route[NumberOfHops(dst)-1]; /* previous module = route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if(src == myID){
		/* Obtain the address of the output buffer.  Note there is no mutual
		 exclusion on this buffer as it is assumed only one command console
		 interface will be used at any one time. */
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

/*-----------------------------------------------------------*/

/* --- Setup a port for remote ST factory bootloader update:
 - Set baudrate to 57600
 - Enable even parity
 - Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port){
	UART_HandleTypeDef *huart =GetUart(port);

	huart->Init.BaudRate =57600;
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);
}

/* --- H17R1 module initialization.
 */
void Module_Peripheral_Init(void){

	/* Array ports */
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();
    MX_GPIO_Init();
	MX_SPI1_Init();
    MX_TIM4_Init();
    //default stepper mode is current mode
   StepperIcInit(0,Accelaration_current,Declaration_current, MaxSpeed_current,Overcurrent_current);	//Circulating DMA Channels ON All Module
	 for(int i=1;i<=NumOfPorts;i++)
		{
		  if(GetUart(i)==&huart1)
				   { index_dma[i-1]=&(DMA1_Channel1->CNDTR); }
		  else if(GetUart(i)==&huart2)
				   { index_dma[i-1]=&(DMA1_Channel2->CNDTR); }
		  else if(GetUart(i)==&huart3)
				   { index_dma[i-1]=&(DMA1_Channel3->CNDTR); }
		  else if(GetUart(i)==&huart4)
				   { index_dma[i-1]=&(DMA1_Channel4->CNDTR); }
		  else if(GetUart(i)==&huart5)
				   { index_dma[i-1]=&(DMA1_Channel5->CNDTR); }
		  else if(GetUart(i)==&huart6)
				   { index_dma[i-1]=&(DMA1_Channel6->CNDTR); }
		}

	/* Create module special task (if needed) */
}

/*-----------------------------------------------------------*/
/* --- H17R1 message processing task.
 */
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift){
	Module_Status result =H17R1_OK;
	motorDir_t Direction=0;
	StoppingMethod mode=0 ;
	uint32_t Steps=0;
	uint32_t Speed=0;
	uint32_t  Accelaration;
	 uint32_t  Declaration;
	uint32_t  MaxSpeed;
	uint32_t  Overcurrent;

int8_t steppermode;
//float Accelaration,Declaration,MaxSpeed,Overcurrent;
	switch(code){

	case CODE_H17R1_StepperIcInit:
		steppermode=cMessage[port - 1][shift];
		Accelaration =((uint32_t )cMessage[port - 1][1 + shift] ) + ((uint32_t )cMessage[port - 1][2 + shift] << 8) + ((uint32_t )cMessage[port - 1][3 + shift] << 16) + ((uint32_t )cMessage[port - 1][4 + shift] << 24);
		Declaration =((uint32_t )cMessage[port - 1][5 + shift] ) + ((uint32_t )cMessage[port - 1][6 + shift] << 8) + ((uint32_t )cMessage[port - 1][7 + shift] << 16) + ((uint32_t )cMessage[port - 1][8 + shift] << 24);
		MaxSpeed =((uint32_t )cMessage[port - 1][9 + shift] ) + ((uint32_t )cMessage[port - 1][10 + shift] << 8) + ((uint32_t )cMessage[port - 1][11 + shift] << 16) + ((uint32_t )cMessage[port - 1][12 + shift] << 24);
		Overcurrent =((uint32_t )cMessage[port - 1][13 + shift] ) + ((uint32_t )cMessage[port - 1][14 + shift] << 8) + ((uint32_t )cMessage[port - 1][15 + shift] << 16) + ((uint32_t )cMessage[port - 1][16 + shift] << 24);
	 StepperIcInit( steppermode, Accelaration, Declaration,  MaxSpeed,  Overcurrent );
	 break;
     case CODE_H17R1_STEPPER_MOVE :
    	 Direction=cMessage[port - 1][shift];
    	 Steps =((uint32_t )cMessage[port - 1][1 + shift] ) + ((uint32_t )cMessage[port - 1][2 + shift] << 8) + ((uint32_t )cMessage[port - 1][3 + shift] << 16) + ((uint32_t )cMessage[port - 1][4 + shift] << 24);
         StepperMove(Direction,Steps);
	    break;
	    //************
     case CODE_H17R1_StepperRun :
         	 Direction=cMessage[port - 1][shift];
         	Speed =((uint32_t )cMessage[port - 1][1 + shift] ) + ((uint32_t )cMessage[port - 1][2 + shift] << 8) + ((uint32_t )cMessage[port - 1][3 + shift] << 16) + ((uint32_t )cMessage[port - 1][4 + shift] << 24);
            StepperRun(  Direction,  Speed);
         break;
    	 //*************
     case CODE_H17R1_StepperStop :
     mode=cMessage[port - 1][shift];
     StepperStop( mode );
      break;
         //*************
		default:
			result =H17R1_ERR_UnknownMessage;
			break;

	}
	
	return result;
}
/* --- Get the port for a given UART. 
 */
uint8_t GetPort(UART_HandleTypeDef *huart){

	if(huart->Instance == USART2)
		return P2;
	else if(huart->Instance == USART3)
		return P3;
	else if(huart->Instance == USART5)
		return P4;
	else if(huart->Instance == USART6)
		return P1;
	
	return 0;
}

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands
 */
void RegisterModuleCLICommands(void){
	FreeRTOS_CLIRegisterCommand(&CLI_StepperIcInitCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_StepperMoveCommandDefinition);
	 FreeRTOS_CLIRegisterCommand(&CLI_StepperRunCommandDefinition);
	 FreeRTOS_CLIRegisterCommand(&CLI_StepperStopCommandDefinition);
}

/*-----------------------------------------------------------*/


/* Module special task function (if needed) */
//void Module_Special_Task(void *argument){
//
//	/* Infinite loop */
//	uint8_t cases; // Test variable.
//	for(;;){
//		/*  */
//		switch(cases){
//
//
//			default:
//				osDelay(10);
//				break;
//		}
//
//		taskYIELD();
//	}
//
//}


/*-----------------------------------------------------------*/



/*-----------------------------------------------------------*/
union powerstep01_Init_u StepperIcInit_current_mode(float Accelaration_current,float Declaration_current, float MaxSpeed_current,float  Overcurrent_current )
{
	 union powerstep01_Init_u initDeviceParameters =
	{
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
	  (
	   POWERSTEP01_ALARM_EN_THERMAL_SHUTDOWN|
	   POWERSTEP01_ALARM_EN_THERMAL_WARNING|
	   POWERSTEP01_ALARM_EN_UVLO|
	   POWERSTEP01_ALARM_EN_STALL_DETECTION|
	   POWERSTEP01_ALARM_EN_SW_TURN_ON|
	   POWERSTEP01_ALARM_EN_WRONG_NPERF_CMD), // Alarm settings via bitmap enum powerstep01_AlarmEn_t
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
	  POWERSTEP01_CONFIG_INT_16MHZ, // Clock setting , enum powerstep01_ConfigOscMgmt_t
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
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
union powerstep01_Init_u StepperIcInit_voltage_mode(float Accelaration_voltage,float Declaration_voltage, float MaxSpeed_voltage,float  Overcurrent_voltage )
{
	union powerstep01_Init_u initDeviceParameters =
			{
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
			  (POWERSTEP01_ALARM_EN_OVERCURRENT|
			   POWERSTEP01_ALARM_EN_THERMAL_SHUTDOWN|
			   POWERSTEP01_ALARM_EN_THERMAL_WARNING|
			   POWERSTEP01_ALARM_EN_UVLO|
			   POWERSTEP01_ALARM_EN_STALL_DETECTION|
			   POWERSTEP01_ALARM_EN_SW_TURN_ON|
			   POWERSTEP01_ALARM_EN_WRONG_NPERF_CMD), // Alarm settings via bitmap enum powerstep01_AlarmEn_t
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
			  POWERSTEP01_CONFIG_INT_16MHZ_OSCOUT_2MHZ, // Clock setting , enum powerstep01_ConfigOscMgmt_t
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
/*-----------------------------------------------------------*/
/* -----------------------------------------------------------------------
 |								  APIs							          | 																 	|
/* -----------------------------------------------------------------------
 */

//**************API1*************************************************
Module_Status StepperIcInit(int8_t steppermode,float Accelaration,float Declaration, float MaxSpeed,float  Overcurrent )
{
	Module_Status status = H17R1_OK;
	if(steppermode !=0 && steppermode !=1  )
	{
		status = H17R1_ERR_WrongParams;
		return status;
	}

	if(Accelaration <14.55 ||  Accelaration > 59590)
		{
			status = H17R1_ERR_WrongParams;
			return status;
		}
	if(Declaration <14.55 ||  Declaration > 59590)
			{
				status = H17R1_ERR_WrongParams;
				return status;
			}

	if(MaxSpeed <15.25 ||  MaxSpeed > 15610)
				{
					status = H17R1_ERR_WrongParams;
					return status;
				}
	//Default Mode is current mode
	// set steppermode=0 to choose Current mode //the current mode is better when high speed is needed
	// set steppermode=1 to choose voltage mode
	if (steppermode==0)
	{
		//current mode
		initDeviceParameters=StepperIcInit_current_mode(Accelaration,Declaration,MaxSpeed,Overcurrent );
    }
	else
	{
		//voltage mode
		initDeviceParameters=StepperIcInit_voltage_mode(Accelaration,Declaration,MaxSpeed,Overcurrent );

	}
	/* Set the Powerstep01 library to use 1 device */
	Powerstep01_SetNbDevices(1);

	/* device with the union declared in the the H17R1.h file and comment the    */
	/* subsequent call having the NULL pointer                                  */
	Powerstep01_Init(&initDeviceParameters);

	/* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
	Powerstep01_AttachFlagInterrupt(MyFlagInterruptHandler);
	/* Attach the function MyBusyInterruptHandler (defined below) to the busy interrupt */
	Powerstep01_AttachBusyInterrupt(MyBusyInterruptHandler);
	/* Attach the function Error_Handler (defined below) to the error Handler*/
	Powerstep01_AttachErrorHandler(MyErrorHandler);
	 return status;
}
//**************API2*************************************************

// StepperMove : the motor wil move depending on the number of steps
Module_Status StepperMove( motorDir_t direction,  uint32_t n_step)
{
	Module_Status status = H17R1_OK;
	if( direction != 0 && direction != 1 )
		{
			status = H17R1_ERR_WrongParams;
			return status;
		}
	 Powerstep01_CmdMove(0, direction, n_step);
     Powerstep01_WaitWhileActive(0);
     return status;
}

//**************API3*************************************************

// StepperRun : the motor will run with the given speed unti it is stopped using StepperStop function
//  speed in 2^-28 step/tick
Module_Status StepperRun( motorDir_t direction, uint32_t speed)
{
	Module_Status status = H17R1_OK;
	if( direction != Backward && direction != Forward )
			{
				status = H17R1_ERR_WrongParams;
				return status;
			}
	if( speed > 15610 )
				{
					status = H17R1_ERR_WrongParams;
					return status;
				}

	 Powerstep01_CmdRun(0, direction, speed);
	 return status;
}

//**************API4*************************************************
Module_Status StepperStop(StoppingMethod mode )
{
	Module_Status status = H17R1_OK;
	if( mode != SoftStop && mode != HardStop && mode != Clock  )
				{
					status = H17R1_ERR_WrongParams;
					return status;
				}

	switch(mode)
	{
	case SoftStop :
		Powerstep01_CmdSoftStop(0);
		break;
	case HardStop :
    	Powerstep01_CmdHardStop(0);
    	break;
	case Clock :
	    Powerstep01_StopStepClock();
	    break;

	   default:
		break;
	}
	return status;
}


/*-----------------------------------------------------------*/


/* -----------------------------------------------------------------------
 |								Commands							      |
   -----------------------------------------------------------------------
 */
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_StepperIcInitCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H17R1_OK;
	int8_t steppermode;
	float Accelaration ,Declaration,MaxSpeed ,Overcurrent;

	static int8_t *pcParameterString1;
	static int8_t *pcParameterString2;
	static int8_t *pcParameterString3;
	static int8_t *pcParameterString4;
	static int8_t *pcParameterString5;

	portBASE_TYPE xParameterStringLength1 =0;
	portBASE_TYPE xParameterStringLength2 =0;
	portBASE_TYPE xParameterStringLength3 =0;
	portBASE_TYPE xParameterStringLength4 =0;
	portBASE_TYPE xParameterStringLength5 =0;


	static const int8_t *pcOKMessage=(int8_t* )"stepper is moving:\r\n %d  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";
	static const int8_t *pcWrongRangeMessage =(int8_t* )"Direction is not true!\n\r";


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
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,steppermode);

	}

	else if(status == H17R1_ERR_WrongParams)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);


	return pdFALSE;
}


/*-----------------------------------------------------------*/
portBASE_TYPE CLI_StepperMoveCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H17R1_OK;
	uint32_t steps;
		uint8_t direction;
	static int8_t *pcParameterString1;
	static int8_t *pcParameterString2;
	portBASE_TYPE xParameterStringLength1 =0;
	portBASE_TYPE xParameterStringLength2 =0;

	static const int8_t *pcOKMessage=(int8_t* )"stepper is moving:\r\n %d  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";
	static const int8_t *pcWrongRangeMessage =(int8_t* )"Direction is not true!\n\r";


	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
	direction =(uint8_t )atol((char* )pcParameterString1);

	pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength2 );
	steps =(uint32_t )atol((char* )pcParameterString2);
    status=StepperMove(direction,steps);

	if(status == H17R1_OK)
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,steps);

	}

	else if(status == H17R1_ERR_WrongParams)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);


	return pdFALSE;
}


/* ----------------------------------------------------------------------------*/
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_StepperRunCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H17R1_OK;
	uint32_t speed;
		uint8_t direction;
	static int8_t *pcParameterString1;
	static int8_t *pcParameterString2;
	portBASE_TYPE xParameterStringLength1 =0;
	portBASE_TYPE xParameterStringLength2 =0;

	static const int8_t *pcOKMessage=(int8_t* )"stepper is moving:\r\n %d  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";
	static const int8_t *pcWrongRangeMessage =(int8_t* )"Direction is not true!\n\r";


	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
	direction =(uint8_t )atol((char* )pcParameterString1);

	pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength2 );
	speed =(uint32_t )atol((char* )pcParameterString2);
    status=StepperRun(direction,speed);

	if(status == H17R1_OK)
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,speed);

	}

	else if(status == H17R1_ERR_WrongParams)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);


	return pdFALSE;
}
/*-----------------------------------------------------------*/
portBASE_TYPE CLI_StepperStopCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ){
	Module_Status status = H17R1_OK;
    uint8_t stopmode;
	static int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 =0;
	static const int8_t *pcOKMessage=(int8_t* )"stepper is stop:\r\n %d  \n\r";
	static const int8_t *pcWrongParamsMessage =(int8_t* )"Wrong Params!\n\r";
	static const int8_t *pcWrongRangeMessage =(int8_t* )"Direction is not true!\n\r";


	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1 );
	stopmode =(uint8_t )atol((char* )pcParameterString1);

	 status=StepperStop(stopmode);

	if(status == H17R1_OK)
	{
		sprintf((char* )pcWriteBuffer,(char* )pcOKMessage,stopmode);

	}

	else if(status == H17R1_ERR_WrongParams)
		strcpy((char* )pcWriteBuffer,(char* )pcWrongParamsMessage);


	return pdFALSE;
}




/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
