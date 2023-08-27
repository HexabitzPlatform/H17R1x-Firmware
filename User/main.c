/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Main function ------------------------------------------------------------*/

int main(void){

	Module_Init();		//Initialize Module &  BitzOS

	//Don't place your code here.
	for(;;){}
}

/*-----------------------------------------------------------*/

/* User Task */
void UserTask(void *argument){
	    int32_t pos=0;
	    uint32_t readData;
	   // StepperIcInit(1,582,582, 488,281.25 );

	    StepperMove(0,10000);
/*
StepperMove(0,10000);
StepperStop(SoftStop);
	   StepperRun( Backward, 3000);
HAL_Delay(5000);
	    StepperStop(SoftStop);
	    HAL_Delay(5000);
	    StepperRun(1, 6000);
	    HAL_Delay(5000);
	    StepperStop(SoftStop);
	    HAL_Delay(5000);
	    StepperRun(Forward, 9000);
	    HAL_Delay(5000);
	    StepperStop(HardStop);
	    HAL_Delay(5000);
	    */
/*
	    //the functions below might be needed
	    Powerstep01_CmdStepClock(0, Forward);
	    Powerstep01_StartStepClock(333);
	    StepperStop(Clock);
	    Powerstep01_StartStepClock(333*3);
	   	StepperStop(Clock);
	   	Powerstep01_StartStepClock(333*6);
	    StepperStop(Clock);
	    Powerstep01_StartStepClock(333*9);
	    StepperStop(Clock);
*/
	//	    Powerstep01_CmdGoHome(0);
	//	    pos = Powerstep01_GetPosition(0);
	//	    Powerstep01_CmdMove(0, Backward, 16000);
	//	    Powerstep01_WaitWhileActive(0);
	//	    pos = Powerstep01_GetPosition(0);
	
	// put your code here, to run repeatedly.
	while(1){

	}
}

/*-----------------------------------------------------------*/
