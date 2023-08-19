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
StepperMove(0,Backward,8000);

	    StepperRun(0, Backward, 3000);

	    StepperStop(0,SoftStop);
	    StepperRun(0, Forward, 6000);
	    StepperRun(0, Forward, 9000);
	    StepperStop(0,HardStop);

	    Powerstep01_CmdStepClock(0, Forward);
	    Powerstep01_StartStepClock(333);
	    StepperStop(0,Clock);
	    Powerstep01_StartStepClock(333*3);
	   	StepperStop(0,Clock);
	   	Powerstep01_StartStepClock(333*6);
	    StepperStop(0,Clock);
	    Powerstep01_StartStepClock(333*9);
	    StepperStop(0,Clock);

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
