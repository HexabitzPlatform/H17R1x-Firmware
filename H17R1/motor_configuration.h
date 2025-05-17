/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : motor.h
 Description   : This file contains all the functions prototypes for motor drivers.

 if you want to add a new motor that is different from the ones in the existing file,
 you must calculate the motor parameters using the formulas below.

 1️) Acceleration (ACC) and Deceleration (DEC) Calculation:
    Step 1: Compute angular acceleration (α):
		a = T/J
		where:
		T:torque(N.M) , J:moment of inertia(Kg.m2) , a:Self-acceleration(red/s2)
		These parameters are found in the stepper motor specifications.
	Step 2: Convert angular acceleration to stepper units:
		acc = a * (200/2pi)
		where:
		200 = steps per revolution (for 1.8° step angle)
		acc = theoretical acceleration in steps/s²
	Step 3: Choose a safe value
		Recommended ACC = acc * Safety Factor
		where:
		Safety Factor = 0.01 to 0.05 (i.e., 1–5% of max acceleration)
	Step 4: set ACC and DEC:
		ACC = DEC = Recommended ACC
 2) Calculate Maximum Speed (MAX_SPEED) and Full-Step Threshold (FS_SPD):
    Step 1: Convert RPM to steps/sec:
    	steps/sec = (RPM * (steps/rev))/60
    	where:
    	steps/rev = usually 200 (for 1.8° motors)
    Step 2: Assign speed values:
    	MAX_SPEED = steps/sec
    	FS_SPD = MAX_SPEED × 0.5
 3) Calculate Torque Control Values (TVAL):
    Step 1: Calculate TVAL in mV
        TVAL(mv) = current * resistance * 1000
        Note : Clamp to 1000 mV maximum (PowerSTEP01 hardware limit)
    Step 2: Set values:
        TVAL_ACC  = TVAL
		TVAL_DEC  = TVAL
		TVAL_RUN  = TVAL
		TVAL_HOLD = TVAL × 0.5
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_PARAMETERS_CONFIG_H
#define __MOTOR_PARAMETERS_CONFIG_H

/* Motors ******************************************************************/
#define MOTOR_23HS8240
//#define MOTOR_17HS4401
//#define MOTOR_SY42STH38_1684A
//#define MOTOR_23HS45_4204S
//#define MOTOR_KL23H256_21_8B
//#define MOTOR_34HS59_5008D

#ifdef __cplusplus
 extern "C" {
#endif

#ifdef MOTOR_23HS8240
#define CONF_PARAM_ACC_DEVICE_0         (5000)
#define CONF_PARAM_DEC_DEVICE_0         (1000)
#define CONF_PARAM_MAX_SPEED_DEVICE_0   (15600)
#define CONF_PARAM_MIN_SPEED_DEVICE_0   (0)
#define CONF_PARAM_FS_SPD_DEVICE_0      (2000)
#define CONF_PARAM_TVAL_ACC_DEVICE_0    (32)
#define CONF_PARAM_TVAL_DEC_DEVICE_0    (32)
#define CONF_PARAM_TVAL_RUN_DEVICE_0    (32)
#define CONF_PARAM_TVAL_HOLD_DEVICE_0   (7.8)
#define CONF_PARAM_STEP_MODE_DEVICE_0   (STEP_MODE_1_16)
#define CONF_PARAM_CM_VM_DEVICE_0       (POWERSTEP01_CM_VM_CURRENT)
#endif

#ifdef MOTOR_17HS4401
#define CONF_PARAM_ACC_DEVICE_0         (3000)
#define CONF_PARAM_DEC_DEVICE_0         (3000)
#define CONF_PARAM_MAX_SPEED_DEVICE_0   (1000)
#define CONF_PARAM_MIN_SPEED_DEVICE_0   (0)
#define CONF_PARAM_FS_SPD_DEVICE_0      (500)
#define CONF_PARAM_TVAL_ACC_DEVICE_0    (1000)
#define CONF_PARAM_TVAL_DEC_DEVICE_0    (1000)
#define CONF_PARAM_TVAL_RUN_DEVICE_0    (1000)
#define CONF_PARAM_TVAL_HOLD_DEVICE_0   (500)
#define CONF_PARAM_STEP_MODE_DEVICE_0   (STEP_MODE_1_16)
#define CONF_PARAM_CM_VM_DEVICE_0       (POWERSTEP01_CM_VM_CURRENT)
#endif

#ifdef MOTOR_SY42STH38_1684A
#define CONF_PARAM_ACC_DEVICE_0         (3500)
#define CONF_PARAM_DEC_DEVICE_0         (3500)
#define CONF_PARAM_MAX_SPEED_DEVICE_0   (1200)
#define CONF_PARAM_MIN_SPEED_DEVICE_0   (0)
#define CONF_PARAM_FS_SPD_DEVICE_0      (600)
#define CONF_PARAM_TVAL_ACC_DEVICE_0    (1000)
#define CONF_PARAM_TVAL_DEC_DEVICE_0    (1000)
#define CONF_PARAM_TVAL_RUN_DEVICE_0    (1000)
#define CONF_PARAM_TVAL_HOLD_DEVICE_0   (500)
#define CONF_PARAM_STEP_MODE_DEVICE_0   (STEP_MODE_1_16)
#define CONF_PARAM_CM_VM_DEVICE_0       (POWERSTEP01_CM_VM_CURRENT)
#endif

#ifdef MOTOR_23HS45_4204S
#define CONF_PARAM_ACC_DEVICE_0         (3000)
#define CONF_PARAM_DEC_DEVICE_0         (3000)
#define CONF_PARAM_MAX_SPEED_DEVICE_0   (800)
#define CONF_PARAM_MIN_SPEED_DEVICE_0   (0)
#define CONF_PARAM_FS_SPD_DEVICE_0      (400)
#define CONF_PARAM_TVAL_ACC_DEVICE_0    (1000)
#define CONF_PARAM_TVAL_DEC_DEVICE_0    (1000)
#define CONF_PARAM_TVAL_RUN_DEVICE_0    (1000)
#define CONF_PARAM_TVAL_HOLD_DEVICE_0   (500)
#define CONF_PARAM_STEP_MODE_DEVICE_0   (STEP_MODE_1_16)
#define CONF_PARAM_CM_VM_DEVICE_0       (POWERSTEP01_CM_VM_CURRENT)
#endif

#ifdef MOTOR_KL23H256_21_8B
#define CONF_PARAM_ACC_DEVICE_0         (3000)
#define CONF_PARAM_DEC_DEVICE_0         (3000)
#define CONF_PARAM_MAX_SPEED_DEVICE_0   (900)
#define CONF_PARAM_MIN_SPEED_DEVICE_0   (0)
#define CONF_PARAM_FS_SPD_DEVICE_0      (450)
#define CONF_PARAM_TVAL_ACC_DEVICE_0    (1000)
#define CONF_PARAM_TVAL_DEC_DEVICE_0    (1000)
#define CONF_PARAM_TVAL_RUN_DEVICE_0    (1000)
#define CONF_PARAM_TVAL_HOLD_DEVICE_0   (500)
#define CONF_PARAM_STEP_MODE_DEVICE_0   (STEP_MODE_1_16)
#define CONF_PARAM_CM_VM_DEVICE_0       (POWERSTEP01_CM_VM_CURRENT)
#endif

#ifdef MOTOR_34HS59_5008D
#define CONF_PARAM_ACC_DEVICE_0         (2500)
#define CONF_PARAM_DEC_DEVICE_0         (2500)
#define CONF_PARAM_MAX_SPEED_DEVICE_0   (600)
#define CONF_PARAM_MIN_SPEED_DEVICE_0   (0)
#define CONF_PARAM_FS_SPD_DEVICE_0      (300)
#define CONF_PARAM_TVAL_ACC_DEVICE_0    (1000)
#define CONF_PARAM_TVAL_DEC_DEVICE_0    (1000)
#define CONF_PARAM_TVAL_RUN_DEVICE_0    (1000)
#define CONF_PARAM_TVAL_HOLD_DEVICE_0   (500)
#define CONF_PARAM_STEP_MODE_DEVICE_0   (STEP_MODE_1_16)
#define CONF_PARAM_CM_VM_DEVICE_0       (POWERSTEP01_CM_VM_CURRENT)
#endif



#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_PARAMETERS_CONFIG_H */

 /***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
