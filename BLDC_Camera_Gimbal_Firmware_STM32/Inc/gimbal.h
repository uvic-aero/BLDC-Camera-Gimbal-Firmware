/*
 * gimbal_tasks.h
 *
 *  Created on: Jun 24, 2019
 *      Author: ntron
 */

#ifndef GIMBAL_H_
#define GIMBAL_H_

#include "angles.h"

/* The idea of this module (header/source) is that it will contain the
 * entire structure of the gimbal application.
 *
 * Main only has to:
 * 		- Call all peripheral init functions (generated by CubeMX)
 * 		- Call the init functions in here
 * 		- Start the FreeRTOS scheduler
 */

/* ================== INIT FUNCTIONS ===================== */
/// Top-level Init function
void Gimbal_Init(void);

/// Init the peripheral sensor modules
void Gimbal_InitSensors(void);

/// Init the queues used for inter-task comms
void Gimbal_InitQueues(void);

/// Init the tasks
void Gimbal_InitTasks(void);

/* ================== TASK FUNCTIONS ===================== */
/// IMU interrupt handler
void vImuIRQHandler(void* pvParameters);

/// Handler for UART receive interrupt
void vUartRxIRQHandler(void* pvParameters);

/// Task for transmitting data over UART
void vUartTxTask(void* pvParameters);

/// Primary Gimbal Control Loop
void vGimbalControlLoopTask(void* pvParameters);

/// Target setting task
void vTargetSettingTask(void* pvParameters);
/// others...

/* ============= HAL IRQ HANDLER CALLBACKS =============== */
/// defined in the source file, declared in various system headers

/* =============== CONTROL MATH FUNCTIONS ================ */
/// Wrapper for the "Get desired motor position" logic
/// This is wrapped since it will change for 2 vs 3 axis
EulerAngles_t Gimbal_CalcMotorTargetPos(EulerAngles_t currIMU, EulerAngles_t targIMU, EulerAngles_t currMotorPos);

#endif /* GIMBAL_H_ */