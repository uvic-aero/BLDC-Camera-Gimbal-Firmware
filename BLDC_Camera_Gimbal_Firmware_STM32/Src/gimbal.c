/*
 * gimbal_tasks.c
 *
 *  Created on: Jun 24, 2019
 *      Author: ntron
 */
/*====================== INCLUDES ======================== */

/// Project Top-Level Headers ///
#include "main.h"
#include "gimbal.h"

/// FreeRTOS Headers ///
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"

///  Module Headers ///
#include "imu.h"
/// others...

/*====================== DEFINES ========================= */
/// Task Priority Levels
/// TODO: these are fairly arbitrary right now, the ordering needs to be given more thought
#define PRIO_IMU					((UBaseType_t)10)
#define PRIO_CONTROL				((UBaseType_t)9)
#define PRIO_TARGETSET				((UBaseType_t)9)
#define PRIO_UARTRX					((UBaseType_t)7)

#define QSIZE_IMU					(1)
#define QSIZE_TARGET				(1)

/* ============= GLOBAL RESOURCE VARIABLES =============== */

static volatile IMU_t imu;

/* ============== SYNCHRONIZATION OBJECTS ================ */

QueueHandle_t xIMUQueue;
QueueHandle_t xTargetQueue;
QueueHandle_t xUartTxQueue;

/* =================== TASK HANDLES ====================== */
/// IMU handler task handle
TaskHandle_t xTaskIMU;
TaskHandle_t xTaskUartRx;
TaskHandle_t xTaskGimbalControl;
TaskHandle_t xTaskTargetSet;

/* ================== INIT FUNCTIONS ===================== */
/// Top-level Init function
void Gimbal_Init(void)
{
	Gimbal_InitSensors();
	Gimbal_InitQueues();
	Gimbal_InitTasks();

	//TODO: whatever else needs to go in here
}

/// Init the peripheral sensor modules
void Gimbal_InitSensors(void)
{
	IMU_Init(&imu, AXIS_IMU);
	/// others... including any calibration required
}

void Gimbal_InitQueues(void)
{
	// IMU queue is of size 1, always overwrite
	xIMUQueue = xQueueCreate(1, sizeof(EulerAngles_t));
	vQueueAddToRegistry(xIMUQueue, "IMU_Q");

	// Target queue will only be read sporadically because
	// target update rate will be low relative to control loop rate
	xTargetQueue = xQueueCreate(1, sizeof(EulerAngles_t));
	vQueueAddToRegistry(xTargetQueue, "Target_Q");

	/// do UART later
}

/// Init the tasks
void Gimbal_InitTasks(void)
{
	xTaskCreate(vImuIRQHandler,"IMUHandler",configMINIMAL_STACK_SIZE, NULL, PRIO_IMU, &xTaskIMU);
	xTaskCreate(vGimbalControlLoopTask, "CtrlLoop", configMINIMAL_STACK_SIZE, NULL, PRIO_CONTROL, &xTaskGimbalControl);
	xTaskCreate(vTargetSettingTask, "TargetSet", configMINIMAL_STACK_SIZE, NULL, PRIO_TARGETSET, &xTaskTargetSet );
	/// others...
}

/* ================== TASK FUNCTIONS ===================== */
/// IMU interrupt handler
void vImuIRQHandler(void* pvParameters)
{
	// initialize the IMU, this needs to go here to prevent the fifo from starting interrupts
	IMU_Start(&imu);

	while(true)
	{
		// wait for IMU interrupt
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		IMU_GetQuaternion(&imu);
		IMU_CalcEulerAngles(&imu);

		xQueueSend(xIMUQueue, (void*)&(imu.pos), (TickType_t)0);
	}
}

void vUartRxIRQHandler(void* pvParameters)
{
	while(true)
	{
		vTaskDelay(1000);
	}
}

void vUartTxTask(void* pvParameters)
{
	while(true)
	{
		vTaskDelay(1000);
	}
}

void vGimbalControlLoopTask(void* pvParameters)
{
	EulerAngles_t currCameraPos = {.pitch = 0.0, .yaw = 0.0, .roll = 0.0 };
	EulerAngles_t targetCameraPos = {.pitch = 0.0, .yaw = 0.0, .roll = 0.0 }; // TODO: find real initial target for startup
	EulerAngles_t currMotorPos = {.pitch = 0.0, .yaw = 0.0, .roll = 0.0 };
	EulerAngles_t targetMotorPos = {.pitch = 0.0, .yaw = 0.0, .roll = 0.0 };

	while(true)
	{
		// synchronize to arrival of IMU data
		xQueueReceive(xIMUQueue, &currCameraPos, portMAX_DELAY);
		printf("%d, %d, %d\n", (int)(currCameraPos.pitch), (int)(currCameraPos.yaw), (int)(currCameraPos.roll));

		// try reading from target queue, if there is nothing, move on, don't wait
		// if there is something, the new target will be updated in targetCameraPos
		xQueueReceive(xTargetQueue, &targetCameraPos, (TickType_t)0);

		/// GET ENCODER VALUES HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// currentMotorPos.pitch = ...
		// currentMotorPos.roll = ...
		// currentMotorPos.yaw = ...

		targetMotorPos = Gimbal_CalcMotorTargetPos(currCameraPos, targetCameraPos, currMotorPos);
		printf("%d\n", targetMotorPos);

		// set the motor positions here // EXAMPLE!!!!! Not necessarily how it will look!
		// the PID loops are in here... maybe? maybe we should break them out and just expose motor duty cycle directly?
		// Motor_SetPos(&yaw_motor, targetMotorPos.yaw, currentMotorPos.yaw);
		// Motor_SetPos(&pitch_motor, targetMotorPos.pitch, currentMotorPos.pitch);
		// Motor_SetPos(&roll_motor, targetMotorPos.roll, currentMotorPos.roll);

	}
}


void vTargetSettingTask(void* pvParameters)
{
	EulerAngles_t newTarget = {.pitch = 0.0, .yaw = 0.0, .roll = 0.0 };
	while(true)
	{
		vTaskDelay(1000);
		xQueueSend(xTargetQueue, (void*)(&newTarget), (TickType_t)0);
	}
}



/* ============= HAL IRQ HANDLER CALLBACKS =============== */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// Handoff to IMU handler task
	if (GPIO_Pin == AXIS_IMU_INT_Pin)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); /// TODO: temporary, remove this
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR( xTaskIMU, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

EulerAngles_t Gimbal_CalcMotorTargetPos(EulerAngles_t currIMU, EulerAngles_t targIMU, EulerAngles_t currMotorPos)
{
	EulerAngles_t targMotorPos = {.pitch = 0.0, .yaw = 0.0, .roll = 0.0,};

#if ENABLED(MODE_2AXIS)

	// assume we are controlling pitch and roll

	// get the error from the IMU position
	float errPitch	=	Angles_CalcDist(targIMU.pitch, currIMU.pitch);
	float errRoll	=	Angles_CalcDist(targIMU.roll,  currIMU.roll);

	// for two axis control, we just need to calculate the next motor position
	// by adding the IMU error to current motor position
	// i.e. move in the direction that brings us towards the target
	targMotorPos.pitch	+= errPitch;
	targMotorPos.roll	+= errRoll;

#elif ENABLED(MODE_3AXIS)
#error 3-axis aint enabled yet yo
#error its an inverse kinematic problem son
#endif
	return targMotorPos;
}
