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
#include "rc_input.h"
#include "Comms.h"
/// others...

/*====================== DEFINES ========================= */
/// Task Priority Levels
/// TODO: these are fairly arbitrary right now, the ordering needs to be given more thought
#define PRIO_IMU					((UBaseType_t)10)
#define PRIO_CONTROL				((UBaseType_t)9)
#define PRIO_TARGETSET				((UBaseType_t)9)
#define PRIO_UARTRX					((UBaseType_t)7)
#define PRIO_RC						((UBaseType_t)6)

#define QSIZE_IMU					(1)
#define QSIZE_TARGET				(1)

/* ============= GLOBAL RESOURCE VARIABLES =============== */

static IMU_t imu;
static RC_Input_t rcPitch;
static RC_Input_t rcYaw;
static RC_Input_t rcMode;


/* ============== SYNCHRONIZATION OBJECTS ================ */

QueueHandle_t xIMUQueue;
QueueHandle_t xTargetQueue;

extern QueueHandle_t xEventsQueue;
extern QueueHandle_t xTargetPanQueue;
extern QueueHandle_t xTargetTiltQueue;
extern QueueHandle_t xSystemTimeQueue;
extern QueueHandle_t xDataTransmitQueue;

/* =================== TASK HANDLES ====================== */
/// IMU handler task handle
TaskHandle_t xTaskIMU;
TaskHandle_t xTaskUartRx;
TaskHandle_t xTaskGimbalControl;
TaskHandle_t xTaskTargetSet;
TaskHandle_t xTaskRcPitch;
TaskHandle_t xTaskRcYaw;
TaskHandle_t xTaskRcMode;

/* ================== INIT FUNCTIONS ===================== */
/// Top-level Init function
void Gimbal_Init(void)
{
	Comms_Init();
	Gimbal_InitSensors();
	Gimbal_InitQueues();
	Gimbal_InitTasks();

	//TODO: whatever else needs to go in here
}

/// Init the peripheral sensor modules
void Gimbal_InitSensors(void)
{
	IMU_Init(&imu, AXIS_IMU);

	RC_Init(&rcPitch, RC_INPUT_PITCH);
	RC_Init(&rcYaw, RC_INPUT_YAW);
	RC_Init(&rcMode, RC_INPUT_MODE);
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
	xTaskCreate(vRcModeHandler, "RcMode", configMINIMAL_STACK_SIZE, NULL, PRIO_RC, &xTaskRcMode);
	xTaskCreate(vRcPitchHandler, "RcPitch", configMINIMAL_STACK_SIZE, NULL, PRIO_RC, &xTaskRcPitch);
	xTaskCreate(vRcYawHandler, "RcYaw", configMINIMAL_STACK_SIZE, NULL, PRIO_RC, &xTaskRcYaw);
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

void vGimbalControlLoopTask(void* pvParameters)
{
	EulerAngles_t currCameraPos = {.pitch = 0.0, .yaw = 0.0, .roll = 0.0 };
	EulerAngles_t targetCameraPos = {.pitch = 0.0, .yaw = 0.0, .roll = 0.0 }; // TODO: find real initial target for startup
	//EulerAngles_t currMotorPos = {.pitch = 0.0, .yaw = 0.0, .roll = 0.0 };
	//EulerAngles_t targetMotorPos = {.pitch = 0.0, .yaw = 0.0, .roll = 0.0 };

	while(true)
	{
		// synchronize to arrival of IMU data
		xQueueReceive(xIMUQueue, &currCameraPos, portMAX_DELAY);
		//printf("%d, %d, %d\n", (int)(currCameraPos.pitch), (int)(currCameraPos.yaw), (int)(currCameraPos.roll));

		// try reading from target queue, if there is nothing, move on, don't wait
		// if there is something, the new target will be updated in targetCameraPos
		xQueueReceive(xTargetQueue, &targetCameraPos, (TickType_t)0);

		/// GET ENCODER VALUES HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// currentMotorPos.pitch = ...
		// currentMotorPos.roll = ...
		// currentMotorPos.yaw = ...

		//targetMotorPos = Gimbal_CalcMotorTargetPos(currCameraPos, targetCameraPos, currMotorPos);
		//printf("%d\n", targetMotorPos);

		// set the motor positions here // EXAMPLE!!!!! Not necessarily how it will look!
		// the PID loops are in here... maybe? maybe we should break them out and just expose motor duty cycle directly?
		// Motor_SetPos(&yaw_motor, targetMotorPos.yaw, currentMotorPos.yaw);
		// Motor_SetPos(&pitch_motor, targetMotorPos.pitch, currentMotorPos.pitch);
		// Motor_SetPos(&roll_motor, targetMotorPos.roll, currentMotorPos.roll);

	}
}


void vTargetSettingTask(void* pvParameters)
{
	//EulerAngles_t newTarget = {.pitch = 0.0, .yaw = 0.0, .roll = 0.0 };

	COMMS_Message newMsg = {.message = {.type = COMMS_Curr_Tilt, .value = 0x8888,}, .event = 0,};

	while(true)
	{
		vTaskDelay(5000);
		//xQueueSend(xTargetQueue, (void*)(&newTarget), (TickType_t)0);

		// send data down the queue
		xQueueSend(xDataTransmitQueue, (void*)(&newMsg), portMAX_DELAY);
	}
}

/// RC Pitch input task
void vRcPitchHandler(void* pvParameters)
{

	RC_StartInterrupts(&rcPitch);

	while(true)
	{
		// wait for IMU interrupt
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		RC_Update(&rcPitch);
	}
}

/// RC yaw input task
void vRcYawHandler(void* pvParameters)
{

	RC_StartInterrupts(&rcYaw);

	while(true)
	{
		// wait for IMU interrupt
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		RC_Update(&rcYaw);
	}
}

/// RC mode input task
void vRcModeHandler(void* pvParameters)
{

	RC_StartInterrupts(&rcMode);

	while(true)
	{
		// wait for IMU interrupt
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		RC_Update(&rcMode);
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

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// RC PITCH
	if ((htim->Instance == TIM2)  && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
	{
		vTaskNotifyGiveFromISR( xTaskRcPitch, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

	// RC YAW
	if ((htim->Instance == TIM8)  && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
	{
		vTaskNotifyGiveFromISR( xTaskRcYaw, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

	// RC MODE IRQ
	if ((htim->Instance == TIM15) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
	{
		vTaskNotifyGiveFromISR( xTaskRcMode, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/* =============== CONTROL MATH FUNCTIONS ================ */

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
