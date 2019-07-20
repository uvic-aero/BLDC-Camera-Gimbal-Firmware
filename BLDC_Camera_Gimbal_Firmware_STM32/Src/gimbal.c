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
#include <math.h>

/// FreeRTOS Headers ///
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"

///  Module Headers ///
#include "encoder.h"
#include "imu.h"
#include "rc_input.h"
#include "Comms.h"
#include "motor.h"
/// others...

/*====================== DEFINES ========================= */
/// Task Priority Levels
/// TODO: these are fairly arbitrary right now, the ordering needs to be given more thought
#define PRIO_MOTOR					((UBaseType_t)11)
#define PRIO_IMU					((UBaseType_t)10)
#define PRIO_CONTROL				((UBaseType_t)9)
#define PRIO_TARGETSET				((UBaseType_t)9)
#define PRIO_UARTRX					((UBaseType_t)7)
#define PRIO_RC						((UBaseType_t)6)

#define QSIZE_IMU					(1)
#define QSIZE_TARGET				(1)
#define QSIZE_MOTOR					(1)

typedef struct PID_t
{
	float kp;
	float ki;
	float kd;
	float error_new;
	float error_old;
	float error_acc;
	float error_diff;
} PID_t;

/* ============= GLOBAL RESOURCE VARIABLES =============== */

extern TIM_HandleTypeDef htim16;
static IMU_t        imu;
static RC_Input_t   rcPitch;
static RC_Input_t   rcYaw;
static RC_Input_t   rcMode;
static Encoder_t 	pitchEncoder;
static Encoder_t 	yawEncoder;
static Encoder_t 	rollEncoder;
static Motor_t 	    pitchMotor;
static Motor_t 	    yawMotor;
static Motor_t 	    rollMotor;


/* ============== SYNCHRONIZATION OBJECTS ================ */

QueueHandle_t xIMUQueue;
QueueHandle_t xTargetQueue;
QueueHandle_t xMotorSpeedQueue;

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
TaskHandle_t xTaskMotor;
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

	//Encoder_Init(&pitchEncoder, PITCH_ENCODER, ENCODER_PITCH_I2C_ADDR);
	//Poll_Encoder(&pitchEncoder);
	//Set_Zero_Position(&pitchEncoder, pitchEncoder.angleRaw);
	Encoder_Init(&yawEncoder, YAW_ENCODER, ENCODER_YAW_I2C_ADDR);
	Poll_Encoder(&yawEncoder);
	Set_Zero_Position(&yawEncoder, yawEncoder.angleFloat);
	//Encoder_Init(&rollEncoder, ROLL_ENCODER, ENCODER_ROLL_I2C_ADDR);
	//Poll_Encoder(&rollEncoder);
	//Set_Zero_Position(&rollEncoder, rollEncoder.angleRaw);

	//Motor_Init(&pitchMotor, PITCH_MOTOR);
	//Set_Operation_Mode(&pitchMotor, COAST);
	Motor_Init(&yawMotor, YAW_MOTOR);
	Set_Operation_Mode(&yawMotor, COAST);
	//Motor_Init(&rollMotor, ROLL_MOTOR);
	//Set_Operation_Mode(&rollMotor, COAST);


	/// others... including any calibration required
}

void Gimbal_InitQueues(void)
{
	// IMU queue is of size 1, always overwrite
	//xIMUQueue = xQueueCreate(1, sizeof(EulerAngles_t));
	//vQueueAddToRegistry(xIMUQueue, "IMU_Q");

	// Target queue will only be read sporadically because
	// target update rate will be low relative to control loop rate
	xTargetQueue = xQueueCreate(1, sizeof(EulerAngles_t));
	vQueueAddToRegistry(xTargetQueue, "Target_Q");

	xMotorSpeedQueue = xQueueCreate(1, sizeof(float));
	vQueueAddToRegistry(xMotorSpeedQueue, "Motor_Q");

	/// do UART later
}

/// Init the tasks
void Gimbal_InitTasks(void)
{
	//xTaskCreate(vImuIRQHandler,"IMUHandler",configMINIMAL_STACK_SIZE, NULL, PRIO_IMU, &xTaskIMU);
	xTaskCreate(vRcModeHandler, "RcMode", configMINIMAL_STACK_SIZE, NULL, PRIO_RC, &xTaskRcMode);
	xTaskCreate(vRcPitchHandler, "RcPitch", configMINIMAL_STACK_SIZE, NULL, PRIO_RC, &xTaskRcPitch);
	xTaskCreate(vRcYawHandler, "RcYaw", configMINIMAL_STACK_SIZE, NULL, PRIO_RC, &xTaskRcYaw);
	xTaskCreate(vGimbalControlLoopTask, "CtrlLoop", configMINIMAL_STACK_SIZE, NULL, PRIO_CONTROL, &xTaskGimbalControl);
	xTaskCreate(vTargetSettingTask, "TargetSet", configMINIMAL_STACK_SIZE, NULL, PRIO_TARGETSET, &xTaskTargetSet );
	xTaskCreate(vMotorCommutationTask, "MotorCom", configMINIMAL_STACK_SIZE, NULL, PRIO_MOTOR, &xTaskMotor);
	/// others...
}

/* ================== TASK FUNCTIONS ===================== */
/// IMU interrupt handler
/*
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
*/

void vGimbalControlLoopTask(void* pvParameters)
{
	EulerAngles_t currCameraPos = {.pitch = 0.0, .yaw = 0.0, .roll = 0.0 };
	EulerAngles_t targetCameraPos = {.pitch = 0.0, .yaw = 0.0, .roll = 0.0 }; // TODO: find real initial target for startup
	EulerAngles_t currMotorPos = {.pitch = 0.0, .yaw = 0.0, .roll = 0.0 };
	EulerAngles_t targetMotorPos = {.pitch = 0.0, .yaw = 0.0, .roll = 0.0 };

	EulerAngles_t temp = {.pitch = 0.0, .yaw = 0.0, .roll = 0.0 };

	//PID_t pitchPID 	= 	{.kp = 1.0, .kd = 0.0, .ki = 0.0, .error_new = 0.0, .error_old = 0.0, .error_acc = 0.0, .error_diff = 0.0 };
	PID_t yawPID 	=	{.kp = YAW_MOTOR_KP, .kd = YAW_MOTOR_KD, .ki = 0.0, .error_new = 0.0, .error_old = 0.0, .error_acc = 0.0, .error_diff = 0.0 };
	//PID_t rollPID 	=	{.kp = 1.0, .kd = 0.0, .ki = 0.0, .error_new = 0.0, .error_old = 0.0, .error_acc = 0.0, .error_diff = 0.0 };

	IMU_Start(&imu);

	//int encoder_counter = 0;

	while(true)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		IMU_GetQuaternion(&imu);
		IMU_CalcEulerAngles(&imu);

		currCameraPos = imu.pos;

		// synchronize to arrival of IMU data
		//xQueueReceive(xIMUQueue, &currCameraPos, portMAX_DELAY);
		//printf("%d, %d, %d\n", (int)(currCameraPos.pitch), (int)(currCameraPos.yaw), (int)(currCameraPos.roll));
		//vTaskDelay(1000);
		//vTaskDelay(20); // every 20 ms = 50 Hz
		// try reading from target queue, if there is nothing, move on, don't wait
		// if there is something, the new target will be updated in targetCameraPos
		if ( xQueueReceive(xTargetQueue, &temp, (TickType_t)0) == pdTRUE )
		{
			targetCameraPos.pitch = temp.pitch;
			targetCameraPos.yaw = temp.yaw;
			targetCameraPos.roll = temp.roll;
		}

		/// Get encoder values
		//currMotorPos.pitch 	= Angles_Normalize180( Poll_Encoder(&pitchEncoder) );
		currMotorPos.yaw 	= Angles_Normalize180( Poll_Encoder(&yawEncoder) );

		/*
		if (encoder_counter == 0)
			printf("Encoder val: %d\n", (int)(currMotorPos.yaw));

		encoder_counter = (encoder_counter + 1 ) % 100;
		*/

		//currMotorPos.roll 	= Angles_Normalize180( Poll_Encoder(&rollEncoder) );

		targetMotorPos = Gimbal_CalcMotorTargetPos(currCameraPos, targetCameraPos, currMotorPos);
		//printf("%d\n", targetMotorPos);

		// save previous PID error values
		//pitchPID.error_old = pitchPID.error_new;
		yawPID.error_old = yawPID.error_new;
		//rollPID.error_old = rollPID.error_new;

		// get new PID error values
		//pitchPID.error_new = Angles_CalcDist(targetMotorPos.pitch, currMotorPos.pitch);
		yawPID.error_new = Angles_CalcDist(targetMotorPos.yaw, currMotorPos.yaw);
		//rollPID.error_new = Angles_CalcDist(targetMotorPos.roll, currMotorPos.roll);

		// calculate accumulation error
		//pitchPID.error_acc += pitchPID.error_new; // TODO: could saturate here or do some sort of attenuation
		yawPID.error_acc += yawPID.error_new; // TODO: could saturate here or do some sort of attenuation
		//rollPID.error_acc += pitchPID.error_new; // TODO: could saturate here or do some sort of attenuation

		// calculate error differential
		//pitchPID.error_diff = Angles_CalcDist(pitchPID.error_new, pitchPID.error_old);
		yawPID.error_diff = yawPID.error_new - yawPID.error_old;
		//rollPID.error_diff = Angles_CalcDist(rollPID.error_new, rollPID.error_old);

		//float pitchSpeed 	= pitchPID.kp * pitchPID.error_new + pitchPID.ki * pitchPID.error_acc + pitchPID.kd * pitchPID.error_diff;
		float yawSpeed 		= yawPID.kp * yawPID.error_new + yawPID.ki * yawPID.error_acc + yawPID.kd * yawPID.error_diff;
		//float rollSpeed 	= rollPID.kp * rollPID.error_new + rollPID.ki * rollPID.error_acc + rollPID.kd * rollPID.error_diff;

		//printf("Yaw speed: %d\n", (int)yawSpeed);

		//xQueueSend(xMotorSpeedQueue, (void*)&yawSpeed, (TickType_t)0);
		xQueueOverwrite(xMotorSpeedQueue, (void*)&yawSpeed);
	}
}


void vTargetSettingTask(void* pvParameters)
{

	COMMS_Data_Message targetPanDelta = {0};
	COMMS_Data_Message targetTiltDelta = {0};
	float targetPan = 0.0;
	float targetTilt = 0.0;
	EulerAngles_t targetPos = {0};

	while(true)
	{
		vTaskDelay(100);
		// wake up and try to read from Pan queue
		if ( xQueueReceive(xTargetPanQueue, (void*)&targetPanDelta, (TickType_t)0) == pdTRUE )
		{
			//printf("Target pan delta: %d\n", (int16_t)(targetPanDelta.value));
			// add target to delta
			targetPan += (float)(targetPanDelta.value);
		}

		if ( xQueueReceive(xTargetTiltQueue, (void*)&targetTiltDelta, (TickType_t)0) == pdTRUE )
		{
			//printf("Target tilt delta: %d\n", (int16_t)(targetTiltDelta.value));
			targetTilt += (float)(targetTiltDelta.value);
		}

		// TODO: translation to target EulerAngle coords happens here

		targetPos.yaw = targetPan;
		targetPos.pitch = targetTilt;

		if (abs(targetTilt) > 80.0)
			targetTilt = copysignf(80.0, targetTilt);

		// send to
		xQueueSend(xTargetQueue, (void*)&targetPos, (TickType_t)0);

		// reset deltas to 0 to make sure it doesn't keep getting added
		targetTiltDelta.value = 0.0;
		targetPanDelta.value = 0.0;
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


void vMotorCommutationTask(void* pvParameters)
{

	Set_Operation_Mode(&yawMotor, COMMUTATE);
	Set_Motor_Parameters(&yawMotor, TURN_CCW, 0);
	float speed = 0.0;
	uint32_t delay = MOTOR_MAX_COMMUTATION_DELAY;
	uint8_t pulse = 0;
	uint8_t dir = TURN_CCW;
	do
	{
		// check to see if theres a new PID output
		xQueueReceive(xMotorSpeedQueue, (void*)&speed, (TickType_t)0);

		Gimbal_CalcMotorParams(speed, &delay, &pulse, &dir);

		Set_Motor_Parameters(&yawMotor, dir, pulse);
		Commutate_Motor(&yawMotor);
		//vTaskDelay(delay);

		// start interrupt for delay and wait for notification from TIM7 ISR
		__HAL_TIM_SET_COUNTER(&htim16, 0xffff - (uint16_t)(delay & 0x0000FFFF));
		HAL_TIM_Base_Start_IT(&htim16);

		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	}
	while(true);
}

/* ============= HAL IRQ HANDLER CALLBACKS =============== */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// Handoff to IMU handler task
	if (GPIO_Pin == AXIS_IMU_INT_Pin)
	{
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); /// TODO: temporary, remove this
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		//vTaskNotifyGiveFromISR( xTaskIMU, &xHigherPriorityTaskWoken );
		vTaskNotifyGiveFromISR( xTaskGimbalControl, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

//static unsigned int led_counter = 0;
/// for the microsecond motor commutation timer
void MOTOR_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM16)
	{
		/*
		if (led_counter == 0)
		{
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		}
		led_counter = (led_counter + 1) % 100;
		*/
		HAL_TIM_Base_Stop_IT(&htim16);
		// yield to motor commutation task
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR( xTaskMotor, &xHigherPriorityTaskWoken );
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

static uint8_t Gimbal_CalcMotorPulse(float speed)
{
	//float pulse_val = 255.0 * sqrt(fabs(speed)) / sqrt(MOTOR_MAX_SPEED);
	float pulse_val = (MOTOR_PULSE_RANGE) * (1.0 - exp(-MOTOR_PULSE_CURVE_VAL*fabs(speed) / MOTOR_MAX_SPEED)) + MOTOR_MIN_PULSE;
	if (pulse_val > 255.0) pulse_val = 255.0;
	return (uint8_t)fabs(pulse_val);
}

// speed in degrees per second
void Gimbal_CalcMotorParams(float speed_in, uint32_t* delay_out, uint8_t* pulse_out, uint8_t* dir_out)
{
	// =========== 1. DIR ====================================
	// extract sign to determine delay and direction
	if (speed_in >= 0.0) *dir_out = TURN_CCW; // + -> CCW
	else *dir_out = TURN_CW;				// - -> CW
	// =======================================================

	// =========== 2. PULSE ==================================

	// get speed magnitude
	float abs_speed = fabs(speed_in);

	// calculate pulse value // TODO: PLACEHOLDER VALUES
	*pulse_out = Gimbal_CalcMotorPulse(abs_speed);
	// =======================================================

	// =========== 3. DELAY ==================================

	// saturate the speed to within the max and min
	// this is very important to prevent ill-conditioned math!
	// the speed_in variable can be very close to 0 or even zero

	if (abs_speed >= MOTOR_MAX_SPEED)
		abs_speed = MOTOR_MAX_SPEED;
	else if (abs_speed <= MOTOR_MIN_SPEED)
		abs_speed = MOTOR_MIN_SPEED;

	// calculate delay from speed
	// there are 384 steps per electrical revolution, and 7*384 = 2688 steps per mechanical revolution
	// there are 7 pole pairs
	// mech_revs_per_sec = (speed (deg/s)) / 360
	// mech_rev_period (ms) = 1000 / (mech_revs_per_sec)
	// elec_rev_period (ms) = mech_rev_period / num_pole_pairs
	// delay (ms) = elec_rev_period / 384
	// total:
	// delay = (1000 / (speed / 360)) / 7 / 384
	// delay = 133.92857 / speed
	uint32_t temp_delay = (uint32_t)(MOTOR_CONVERSION_CONSTANT / abs_speed);

	// saturate delay (in case of ill conditioning leading to slightly out of bounds results)
	if (temp_delay >= MOTOR_MAX_COMMUTATION_DELAY)
		temp_delay = MOTOR_MAX_COMMUTATION_DELAY;
	else if (temp_delay <= MOTOR_MIN_COMMUTATION_DELAY)
		temp_delay = MOTOR_MIN_COMMUTATION_DELAY;

	*delay_out = temp_delay;

	// ===========================================================

}


EulerAngles_t Gimbal_CalcMotorTargetPos(EulerAngles_t currIMU, EulerAngles_t targIMU, EulerAngles_t currMotorPos)
{
	EulerAngles_t targMotorPos = {.pitch = 0.0, .yaw = 0.0, .roll = 0.0,};

#if ENABLED(MODE_1AXIS)
	float errYaw	=	Angles_CalcDist(targIMU.pitch,  -currIMU.pitch); // MOTOR YAW MAPPED TO IMU PITCH TEMPORARILY!!!!!!!!!!!!!!!!

	if (fabs(errYaw) < 3.0)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	targMotorPos.yaw	= Angles_Normalize180(currMotorPos.yaw + errYaw);

	// try to cap the value so we don't go past +/- 90deg
	if (fabs(targMotorPos.yaw) > 70.0 )
		targMotorPos.yaw = copysignf(70.0, targMotorPos.yaw);


#elif ENABLED(MODE_2AXIS)

	// assume we are controlling pitch and roll

	// get the error from the IMU position
	float errPitch	=	Angles_CalcDist(targIMU.pitch, currIMU.pitch);
	float errRoll	=	Angles_CalcDist(targIMU.roll,  currIMU.roll);

	// for two axis control, we just need to calculate the next motor position
	// by adding the IMU error to current motor position
	// i.e. move in the direction that brings us towards the target
	targMotorPos.pitch	= Angles_Normalize180(currMotorPos.pitch + errPitch);
	targMotorPos.roll	= Angles_Normalize180(currMotorPos.roll + errRoll);

#elif ENABLED(MODE_3AXIS)
#error 3-axis aint enabled yet yo
#error its an inverse kinematic problem son
#endif
	return targMotorPos;
}
