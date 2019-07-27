/*
 * motor.c
 *
 *  Created on: Jun. 9, 2019
 *      Author: Jake
 */

#include "motor.h"
// include encoder .h .c files

#define NUM_STEPS (384)

static const uint8_t waveform[NUM_STEPS] =	{
	  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	  0,  0,  0,  0,  0,  0,  0,  0,  4,  8, 12, 16, 20, 25, 29, 33, 37, 41, 45, 49, 54, 58, 62, 66,
	 70, 74, 78, 82, 86, 90, 94, 97,101,105,109,113,116,120,124,128,131,135,138,142,145,149,152,155,
	159,162,165,168,171,174,178,181,183,186,189,192,195,197,200,203,205,208,210,212,215,217,219,221,
	223,225,227,229,231,233,234,236,238,239,241,242,243,244,246,247,248,249,250,251,251,252,253,253,
	254,254,255,255,255,255,255,255,255,255,255,255,255,254,254,253,253,252,251,251,250,249,248,247,
	246,244,243,242,241,239,238,236,234,233,231,229,227,225,223,221,221,223,225,227,229,231,233,234,
	236,238,239,241,242,243,244,246,247,248,249,250,251,251,252,253,253,254,254,255,255,255,255,255,
	255,255,255,255,255,255,254,254,253,253,252,251,251,250,249,248,247,246,244,243,242,241,239,238,
	236,234,233,231,229,227,225,223,221,219,217,215,212,210,208,205,203,200,197,195,192,189,186,183,
	181,178,174,171,168,165,162,159,155,152,149,145,142,138,135,131,128,124,120,116,113,109,105,101,
	 97, 94, 90, 86, 82, 78, 74, 70, 66, 62, 58, 54, 49, 45, 41, 37, 33, 29, 25, 20, 16, 12,  8,  4
	};

void Motor_Init(MotorHandle_t motor, MotorIdentity_t identity)
{
	motor->identity = identity;
	motor->direction = MOTOR_TURN_CW;
	motor->phaseIndex[0] = 0;
	motor->phaseIndex[1] = motor->phaseIndex[0] + (NUM_STEPS / 3);
	motor->phaseIndex[2] = motor->phaseIndex[1] + (NUM_STEPS / 3);
	motor->maxPulseSize = 0x0;

	switch(identity)
	{
		case YAW_MOTOR:
		{
			motor->timer = &htim1;

			motor->phaseChannel[0] 	= 	TIM_CHANNEL_1;
			motor->phasePin[0] 		= 	MOTOR1_IN1_Pin;
			motor->phasePort[0] 	= 	MOTOR1_IN1_GPIO_Port;
			motor->enablePin[0]		= 	MOTOR1_EN1_Pin;
			motor->enablePort[0]	= 	MOTOR1_EN1_GPIO_Port;

			motor->phaseChannel[1] 	= 	TIM_CHANNEL_2;
			motor->phasePin[1] 		= 	MOTOR1_IN2_Pin;
			motor->phasePort[1] 	= 	MOTOR1_IN2_GPIO_Port;
			motor->enablePin[1] 	= 	MOTOR1_EN2_Pin;
			motor->enablePort[1] 	= 	MOTOR1_EN2_GPIO_Port;

			motor->phaseChannel[2] 	=	TIM_CHANNEL_3;
			motor->phasePin[2] 		=	MOTOR1_IN3_Pin;
			motor->phasePort[2]		=	MOTOR1_IN3_GPIO_Port;
			motor->enablePin[2]		=	MOTOR1_EN3_Pin;
			motor->enablePort[2]	=	MOTOR1_EN3_GPIO_Port;

			motor->nResetPort		=	MOTOR1_NRESET_GPIO_Port;
			motor->nResetPin		=	MOTOR1_NRESET_Pin;
			motor->nFaultPort		=	MOTOR1_NFAULT_GPIO_Port;
			motor->nFaultPin		=	MOTOR1_NFAULT_Pin;

			break;
		}
		case PITCH_MOTOR:
		{
			motor->timer = &htim3;

			motor->phaseChannel[0]	=	TIM_CHANNEL_1;
			motor->phasePin[0]		=	MOTOR2_IN1_Pin;
			motor->phasePort[0]		=	MOTOR2_IN1_GPIO_Port;
			motor->enablePin[0]		=	MOTOR2_EN1_Pin;
			motor->enablePort[0]	=	MOTOR2_EN1_GPIO_Port;

			motor->phaseChannel[1]	=	TIM_CHANNEL_2;
			motor->phasePin[1]		=	MOTOR2_IN2_Pin;
			motor->phasePort[1]		=	MOTOR2_IN2_GPIO_Port;
			motor->enablePin[1]		=	MOTOR2_EN2_Pin;
			motor->enablePort[1]	=	MOTOR2_EN2_GPIO_Port;

			motor->phaseChannel[2]	=	TIM_CHANNEL_3;
			motor->phasePin[2]		=	MOTOR2_IN3_Pin;
			motor->phasePort[2]		=	MOTOR2_IN3_GPIO_Port;
			motor->enablePin[2]		=	MOTOR2_EN3_Pin;
			motor->enablePort[2]	=	MOTOR2_EN3_GPIO_Port;

			motor->nResetPort		=	MOTOR2_NRESET_GPIO_Port;
			motor->nResetPin		=	MOTOR2_NRESET_Pin;
			motor->nFaultPort		=	MOTOR2_NFAULT_GPIO_Port;
			motor->nFaultPin		=	MOTOR2_NFAULT_Pin;

			break;
		}
		case ROLL_MOTOR:
		{
			motor->timer = &htim4;

			motor->phaseChannel[0]	=	TIM_CHANNEL_1;
			motor->phasePin[0]		=	MOTOR3_IN1_Pin;
			motor->phasePort[0]		=	MOTOR3_IN1_GPIO_Port;
			motor->enablePin[0]		=	MOTOR3_EN1_Pin;
			motor->enablePort[0]	=	MOTOR3_EN1_GPIO_Port;

			motor->phaseChannel[1]	=	TIM_CHANNEL_2;
			motor->phasePin[1]		=	MOTOR3_IN2_Pin;
			motor->phasePort[1]		=	MOTOR3_IN2_GPIO_Port;
			motor->enablePin[1]		=	MOTOR3_EN2_Pin;
			motor->enablePort[1]	=	MOTOR3_EN2_GPIO_Port;

			motor->phaseChannel[2]	=	TIM_CHANNEL_3;
			motor->phasePin[2]		=	MOTOR3_IN3_Pin;
			motor->phasePort[2]		=	MOTOR3_IN3_GPIO_Port;;
			motor->enablePin[2]		=	MOTOR3_EN3_Pin;
			motor->enablePort[2]	=	MOTOR3_EN3_GPIO_Port;

			motor->nResetPort		=	MOTOR3_NRESET_GPIO_Port;
			motor->nResetPin		=	MOTOR3_NRESET_Pin;
			motor->nFaultPort		=	MOTOR3_NFAULT_GPIO_Port;
			motor->nFaultPin		=	MOTOR3_NFAULT_Pin;

			break;
		}
	}

	Motor_EnableDriver(motor);
}

void Motor_SetOperationMode(MotorHandle_t motor, MotorOperationMode_t mode)
{
	HAL_TIM_PWM_Stop(motor->timer, motor->phaseChannel[0]);
	HAL_TIM_PWM_Stop(motor->timer, motor->phaseChannel[1]);
	HAL_TIM_PWM_Stop(motor->timer, motor->phaseChannel[2]);

	switch(mode)
	{
		case COMMUTATE:
		{
			HAL_GPIO_WritePin(motor->enablePort[0], motor->enablePin[0], GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor->enablePort[1], motor->enablePin[1], GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor->enablePort[2], motor->enablePin[2], GPIO_PIN_SET);
			break;
		}
		case BRAKE:
		{
			HAL_GPIO_WritePin(motor->enablePort[0], motor->enablePin[0], GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor->enablePort[1], motor->enablePin[1], GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor->enablePort[2], motor->enablePin[2], GPIO_PIN_SET);
			break;
		}
		case COAST:
		{
			HAL_GPIO_WritePin(motor->enablePort[0], motor->enablePin[0], GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->enablePort[1], motor->enablePin[1], GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->enablePort[2], motor->enablePin[2], GPIO_PIN_RESET);
			break;
		}
	}

	motor->mode = mode;
}

void Motor_Commutate(MotorHandle_t motor)
{
	if(motor->direction == MOTOR_TURN_CW)
	{
		motor->phaseIndex[0]++;
		if(motor->phaseIndex[0] >= NUM_STEPS) motor->phaseIndex[0] = 0;

		motor->phaseIndex[1]++;
		if(motor->phaseIndex[1] >= NUM_STEPS) motor->phaseIndex[1] = 0;

		motor->phaseIndex[2]++;
		if(motor->phaseIndex[2] >= NUM_STEPS) motor->phaseIndex[2] = 0;
	}

	else
	{
		motor->phaseIndex[0] = motor->phaseIndex[0] <= 0 ? (NUM_STEPS - 1) : (motor->phaseIndex[0] - 1);

		motor->phaseIndex[1] = motor->phaseIndex[1] <= 0 ? (NUM_STEPS - 1) : (motor->phaseIndex[1] - 1);

		motor->phaseIndex[2] = motor->phaseIndex[2] <= 0 ? (NUM_STEPS - 1) : (motor->phaseIndex[2] - 1);
	}

	Motor_Run(motor);
}

void Motor_Run(MotorHandle_t motor)
{
	if(waveform[motor->phaseIndex[0]] != 0)
	{
		__HAL_TIM_SET_COMPARE(motor->timer, motor->phaseChannel[0], ((motor->maxPulseSize * waveform[motor->phaseIndex[0]]) >> 8));
		HAL_TIM_PWM_Start(motor->timer, motor->phaseChannel[0]);
	}

	else
	{
		HAL_TIM_PWM_Stop(motor->timer, motor->phaseChannel[0]);
	}

	if(waveform[motor->phaseIndex[1]] != 0)
	{
		__HAL_TIM_SET_COMPARE(motor->timer, motor->phaseChannel[1], ((motor->maxPulseSize * waveform[motor->phaseIndex[1]]) >> 8));
		HAL_TIM_PWM_Start(motor->timer, motor->phaseChannel[1]);
	}

	else
	{
		HAL_TIM_PWM_Stop(motor->timer, motor->phaseChannel[1]);
	}

	if(waveform[motor->phaseIndex[2]] != 0)
	{
		__HAL_TIM_SET_COMPARE(motor->timer, motor->phaseChannel[2], ((motor->maxPulseSize * waveform[motor->phaseIndex[2]]) >> 8));
		HAL_TIM_PWM_Start(motor->timer, motor->phaseChannel[2]);
	}

	else
	{
		HAL_TIM_PWM_Stop(motor->timer, motor->phaseChannel[2]);
	}
}

void Motor_SetParams(MotorHandle_t motor, MotorDirection_t direction, uint8_t pulse)
{
	motor->direction = direction;
	motor->maxPulseSize = pulse;
}

void Motor_EnableDriver(MotorHandle_t motor)
{
	HAL_GPIO_WritePin(motor->nResetPort, motor->nResetPin, GPIO_PIN_SET);
}

void Motor_DisableDriver(MotorHandle_t motor)
{
	HAL_GPIO_WritePin(motor->nResetPort, motor->nResetPin, GPIO_PIN_RESET);
}
