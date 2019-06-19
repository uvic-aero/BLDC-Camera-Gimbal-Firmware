/*
 * motor.c
 *
 *  Created on: Jun. 9, 2019
 *      Author: Jake
 */

#include "motor.h"
// include encoder .h .c files

void Motor_Init(Motor_Handle_t motor, Motor_Identity_t identity, Operation_Mode_t mode)
{
	switch(identity)
	{
		case PITCH_MOTOR:
		{
			motor->identity = identity;
			motor->mode = mode;
			motor->state = STATE_1;
			motor->direction = TURN_CW;
			motor->speedTarget = 0.0;
			motor->angleTarget = 0.0;

			motor->timer = &htim1;

			motor->phaseChannelA = TIM_CHANNEL_1;
			motor->phasePinA = MOTOR1_IN1_Pin;
			motor->phasePortA = MOTOR1_IN1_GPIO_Port;
			motor->enablePinA = MOTOR1_EN1_Pin;
			motor->enablePortA = MOTOR1_EN1_GPIO_Port;

			motor->phaseChannelB = TIM_CHANNEL_2;
			motor->phasePinB = MOTOR1_IN2_Pin;
			motor->phasePortB = MOTOR1_IN2_GPIO_Port;
			motor->enablePinB = MOTOR1_EN2_Pin;
			motor->enablePortB = MOTOR1_EN2_GPIO_Port;

			motor->phaseChannelC = TIM_CHANNEL_3;
			motor->phasePinC = MOTOR1_IN3_Pin;
			motor->phasePortC = MOTOR1_IN3_GPIO_Port;
			motor->enablePinC = MOTOR1_EN3_Pin;
			motor->enablePortC = MOTOR1_EN2_GPIO_Port;
			break;
		}
		case YAW_MOTOR:
		{
			motor->identity = identity;
			motor->mode = mode;
			motor->state = STATE_1;
			motor->direction = TURN_CW;
			motor->speedTarget = 0.0;
			motor->angleTarget = 0.0;

			motor->timer = &htim3;

			motor->phaseChannelA = TIM_CHANNEL_1;
			motor->phasePinA = MOTOR2_IN1_Pin;
			motor->phasePortA = MOTOR2_IN1_GPIO_Port;
			motor->enablePinA = MOTOR2_EN1_Pin;
			motor->enablePortA = MOTOR2_EN1_GPIO_Port;

			motor->phaseChannelB = TIM_CHANNEL_2;
			motor->phasePinB = MOTOR2_IN2_Pin;
			motor->phasePortB = MOTOR2_IN2_GPIO_Port;
			motor->enablePinB = MOTOR2_EN2_Pin;
			motor->enablePortB = MOTOR2_EN2_GPIO_Port;

			motor->phaseChannelC = TIM_CHANNEL_3;
			motor->phasePinC = MOTOR2_IN3_Pin;
			motor->phasePortC = MOTOR2_IN3_GPIO_Port;
			motor->enablePinC = MOTOR2_EN3_Pin;
			motor->enablePortC = MOTOR2_EN3_GPIO_Port;
			break;
		}
		case ROLL_MOTOR:
		{
			motor->identity = identity;
			motor->mode = mode;
			motor->state = STATE_1;
			motor->direction = TURN_CW;
			motor->speedTarget = 0.0;
			motor->angleTarget = 0.0;

			motor->timer = &htim4;

			motor->phaseChannelA = TIM_CHANNEL_1;
			motor->phasePinA = MOTOR3_IN1_Pin;
			motor->phasePortA = MOTOR3_IN1_GPIO_Port;
			motor->enablePinA = MOTOR3_EN1_Pin;
			motor->enablePortA = MOTOR3_EN1_GPIO_Port;

			motor->phaseChannelB = TIM_CHANNEL_2;
			motor->phasePinB = MOTOR3_IN2_Pin;
			motor->phasePortB = MOTOR3_IN2_GPIO_Port;
			motor->enablePinB = MOTOR3_EN2_Pin;
			motor->enablePortB = MOTOR3_EN2_GPIO_Port;

			motor->phaseChannelC = TIM_CHANNEL_3;
			motor->phasePinC = MOTOR3_IN3_Pin;
			motor->phasePortC = MOTOR3_IN3_GPIO_Port;;
			motor->enablePinC = MOTOR2_EN3_Pin;
			motor->enablePortC = MOTOR3_EN3_GPIO_Port;
			break;
		}
	}
}

void Set_Operation_Mode(Motor_Handle_t motor, Operation_Mode_t mode)
{
	HAL_TIM_PWM_Stop(motor->timer, motor->phaseChannelA);
	HAL_TIM_PWM_Stop(motor->timer, motor->phaseChannelB);
	HAL_TIM_PWM_Stop(motor->timer, motor->phaseChannelC);

	switch(mode)
	{
		case COMMUTATE:
		{
			switch(motor->state)
			{
				case STATE_1:
				{
					if (motor->direction == TURN_CW) {motor->state = STATE_2;}
					else {motor->state = STATE_6;}
					break;
				}
				case STATE_2:
				{
					if (motor->direction == TURN_CW) {motor->state = STATE_3;}
					else {motor->state = STATE_1;}
					break;
				}
				case STATE_3:
				{
					if (motor->direction == TURN_CW) {motor->state = STATE_4;}
					else {motor->state = STATE_2;}
					break;
				}
				case STATE_4:
				{
					if (motor->direction == TURN_CW) {motor->state = STATE_5;}
					else {motor->state = STATE_3;}
					break;
				}
				case STATE_5:
				{
					if (motor->direction == TURN_CW) {motor->state = STATE_6;}
					else {motor->state = STATE_4;}
					break;
				}
				case STATE_6:
				{
					if (motor->direction == TURN_CW) {motor->state = STATE_1;}
					else {motor->state = STATE_5;}
					break;
				}
			}

			Run_Motor(motor);
			break;
		}
		case BRAKE:
		{
			HAL_GPIO_WritePin(motor->phasePortA, motor->phasePinA, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->phasePortB, motor->phasePinB, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->phasePortC, motor->phasePinC, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(motor->enablePortA, motor->enablePinA, GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor->enablePortB, motor->enablePinB, GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor->enablePortC, motor->enablePinC, GPIO_PIN_SET);
			break;
		}
		case COAST:
		{
			HAL_GPIO_WritePin(motor->phasePortA, motor->phasePinA, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->phasePortB, motor->phasePinB, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->phasePortC, motor->phasePinC, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(motor->enablePortA, motor->enablePinA, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->enablePortB, motor->enablePinB, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->enablePortC, motor->enablePinC, GPIO_PIN_RESET);
			break;
		}
	}

	motor->mode = mode;
}

void Run_Motor(Motor_Handle_t motor)
{
	switch(motor->state)
	{
		case STATE_1:
		{
			__HAL_TIM_SET_COMPARE(motor->timer, motor->phaseChannelA, 0);
			HAL_TIM_PWM_Start(motor->timer, motor->phaseChannelA);
			HAL_GPIO_WritePin(motor->enablePortA, motor->enablePinA, GPIO_PIN_RESET);

			__HAL_TIM_SET_COMPARE(motor->timer, motor->phaseChannelB, DUTY_CYCLE);
			HAL_TIM_PWM_Start(motor->timer, motor->phaseChannelB);
			HAL_GPIO_WritePin(motor->enablePortB, motor->enablePinB, GPIO_PIN_SET);

			__HAL_TIM_SET_COMPARE(motor->timer, motor->phaseChannelC, 0);
			HAL_TIM_PWM_Start(motor->timer, motor->phaseChannelC);
			HAL_GPIO_WritePin(motor->enablePortC, motor->enablePinC, GPIO_PIN_SET);
			break;
		}
		case STATE_2:
		{
			__HAL_TIM_SET_COMPARE(motor->timer, motor->phaseChannelA, DUTY_CYCLE);
			HAL_TIM_PWM_Start(motor->timer, motor->phaseChannelA);
			HAL_GPIO_WritePin(motor->enablePortA, motor->enablePinA, GPIO_PIN_SET);

			__HAL_TIM_SET_COMPARE(motor->timer, motor->phaseChannelB, 0);
			HAL_TIM_PWM_Start(motor->timer, motor->phaseChannelB);
			HAL_GPIO_WritePin(motor->enablePortB, motor->enablePinB, GPIO_PIN_RESET);

			__HAL_TIM_SET_COMPARE(motor->timer, motor->phaseChannelC, 0);
			HAL_TIM_PWM_Start(motor->timer, motor->phaseChannelC);
			HAL_GPIO_WritePin(motor->enablePortC, motor->enablePinC, GPIO_PIN_SET);
			break;
		}
		case STATE_3:
		{
			__HAL_TIM_SET_COMPARE(motor->timer, motor->phaseChannelA, DUTY_CYCLE);
			HAL_TIM_PWM_Start(motor->timer, motor->phaseChannelA);
			HAL_GPIO_WritePin(motor->enablePortA, motor->enablePinA, GPIO_PIN_SET);

			__HAL_TIM_SET_COMPARE(motor->timer, motor->phaseChannelB, 0);
			HAL_TIM_PWM_Start(motor->timer, motor->phaseChannelB);
			HAL_GPIO_WritePin(motor->enablePortB, motor->enablePinB, GPIO_PIN_SET);

			__HAL_TIM_SET_COMPARE(motor->timer, motor->phaseChannelC, 0);
			HAL_TIM_PWM_Start(motor->timer, motor->phaseChannelC);
			HAL_GPIO_WritePin(motor->enablePortC, motor->enablePinC, GPIO_PIN_RESET);
			break;
		}
		case STATE_4:
		{
			__HAL_TIM_SET_COMPARE(motor->timer, motor->phaseChannelA, 0);
			HAL_TIM_PWM_Start(motor->timer, motor->phaseChannelA);
			HAL_GPIO_WritePin(motor->enablePortA, motor->enablePinA, GPIO_PIN_RESET);

			__HAL_TIM_SET_COMPARE(motor->timer, motor->phaseChannelB, 0);
			HAL_TIM_PWM_Start(motor->timer, motor->phaseChannelB);
			HAL_GPIO_WritePin(motor->enablePortB, motor->enablePinB, GPIO_PIN_SET);

			__HAL_TIM_SET_COMPARE(motor->timer, motor->phaseChannelC, DUTY_CYCLE);
			HAL_TIM_PWM_Start(motor->timer, motor->phaseChannelC);
			HAL_GPIO_WritePin(motor->enablePortC, motor->enablePinC, GPIO_PIN_SET);
			break;
		}
		case STATE_5:
		{
			__HAL_TIM_SET_COMPARE(motor->timer, motor->phaseChannelA, 0);
			HAL_TIM_PWM_Start(motor->timer, motor->phaseChannelA);
			HAL_GPIO_WritePin(motor->enablePortA, motor->enablePinA, GPIO_PIN_SET);

			__HAL_TIM_SET_COMPARE(motor->timer, motor->phaseChannelB, 0);
			HAL_TIM_PWM_Start(motor->timer, motor->phaseChannelB);
			HAL_GPIO_WritePin(motor->enablePortB, motor->enablePinB, GPIO_PIN_RESET);

			__HAL_TIM_SET_COMPARE(motor->timer, motor->phaseChannelC, DUTY_CYCLE);
			HAL_TIM_PWM_Start(motor->timer, motor->phaseChannelC);
			HAL_GPIO_WritePin(motor->enablePortC, motor->enablePinC, GPIO_PIN_SET);
			break;
		}
		case STATE_6:
		{
			__HAL_TIM_SET_COMPARE(motor->timer, motor->phaseChannelA, 0);
			HAL_TIM_PWM_Start(motor->timer, motor->phaseChannelA);
			HAL_GPIO_WritePin(motor->enablePortA, motor->enablePinA, GPIO_PIN_SET);

			__HAL_TIM_SET_COMPARE(motor->timer, motor->phaseChannelB, DUTY_CYCLE);
			HAL_TIM_PWM_Start(motor->timer, motor->phaseChannelB);
			HAL_GPIO_WritePin(motor->enablePortB, motor->enablePinB, GPIO_PIN_SET);

			__HAL_TIM_SET_COMPARE(motor->timer, motor->phaseChannelC, 0);
			HAL_TIM_PWM_Start(motor->timer, motor->phaseChannelC);
			HAL_GPIO_WritePin(motor->enablePortC, motor->enablePinC, GPIO_PIN_RESET);
			break;
		}
	}
}

void Set_Motor_Parameters(Motor_Handle_t motor, uint8_t direction, float speed, float angle)
{
	motor->direction = direction;
	motor->speedTarget = speed;
	motor->angleTarget = angle;
}
