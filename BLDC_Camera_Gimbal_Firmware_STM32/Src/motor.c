/*
 * motor.c
 *
 *  Created on: Jun. 9, 2019
 *      Author: Jake
 */

#include "motor.h"
// include encoder .h .c files

#define DUTY_CYCLE (uint8_t)(0x04)

void Motor_Init(Motor_Handle_t motor, Motor_Identity_t identity)
{
	switch(identity)
	{
		case PITCH_MOTOR:
		{
			motor->identity = identity;
			motor->state = COMMUTATE;
			motor->step = STATE_1;
			motor->direction = TURN_CW;
			motor->speedTarget = 0.0;
			motor->angleTarget = 0.0;

			motor->phasePinA = MOTOR1_IN1_Pin;
			motor->phasePortA = MOTOR1_IN1_GPIO_Port;
			motor->phaseTimerA = &htim1;
			motor->phaseChannelA = TIM_CHANNEL_1;
			motor->enablePinA = MOTOR1_EN1_Pin;
			motor->enablePortA = MOTOR1_EN1_GPIO_Port;

			motor->phasePinB = MOTOR1_IN2_Pin;
			motor->phasePortB = MOTOR1_IN2_GPIO_Port;
			motor->phaseTimerB = &htim1;
			motor->phaseChannelB = TIM_CHANNEL_2;
			motor->enablePinB = MOTOR1_EN2_Pin;
			motor->enablePortB = MOTOR1_EN2_GPIO_Port;

			motor->phasePinC = MOTOR1_IN3_Pin;
			motor->phasePortC = MOTOR1_IN3_GPIO_Port;
			motor->phaseTimerC = &htim1;
			motor->phaseChannelC = TIM_CHANNEL_3;
			motor->enablePinC = MOTOR1_EN3_Pin;
			motor->enablePortC = MOTOR1_EN2_GPIO_Port;
		}
		case YAW_MOTOR:
		{
			motor->identity = identity;
			motor->state = COMMUTATE;
			motor->step = STATE_1;
			motor->direction = TURN_CW;
			motor->speedTarget = 0.0;
			motor->angleTarget = 0.0;

			motor->phasePinA = MOTOR2_IN1_Pin;
			motor->phasePortA = MOTOR2_IN1_GPIO_Port;
			motor->phaseTimerA = &htim3;
			motor->phaseChannelA = TIM_CHANNEL_1;
			motor->enablePinA = MOTOR2_EN1_Pin;
			motor->enablePortA = MOTOR2_EN1_GPIO_Port;

			motor->phasePinB = MOTOR2_IN2_Pin;
			motor->phasePortB = MOTOR2_IN2_GPIO_Port;
			motor->phaseTimerB = &htim3;
			motor->phaseChannelB = TIM_CHANNEL_2;
			motor->enablePinB = MOTOR2_EN2_Pin;
			motor->enablePortB = MOTOR2_EN2_GPIO_Port;

			motor->phasePinC = MOTOR2_IN3_Pin;
			motor->phasePortC = MOTOR2_IN3_GPIO_Port;
			motor->phaseTimerC = &htim3;
			motor->phaseChannelC = TIM_CHANNEL_3;
			motor->enablePinC = MOTOR2_EN3_Pin;
			motor->enablePortC = MOTOR2_EN3_GPIO_Port;
		}
		case ROLL_MOTOR:
		{
			motor->identity = identity;
			motor->state = COMMUTATE;
			motor->step = STATE_1;
			motor->direction = TURN_CW;
			motor->speedTarget = 0.0;
			motor->angleTarget = 0.0;

			motor->phasePinA = MOTOR3_IN1_Pin;
			motor->phasePortA = MOTOR3_IN1_GPIO_Port;
			motor->phaseTimerA = &htim4;
			motor->phaseChannelA = TIM_CHANNEL_1;
			motor->enablePinA = MOTOR3_EN1_Pin;
			motor->enablePortA = MOTOR3_EN1_GPIO_Port;

			motor->phasePinB = MOTOR3_IN2_Pin;
			motor->phasePortB = MOTOR3_IN2_GPIO_Port;
			motor->phaseTimerB = &htim4;
			motor->phaseChannelB = TIM_CHANNEL_2;
			motor->enablePinB = MOTOR3_EN2_Pin;
			motor->enablePortB = MOTOR3_EN2_GPIO_Port;

			motor->phasePinC = MOTOR3_IN3_Pin;
			motor->phasePortC = MOTOR3_IN3_GPIO_Port;
			motor->phaseTimerC = &htim4;
			motor->phaseChannelC = TIM_CHANNEL_3;
			motor->enablePinC = MOTOR2_EN3_Pin;
			motor->enablePortC = MOTOR3_EN3_GPIO_Port;
		}
	}
}

void Set_Commutation_State(Motor_Handle_t motor, Commutation_State_t state)
{
	switch(state)
	{
		case COMMUTATE:
		{
			HAL_TIM_PWM_Stop(motor->phaseTimerA, motor->phaseChannelA);
			HAL_TIM_PWM_Stop(motor->phaseTimerB, motor->phaseChannelB);
			HAL_TIM_PWM_Stop(motor->phaseTimerC, motor->phaseChannelC);

			switch(motor->step)
			{
				case STATE_1:
				{
					HAL_GPIO_WritePin(motor->phasePortA, motor->phasePinA, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(motor->enablePortA, motor->enablePinA, GPIO_PIN_RESET);

					__HAL_TIM_SET_COMPARE(motor->phaseTimerB, motor->phaseChannelB, DUTY_CYCLE);
					HAL_TIM_PWM_Start(motor->phaseTimerB, motor->phaseChannelB);
					HAL_GPIO_WritePin(motor->enablePortB, motor->enablePinB, GPIO_PIN_SET);

					HAL_GPIO_WritePin(motor->phasePortC, motor->phasePinC, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(motor->enablePortC, motor->enablePinC, GPIO_PIN_SET);

					if (motor->direction == TURN_CW)
					{
						motor->step = STATE_2;
					}
					else
					{
						motor->step = STATE_6;
					}
				}
				case STATE_2:
				{
					__HAL_TIM_SET_COMPARE(motor->phaseTimerA, motor->phaseChannelA, DUTY_CYCLE);
					HAL_TIM_PWM_Start(motor->phaseTimerA, motor->phaseChannelA);
					HAL_GPIO_WritePin(motor->enablePortA, motor->enablePinA, GPIO_PIN_SET);

					HAL_GPIO_WritePin(motor->phasePortB, motor->phasePinB, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(motor->enablePortB, motor->enablePinB, GPIO_PIN_RESET);

					HAL_GPIO_WritePin(motor->phasePortC, motor->phasePinC, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(motor->enablePortC, motor->enablePinC, GPIO_PIN_SET);

					if (motor->direction == TURN_CW)
					{
						motor->step = STATE_3;
					}
					else
					{
						motor->step = STATE_1;
					}
				}
				case STATE_3:
				{
					__HAL_TIM_SET_COMPARE(motor->phaseTimerA, motor->phaseChannelA, DUTY_CYCLE);
					HAL_TIM_PWM_Start(motor->phaseTimerA, motor->phaseChannelA);
					HAL_GPIO_WritePin(motor->enablePortA, motor->enablePinA, GPIO_PIN_SET);

					HAL_GPIO_WritePin(motor->phasePortB, motor->phasePinB, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(motor->enablePortB, motor->enablePinB, GPIO_PIN_SET);

					HAL_GPIO_WritePin(motor->phasePortC, motor->phasePinC, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(motor->enablePortC, motor->enablePinC, GPIO_PIN_RESET);

					if (motor->direction == TURN_CW)
					{
						motor->step = STATE_4;
					}
					else
					{
						motor->step = STATE_2;
					}
				}
				case STATE_4:
				{
					HAL_GPIO_WritePin(motor->phasePortA, motor->phasePinA, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(motor->enablePortA, motor->enablePinA, GPIO_PIN_RESET);

					HAL_GPIO_WritePin(motor->phasePortB, motor->phasePinB, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(motor->enablePortB, motor->enablePinB, GPIO_PIN_SET);

					__HAL_TIM_SET_COMPARE(motor->phaseTimerC, motor->phaseChannelC, DUTY_CYCLE);
					HAL_TIM_PWM_Start(motor->phaseTimerC, motor->phaseChannelC);
					HAL_GPIO_WritePin(motor->enablePortC, motor->enablePinC, GPIO_PIN_SET);

					if (motor->direction == TURN_CW)
					{
						motor->step = STATE_5;
					}
					else
					{
						motor->step = STATE_3;
					}
				}
				case STATE_5:
				{
					HAL_GPIO_WritePin(motor->phasePortA, motor->phasePinA, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(motor->enablePortA, motor->enablePinA, GPIO_PIN_SET);

					HAL_GPIO_WritePin(motor->phasePortB, motor->phasePinB, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(motor->enablePortB, motor->enablePinB, GPIO_PIN_RESET);

					__HAL_TIM_SET_COMPARE(motor->phaseTimerC, motor->phaseChannelC, DUTY_CYCLE);
					HAL_TIM_PWM_Start(motor->phaseTimerC, motor->phaseChannelC);
					HAL_GPIO_WritePin(motor->enablePortC, motor->enablePinC, GPIO_PIN_SET);

					if (motor->direction == TURN_CW)
					{
						motor->step = STATE_6;
					}
					else
					{
						motor->step = STATE_4;
					}
				}
				case STATE_6:
				{
					HAL_GPIO_WritePin(motor->phasePortA, motor->phasePinA, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(motor->enablePortA, motor->enablePinA, GPIO_PIN_SET);

					__HAL_TIM_SET_COMPARE(motor->phaseTimerB, motor->phaseChannelB, DUTY_CYCLE);
					HAL_TIM_PWM_Start(motor->phaseTimerB, motor->phaseChannelB);
					HAL_GPIO_WritePin(motor->enablePortB, motor->enablePinB, GPIO_PIN_SET);

					HAL_GPIO_WritePin(motor->phasePortC, motor->phasePinC, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(motor->enablePortC, motor->enablePinC, GPIO_PIN_RESET);

					if (motor->direction == TURN_CW)
					{
						motor->step = STATE_1;
					}
					else
					{
						motor->step = STATE_5;
					}
				}
			}
		}
		case BRAKE:
		{
			HAL_GPIO_WritePin(motor->phasePortA, motor->phasePinA, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->phasePortB, motor->phasePinB, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->phasePortC, motor->phasePinC, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(motor->enablePortA, motor->enablePinA, GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor->enablePortB, motor->enablePinB, GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor->enablePortC, motor->enablePinC, GPIO_PIN_SET);
		}
		case COAST:
		{
			HAL_GPIO_WritePin(motor->phasePortA, motor->phasePinA, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->phasePortB, motor->phasePinB, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->phasePortC, motor->phasePinC, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(motor->enablePortA, motor->enablePinA, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->enablePortB, motor->enablePinB, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->enablePortC, motor->enablePinC, GPIO_PIN_RESET);
		}
	}

	motor->state = state;
}

void Set_Motor_Parameters(Motor_Handle_t motor, uint8_t direction, float speed, float angle)
{
	motor->direction = direction;
	motor->speedTarget = speed;
	motor->angleTarget = angle;
}
