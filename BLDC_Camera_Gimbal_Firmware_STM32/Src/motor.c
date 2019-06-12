/*
 * motor.c
 *
 *  Created on: Jun. 9, 2019
 *      Author: Jake
 */

#include "motor.h"
// include encoder .h .c files

void Motor_Init(Motor_Handle_t motor, Motor_Identity_t identity)
{
	switch(identity)
	{
		case PITCH_MOTOR:
		{
			motor->identity = identity;
			motor->state = IDLE;

			motor->phasePinA = MOTOR1_IN1_Pin;
			motor->phasePortA = MOTOR1_IN1_GPIO_Port;
			motor->phaseTimerA = TIM1;
			motor->phaseChannelA = TIM_CHANNEL_1;
			motor->enablePinA = MOTOR1_EN1_Pin;
			motor->enablePortA = MOTOR1_EN1_GPIO_Port;

			motor->phasePinB = MOTOR1_IN2_Pin;
			motor->phasePortB = MOTOR1_IN2_GPIO_Port;
			motor->phaseTimerB = TIM1;
			motor->phaseChannelB = TIM_CHANNEL_2;
			motor->enablePinB = MOTOR1_EN2_Pin;
			motor->enablePortB = MOTOR1_EN2_GPIO_Port;

			motor->phasePinC = MOTOR1_IN3_Pin;
			motor->phasePortC = MOTOR1_IN3_GPIO_Port;
			motor->phaseTimerC = TIM1;
			motor->phaseChannelC = TIM_CHANNEL_3;
			motor->enablePinC = MOTOR1_EN3_Pin;
			motor->enablePortC = MOTOR1_EN2_GPIO_Port;
		}
		case YAW_MOTOR:
		{
			motor->identity = identity;
			motor->state = IDLE;

			motor->phasePinA = MOTOR2_IN1_Pin;
			motor->phasePortA = MOTOR2_IN1_GPIO_Port;
			motor->phaseTimerA = TIM3;
			motor->phaseChannelA = TIM_CHANNEL_1;
			motor->enablePinA = MOTOR2_EN1_Pin;
			motor->enablePortA = MOTOR2_EN1_GPIO_Port;

			motor->phasePinB = MOTOR2_IN2_Pin;
			motor->phasePortB = MOTOR2_IN2_GPIO_Port;
			motor->phaseTimerB = TIM3;
			motor->phaseChannelB = TIM_CHANNEL_2;
			motor->enablePinB = MOTOR2_EN2_Pin;
			motor->enablePortB = MOTOR2_EN2_GPIO_Port;

			motor->phasePinC = MOTOR2_IN3_Pin;
			motor->phasePortC = MOTOR2_IN3_GPIO_Port;
			motor->phaseTimerC = TIM3;
			motor->phaseChannelC = TIM_CHANNEL_3;
			motor->enablePinC = MOTOR2_EN3_Pin;
			motor->enablePortC = MOTOR2_EN3_GPIO_Port;
		}
		case ROLL_MOTOR:
		{
			motor->identity = identity;
			motor->state = IDLE;

			motor->phasePinA = MOTOR3_IN1_Pin;
			motor->phasePortA = MOTOR3_IN1_GPIO_Port;
			motor->phaseTimerA = TIM4;
			motor->phaseChannelA = TIM_CHANNEL_1;
			motor->enablePinA = MOTOR3_EN1_Pin;
			motor->enablePortA = MOTOR3_EN1_GPIO_Port;

			motor->phasePinB = MOTOR3_IN2_Pin;
			motor->phasePortB = MOTOR3_IN2_GPIO_Port;
			motor->phaseTimerB = TIM4;
			motor->phaseChannelB = TIM_CHANNEL_2;
			motor->enablePinB = MOTOR3_EN2_Pin;
			motor->enablePortB = MOTOR3_EN2_GPIO_Port;

			motor->phasePinC = MOTOR3_IN3_Pin;
			motor->phasePortC = MOTOR3_IN3_GPIO_Port;
			motor->phaseTimerC = TIM4;
			motor->phaseChannelC = TIM_CHANNEL_3;
			motor->enablePinC = MOTOR2_EN3_Pin;
			motor->enablePortC = MOTOR3_EN3_GPIO_Port;
		}
	}
}

void Set_Commutation_State(Motor_Handle_t motor, Commutation_State_t state)
{
	if ((motor->state == COMMUTATE) && (state != COMMUTATE))
	{
		//disable PWM output so that normal GPIO function can be used
		//HAL_TIM_PWM_STOP();
	}

	switch(state)
	{
		case IDLE:
		{

		}
		case COMMUTATE:
		{
			//use TIM register directly to change PWM stats, see STM MCU manual pg. 550
			if (motor->state != COMMUTATE)
			{
				//start PWM generation after registers have been configured
				//HAL_TIM_PWM_Start();
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
