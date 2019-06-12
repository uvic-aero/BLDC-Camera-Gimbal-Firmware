/*
 * motor.h
 *
 *  Created on: Jun. 9, 2019
 *      Author: Jake
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "main.h"

typedef enum Motor_Identity_t
{
	PITCH_MOTOR,
	YAW_MOTOR,
	ROLL_MOTOR

} Motor_Identity_t;

typedef enum Commutation_State_t
{
	COMMUTATE,
	BRAKE,
	COAST,
	IDLE
} Commutation_State_t;

typedef struct Motor_t
{
	Motor_Identity_t identity;
	Commutation_State_t state;

	uint16_t phasePinA;
	GPIO_TypeDef* phasePortA;
	TIM_TypeDef* phaseTimerA;
	uint16_t phaseChannelA;
	uint16_t enablePinA;
	GPIO_TypeDef* enablePortA;

	uint16_t phasePinB;
	GPIO_TypeDef* phasePortB;
	TIM_TypeDef* phaseTimerB;
	uint16_t phaseChannelB;
	uint16_t enablePinB;
	GPIO_TypeDef* enablePortB;

	uint16_t phasePinC;
	GPIO_TypeDef* phasePortC;
	TIM_TypeDef* phaseTimerC;
	uint16_t phaseChannelC;
	uint16_t enablePinC;
	GPIO_TypeDef* enablePortC;

} Motor_t;

typedef Motor_t* Motor_Handle_t;

void Motor_Init(Motor_Handle_t motor, Motor_Identity_t identity);

void Set_Commutation_State(Motor_Handle_t motor, Commutation_State_t state);

#endif /* MOTOR_H_ */
