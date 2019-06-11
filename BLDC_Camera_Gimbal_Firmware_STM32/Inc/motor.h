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

typedef struct Motor_t
{
	Motor_Identity_t identity;

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

	/*float currPosition;
	float currSpeed;*/

	//MAKE THESE NORMAL FUNCTIONS
	/*void (*setSpeed)(float speed); // set PWM duty and pulse length
	void (*getSpeed)(void);

	void (*commutate)(void);
	void (*brake)(void);
	void (*coast)(void);

	void (*speedToDuty)(void);
	void (*dutyToSpeed)(void);*/

} Motor_t;

typedef Motor_t* Motor_Handle_t;

void Motor_Init(Motor_Handle_t motor, Motor_Identity_t ident);

#endif /* MOTOR_H_ */
