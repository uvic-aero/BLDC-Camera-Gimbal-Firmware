/*
 * motor.h
 *
 *  Created on: Jun. 9, 2019
 *      Author: Jake
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "main.h"

#define TURN_CW (uint8_t)0x00
#define TURN_CCW (uint8_t)0x01

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

typedef enum Motor_Identity_t
{
	PITCH_MOTOR,
	YAW_MOTOR,
	ROLL_MOTOR

} Motor_Identity_t;

typedef enum Operation_Mode_t
{
	COMMUTATE,
	BRAKE,
	COAST
} Operation_Mode_t;

typedef enum Commutation_State_t
{
	STATE_1,
	STATE_2,
	STATE_3,
	STATE_4,
	STATE_5,
	STATE_6
} Commutation_State_t;

typedef struct Motor_t
{
	Motor_Identity_t identity;
	Operation_Mode_t mode;
	Commutation_State_t state;

	TIM_HandleTypeDef* timer;

	uint16_t phaseChannelA;
	uint16_t phasePinA;
	GPIO_TypeDef* phasePortA;
	uint16_t enablePinA;
	GPIO_TypeDef* enablePortA;

	uint16_t phaseChannelB;
	uint16_t phasePinB;
	GPIO_TypeDef* phasePortB;
	uint16_t enablePinB;
	GPIO_TypeDef* enablePortB;

	uint16_t phaseChannelC;
	uint16_t phasePinC;
	GPIO_TypeDef* phasePortC;
	uint16_t enablePinC;
	GPIO_TypeDef* enablePortC;

	uint8_t direction;
	float speedTarget;
	float angleTarget;

} Motor_t;

typedef Motor_t* Motor_Handle_t;

void Motor_Init(Motor_Handle_t motor, Motor_Identity_t identity, Operation_Mode_t mode);
void Set_Operation_Mode(Motor_Handle_t motor, Operation_Mode_t mode);
void Run_Motor(Motor_Handle_t motor);
void Set_Motor_Parameters(Motor_Handle_t motor, uint8_t direction, float speed, float angle);

#endif /* MOTOR_H_ */
