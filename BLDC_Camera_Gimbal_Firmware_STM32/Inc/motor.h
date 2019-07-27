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

typedef struct Motor_t
{
	Motor_Identity_t identity;
	Operation_Mode_t mode;

	TIM_HandleTypeDef* timer;

	uint16_t phaseChannel[3];
	uint16_t phasePin[3];
	GPIO_TypeDef* phasePort[3];
	uint16_t enablePin[3];
	GPIO_TypeDef* enablePort[3];
	uint16_t phaseIndex[3];

	uint16_t nResetPin;
	GPIO_TypeDef* nResetPort;
	uint16_t nFaultPin;
	GPIO_TypeDef* nFaultPort;

	uint8_t direction;
	uint8_t maxPulseSize;

} Motor_t;

typedef Motor_t* Motor_Handle_t;

void Motor_Init(Motor_Handle_t motor, Motor_Identity_t identity);
void Set_Operation_Mode(Motor_Handle_t motor, Operation_Mode_t mode);
void Commutate_Motor(Motor_Handle_t motor);
void Run_Motor(Motor_Handle_t motor);
void Set_Motor_Parameters(Motor_Handle_t motor, uint8_t direction, uint8_t pulse);
void Motor_EnableDriver(Motor_Handle_t motor);
void Motor_DisableDriver(Motor_Handle_t motor);

#endif /* MOTOR_H_ */
