/*
 * motor.h
 *
 *  Created on: Jun. 9, 2019
 *      Author: Jake
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

typedef enum MotorIdentity_t
{
	PITCH_MOTOR,
	YAW_MOTOR,
	ROLL_MOTOR

} MotorIdentity_t;

typedef enum MotorOperationMode_t
{
	COMMUTATE,
	BRAKE,
	COAST
} MotorOperationMode_t;

typedef enum MotorDirection_t
{
	MOTOR_TURN_CCW,
	MOTOR_TURN_CW,
} MotorDirection_t;

typedef struct Motor_t
{
	MotorIdentity_t identity;
	MotorOperationMode_t mode;

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

typedef Motor_t* MotorHandle_t;

void Motor_Init(MotorHandle_t motor, MotorIdentity_t identity);
void Motor_SetOperationMode(MotorHandle_t motor, MotorOperationMode_t mode);
void Motor_Commutate(MotorHandle_t motor);
void Motor_Run(MotorHandle_t motor);
void Motor_SetParams(MotorHandle_t motor, MotorDirection_t direction, uint8_t pulse);
void Motor_EnableDriver(MotorHandle_t motor);
void Motor_DisableDriver(MotorHandle_t motor);

#endif /* MOTOR_H_ */
