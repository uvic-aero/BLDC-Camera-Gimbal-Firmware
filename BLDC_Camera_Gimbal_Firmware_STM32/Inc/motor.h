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
	//VARIABLES
	//associated timer
	//attach phases to each timer channel
	//attach associated enable GPIO pins
	//Pitch/yaw/roll (which motor)
	//position
	//speed

	//FUNCTIONS
	//set speed (PWM duty and pulse length)
	//commutation
	//brake
	//coast
	//conversion helper functions (speed to duty cycle)
} Motor_t;

typedef Motor_t* Motor_Handle_t;

void Motor_Init(Motor_Handle_t motor, Motor_Identity_t ident);

#endif /* MOTOR_H_ */
