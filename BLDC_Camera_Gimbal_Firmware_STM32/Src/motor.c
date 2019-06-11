/*
 * motor.c
 *
 *  Created on: Jun. 9, 2019
 *      Author: Jake
 */

#include "motor.h"
//include encoder files

// initialize 3 instances of the motor struct to keep track of each motor
//run an init function for each struct to configure to a particular motor
void Motor_Init(Motor_Handle_t motor, Motor_Identity_t ident)
{
	switch(ident)
	{
		case PITCH_MOTOR:
		{

		}
		case YAW_MOTOR:
		{

		}
		case ROLL_MOTOR:
		{

		}
	}
}
