/*
 * motor.h
 *
 *  Created on: Jun. 9, 2019
 *      Author: Jake
 */

#ifndef MOTOR_H_
#define MOTOR_H_

typedef struct Motor_t {
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
};

#endif /* MOTOR_H_ */
