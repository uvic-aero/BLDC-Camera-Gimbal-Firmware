/*
 * rc_input.h
 *
 *  Created on: Jul 3, 2019
 *      Author: ntron
 */

/*
 * This module abstracts the Remote Control (RC) inputs of the system
 * They are PWM inputs with the following characteristics:
 * 		- Frequency: 70 to 400 Hz
 * 		- Duty cycle: 1000 us to 2000 us
 * This module requires that the associated timers use PWM input mode
 * with combined channel 1 and channel 2:
 * 		- Channel 1: Direct, Rising edge
 * 		- Channel 2: Indirect, Falling edge
 */

#ifndef RC_INPUT_H_
#define RC_INPUT_H_

#include "main.h"

/// identity of an RC input
/// defined byte-wise so that these can be directly used in task notification or event groups
typedef enum RC_InputIdentity_t
{
	RC_INPUT_PITCH 	= 0b001,
	RC_INPUT_YAW 	= 0b010,
	RC_INPUT_MODE 	= 0b100,
} RC_InputIdentity_t;

/// represents one of the system RC inputs
typedef struct RC_Input_t
{
	RC_InputIdentity_t id;
	TIM_HandleTypeDef* timer;
	uint32_t pulse_width_us;
	uint32_t period_us;
} RC_Input_t;

typedef RC_Input_t* RC_InputHandle_t;

/// Initialize the struct data
void RC_Init(RC_InputHandle_t rc, RC_InputIdentity_t id);
/// Start the timer interrupts (begins measurement and interrupts once every measurement is ready)
void RC_StartInterrupts(RC_InputHandle_t rc);
/// Stop the timer interrupts
void RC_StopInterrupts(RC_InputHandle_t rc);
/// Get the newest period and pulse values in microseconds
/// Should only be called from an interrupt handler to ensure correctness
void RC_Update(RC_InputHandle_t rc);

#endif /* RC_INPUT_H_ */
