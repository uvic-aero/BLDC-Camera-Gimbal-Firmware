/*
 * rc_input.c
 *
 *  Created on: Jul 3, 2019
 *      Author: ntron
 */


#include "rc_input.h"

extern TIM_HandleTypeDef htim2, htim8, htim15;

void RC_Init(RC_InputHandle_t rc, RC_InputIdentity_t id)
{
	switch(id)
	{
	case RC_INPUT_PITCH:
		{
			rc->id = RC_INPUT_PITCH;
			rc->timer = &htim2;
			break;
		}
	case RC_INPUT_YAW:
		{
			rc->id = RC_INPUT_YAW;
			rc->timer = &htim8;
			break;
		}
	case RC_INPUT_MODE:
		{
			rc->id = RC_INPUT_MODE;
			rc->timer = &htim15;
			break;
		}
	default:
		{
			while(true); // should never get here!!!! spin forever
			break;
		}
	}

	rc->period_us = 0;
	rc->pulse_width_us = 0;
}

void RC_StartInterrupts(RC_InputHandle_t rc)
{
	HAL_TIM_IC_Start_IT(rc->timer, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(rc->timer, TIM_CHANNEL_2);
}

void RC_StopInterrupts(RC_InputHandle_t rc)
{
	HAL_TIM_IC_Stop_IT(rc->timer, TIM_CHANNEL_1);
	HAL_TIM_IC_Stop_IT(rc->timer, TIM_CHANNEL_2);
}

void RC_Update(RC_InputHandle_t rc)
{
	rc->period_us 		= HAL_TIM_ReadCapturedValue(rc->timer, TIM_CHANNEL_1);
	rc->pulse_width_us 	= HAL_TIM_ReadCapturedValue(rc->timer, TIM_CHANNEL_2);
}

/*

void vRcInputHandler (void* pvParameters)
{
	// initialize the IMU, this needs to go here to prevent the fifo from starting interrupts

	uint32_t period_ticks = 0;
	uint32_t pulse_ticks = 0;
	uint32_t duty_cycle = 0;

	int counter = 0;

	// start interrupts
	HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_2);

	// testing!
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 7000);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	while(true)
	{
		// wait for IMU interrupt
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		// period and pulse in timer ticks
		period_ticks = HAL_TIM_ReadCapturedValue(&htim15, TIM_CHANNEL_1);
		pulse_ticks = HAL_TIM_ReadCapturedValue(&htim15, TIM_CHANNEL_2);

		if (period_ticks != 0)
			duty_cycle = (pulse_ticks * 100) / period_ticks;
		else
			duty_cycle = 0;

		if (counter == 0)
			printf("period ticks: %d; pulse ticks: %d\n", period_ticks, pulse_ticks);

		counter = (counter + 1) % 70;
	}

}*/


