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


