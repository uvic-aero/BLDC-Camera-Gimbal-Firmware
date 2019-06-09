/*
 * imu.c
 *
 *  Created on: Jun 9, 2019
 *      Author: ntron
 */

#include "imu.h"

void IMU_Init(IMU_Handle_t hImu, IMU_Identity_t ident)
{
	/* Configuration specific to the identity of the IMU*/

	switch(ident)
	{
	case AXIS_IMU:
		{
			hImu->identity = ident;
			hImu->i2c_address = AXIS_IMU_ADDR;
			//hImu->interrupt_port = TODO;
			//hImu->interrupt_pin = TODO;
			break;
		}
	case BASE_IMU:
		{
			hImu->identity = ident;
			hImu->i2c_address = BASE_IMU_ADDR;
			//hImu->interrupt_port = TODO;
			//hImu->interrupt_pin = TODO;
			break;
		}
	}

	/* Configuration applicable to all IMUs in system */



}
