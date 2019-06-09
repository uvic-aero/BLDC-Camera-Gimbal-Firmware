/*
 * imu.h
 *
 *  Created on: Jun 9, 2019
 *      Author: ntron
 */

#ifndef IMU_H_
#define IMU_H_

#include "main.h"

typedef enum IMU_Identity_t
{
	AXIS_IMU,
	BASE_IMU,
} IMU_Identity_t;

typedef struct IMU_t
{
	IMU_Identity_t	identity;
	uint16_t		i2c_address;
	uint16_t		interrupt_port;
	uint16_t		interrupt_pin;
} IMU_t;

typedef IMU_t* IMU_Handle_t;

void IMU_Init(IMU_Handle_t imu, IMU_Identity_t ident);


#endif /* IMU_H_ */
