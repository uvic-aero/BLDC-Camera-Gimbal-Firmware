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
	/* Addressing Fields */
	IMU_Identity_t	identity;
	uint16_t		i2c_address;
	GPIO_TypeDef*	interrupt_port;
	uint16_t		interrupt_pin;

	/* Data Fields */
	bool initialized;
	unsigned short dmpRate;
	unsigned short aSens; 	// accel sensitivity scalar
	float gSens;		  	// gyro sensitivity scalar
	float mSens; 			// magnetic sensitivity scalar
	float pitch;  			// degrees
	float yaw;				// degrees
	float roll;				// degrees
	long quat[4];			// raw sensor output
	unsigned long timestamp; // used for other computation

} IMU_t;

typedef IMU_t* IMU_Handle_t;

void IMU_Init(IMU_Handle_t imu, IMU_Identity_t ident);

void IMU_GetQuaternion(IMU_Handle_t imu);

void IMU_CalcEulerAngles(IMU_Handle_t imu);


#endif /* IMU_H_ */
