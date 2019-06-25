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
	uint16_t		exti_line; /*EXTI##_##_IRQn*/

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

/** Initialize the IMU
 *  Init the struct and start the IMU in raw mode.
 *  Does not enable the IMU interrupt.
 *  Disables the NVIC interrupt and leaves it disabled
 *  to prevent interruption before FreeRTOS scheduler begins
 *
 *  \param[in] imu: struct to initialize
 *  \param[in] ident: identity (AXIS_IMU, BASE_IMU)
 */
void IMU_Init(IMU_Handle_t imu, IMU_Identity_t ident);

/** Start the IMU Digital Motion Process
 *  Configure the DMP and start it
 *  Start the DMP and IMU interrupts
 *  Re-enable the NVIC EXTI line for the IMU
 *
 *  \param[in] imu: the IMU to start
 */
void IMU_Start(IMU_Handle_t imu);

/// Enable NVIC EXTI interrupts for this IMU
void IMU_EnableInterrupt(IMU_Handle_t imu);

/// Disable NVIC EXTI interrupts for this IMU
void IMU_DisableInterrupt(IMU_Handle_t imu);

/** Get 6-axis quaternion from the IMU
 *  Store in the IMU structure
 *
 *  \param[out] imu: quaternion stored here
 */
void IMU_GetQuaternion(IMU_Handle_t imu);

/** Calculate Euler angles from quaternion data
 *  Store in the IMU structure
 *
 *  \param[out] imu: euler angles stored here
 */
void IMU_CalcEulerAngles(IMU_Handle_t imu);


#endif /* IMU_H_ */
