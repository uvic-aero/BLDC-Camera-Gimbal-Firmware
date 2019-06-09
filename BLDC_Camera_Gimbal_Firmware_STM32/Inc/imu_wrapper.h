/*
 * imu_i2c_wrapper.h
 *
 *  Created on: Jun 8, 2019
 *      Author: ntron
 *
 *  This file defined wrapper functions around standard HAL
 *  I2C functions for interop with the eMD6 driver
 */

#ifndef IMU_WRAPPER_H_
#define IMU_WRAPPER_H_

#include "main.h"
// Need these defines for driver to build properly
#define EMPL_TARGET_STM32F3
#define MPU9250
#define __no_operation(...) __asm volatile ("	nop				\n");

// These defines are for config options
#define IMU_I2C_TIMEOUT (100)

// Wrapper functions


bool IMU_I2C_Write_Wrapper(
		unsigned char slave_addr,
		unsigned char reg_addr,
		unsigned char length,
		unsigned char* data);

bool IMU_I2C_Read_Wrapper(
		unsigned char slave_addr,
		unsigned char reg_addr,
		unsigned char length,
		unsigned char* data);

void IMU_Delay_Wrapper(unsigned long ms);
void IMU_GetTick_Wrapper(unsigned long* timestamp);

#endif /* IMU_WRAPPER_H_ */
