/*
 * imu_i2c_wrapper.c
 *
 *  Created on: Jun 8, 2019
 *      Author: ntron
 */

#include "main.h"
#include "imu_wrapper.h"

extern I2C_HandleTypeDef IMU_I2C_CHANNEL;

bool IMU_I2C_Write_Wrapper(
		unsigned char slave_addr,
		unsigned char reg_addr,
		unsigned char length,
		unsigned char* data)
{
	if (HAL_OK == HAL_I2C_Mem_Write(
			&IMU_I2C_CHANNEL,				// using I2C2... TODO: for now
			(((uint16_t)slave_addr) << 1),	// for the HAL, the slave_addr should be shifted by one
			(uint16_t)reg_addr,				// Register we want
			1,								// MemAddressSize: this is 1byte for this device, always
			data,							// Data buffer
			(uint16_t)length,				// Length of data
			IMU_I2C_TIMEOUT) 				// Timeout... TODO: figure this out
		)
	{
		return true;
	}
	else
		return false;
}

bool IMU_I2C_Read_Wrapper(
		unsigned char slave_addr,
		unsigned char reg_addr,
		unsigned char length,
		unsigned char* data)
{
	if ( HAL_OK == HAL_I2C_Mem_Read(
			&IMU_I2C_CHANNEL,				// using I2C2... TODO: for now
			(((uint16_t)slave_addr) << 1),	// for the HAL, the slave_addr should be shifted by one
			(uint16_t)reg_addr,				// Register we want
			1,								// MemAddressSize: this is 1byte for this device, always
			data,							// Data buffer
			(uint16_t)length,				// Length of data
			IMU_I2C_TIMEOUT)				// Timeout... TODO: figure this out
		)
	{
		return true;
	}
	else
		return false;
}

void IMU_Delay_Wrapper(unsigned long ms)
{
	HAL_Delay((uint32_t)ms);
}

void IMU_GetTick_Wrapper(unsigned long* timestamp)
{
	*timestamp = (unsigned long)HAL_GetTick();
}
