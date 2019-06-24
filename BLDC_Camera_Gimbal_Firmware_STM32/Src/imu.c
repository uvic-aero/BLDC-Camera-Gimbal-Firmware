/*
 * imu.c
 *
 *  Created on: Jun 9, 2019
 *      Author: ntron
 */

// Header
#include "imu.h"

// Invensense Headers
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include <math.h>


/* ************ INVENSENSE-REQUIRED DEFINITIONS (COPY PASTE FROM DEMO) *************/

/* The sensors can be mounted onto the board in any orientation. The mounting
* matrix seen below tells the MPL how to rotate the raw data from the
* driver(s).
*/
/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
    signed char orientation[9];
};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

#if defined MPU9150 || defined MPU9250
static struct platform_data_s compass_pdata = {
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0, -1}
};
#define COMPASS_ENABLED 1
#elif defined AK8975_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0, 1, 0,
                     0, 0,-1}
};
#define COMPASS_ENABLED 1
#elif defined AK8963_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0,-1, 0,
                     0, 0, 1}
};
#define COMPASS_ENABLED 1
#endif
/************************************************************************************/

/**************************** FUNCTION DEFINITIONS **********************************/

static void tap_cb(unsigned char direction, unsigned char count)
{
	return; // this should be empty
}

static void android_orient_cb(unsigned char orientation)
{
	return; // this should be empty
}

float qToFloat(long number, unsigned char q)
{
	unsigned long mask = 0;
	for (int i=0; i<q; i++)
	{
		mask |= (1<<i);
	}
	return (number >> q) + ((number & mask) / (float) (2<<(q-1)));
}

void IMU_Init(IMU_Handle_t imu, IMU_Identity_t ident)
{
	/* Configuration specific to the identity of the IMU*/

	// NOTE!!!!!
	// TODO: Current Invensense driver only supports 1 IMU at a time
	// TODO: Mod the driver layer to take which IMU it operates on as a parameter, if possible
	// TODO: this may not be possible
	// In the meantime, just "assert" that the correct setup is used

	ASSERT_PRINT(ident != BASE_IMU, "Cannot use BASE IMU with current Invensense Driver");

	switch(ident)
	{
	case AXIS_IMU:
		{
			imu->identity = ident;
			imu->i2c_address = AXIS_IMU_ADDR;
			imu->interrupt_port = GPIOA;		// TODO: THIS IS NOT THE RIGHT PORT
			imu->interrupt_pin = GPIO_PIN_8;	// TODO: THIS IS NOT THE RIGHT PIN
			break;
		}
	case BASE_IMU:
		{
			imu->identity = ident;
			imu->i2c_address = BASE_IMU_ADDR;
			imu->interrupt_port = GPIOC;		// TODO: THIS IS NOT THE RIGHT PORT
			imu->interrupt_pin = GPIO_PIN_8;	// TODO: THIS IS NOT THE RIGHT PIN
			break;
		}
	}

	/* Configuration applicable to all IMUs in system */

	struct int_param_s int_params;
	imu->dmpRate = 100;	// chosen rate Hz
	imu->mSens = 0.15;	// constant

	// IF ANY OF THESE CALLS FAIL IT WILL LOOP FOREVER!!!!!!!!!!!!!!!!!!
	mpu_init(&int_params);
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
	mpu_get_accel_sens(&(imu->aSens));
	mpu_get_gyro_sens(&(imu->gSens));
	dmp_load_motion_driver_firmware();
	dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
	dmp_register_tap_cb(tap_cb);
	dmp_register_android_orient_cb(android_orient_cb);
	dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_TAP);
	dmp_set_fifo_rate(imu->dmpRate);
	mpu_set_dmp_state(1);

}

void IMU_GetQuaternion(IMU_Handle_t imu)
{
	short g_temp[3]; // for throwaway data, might be able to replace with NULL
	short a_temp[3]; // for throwaway data, might be able to repalce with NULL
	long q_temp[4];
	unsigned long t_temp;
	unsigned char more;
	short sensors;

	if (dmp_read_fifo(g_temp, a_temp, q_temp, &t_temp, &sensors, &more) == INV_SUCCESS )
	// theres potential for getting again if there is more, so that we get the most up to date data
	{
		if (sensors & INV_WXYZ_QUAT)
		{
			//memcpy((void*)(imu->quat), (void*)q_temp, 4*sizeof(long));
			imu->quat[0] = q_temp[0];
			imu->quat[1] = q_temp[1];
			imu->quat[2] = q_temp[2];
			imu->quat[3] = q_temp[3];
		}
	}
}

void IMU_CalcEulerAngles(IMU_Handle_t imu)
{
	// Temporary implementation, might use MPL instead, haven't explored that yet

	float dqw = qToFloat(imu->quat[0], 30);
	float dqx = qToFloat(imu->quat[1], 30);
	float dqy = qToFloat(imu->quat[2], 30);
	float dqz = qToFloat(imu->quat[3], 30);

	float ysqr = dqy * dqy;
	float t0 = -2.0f * (ysqr + dqz * dqz) + 1.0f;
	float t1 = +2.0f * (dqx * dqy - dqw * dqz);
	float t2 = -2.0f * (dqx * dqz + dqw * dqy);
	float t3 = +2.0f * (dqy * dqz - dqw * dqx);
	float t4 = -2.0f * (dqx * dqx + ysqr) + 1.0f;

	// Keep t2 within range of asin (-1, 1)
	t2 = t2 > 1.0f ? 1.0f : t2;
	t2 = t2 < -1.0f ? -1.0f : t2;

	imu->pitch = asin(t2) * 2;
	imu->roll = atan2(t3, t4);
	imu->yaw = atan2(t1, t0);

	imu->pitch *= (180.0 / M_PI);
	imu->roll *= (180.0 / M_PI);
	imu->yaw *= (180.0 / M_PI);
	if (imu->pitch < 0) imu->pitch = 360.0 + imu->pitch;
	if (imu->roll < 0) imu->roll = 360.0 + imu->roll;
	if (imu->yaw < 0) imu->yaw = 360.0 + imu->yaw;
}
