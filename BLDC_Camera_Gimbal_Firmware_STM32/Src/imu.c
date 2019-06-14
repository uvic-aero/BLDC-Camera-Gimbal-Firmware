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

void IMU_Init(IMU_Handle_t hImu, IMU_Identity_t ident)
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
	/*struct int_param_s int_param;
	inv_error_t result;

	// initialize IMU device, TODO: currently only for address 0x68
	result = mpu_init(&int_param);
	ASSERT_PRINT(result != 0, "mpu_init() failed");
	*/








}
