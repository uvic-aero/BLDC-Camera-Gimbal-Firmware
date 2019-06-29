/*
 * angles.h
 *
 *  Created on: Jun. 28, 2019
 *      Author: ntron
 */

#ifndef ANGLES_H_
#define ANGLES_H_


typedef struct EulerAngles_t
{
	float pitch;
	float yaw;
	float roll;
} EulerAngles_t;

/// Normalize an angle to between -180 and +180 degrees
float Angles_Normalize180(float theta);
/// Normalize an angle to between 0 and 360 degrees
float Angles_Normalize360(float theta);

/// Function to calculate angular distance
/// Caculate the angular distance from current to target
/// return is positive => movement to target is CCW
/// return is negative => movement to target is CW
/// return range: (-180.0 , +180.0]
float Angles_CalcDist(float target, float current);

#endif /* ANGLES_H_ */
