/*
 * angles.c
 *
 *  Created on: Jun. 28, 2019
 *      Author: ntron
 */

#include "angles.h"

float Angles_Normalize180(float theta)
{
	// remove or add 360 to normalize to within  +/-180
	// TODO: make this more robust to avoid floating point delta errors

	// these two loops are, in theory, exclusive
	// the result of the first should never give a result like the second
	// could be a problem for floating point epsilon
	// TODO: might want to use an epsilon value
	while (theta > 180.0)
		theta = theta - 360.0;

	while (theta < -180.0)
		theta = theta + 360.0;

	// fall through
	return theta;
}

float Angles_Normalize360(float theta)
{

	//TODO: use an epsilon value to compare
	// this will make it more robust
	while ( theta > 360.0 )
		theta = theta - 360.0;

	while ( theta < 0.0 )
		theta = theta + 360;

	return theta;

}

/// Function to calculate angular distance
/// Caculate the angular distance from current to target
/// return is positive => movement to target is CCW
/// return is negative => movement to target is CW
/// return range: (-180.0 , +180.0]
float Angles_CalcDist(float target, float current)
{
	float t = Angles_Normalize180(target);
	float c = Angles_Normalize180(current);

	return Angles_Normalize180(t - c);
}
