/*
 * encoder.c
 *
 *  Created on: Jun 19, 2019
 *      Author: Jake
 */

#include "encoder.h"
#include "angles.h"

void Encoder_Init(Encoder_Handle_t encoder, Encoder_Identity_t identity, uint16_t pinconfig)
{
	encoder->identity = identity;
	encoder->i2c = &hi2c1;

	encoder->zeroPosition = 0.0;

	encoder->A1A2PinConfig = pinconfig;
	encoder->address = (ENCODER_ADDRESS_BASE | encoder->A1A2PinConfig);

	encoder->dataHigh = 0x0;
	encoder->dataLow = 0x0;

	encoder->angleRaw = 0x0;
	encoder->angleFloat = 0.0;
}

float Poll_Encoder(Encoder_Handle_t encoder)
{
	HAL_I2C_Mem_Read(encoder->i2c, (encoder->address << 1), MEM_ANGLE_ADDRESS_HIGH, MEM_SIZE, &encoder->dataHigh, DATA_SIZE, TIMEOUT);
	HAL_I2C_Mem_Read(encoder->i2c, (encoder->address << 1), MEM_ANGLE_ADDRESS_LOW, MEM_SIZE, &encoder->dataLow, DATA_SIZE, TIMEOUT);

	encoder->angleRaw = (encoder->dataHigh << 6) | (encoder->dataLow & 0x3F);

	/*
	uint16_t angleSum = (encoder->angleRaw + encoder->zeroPosition);
	uint16_t angleCorrected = angleSum > 0x3FFF ? (angleSum - 0x3FFF) : angleSum;
	*/

	encoder->angleFloat = Angles_Normalize360( ((((float)(encoder->angleRaw))/ 16383.0) * 360.0) - encoder->zeroPosition );

	return encoder->angleFloat;
}

void Set_Zero_Position(Encoder_Handle_t encoder, float zeropos)
{
	encoder->zeroPosition = zeropos;
}
