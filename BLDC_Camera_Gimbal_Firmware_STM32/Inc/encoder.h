/*
 * encoder.h
 *
 *  Created on: Jun 19, 2019
 *      Author: Jake
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "main.h"

#define TIMEOUT (uint32_t)0xFFFFFFFF
#define MEM_ANGLE_ADDRESS_HIGH (uint16_t)0xFE
#define MEM_ANGLE_ADDRESS_LOW (uint16_t)0xFF
#define MEM_SIZE (uint16_t)0x1
#define DATA_SIZE (uint16_t)0x1
#define ENCODER_ADDRESS_BASE (uint16_t)0x40

typedef enum EncoderIdentity_t
{
	PITCH_ENCODER,
	YAW_ENCODER,
	ROLL_ENCODER

} EncoderIdentity_t;

typedef enum EncoderAddrConfig_t
{
	A2A1_00 = (uint8_t)0x0,
	A2A1_01 = (uint8_t)0x1,
	A2A1_10 = (uint8_t)0x2,
	A2A1_11 = (uint8_t)0x3,
} EncoderAddrConfig_t;

typedef struct Encoder_t
{
	EncoderIdentity_t identity;
	I2C_HandleTypeDef* i2c;

	float zeroPosition;

	uint16_t A1A2PinConfig;
	uint16_t address;

	uint8_t dataHigh;
	uint8_t dataLow;

	uint16_t angleRaw;
	float angleFloat;

} Encoder_t;

typedef Encoder_t* EncoderHandle_t;

void	Encoder_Init(EncoderHandle_t encoder, EncoderIdentity_t identity, EncoderAddrConfig_t pinconfig);
float	Encoder_GetAngle(EncoderHandle_t encoder);
void	Encoder_SetZeroPosition(EncoderHandle_t encoder, float zeropos);

#endif /* ENCODER_H_ */
