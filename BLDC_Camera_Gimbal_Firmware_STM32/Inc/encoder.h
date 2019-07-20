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

extern I2C_HandleTypeDef hi2c1;

typedef enum Encoder_Identity_t
{
	PITCH_ENCODER,
	YAW_ENCODER,
	ROLL_ENCODER

} Encoder_Identity_t;

typedef struct Encoder_t
{
	Encoder_Identity_t identity;
	I2C_HandleTypeDef* i2c;

	float zeroPosition;

	uint16_t A1A2PinConfig;
	uint16_t address;

	uint8_t dataHigh;
	uint8_t dataLow;

	uint16_t angleRaw;
	float angleFloat;

} Encoder_t;

typedef Encoder_t* Encoder_Handle_t;

void Encoder_Init(Encoder_Handle_t encoder, Encoder_Identity_t identity, uint16_t pinconfig);
float Poll_Encoder(Encoder_Handle_t encoder);
void Set_Zero_Position(Encoder_Handle_t encoder, float zeropos);

#endif /* ENCODER_H_ */
