#pragma once

#include "main.h"
#include <stdint.h>

typedef enum { false, true } bool;

// ***************** Message format ***************

typedef uint8_t COMMS_Header;

typedef struct {
	COMMS_Header type;
	uint16_t value;
} COMMS_Data_Message;

typedef struct {
	COMMS_Header type;
	uint32_t value;
} COMMS_Time_Message;

typedef struct {
	uint8_t start; // start byte sequence

	uint8_t size_payload; // size of the entire payload (includes the message headers) (in number of bytes)
	uint8_t size_mssgs; // total size of all data messages (in bytes)

	COMMS_Time_Message sys_time;
	COMMS_Data_Message* messages;
	COMMS_Header* events;

	uint8_t stop; // stop byte sequence
} COMMS_Payload;

// ***************** Struct Handles ***************

typedef COMMS_Payload* COMMS_PayloadHandle;

// ***************** Function prototypes  ***************


//// Sends data to SW
//// returns true or false indicating if it succeeded
bool SendData(COMMS_Data_Message messages[], uint8_t mssg_size, COMMS_Header events[], uint8_t evt_size);
uint8_t* EncodeStruct(COMMS_PayloadHandle);

//COMMS_PkgRxHandle GetData();





