#pragma once


#include <stdbool.h>

#define ARRAY_LEN(x)	(sizeof(x) / sizeof((x)[0]))


// ***************** Message format ***************

typedef uint8_t COMMS_Header;

typedef uint8_t COMMS_Event_Message;

typedef struct __attribute__((packed)) {
	COMMS_Header type;
	uint16_t value;
} COMMS_Data_Message;


typedef struct __attribute__((packed)) {
	COMMS_Header type;
	uint32_t value;
} COMMS_Time_Message;

typedef struct __attribute__((packed)) {
	uint8_t start; // start byte sequence

	uint8_t size_data; // size of the data messages with the headers (in bytes)
	uint8_t size_events; // total size of all event messages (in bytes)

	COMMS_Time_Message sys_time; // 5 bytes
	COMMS_Data_Message* messages; // 3 bytes
	COMMS_Event_Message* events; // 1 bytes

	uint8_t stop; // stop byte sequence
} COMMS_Payload;

typedef struct __attribute__((packed)) Message {
	COMMS_Event_Message event;
	COMMS_Data_Message message;
} COMMS_Message;


typedef enum Data_Messages {
	COMMS_START = 0b10101010,
	COMMS_STOP = 0b11011011,
	COMMS_SysTime = 0b00000001,
	COMMS_Curr_Pan = 0b00000010,
	COMMS_Target_Pan = 0b00000100,
	COMMS_Curr_Tilt = 0b00000011,
	COMMS_Target_Tilt = 0b00000101,
} COMMS_Data_Message_Type;

typedef enum Event_Messages {
	COMMS_Switch_RC = 0b10000001,
	COMMS_Switch_USB = 0b10000010,
} COMMS_Event_Message_Type;

// ***************** Struct Handles ***************

typedef COMMS_Payload* COMMS_PayloadHandle;

// ***************** Function prototypes  ***************

bool __COMMS_IsValidDataStream(void);

void __COMMS_ProcessData(void* data, size_t len);

char* __EncodePayload(COMMS_PayloadHandle packet, uint8_t mssg_size, uint8_t evt_size, uint8_t* txPackage);

bool __COMMS_GetMessage(COMMS_Data_Message* messageRef, COMMS_Data_Message* head, uint8_t data_messages_count, COMMS_Data_Message_Type type);

bool __COMMS_GetEvent(COMMS_Event_Message* event, COMMS_Event_Message* head, uint8_t events_count, COMMS_Event_Message_Type type);

void __DMABuffer_to_UARTBuffer(void);

void Comms_Init(void);

void Comms_InitTasks(void);

void Comms_InitQueues(void);

void vCommsRxData(void);

void vCommsDecodePayload();

void vCommsTxData(void);
