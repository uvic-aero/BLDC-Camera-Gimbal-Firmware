#pragma once
#pragma pack(1)

#include "main.h"
#include <stdint.h>
#include <string.h>

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;


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

	uint8_t size_data; // size of the data messages with the headers (in bytes)
	uint8_t size_events; // total size of all event messages (in bytes)

	COMMS_Time_Message sys_time;
	COMMS_Data_Message* messages;
	COMMS_Header* events;

	uint8_t stop; // stop byte sequence
} COMMS_Payload;

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


//// Sends data to SW
//// returns true or false indicating if it succeeded
bool SendData(COMMS_Data_Message messages[], uint8_t mssg_size, COMMS_Header events[], uint8_t evt_size);
char* EncodePayload(COMMS_PayloadHandle packet, uint8_t mssg_size, uint8_t evt_size, uint8_t* txPackage);

// private functions
void __COMMS_ProcessData(const void* data, size_t len);

// public functions
void COMMS_RX_Check(void);
bool DecodePayload(COMMS_PayloadHandle payload);
//COMMS_PayloadHandle DecodePayload(void);

// TODO: Remove these as i dont used them
void COMMS_DMA_IrqHandler(DMA_HandleTypeDef *hdma, UART_HandleTypeDef *huart);
void COMMS_USART2_IrqHandler(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);



