/*
 * Comms.c
 *
 *  Created on: Jun 11, 2019
 *      Author: cbest
 */

#include "Comms.h"
#include "cmsis_os.h"

// Prepares sends off package
bool SendData(COMMS_Data_Message messages[], uint8_t mssg_size, COMMS_Header events[], uint8_t evt_size)
{
	const uint8_t start = 0b10101010;
	const uint8_t stop = 0b11011011;

	// ******** SIZE OF DATA (INCUDING HEADERS)  ********
	uint8_t events_size_h = events != NULL ? sizeof(events[0]) * evt_size : 0;

	uint8_t time_mssg_size_h = sizeof(uint32_t) + sizeof(COMMS_Header);
	uint8_t data_message_size_h = (sizeof(messages[0].type) + sizeof(messages[0].value)) * mssg_size;
	uint8_t total_messages_size_h = data_message_size_h + time_mssg_size_h + events_size_h;

	// ******** SIZE OF DATA (EXCLUDING HEADERS)  ********
	uint8_t time_mssg_size = sizeof(uint32_t);
	uint8_t data_message_size = sizeof(messages[0].value) * mssg_size;
	uint8_t total_messages_size = time_mssg_size + data_message_size;

//	COMMS_Time_Message sys_time = { .type = 0b00000001, .value = HAL_GetTick() };
	COMMS_Payload payload = {
			.start = start,
			.size_payload = total_messages_size_h, // includes headers
			.size_mssgs = total_messages_size, // excluding headers
			.sys_time = { .type = 0b00000001, .value = HAL_GetTick() }, // return number of milliseconds that have elapsed since startup
			.messages = messages,
			.events = events,
			.stop = stop,
	};

	COMMS_PayloadHandle payload_handle = &payload;
	int payload_size = sizeof(payload);
	char byteStream[payload_size];

	EncodePayload(payload_handle, mssg_size, evt_size, byteStream);

//	printf("Size of Struct is: %d, and it should be ??", sizeof(payload));


//	extern huart2;
//	HAL_StatusTypeDef result = HAL_UART_Transmit(&uartHandle, pData, Size, Timeout)

}


char* EncodePayload(COMMS_PayloadHandle packet, uint8_t mssg_size, uint8_t evt_size, char* txPackage)
{
	char* block = txPackage;
	*block = packet->start;				block++;
	*block = packet->size_payload;		block++;
	*block = packet->size_mssgs;		block++;

	// little endian (M3)
	//	high byte eg: 0x12 34 56 78 => 0x12

	// serializing sys_time (32bit/4bytes)
	*block = *((char*)&(packet->sys_time.type));		block++;
	*block = *((char*)&(packet->sys_time.value)+3);		block++;
	*block = *((char*)&(packet->sys_time.value)+2);		block++;
	*block = *((char*)&(packet->sys_time.value)+1);		block++;
	*block = *((char*)&(packet->sys_time.value)+0);		block++;

	// serializing data messages (variable length)
	COMMS_Data_Message* pMssg = packet->messages;
	for (uint8_t i = 0; i < mssg_size; i++, pMssg++)
	{
		*block = *((char*)&(pMssg->type));			block++;
		*block = *((char*)&(pMssg->value)+1);		block++;
		*block = *((char*)&(pMssg->value)+0);		block++;
	}

	// serializing events (variable length)
	COMMS_Header* pEvts = packet->events;
	for (uint8_t i = 0; i < evt_size; i++, pEvts++)
	{
		*block = *((char*)pEvts);					block++;
	}

	*block = *((char*)&(packet->stop));

	// in case the original handle is needed, but not needed at this time
	return txPackage;
}
