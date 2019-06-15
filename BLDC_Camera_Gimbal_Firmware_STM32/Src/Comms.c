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

	COMMS_Payload payload = {
			.start = start,
			.size_payload = total_messages_size_h, // includes headers
			.size_mssgs = total_messages_size, // excluding headers
			.sys_time = HAL_GetTick(), // return number of milliseconds that have elapsed since startup
			.messages = messages,
			.events = events,
			.stop = stop,
	};

	COMMS_PayloadHandle payload_handle = &payload;

	uint8_t* arryPtr = EncodeStruct(COMMS_PayloadHandle);

//	UART_HandleTypeDef uartHandle;
//	HAL_StatusTypeDef result = HAL_UART_Transmit(&uartHandle, pData, Size, Timeout)

}


uint8_t* EncodeStruct(COMMS_PayloadHandle p_handle)
{
	UART_HandleTypeDef uartHandle;
	HAL_StatusTypeDef status = HAL_UART_Transmit(&uartHandle, pData, Size, 20);
}
