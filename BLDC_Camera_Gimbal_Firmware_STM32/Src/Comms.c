/*
 * Comms.c
 *
 *  Created on: Jun 11, 2019
 *      Author: cbest
 */

#include "cmsis_os.h"
#include "Comms.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

#define DMA_RX_BUFFER_SIZE          64
extern uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];

#define UART_BUFFER_SIZE            20
extern uint8_t UART_Buffer[UART_BUFFER_SIZE];

#define ARRAY_LEN(x)	(sizeof(x) / sizeof((x)[0]))

uint8_t uart_index_pos;
size_t old_pos;

size_t length, toCopy, Write;
uint8_t* ptr;

bool new_dataStream_available = false;

bool __COMMS_IsValidDataStream(void)
{
	// check that the first element is a start byte
	if (UART_Buffer[0] != COMMS_START)
		return false;

	uint8_t count = 1;
	while(count < UART_BUFFER_SIZE)
	{
		if (UART_Buffer[count++] == COMMS_STOP)
			return true;
	}

	return false;
}

uint8_t __COMMS_GetPayloadLength(void)
{
	return uart_index_pos + 1;
}

bool DecodePayload(COMMS_PayloadHandle payload)
{
	// if there is no new data to process or the data to process is bad, then return false
	if (!new_dataStream_available || !__COMMS_IsValidDataStream())
	{
		memset(UART_Buffer, 0, UART_BUFFER_SIZE);	// clear the array
		uart_index_pos = 0;							// reset the ptr value;
		return false;
	}

//	payload = (COMMS_PayloadHandle)malloc(sizeof (*payload));
	 // for testing
	// 0xAA 0x0B 0x02 0x01 0x50 0x35 0x6E 0x3E 0xAB 0x00 0xFF 0xAB 0x00 0xAA 0x33 0x82 0xDB

	// sys time => 0x50356e3e

	//AA 0B 02 01 50 35 6E 3E   AB 00 FF AB 00 AA 33 69 DB


	// 0xAA 0x0B 0x05 0x01 0x50 0x35 0x6E 0x3E 0xAB 0x00 0xFF 0xAB 0x00 0xAA 0x33 0x82 0x66 0x77 0x88 0xDB

	// get size of Data Messages (in bytes)
	uint8_t size_data_messages = UART_Buffer[1]; // includes 5 bytes of sys time (1 byte header + 4 bytes value)
	uint8_t size_event_messages = UART_Buffer[2];
//	uint8_t sys_time = (UART_Buffer[3] == COMMS_SysTime) ? UART_Buffer[3] : 0; // TODO: Implement better error handling

	// *********** extract the values from the payload **********

	payload->start = UART_Buffer[0];
	payload->size_data = UART_Buffer[1];
	payload->size_events = UART_Buffer[2];

	// extract the System Time
	uint8_t* ptr = &UART_Buffer[3]; // points to first element of Sys time message struct
	payload->sys_time.type = *(ptr++);
	payload->sys_time.value = *(ptr++) << 24 | *(ptr++) << 16 | *(ptr++) << 8 | *(ptr++);

	// extract Data Messages

	// divide by 3 to normalize size for the struct
	uint8_t data_messages_bytes_count = size_data_messages - sizeof(COMMS_Time_Message);
	uint8_t data_messages_count = data_messages_bytes_count / sizeof(COMMS_Data_Message);

	// took 2 weeks to realize that malloc wasnt working. (thought snytax was wrong) (S#!^@&#*@&!!)
	COMMS_Data_Message* data_mssg_ptr = pvPortMalloc(data_messages_count * sizeof(*data_mssg_ptr));
//	vPortFree(payload->messages); // cannot use this in heap_1 (https://freertos.org/a00111.html#heap_1)
	payload->messages = data_mssg_ptr;
	size_t size = data_messages_count * sizeof(COMMS_Data_Message);

	if (data_mssg_ptr == NULL)
		return false;

	uint8_t data_count = 0;

	// fill memory with data messages
	while (data_count < data_messages_count) // remove the sys time message
	{
		COMMS_Header type = *(ptr++);
		uint16_t value = *(ptr++) << 8 | *(ptr++);
		data_mssg_ptr[data_count].type = type;
		data_mssg_ptr[data_count].value = value;
		data_count++;
	}

	// Extract event message
	COMMS_Header* evt_mssg_ptr = pvPortMalloc(sizeof(COMMS_Header) * size_event_messages);
//	vPortFree(payload->events);  // cannot use this in heap_1 (https://freertos.org/a00111.html#heap_1)
	payload->events = evt_mssg_ptr;
	uint8_t event_mssgs_count = 0;

	// fill allocated memory with event messages
	while (event_mssgs_count < size_event_messages)
	{
		evt_mssg_ptr[event_mssgs_count] = *((COMMS_Header*)ptr);
		ptr += sizeof(COMMS_Header);
		event_mssgs_count++;
	}

	payload->stop = *ptr; // get stop byte

	new_dataStream_available = false;	// let system know that there is not new data stream

	return true;
}

// Wraps params in payload ans sends off via UART
bool SendData(COMMS_Data_Message messages[], uint8_t mssg_size, COMMS_Header events[], uint8_t evt_size)
{
	const uint8_t start = COMMS_START;
	const uint8_t stop = COMMS_STOP;

	// ******** SIZE OF DATA (INCUDING HEADERS) in bytes ********
	uint8_t time_mssg_size_h = sizeof(COMMS_Time_Message);
	uint8_t data_message_size_h = sizeof(COMMS_Data_Message) * mssg_size;
	uint8_t total_messages_size_h = data_message_size_h + time_mssg_size_h;

	uint8_t events_size = (events != NULL) ? sizeof(events[0]) * evt_size : 0;

	COMMS_Payload payload = {
			.start = start,
			.size_data = total_messages_size_h, // size of all data messages (incl headers)
			.size_events = events_size, // size of all event messages
			.sys_time = { .type = 0b00000001, .value = 1345678910 },// HAL_GetTick() }, // return number of milliseconds that have elapsed since startup
			.messages = messages,
			.events = events,
			.stop = stop,
	};

	COMMS_PayloadHandle payload_handle = &payload;
	uint16_t payload_size = sizeof(payload);
	uint8_t byteStream[payload_size];

	EncodePayload(payload_handle, mssg_size, evt_size, byteStream);

	HAL_StatusTypeDef result_tx = HAL_UART_Transmit(&huart2, byteStream, payload_size, 1000);

	return true;
}


char* EncodePayload(COMMS_PayloadHandle packet, uint8_t mssg_size, uint8_t evt_size, uint8_t* txPackage)
{
	char* block = txPackage;
	*block = packet->start;								block++;
	*block = packet->size_data;							block++;
	*block = packet->size_events;						block++;

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
		*block = *((char*)&(pMssg->type));				block++;
		*block = *((char*)&(pMssg->value)+1);			block++;
		*block = *((char*)&(pMssg->value)+0);			block++;
	}

	// serializing events (variable length)
	COMMS_Header* pEvts = packet->events;
	for (uint8_t i = 0; i < evt_size; i++, pEvts++)
	{
		*block = *((char*)pEvts);						block++;
	}

	*block = *((char*)&(packet->stop));

	// in case the original handle is needed, but not needed at this time
	return txPackage;
}



// ************************ Code for checking DMA RX buffer for new data and copying the value to the UART buffer for processing  ************

// Purpose
// 		Copies data of size (len) from staring point of a given buffer/array to the global UART buffer
void __COMMS_ProcessData(const void* data, size_t len)
{
	// TODO: Add a flag (global var) to indicate when receiving new data, so i can only add data starting from the beginning of UART_Buffer
	// meaning i'll reset the uart_index_pos value to 0
	// can be based of when i've read the stop byte in the transmission protocol
	const uint8_t* d = data;
	while (len--)
	{
		if (uart_index_pos >= ARRAY_LEN(UART_Buffer))
		{
			uart_index_pos = 0;
		}

		UART_Buffer[uart_index_pos++] = *d;
		d++;
	}
}

// Purpose
// 		Check if New data has been received from DMA transfer
// Notes:
// 		possibly make this a task and have a hard deadline for this
// 		can have this run every 1ms or something like that
void COMMS_RX_Check(void)
{
//	__HAL_UART_ENABLE_IT (&huart2, UART_IT_IDLE);		// Enable idle line interrupt

	// calculate current position in buffer
	// (the total buffer size - the remaining data to transfer) = current pos in buffer
	size_t curr_pos = ARRAY_LEN(DMA_RX_Buffer) - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
	if (curr_pos != old_pos) // check that the pos ptr has changed
	{
		if (curr_pos > old_pos)		// current position is past the prev
		{
			// we are in "linear" mode (all the data we need is contiguous in buffer)
			__COMMS_ProcessData(&DMA_RX_Buffer[old_pos], curr_pos - old_pos);
		}
		else
		{
			// we are in "overflow mode" (the data received from UART has wrapped around the buffer)
			// First process data up to the end of the buffer
			__COMMS_ProcessData(&DMA_RX_Buffer[old_pos], ARRAY_LEN(DMA_RX_Buffer) - old_pos);
			if (curr_pos > 0)
			{
				// then process the data from beginning to the last data point
				__COMMS_ProcessData(&DMA_RX_Buffer[0], curr_pos);
			}
		}

		new_dataStream_available = true;	// global variable that indicates that new data is available for use
	}

	old_pos = curr_pos;	// save the current position as old

	if (old_pos == ARRAY_LEN(DMA_RX_Buffer))
	{
		old_pos = 0;
	}
}


