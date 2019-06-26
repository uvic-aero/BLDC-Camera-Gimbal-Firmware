/*
 * Comms.c
 *
 *  Created on: Jun 11, 2019
 *      Author: cbest
 */

#include "Comms.h"
#include "cmsis_os.h"


extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

#define DMA_RX_BUFFER_SIZE          64
extern uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];

#define UART_BUFFER_SIZE            20
extern uint8_t UART_Buffer[UART_BUFFER_SIZE];

#define ARRAY_LEN(x)	(sizeof(x) / sizeof((x)[0]))

uint8_t uart_ptr_pos;
size_t old_pos;

size_t length, toCopy, Write;
uint8_t* ptr;

// Prepares sends off package
bool SendData(COMMS_Data_Message messages[], uint8_t mssg_size, COMMS_Header events[], uint8_t evt_size)
{
	const uint8_t start = 0b10101010;
	const uint8_t stop = 0b11011011;

	// ******** SIZE OF DATA (INCUDING HEADERS)  ********
	uint8_t time_mssg_size_h = sizeof(uint32_t) + sizeof(COMMS_Header);
	uint8_t data_message_size_h = (sizeof(messages[0].type) + sizeof(messages[0].value)) * mssg_size;
	uint8_t total_messages_size_h = data_message_size_h + time_mssg_size_h;

	uint8_t events_size = (events != NULL) ? sizeof(events[0]) * evt_size : 0;

	// ******** SIZE OF DATA (EXCLUDING HEADERS)  ********
//	uint8_t time_mssg_size = sizeof(uint32_t);
//	uint8_t data_message_size = sizeof(messages[0].value) * mssg_size;
//	uint8_t total_messages_size = time_mssg_size + data_message_size;

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

//	printf("Size of Struct is: %d, and it should be ??", sizeof(payload));

	// from Main()
	HAL_StatusTypeDef result_tx = HAL_UART_Transmit(&huart2, byteStream, payload_size, 1000);
	uint8_t rx_buffer[payload_size];
//	memset(rx_buffer, 0, payload_size);

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


void COMMS_USART2_IrqHandler(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma)
{
	// if the UART idle flag is set, manually disable the dma to trigger a DMA Transfer complete interrupt
	if (huart->Instance->ISR & UART_FLAG_IDLE)
	{
		volatile uint32_t tmp; // discard
		tmp = huart->Instance->ISR;
		tmp = huart->Instance->RDR;
		__HAL_DMA_DISABLE(hdma); // disable DMA to force transfer complete interrupt 
		
		// call the DMA irq handler
		COMMS_DMA_IrqHandler(hdma, huart);
	}
	else
	{
		HAL_UART_IRQHandler(huart);
	}
}

void COMMS_DMA_IrqHandler(DMA_HandleTypeDef *hdma, UART_HandleTypeDef *huart)
{
	DMA_TypeDef* baseAddr = hdma->DmaBaseAddress; // get access to the DMA registers

	if (__HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_TC) != RESET) // if the interrupt is Transfer complete
	{
		// clear the transfer complete flag
		__HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));

		// get the length of the data
		length = DMA_RX_BUFFER_SIZE - hdma->Instance->CNDTR;

		/* Get number of bytes we can copy to the end of buffer */
		toCopy = UART_BUFFER_SIZE - Write;

		// check how many bytes to copy
		if (toCopy > length)
			toCopy = length;

		// check how many bytes to copy
		ptr = DMA_RX_Buffer;
		memcpy(&UART_Buffer[Write], ptr, toCopy); // copy first part

		Write += toCopy;
		length -= toCopy;
		ptr += toCopy;

		if (length)
		{
			memcpy(&UART_Buffer[0], ptr, length);
			Write = length;
		}

		/* Prepare DMA for next transfer */
        /* Important! DMA stream won't start if all flags are not cleared first */

		baseAddr->IFCR = 0x3FU << hdma->ChannelIndex; 		// Clear all interrupts
		hdma->Instance->CMAR = (uint32_t)DMA_RX_Buffer; 	// set memory address for DMA again
		hdma->Instance->CNDTR = DMA_RX_BUFFER_SIZE; 		// Set number of bytes to receive;
		hdma->Instance->CCR |= DMA_CCR_EN;					// Start DMA transfer
	}
}

// move data from DMA buffer to another array so we can process it later
void __COMMS_Process_Data(const void* data, size_t len)
{
	// TODO: Add a flag (global var) to indicate when receiving new data, so i can only add data starting from the beginning of UART_Buffer
	// meaning i'll reset the uart_ptr_pos value to 0
	// can be based of when i've read the stop byte in the transmission protocol
	const uint8_t* d = data;
//	memcpy(&UART_Buffer[uart_ptr_pos], d, len); // copy the newly received data into the UART array
	while (len--)
	{
		if (uart_ptr_pos >= ARRAY_LEN(UART_Buffer))
		{
			uart_ptr_pos = 0;
		}

		UART_Buffer[uart_ptr_pos++] = *d;
		d++;
	}
}


// possible make this a task and have a hard deadline for this
// can have this run every 1ms or something like that
void COMMS_RX_Check(void)
{
//	__HAL_UART_ENABLE_IT (&huart2, UART_IT_IDLE);		// Enable idle line interrupt
//	static size_t old_pos;

	// calculate current position in buffer
	// (the total buffer size - the remaining data to transfer) = current pos in buffer
	size_t curr_pos = ARRAY_LEN(DMA_RX_Buffer) - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
	if (curr_pos != old_pos) // check that the pos ptr has changed
	{
		if (curr_pos > old_pos)		// current position is past the prev
		{
			// we are in "linear" mode (all the data we need is contiguous in buffer)
			__COMMS_Process_Data(&DMA_RX_Buffer[old_pos], curr_pos - old_pos);
		}
		else
		{
			// we are in "overflow mode" (the data received from UART has wrapped around the buffer)
			// First process data up to the end of the buffer
			__COMMS_Process_Data(&DMA_RX_Buffer[old_pos], ARRAY_LEN(DMA_RX_Buffer) - old_pos);
			if (curr_pos > 0)
			{
				// then process the data from beginning to the last data point
				__COMMS_Process_Data(&DMA_RX_Buffer[0], curr_pos);
			}
		}
	}

	old_pos = curr_pos;	// save the current position as old

	if (old_pos == ARRAY_LEN(DMA_RX_Buffer))
	{
		old_pos = 0;
	}
}


