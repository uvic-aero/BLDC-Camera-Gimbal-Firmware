/*
 * Comms.c
 *
 *  Created on: Jun 11, 2019
 *      Author: cbest
 */


/* ================= INCLUDES  ================= */

/// PROJECT LEVEL HEADERS ///
#include "main.h"

/// FreeRTOS Headers ///
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"

/// Module Headers ///
#include "cmsis_os.h"
#include "Comms.h"

/* ================= DEFINES  ================= */

/// TASK PRIORITY LEVELS
#define PRIO_RXDATA					((UBaseType_t)2)
#define PRIO_DECODE					((UBaseType_t)2)
#define PRIO_TXDATA					((UBaseType_t)2)

/// ARRAY SIZE CONSTANTS ///
#define DMA_RX_BUFFER_SIZE          64
#define UART_BUFFER_SIZE            20

/// QUEUE SIZE CONSTANTS ///
#define QSIZE_PTICH					(1)
#define QSIZE_YAW					(1)
#define QSIZE_ROLL					(1)

// MACROS
#define ARRAY_LEN(x)	(sizeof(x) / sizeof((x)[0]))

/* ================= SHARED GLOBAL VARIABLES  ================= */

/// EXTERNAL VARIABLES ///
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

/// MODULE LEVEL SHARED VARIABLES ///
// Contains RAW unprocessed data received by UART and transfered by DMA
static uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];
// Contains the payload data as copied over from DMA_RX_Buffer
static uint8_t UART_Buffer[UART_BUFFER_SIZE];

static uint8_t uart_index_pos;
static uint8_t* ptr;
static size_t old_pos;
static size_t length, toCopy, Write;
static bool new_dataStream_available = false;

// pointer values
// these memory locations are freed as soon as the messages are placed in there respective queues
// will have to switch the a different heap to free them;
COMMS_Header* evt_mssg_ptr;
COMMS_Data_Message* data_mssg_ptr;

/// TASK HANDLES ///
TaskHandle_t xTaskSerialRx;
TaskHandle_t xTaskDecodePayload;
TaskHandle_t xTaskSerialTx;

/* ================= FUNCTIONS  ================= */

/// IRQ Handler Callbacks ///
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  puts("from HAL_UART_RxCpltCallback");
  HAL_UART_Transmit(&huart2, DMA_RX_Buffer, DMA_RX_BUFFER_SIZE, 10000);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	puts("from HAL_UART_RxHalfCpltCallback");
	uint16_t rxSize = huart->RxXferSize;
}


/// Private Module Utility Functions ///

// Checks if the data stream transferred from DMA buffer is of valid format
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

// Purpose
// 		Copies data of size (len) from staring point of a given buffer/array to the global UART buffer
// Private method
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


char* __EncodePayload(COMMS_PayloadHandle packet, uint8_t mssg_size, uint8_t evt_size, uint8_t* txPackage)
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


/// INIT FUNCTIONS ///

// Initializes the Module.
/*
 * A DMA controller is only able to issue interrupts when its buffer is either full or halfway full.
 * however considering UART communication, in most cases the received amount of data is not known in advance
 * and the end of transfer cannot be detected.
 * */
void Comms_Init(void)
{
  // Setup DMA to transfer data from UART's internal FIFO onto DMA_RX_Buffer (global)
  HAL_UART_Receive_DMA(&huart2, DMA_RX_Buffer, DMA_RX_BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); 	// Disable half transfer complete interrupt
  __HAL_UART_ENABLE_IT (&huart2, UART_IT_IDLE);		// Enable idle line interrupt

  Comms_InitTasks();
  Comms_InitQueues();
}

void Comms_InitTasks(void)
{
	xTaskCreate(vCommsRxData, "ReceiveDMATransfer", configMINIMAL_STACK_SIZE, NULL, PRIO_RXDATA, xTaskSerialRx);
//	xTaskCreate(vCommsTxData, "SendPayload", configMINIMAL_STACK_SIZE, NULL, PRIO_TXDATA, xTaskSerialTx);
//	xTaskCreate(vCommsDecodePaylod, "DecodePayload", configMINIMAL_STACK_SIZE, NULL, PRIO_DECODE, xTaskDecodePayload);
}

void Comms_InitQueues(void)
{

}


/// TASK FUNCTIONS ///

// Purpose
// 		Check if new data has been received from DMA transfer
//		Places any new data from DMA_RX_Buffer to UART_Buffer
// Notes:
// 		possibly make this a task and have a hard deadline for this
// 		can have this run every 1ms or something like that
void vCommsRxData(void)
{
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

		// TODO: Start the decoding Task here
		// doesnt have to have a high priority, as we can resume decoding at any point
		// but decoding has to be completed before new data is transfered to UART_Buffer
		xTaskCreate(vCommsDecodePayload, "DecodePayload", configMINIMAL_STACK_SIZE, NULL, PRIO_DECODE, NULL);

//			new_dataStream_available = true;	// global variable that indicates that new data is available for use
	}

	old_pos = curr_pos;	// save the current position as old

	if (old_pos == ARRAY_LEN(DMA_RX_Buffer))
	{
		old_pos = 0;
	}

	vTaskDelete(NULL); // DELETE SELF
//	vTaskSuspend(NULL);	// Suspend Self Till triggered by ISR
}

void vCommsDecodePayload(void* pvParams)
{
	COMMS_PayloadHandle payload = (COMMS_PayloadHandle) pvParams;
	// if there is no new data to process or the data to process is bad, then return false
	if (!new_dataStream_available || !__COMMS_IsValidDataStream())
	{
		memset(UART_Buffer, 0, UART_BUFFER_SIZE);	// clear the array
		uart_index_pos = 0;							// reset the ptr value;
	}

	// get size of Data Messages (in bytes)
	uint8_t size_data_messages = UART_Buffer[1]; // includes 5 bytes of sys time (1 byte header + 4 bytes value)
	uint8_t size_event_messages = UART_Buffer[2];

	// *********** extract the values from the payload **********

	payload->start = UART_Buffer[0];
	payload->size_data = UART_Buffer[1];
	payload->size_events = UART_Buffer[2];

	// extract the System Time
	ptr = &UART_Buffer[3]; // points to first element of Sys time message struct
	payload->sys_time.type = *(ptr++);
	payload->sys_time.value = *(ptr++) << 24 | *(ptr++) << 16 | *(ptr++) << 8 | *(ptr++);

	// extract Data Messages
	uint8_t data_messages_bytes_count = size_data_messages - sizeof(COMMS_Time_Message);
	uint8_t data_messages_count = data_messages_bytes_count / sizeof(COMMS_Data_Message);

	data_mssg_ptr = pvPortMalloc(data_messages_count * sizeof(*data_mssg_ptr));
	payload->messages = data_mssg_ptr;
	size_t size = data_messages_count * sizeof(COMMS_Data_Message);

	if (data_mssg_ptr == NULL)
		return false;

	// fill memory with data messages
	uint8_t data_count = 0;
	while (data_count < data_messages_count) // remove the sys time message
	{
		COMMS_Header type = *(ptr++);
		uint16_t value = *(ptr++) << 8 | *(ptr++);
		data_mssg_ptr[data_count].type = type;
		data_mssg_ptr[data_count].value = value;
		data_count++;
	}

	// Extract event message
	evt_mssg_ptr = pvPortMalloc(sizeof(COMMS_Header) * size_event_messages);
	if (evt_mssg_ptr == NULL)
		return false;

	payload->events = evt_mssg_ptr;

	// fill allocated memory with event messages
	uint8_t event_mssgs_count = 0;
	while (event_mssgs_count < size_event_messages)
	{
		evt_mssg_ptr[event_mssgs_count] = *((COMMS_Header*)ptr);
		ptr += sizeof(COMMS_Header);
		event_mssgs_count++;
	}

	payload->stop = *ptr; // get stop byte

	new_dataStream_available = false;	// let system know that there is no new data stream

	vTaskDelete(NULL); // DELETE SELF
}

void vCommsTxData(void* pvParam)
{
	COMMS_SendData_Params* params = (COMMS_SendData_Params*) pvParam;
	const uint8_t start = COMMS_START;
	const uint8_t stop = COMMS_STOP;

	// ******** SIZE OF DATA (INCUDING HEADERS) in bytes ********
	uint8_t time_mssg_size_h = sizeof(COMMS_Time_Message);
	uint8_t data_message_size_h = sizeof(COMMS_Data_Message) * params->mssg_size;
	uint8_t total_messages_size_h = data_message_size_h + time_mssg_size_h;

	uint8_t events_size = (params->events != NULL) ? sizeof(params->events[0]) * params->evt_size : 0;

	COMMS_Payload payload = {
			.start = start,
			.size_data = total_messages_size_h, // size of all data messages (incl headers)
			.size_events = events_size, // size of all event messages
			.sys_time = { .type = 0b00000001, .value = 1345678910 },// HAL_GetTick() }, // return number of milliseconds that have elapsed since startup
			.messages = params->messages,
			.events = params->events,
			.stop = stop,
	};

	COMMS_PayloadHandle payload_handle = &payload;
	uint16_t payload_size = sizeof(payload);
	uint8_t byteStream[payload_size];

	__EncodePayload(payload_handle, params->mssg_size, params->evt_size, byteStream);

	HAL_StatusTypeDef result_tx = HAL_UART_Transmit(&huart2, byteStream, payload_size, 10000);

	vTaskDelete(NULL); // DELETE SELF AFTER CALL
}

//// Should have a very high priority
//void vCommsSendData(void* pvParam)
//{
//	while (true)
//	{
//		COMMS_SendData_Params* params = (COMMS_SendData_Params*) pvParam;
//		xTaskCreate(vCommsTxData, "SendPayload", configMINIMAL_STACK_SIZE, (void*) params, PRIO_TXDATA, NULL);
//		vTaskSuspend(NULL); // Suspend Self
//	}
//}

//void vComms
