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
#define PRIO_RXDATA					((UBaseType_t)5)
#define PRIO_DECODE					((UBaseType_t)5)
#define PRIO_TXDATA					((UBaseType_t)5)

/// ARRAY SIZE CONSTANTS ///
#define DMA_RX_BUFFER_SIZE          64
#define UART_BUFFER_SIZE            20

/// QUEUE SIZE CONSTANTS ///
#define QSIZE_PTICH					(1)
#define QSIZE_YAW					(1)
#define QSIZE_ROLL					(1)

/// Constant Types ///
//#define Data_Message_Default		(COMMS_Data_Message defaultVal = { .type = 0, .value = 0 })

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

/// POINTER VARIABLES ///
// these memory locations are freed as soon as the messages are placed in there respective queues
// will have to switch to a different heap to free them;
COMMS_Header* evt_mssg_ptr;
COMMS_Data_Message* data_mssg_ptr;

/// QUEUE HANDLES ///
QueueHandle_t xCurrentPanQueue;
QueueHandle_t xCurrentTiltQueue;
QueueHandle_t xTargetPanQueue;
QueueHandle_t xTargetTiltQueue;

QueueHandle_t xEventsQueue;

QueueHandle_t xPayloadTransferQueue;
QueueHandle_t xPayloadDecodeQueue;


/// TASK HANDLES ///
TaskHandle_t xTaskSerialRx;
TaskHandle_t xTaskSerialTx;
TaskHandle_t xTaskDecodePayload;

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

COMMS_Data_Message __COMMS_DefaultDataMessage(void)
{
	COMMS_Data_Message defaultVal = { .type = 0, .value = 0 };
	return defaultVal;
}

// Grabs a specified message type from the decoded payload if it exists;
COMMS_Data_Message* __COMMS_GetMessage(COMMS_Data_Message* head, uint8_t data_messages_count, COMMS_Data_Message_Type type)
{
	while (data_messages_count-- > 0)
	{
		if (head->type == (COMMS_Header)type)
		{
			return head;
		}
		head++;
	}

	return NULL;
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
	// Init The tasks and Queues before enabling the interrupts
	Comms_InitQueues();
	Comms_InitTasks();

	// Setup DMA to transfer data from UART's internal FIFO onto DMA_RX_Buffer (global)
	HAL_UART_Receive_DMA(&huart2, DMA_RX_Buffer, DMA_RX_BUFFER_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); 	// Disable half transfer complete interrupt
	__HAL_UART_ENABLE_IT (&huart2, UART_IT_IDLE);		// Enable idle line interrupt
}

void Comms_InitTasks(void)
{
	xTaskCreate(vCommsRxData, "ReceiveDMATransfer", configMINIMAL_STACK_SIZE, NULL, PRIO_RXDATA, &xTaskSerialRx);
	xTaskCreate(vCommsTxData, "SendPayload", configMINIMAL_STACK_SIZE, NULL, PRIO_TXDATA, &xTaskSerialTx);
	xTaskCreate(vCommsDecodePayload, "DecodePayload", configMINIMAL_STACK_SIZE, NULL, PRIO_DECODE, &xTaskDecodePayload);
}

void Comms_InitQueues(void)
{
	xTargetPanQueue= xQueueCreate(1, sizeof(COMMS_Data_Message));
	xCurrentPanQueue = xQueueCreate(1, sizeof(COMMS_Data_Message));

	xTargetTiltQueue= xQueueCreate(1, sizeof(COMMS_Data_Message));
	xCurrentTiltQueue = xQueueCreate(1, sizeof(COMMS_Data_Message));

	xEventsQueue = xQueueCreate(100, sizeof(COMMS_Header));
	xPayloadDecodeQueue = xQueueCreate(10, sizeof(COMMS_Payload));
	xPayloadTransferQueue = xQueueCreate(100, sizeof(COMMS_Messages_t));

	// These are used for debugging
	vQueueAddToRegistry(xTargetPanQueue, "Target_Pan");
	vQueueAddToRegistry(xCurrentPanQueue, "Current_Pan");

	vQueueAddToRegistry(xTargetTiltQueue, "Target_Tilt");
	vQueueAddToRegistry(xCurrentTiltQueue, "Current_Tilt");

	vQueueAddToRegistry(xEventsQueue, "Event Queue");
	vQueueAddToRegistry(xPayloadTransferQueue, "Payload_Tx_Queue");
	vQueueAddToRegistry(xPayloadDecodeQueue, "Payload_Decode_Queue");
}


/// TASK FUNCTIONS ///

// Purpose
// 		Check if new data has been received from DMA transfer
//		Places any new data from DMA_RX_Buffer to UART_Buffer
// Notes:
// 		possibly make this a task and have a hard deadline for this
// 		can have this run every 1ms or something like that
// 		Can Have this check if the current total data processed is valid enough to begin decoding
void vCommsRxData(void)
{
	while(true)
	{
		vTaskSuspend(NULL); // Suspend itself (Resumed by ISR)

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


			// Place the payload data onto the decode queue
			// the decode Task is responsible for decoding and clearing out the buffer
			if (__COMMS_IsValidDataStream())
			{
				xQueueSendToBack(xPayloadDecodeQueue, UART_Buffer, (TickType_t)0);
			}
		}

		old_pos = curr_pos;	// save the current position as old

		if (old_pos == ARRAY_LEN(DMA_RX_Buffer))
		{
			old_pos = 0;
		}
	}
}

// Decodes the Payload and place its contents onto their respective queues
// Receive Data from its own queue
//void vCommsDecodePayload(void* pvParams)
void vCommsDecodePayload()
{
	COMMS_PayloadHandle payload;
	while (true)
	{
		/// Read the data from the Queue if any exists
		// using portMAX_DELAY will allow this task to wait till data is on the queue
		// The task is put in the blocked stated till there is a message on the queue
		if ( xQueueReceive(xPayloadDecodeQueue, payload, portMAX_DELAY) != pdPASS )
		{
			return;
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
		{
			return;
		}

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
		{
			return;
		}

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

		// Place the decoded messages onto their respective queues
		COMMS_Data_Message* targetPan = __COMMS_GetMessage(data_mssg_ptr, data_messages_count, COMMS_Target_Pan);
		COMMS_Data_Message* targetTilt =__COMMS_GetMessage(data_mssg_ptr, data_messages_count, COMMS_Target_Tilt);
		COMMS_Data_Message* currentPan = __COMMS_GetMessage(data_mssg_ptr, data_messages_count, COMMS_Curr_Pan);
		COMMS_Data_Message* currentTilt = __COMMS_GetMessage(data_mssg_ptr, data_messages_count, COMMS_Curr_Tilt);

		// place on queue
		if (targetPan != NULL)
		{
			xQueueSendToBack(xTargetPanQueue, (void*)(targetPan), (TickType_t)0);
		}

		if (targetTilt != NULL)
		{
			xQueueSendToBack(xTargetTiltQueue, (void*)(targetTilt), (TickType_t)0);
		}

		if (currentPan != NULL)
		{
			xQueueSendToBack(xCurrentPanQueue, (void*)(currentPan), (TickType_t)0);
		}

		if (currentTilt != NULL)
		{
			xQueueSendToBack(xCurrentTiltQueue, (void*)(currentTilt), (TickType_t)0);
		}

		// Clear out the Buffer to allow for new input
		memset(UART_Buffer, 0, UART_BUFFER_SIZE);	// clear the array
		uart_index_pos = 0;							// reset the ptr value;
	}
}

// Created when needed and discarded after its done
// Receive the data from a queue and
// task should exist forever
// should send the data from the queue
//void vCommsTxData(void* pvParam)
void vCommsTxData(void)
{
	const uint8_t start = COMMS_START;
	const uint8_t stop = COMMS_STOP;

	COMMS_Messages_t queueData;
	// Task is started whenever there is new data on its Queue
	while (true)
	{
//		vTaskSuspend(NULL);	// SUSPEND SELF

		/// Read the data from the Queue if any exists
		// Queue could not retrieve the queue Data value OR queue is empty
		if ( xQueueReceive(xPayloadTransferQueue, &queueData, portMAX_DELAY) != pdPASS )
		{
			// TODO: Implement better error handling
			return;
		}

		// ******** SIZE OF DATA (INCUDING HEADERS) in bytes ********
		uint8_t time_mssg_size_h = sizeof(COMMS_Time_Message);
		uint8_t data_message_size_h = sizeof(COMMS_Data_Message) * queueData.mssg_size;
		uint8_t total_messages_size_h = data_message_size_h + time_mssg_size_h;

		uint8_t events_size = (queueData.events != NULL) ? sizeof(queueData.events[0]) * queueData.evt_size : 0;

		COMMS_Payload payload = {
				.start = start,
				.size_data = total_messages_size_h, // size of all data messages (incl headers)
				.size_events = events_size, // size of all event messages
				.sys_time = { .type = COMMS_SysTime, .value = 1345678910 },// HAL_GetTick() }, // return number of milliseconds that have elapsed since startup
				.messages = queueData.messages,
				.events = queueData.events,
				.stop = stop,
		};

		COMMS_PayloadHandle payload_handle = &payload;
		uint16_t payload_size = sizeof(payload);
		uint8_t byteStream[payload_size];

		__EncodePayload(payload_handle, queueData.mssg_size, queueData.evt_size, byteStream);

		HAL_StatusTypeDef result_tx = HAL_UART_Transmit(&huart2, byteStream, payload_size, 10000);

		if (result_tx != HAL_OK)
		{
			// TODO: Implement Better Error handling
			return;
		}
	}
}
