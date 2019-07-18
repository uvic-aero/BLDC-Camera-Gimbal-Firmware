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
#define PRIO_TXDATA					((UBaseType_t)4)
#define PRIO_RXDATA					((UBaseType_t)5)
#define PRIO_DECODE					((UBaseType_t)6)

/// ARRAY SIZE CONSTANTS ///
#define DMA_RX_BUFFER_SIZE          (100)
#define UART_BUFFER_SIZE            (50)


/// QUEUE SIZE CONSTANTS ///
#define QSIZE_PTICH					(1)
#define QSIZE_YAW					(1)
#define QSIZE_ROLL					(1)

// MACROS
#define ARRAY_LEN(x)				(sizeof(x) / sizeof((x)[0]))
#define COMMS_DEFAULT_STACK_SIZE 	((configMINIMAL_STACK_SIZE) * (2))

/* ================= SHARED GLOBAL VARIABLES  ================= */

/// EXTERNAL VARIABLES ///
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

/// MODULE LEVEL SHARED VARIABLES ///
// Contains RAW unprocessed data received by UART and transfered by DMA
static volatile uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];
// Contains the payload data as copied over from DMA_RX_Buffer
static volatile uint8_t UART_Buffer[UART_BUFFER_SIZE];

static uint8_t uart_index_pos;
static uint8_t* ptr;
static size_t old_pos;

/// QUEUE HANDLES ///
QueueHandle_t xEventsQueue;
QueueHandle_t xTargetPanQueue;
QueueHandle_t xTargetTiltQueue;
QueueHandle_t xSystemTimeQueue;
QueueHandle_t xDataTransmitQueue;

/// TASK HANDLES ///
TaskHandle_t xTaskSerialRx;
TaskHandle_t xTaskSerialTx;
TaskHandle_t xTaskDecodePayload;

/* ======================================== FUNCTIONS  ======================================== */


/* ================= Private Module Utility Functions  ================ */

// Purpose
// 		Checks if the data stream transferred from DMA buffer is of valid format
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

// Purpose
// 		Copies data of size (len) from staring point of a given buffer/array to the global UART buffer
void __COMMS_ProcessData(void* data, size_t len)
{
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
// 		Serializes the payload into a byte-stream
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

	// serializing data messages
	COMMS_Data_Message* pMssg = packet->messages;
	for (uint8_t i = 0; i < mssg_size; i++, pMssg++)
	{
		*block = *((char*)&(pMssg->type));				block++;
		*block = *((char*)&(pMssg->value)+1);			block++;
		*block = *((char*)&(pMssg->value)+0);			block++;
	}

	// serializing events (variable length)
	COMMS_Event_Message* pEvts = packet->events;
	for (uint8_t i = 0; i < evt_size; i++, pEvts++)
	{
		*block = *((char*)pEvts);						block++;
	}

	*block = *((char*)&(packet->stop));

	// in case the original handle is needed, but not needed at this time
	return txPackage;
}

// Purpose
// 		Grabs a specified message type from the payload
bool __COMMS_GetMessage(COMMS_Data_Message* messageRef, COMMS_Data_Message* head, uint8_t data_messages_count, COMMS_Data_Message_Type type)
{
	while (data_messages_count-- > 0)
	{
		if (head->type == (COMMS_Header)type)
		{
			messageRef->type = head->type;
			messageRef->value = head->value << 8 | head->value >> 8; // reverse order of byte
			return true;
		}
		head++;
	}

	// No message of the type --COMMS_Data_Message_Type-- exists
	return false;
}

// Purpose
// 		Grabs a specified Event type type from the payload
bool __COMMS_GetEvent(COMMS_Event_Message* event, COMMS_Event_Message* head, uint8_t events_count, COMMS_Event_Message_Type type)
{
	while (events_count-- > 0)
	{
		if (*head == (COMMS_Event_Message)type)
		{
			*event = *head;
			return true;
		}
		head++;
	}

	return false;
}

/// Purpose
/// 	Processes the DMA and UART buffer, moving data from DMA to UART buffer so it can be decoded
void __DMABuffer_to_UARTBuffer(void)
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

		// Place the payload data onto the decode queue
		// the decode Task is responsible for decoding and clearing out the buffer
		if (__COMMS_IsValidDataStream())
		{
//			xQueueSend(xPayloadDecodeQueue, UART_Buffer, (TickType_t)0);
			vTaskResume(xTaskDecodePayload);
		}
	}

	old_pos = curr_pos;	// save the current position as old

	if (old_pos == ARRAY_LEN(DMA_RX_Buffer))
	{
		old_pos = 0;
	}
}


/* ================= Initialization Functions  ================ */

// Purpose
// 		Initializes the Module.
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

// Purpose
// 		Initializes the Tasks.
void Comms_InitTasks(void)
{
	xTaskCreate(vCommsTxData, "SendPayload", COMMS_DEFAULT_STACK_SIZE, NULL, PRIO_TXDATA, &xTaskSerialTx);
	xTaskCreate(vCommsRxData, "ReceiveDMATransfer", COMMS_DEFAULT_STACK_SIZE, NULL, PRIO_RXDATA, &xTaskSerialRx);
	xTaskCreate(vCommsDecodePayload, "DecodePayload", COMMS_DEFAULT_STACK_SIZE, NULL, PRIO_DECODE, &xTaskDecodePayload);
}

// Purpose
// 		Initializes the Queues.
void Comms_InitQueues(void)
{
	xTargetPanQueue= xQueueCreate(1, sizeof(COMMS_Data_Message));
	xTargetTiltQueue= xQueueCreate(1, sizeof(COMMS_Data_Message));

	xEventsQueue = xQueueCreate(10, sizeof(COMMS_Event_Message));
	xSystemTimeQueue = xQueueCreate(1, sizeof(COMMS_Time_Message));
	xDataTransmitQueue = xQueueCreate(10, sizeof(COMMS_Message));

	// These are used for debugging
	vQueueAddToRegistry(xTargetPanQueue, "Target_Pan");
	vQueueAddToRegistry(xTargetTiltQueue, "Target_Tilt");

	vQueueAddToRegistry(xEventsQueue, "Event Queue");
	vQueueAddToRegistry(xSystemTimeQueue, "System_Time_Queue");
	vQueueAddToRegistry(xDataTransmitQueue, "Payload_Tx_Queue");
}


/* ================= Task Functions  ================ */


// Purpose
// 		Check if new data has been received from DMA transfer
//		Places any new data from DMA_RX_Buffer to UART_Buffer
// Notes:
// 		possibly make this a task and have a hard deadline for this
// 		can have this run every 1ms or something like that
// 		Can Have this check if the current total data processed is valid enough to begin decoding
void vCommsRxData(void)
{
	uint32_t ulNotifiedValue;

	while(true)
	{
		// ulNotifiedValue is the counting semaphore's count before being decremented
		ulNotifiedValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Block task indefinitely till it is notified

		// No notification was received
		if (ulNotifiedValue == 0)
		{
			// TODO: Error Handling
		}
		else
		{
			// ulNotifiedValue holds the number of outstanding interrupts. We processes each one;
			while (ulNotifiedValue > 0)
			{
				__DMABuffer_to_UARTBuffer();
				ulNotifiedValue--;
			}
		}

	}
}

// Purpose
// 		Decodes the Payload and place its contents onto their respective queues
//		Receive Data from its own queue
void vCommsDecodePayload()
{
	uint8_t size_data_messages;
	uint8_t size_event_messages;
	uint8_t data_messages_bytes_count;
	uint8_t data_messages_count;

	COMMS_Time_Message time_mssg = {0};

	COMMS_Data_Message* data_mssg_head;
	COMMS_Event_Message* evt_mssg_head;

	COMMS_Data_Message targetPan;
	COMMS_Data_Message targetTilt;
	COMMS_Event_Message switch_rc;
	COMMS_Event_Message switch_usb;

	while (true)
	{
		vTaskSuspend(NULL);	// SUSPEND SELF

		// get size of Data Messages (in bytes)
		size_data_messages = UART_Buffer[1]; // includes 5 bytes of sys time (1 byte header + 4 bytes value)
		size_event_messages = UART_Buffer[2];
		data_messages_bytes_count = size_data_messages - sizeof(COMMS_Time_Message);
		data_messages_count = data_messages_bytes_count / sizeof(COMMS_Data_Message);

		// **************** Extract the System Time **************** //
		ptr = &UART_Buffer[3]; // points to first element of Sys time message struct
		time_mssg.type = *(ptr++);
		time_mssg.value = *(ptr++) << 24 | *(ptr++) << 16 | *(ptr++) << 8 | *(ptr++);

		// **************** Extract Start Of Data Messages **************** //
		data_mssg_head = ptr;
		ptr += data_messages_bytes_count;	// Move ptr past the data messages (point to events)

		// **************** Extract Start Of Event Messages **************** //
		evt_mssg_head = ptr;
		ptr += size_event_messages;	// move past events and point to end byte

		// **************** Place values on their respective queues **************** //
		xQueueSend(xSystemTimeQueue, (void*)(&time_mssg), (TickType_t)0);

		if (__COMMS_GetMessage(&targetPan, data_mssg_head, data_messages_count, COMMS_Target_Pan))
		{
			xQueueSend(xTargetPanQueue, (void*)(&targetPan), (TickType_t)0);
		}

		if (__COMMS_GetMessage(&targetTilt, data_mssg_head, data_messages_count, COMMS_Target_Tilt))
		{
			xQueueSend(xTargetTiltQueue, (void*)(&targetTilt), (TickType_t)0);
		}

		if (__COMMS_GetEvent(&switch_rc, evt_mssg_head, size_event_messages, COMMS_Switch_RC))
		{
			xQueueSend(xEventsQueue, (void*)(&switch_rc), (TickType_t)0);
		}

		if (__COMMS_GetEvent(&switch_usb, evt_mssg_head, size_event_messages, COMMS_Switch_USB))
		{
			xQueueSend(xEventsQueue, (void*)(&switch_usb), (TickType_t)0);
		}

		// Clear out the Buffer to allow for new input
		memset(UART_Buffer, 0, UART_BUFFER_SIZE);	// clear the array
		uart_index_pos = 0;							// reset the ptr value;
	}
}

//
// Purpose
// 		Wraps type --COMMS_Message-- into type --COMMS_Payload-- and transmits via UART
//		Receive Data from its own queue
void vCommsTxData(void)
{
	uint8_t events_size;
	uint8_t time_mssg_size_h;
	uint8_t data_message_size_h;
	uint8_t total_messages_size_h;

	COMMS_Payload payload;
	COMMS_Event_Message Default_Event = 0;
	COMMS_Data_Message Default_Data_Message = {0};
	COMMS_Message Default_Message = {0};

	COMMS_Message queueData;

	while (true)
	{
		// Queue could not retrieve the queue Data value OR queue is empty
		if ( xQueueReceive(xDataTransmitQueue, &queueData, portMAX_DELAY) != pdPASS )
		{
			continue;
		}

		// ******** SIZE OF DATA (INCUDING HEADERS) in bytes ********
		time_mssg_size_h = sizeof(COMMS_Time_Message);
		data_message_size_h = sizeof(COMMS_Data_Message);
		total_messages_size_h = (queueData.message.type != 0) ? data_message_size_h + time_mssg_size_h : time_mssg_size_h;

		events_size = (queueData.event != Default_Event) ? 1 : 0;

		payload.start = COMMS_START;
		payload.size_data = total_messages_size_h;
		payload.size_events = events_size;

		payload.sys_time.type = COMMS_SysTime;
		payload.sys_time.value = HAL_GetTick();

		payload.messages = (queueData.message.type != 0) ? &queueData.message : &Default_Data_Message;
		payload.events = (queueData.event != Default_Event) ? &queueData.event : &Default_Event;
		payload.stop = COMMS_STOP;

		uint16_t payload_size = sizeof(COMMS_Payload) - 4;
		uint8_t byteStream[payload_size];

		__EncodePayload(&payload, 1, 1, byteStream);

		// Use a non-blocking call if possible (DMA)?
		HAL_StatusTypeDef result_tx = HAL_UART_Transmit(&huart2, byteStream, payload_size, 10000);

		if (result_tx != HAL_OK)
		{
			// TODO: Implement Better Error handling
			continue;
		}
	}
}
