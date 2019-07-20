#include  "Comms.h"
/*
 Name:		COMMS_Arduino.ino
 Created:	7/19/2019 12:36:15 PM
 Author:	cbest
*/

/// ARRAY SIZE CONSTANTS ///
#define UART_BUFFER_SIZE            (50)
#define NUMBER_OF_MESSAGES          (5)

/// MODULE LEVEL SHARED VARIABLES ///
// Contains the payload data as copied over from DMA_RX_Buffer
volatile uint8_t UART_Buffer[UART_BUFFER_SIZE];
static COMMS_Message messages[NUMBER_OF_MESSAGES];

// =================================================== Main Program ======================================================


const int buttonPin = 2;     // the number of the pushbutton pin
bool newData = false;
int8_t messages_ptr = 0;

// variables will change:
//volatile int buttonState = 0;         // variable for reading the pushbutton status

void setup() {
    memset((void*)UART_Buffer, 0, UART_BUFFER_SIZE);
    SetupSendingBuffer();
    // initialize the pushbutton pin as an input:
    pinMode(buttonPin, INPUT_PULLUP);
    // Attach an interrupt to the ISR vector
    attachInterrupt(digitalPinToInterrupt(buttonPin), pin_ISR, CHANGE);
    Serial.begin(38400);
    //Serial.println("<Arduino is ready>");
}

void loop() {
    // Nothing here!
    _USART_RX();
    //SendNextPayload();
    //delay(5000);
}

void SetupSendingBuffer(void)
{
    // 	/***********  Tx data Structure (For Testing)  *************/
    // 0XAA (d_size)0X08 (evt_size)0X01 (sys_time)0X01  0x12 0x34 0x56 0x79 (d_mssg) 0x03  0x00 0x12 (evt) 0x81 (stop) 0xDB
    // 0XAA 0X08 0X01 0X01 0x12 0x34 0x56 0x79 0X03 0X00 0X12 0X81 0XDB  <==== EXPECTED 
    // 0XAA 0X08 0X01 0X01 0X12 0X34 0X56 0X79 0X03 0X00 0X12 0X81 0XDB  <==== ACTUAL
    // AA 08 01 01 12 34 56 79 03 00 12 81 DB

    COMMS_Message mssg1;
    mssg1.event = 0x81;
    mssg1.message.type = COMMS_Curr_Tilt;
    mssg1.message.value = 0X12;

    // 0XAA (d_size)0X08 (evt_size)0X01 (sys_time)0X01  0x12 0x34 0x56 0x79 (d_mssg) 0x02  0x00 0x34 (evt) 0x82 (stop) 0xDB
    // 0XAA 0X08 0X01 0X01 0x12 0x34 0x56 0x79 0X02 0X00 0X34 0X82 0XDB  <==== EXPECTED 
    // 0XAA 0X08 0X01 0X01 0X12 0X34 0X56 0X79 0X02 0X00 0X34 0X82 0XDB  <==== ACTUAL
    COMMS_Message mssg2;
    mssg2.event = 0x82;
    mssg2.message.type = COMMS_Curr_Pan;
    mssg2.message.value = 0X34;

    // 0XAA (d_size)0X08 (evt_size)0X00 (sys_time)0X01  0x12 0x34 0x56 0x79 (d_mssg) 0x03  0x00 0x56 (evt) 0x00 (stop) 0xDB
    // 0XAA 0X08 0X00 0X01 0x12 0x34 0x56 0x79 0X03 0X00 0X56 0X00 0XDB  <==== EXPECTED 
    // 0XAA 0X08 0X00 0X01 0X12 0X34 0X56 0X79 0X03 0X00 0X56 0X00 0XDB  <==== ACTUAL
    COMMS_Message mssg3;
    mssg3.message.type = COMMS_Curr_Tilt;
    mssg3.message.value = 0X56;

    // 0XAA (d_size)0X08 (evt_size)0X00 (sys_time)0X01  0x12 0x34 0x56 0x79 (d_mssg) 0x02  0x00 0x78 (evt) 0x00 (stop) 0xDB
    // 0XAA 0X08 0X00 0X01 0x12 0x34 0x56 0x79 0X02 0X00 0X78 0X00 0XDB  <==== EXPECTED 
    // 0XAA 0X08 0X00 0X01 0X12 0X34 0X56 0X79 0X02 0X00 0X78 0X00 0XDB  <==== ACTUAL
    COMMS_Message mssg4;
    mssg4.message.type = COMMS_Curr_Pan;
    mssg4.message.value = 0X78;

    // 0XAA (d_size)0X08 (evt_size)0X01 (sys_time)0X01  0x12 0x34 0x56 0x79 (d_mssg) 0x00  0x00 0x00 (evt) 0x82 (stop) 0xDB
    // 0XAA 0X08 0X01 0X01 0x12 0x34 0x56 0x79 0X00 0X00 0X00 0X82 0XDB  <==== EXPECTED 
    // 0XAA 0X05 0X01 0X01 0X12 0X34 0X56 0X79 0X00 0X00 0X00 0X82 0XDB  <==== ACTUAL
    COMMS_Message mssg5;
    mssg5.event = 0x82;

    messages[0] = mssg1;
    messages[1] = mssg2;
    messages[2] = mssg3;
    messages[3] = mssg4;
    messages[4] = mssg5;
}

void SendNextPayload(void)
{
    SendMessage(messages[messages_ptr++]);
    //Serial.println();
    messages_ptr = messages_ptr % NUMBER_OF_MESSAGES;
}

// ================================================ Plan  =================================================

/*
    1. setup external interrupt from a button press
        a. Use ext interrupt 0
        b. hitting a button will iterate through list of messages to send via UART as a Payload type
        c. utilize encoder logic
  ======================================================================================================== */

void pin_ISR() {
    SendNextPayload();
}



/*
    2. Setup decode fuctionality
    ========================================================================================================== */

COMMS_Messages_Struct DecodePayload(uint8_t* UART_Buffer)
{
    uint8_t size_data_messages;
    uint8_t size_event_messages;
    uint8_t data_messages_bytes_count;
    uint8_t data_messages_count;

    COMMS_Time_Message time_mssg = { 0 };

    COMMS_Data_Message* data_mssg_head;
    COMMS_Event_Message* evt_mssg_head;

    COMMS_Data_Message targetPan;
    COMMS_Data_Message targetTilt;
    COMMS_Event_Message switch_rc;
    COMMS_Event_Message switch_usb;

    // get size of Data Messages (in bytes)
    size_data_messages = UART_Buffer[1]; // includes 5 bytes of sys time (1 byte header + 4 bytes value)
    size_event_messages = UART_Buffer[2];
    data_messages_bytes_count = size_data_messages - sizeof(COMMS_Time_Message);
    data_messages_count = data_messages_bytes_count / sizeof(COMMS_Data_Message);

    // **************** Extract the System Time **************** //
    uint8_t* ptr = &UART_Buffer[3]; // points to first element of Sys time message struct
    time_mssg.type = *(ptr++);
    time_mssg.value = *(ptr++) << 24 | *(ptr++) << 16 | *(ptr++) << 8 | *(ptr++);

    // **************** Extract Start Of Data Messages **************** //
    data_mssg_head = (COMMS_Data_Message*)ptr;
    ptr += data_messages_bytes_count;	// Move ptr past the data messages (point to events)

    // **************** Extract Start Of Event Messages **************** //
    evt_mssg_head = (COMMS_Event_Message*)ptr;
    ptr += size_event_messages;	// move past events and point to end byte

    // **************** Place values on their respective queues **************** //

    for (int i = 0; i < data_messages_count; i++)
    {
        switch (data_mssg_head[i].type)
        {
        case COMMS_Target_Pan:
            targetPan.type = COMMS_Target_Pan;
            targetPan.value = data_mssg_head[i].value << 8 | data_mssg_head[i].value >> 8;
            break;

        case COMMS_Target_Tilt:
            targetTilt.type = COMMS_Target_Tilt;
            targetTilt.value = data_mssg_head[i].value << 8 | data_mssg_head[i].value >> 8;
            break;
        }
    }

    for (int i = 0; i < size_event_messages; i++)
    {
        switch (evt_mssg_head[i])
        {
        case COMMS_Switch_RC:
            switch_rc = COMMS_Switch_RC;
            break;

        case COMMS_Switch_USB:
            switch_usb = COMMS_Switch_USB;
            break;
        }
    }

    COMMS_Messages_Struct returnVal = { targetPan, targetTilt, switch_rc, switch_usb };
    return returnVal;
}

/*
    3. Setup Transmit functionality
        a. Wrap a message type in a payload type
        b. tranmit over UART
========================================================================================================== */


void SendMessage(COMMS_Message message)
{
    COMMS_Payload payload = ConvertToPayload(message);
    uint16_t payload_size = sizeof(COMMS_Payload) - 4;
    uint8_t byteStream[13];

    EncodePayload(&payload, 1, 1, byteStream);

    Serial.write(byteStream, 13);
    //PrintSentByteStream(byteStream, 13);
}

COMMS_Payload ConvertToPayload(COMMS_Message message)
{
    uint8_t events_size;
    uint8_t time_mssg_size_h;
    uint8_t data_message_size_h;
    uint8_t total_messages_size_h;

    COMMS_Payload payload;
    static COMMS_Event_Message Default_Event = { 0 };
    static COMMS_Data_Message Default_Data_Message = { 0 };
    static COMMS_Message Default_Message = { 0 };

    //COMMS_Message message = mssg;

    // ******** SIZE OF DATA (INCUDING HEADERS) in bytes ********
    time_mssg_size_h = sizeof(COMMS_Time_Message);
    data_message_size_h = sizeof(COMMS_Data_Message);
    total_messages_size_h = isValidDataMessage(message.message) ? data_message_size_h + time_mssg_size_h : time_mssg_size_h;

    events_size = (message.event != Default_Event) ? 1 : 0;

    payload.start = COMMS_START;
    payload.size_data = total_messages_size_h;
    payload.size_events = events_size;

    payload.sys_time.type = COMMS_SysTime;
    payload.sys_time.value = 0x12345679;

    payload.messages = isValidDataMessage(message.message) ? &message.message : &Default_Data_Message;
    payload.events = (message.event != Default_Event) ? &message.event : &Default_Event;
    payload.stop = COMMS_STOP;

    return payload;
}

bool isValidDataMessage(COMMS_Data_Message message)
{
    switch (message.type)
    {
    case COMMS_SysTime:
    case COMMS_Curr_Pan:
    case COMMS_Target_Pan:
    case COMMS_Curr_Tilt:
    case COMMS_Target_Tilt:
        return true;
    default:
        return false;
    }
}

void EncodePayload(COMMS_PayloadHandle packet, uint8_t mssg_size, uint8_t evt_size, uint8_t* txPackage)
{
    uint8_t* block = txPackage;
    *block = packet->start;								block++;
    *block = packet->size_data;							block++;
    *block = packet->size_events;						block++;

    // little endian (M3)
    //	high byte eg: 0x12 34 56 78 => 0x12

    // serializing sys_time (32bit/4bytes)
    *block = *((char*) & (packet->sys_time.type));		    block++;
    *block = *((char*) & (packet->sys_time.value) + 3);		block++;
    *block = *((char*) & (packet->sys_time.value) + 2);		block++;
    *block = *((char*) & (packet->sys_time.value) + 1);		block++;
    *block = *((char*) & (packet->sys_time.value) + 0);		block++;

    // serializing data messages
    COMMS_Data_Message* pMssg = packet->messages;
    for (uint8_t i = 0; i < mssg_size; i++, pMssg++)
    {
        *block = *((char*) & (pMssg->type));				block++;
        *block = *((char*) & (pMssg->value) + 1);			block++;
        *block = *((char*) & (pMssg->value) + 0);			block++;
    }

    // serializing events (variable length)
    COMMS_Event_Message* pEvts = packet->events;
    for (uint8_t i = 0; i < evt_size; i++, pEvts++)
    {
        *block = *((char*)pEvts);						    block++;
    }

    *block = *((char*) & (packet->stop));
}


/*  4. setup USART Rx complete interrupt
        a. Decode the payload
        b. Extract all the values on the payload (data and events)
        c. Transmit the payload (Loop-back)

========================================================================================================== */
void _USART_RX()
{
/**
 * Receives COMMS_Target_Pan => 0x55 0x44 | COMMS_Target_Tilt => 0xDC 0xCD
 *  // Start => 					0xAA
	// Size of data messages => 	0x0B
	// Size of event messages => 	0x05
	// Sys time mssg => 			(header)0x01 (value) 0x50 0x35 0x6E 0x3E
	// data messages => 			(header)0x04 (value)0x55 0x44
	//				   				(header)0x05 (value)0xDC 0xCD
	// Events 		=> 				(header)0x81 (header)0x55 (header)0x66 (header)0x77 (header)0x82
	// Stop 		=> 				0xDB
 *
 * 0xAA 0x0B 0x05 0x01 0x50 0x35 0x6E 0x3E 0x04 0x55 0x44 0x05 0xDC 0xCD 0x81 0x55 0x66 0x77 0x82 0xDB
 */
    static bool rxInProgress = false;
    static uint8_t index = 0;

    COMMS_Message mssg1 = { 0 };
    COMMS_Message mssg2 = { 0 };
    COMMS_Message mssg3 = { 0 };
    
    uint8_t _byte;
    COMMS_Messages_Struct result;

    if (!Serial.available())
    {
        return;
    }

    Serial.readBytes((uint8_t*)UART_Buffer, UART_BUFFER_SIZE);
    //while (Serial.available() > 0 && newData == false)
    //{
    //    _byte = Serial.read();

    //    if (_byte == COMMS_START)
    //    {
    //        rxInProgress = true;
    //        if (_byte != COMMS_STOP)
    //        {
    //            UART_Buffer[index++] = _byte;
    //            if (index >= UART_BUFFER_SIZE)
    //            {
    //                index = UART_BUFFER_SIZE - 1; // place at last element of buffer
    //            }
    //        }
    //        else {
    //            rxInProgress = false;
    //            index = 0;
    //            newData = true;
    //        }
    //    }
    //    else if (_byte == COMMS_START)
    //    {
    //        rxInProgress = true;
    //    }
    //}

    COMMS_Messages_Struct rxMssgs = DecodePayload((uint8_t*)UART_Buffer);
    mssg1.event = rxMssgs.switch_rc;
    mssg2.event = 0; mssg2.message = rxMssgs.targetPan;
    mssg3.event = rxMssgs.switch_usb; mssg3.message = rxMssgs.targetTilt;

    // 0XAA (d_size)0X05 (evt_size)0X01 (sys_time)0X01  0x12 0x34 0x56 0x79 (d_mssg) 0x00  0x00 0x00 (evt) 0x81 (stop) 0xDB
    // 0XAA 0X05 0X01 0X01 0x12 0x34 0x56 0x79 0X00 0X00 0X00 0X81 0XDB  <==== EXPECTED 
    // 0XAA 0X05 0X01 0X01 0X12 0X34 0X56 0X79 0X00 0X00 0X00 0X81 0XDB  <==== ACTUAL
    //   
    SendMessage(mssg1);
    delay(2000);

    // 0XAA (d_size)0X08 (evt_size)0X00 (sys_time)0X01  0x12 0x34 0x56 0x79 (d_mssg) 0x04  0x55 0x44 (evt) 0x00 (stop) 0xDB
    // 0XAA 0X08 0X00 0X01 0x12 0x34 0x56 0x79 0X04 0x55 0x44 0X00 0XDB  <==== EXPECTED 
    // 0XAA 0X08 0X00 0X01 0X12 0X34 0X56 0X79 0X04 0X55 0X44 0X00 0XDB  <==== ACTUAL
    SendMessage(mssg2);
    delay(2000);

    // 0XAA (d_size)0X08 (evt_size)0X01 (sys_time)0X01  0x12 0x34 0x56 0x79 (d_mssg) 0x05  0xDC 0xCD (evt) 0x82 (stop) 0xDB
    // 0XAA 0X08 0X01 0X01 0x12 0x34 0x56 0x79 0X05 0xDC 0xCD 0X82 0XDB  <==== EXPECTED 
    // 0XAA 0X08 0X01 0X01 0X12 0X34 0X56 0X79 0X05 0XDC 0XCD 0X82 0XDB  <==== ACTUAL
    SendMessage(mssg3);
    delay(2000);

    // clear uart buffer
    memset((void*)UART_Buffer, 0, UART_BUFFER_SIZE);
}

void showNewData() 
{
    if (newData == true) {
        uint8_t idx = 0;
        Serial.println("This just in ... ");
        while (idx < UART_BUFFER_SIZE)
        {
            Serial.print(UART_Buffer[idx]);
        }
        Serial.println();
        //Serial.println(UART_Buffer);
        newData = false;
    }
}

void PrintSentByteStream(uint8_t buffer[], size_t size)
{
    Serial.println();
    Serial.println("Printing sent data");
    uint8_t idx = 0;
    while (idx < size)
    {
        Serial.print((uint8_t)buffer[idx++]);
        Serial.print("  |  ");
    }
    Serial.println();
}