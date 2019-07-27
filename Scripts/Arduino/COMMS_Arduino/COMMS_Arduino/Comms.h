#pragma once

#define ARRAY_LEN(x)	(sizeof(x) / sizeof((x)[0]))

// ================================================ Message Types  =================================================

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

typedef struct __attribute__((packed)) {
    COMMS_Data_Message targetPan;
    COMMS_Data_Message targetTilt;
    COMMS_Data_Message currentPan;
    COMMS_Data_Message currentTilt;
    COMMS_Event_Message switch_rc;
    COMMS_Event_Message switch_usb;
} COMMS_Messages_Struct;

// ***************** Struct Handles ***************

typedef COMMS_Payload* COMMS_PayloadHandle;

// ***************** Function prototypes  ***************

void SendNextPayload(void);

void SendMessage(COMMS_Message message);

COMMS_Payload ConvertToPayload(COMMS_Message mssg);

void EncodePayload(COMMS_PayloadHandle packet, uint8_t mssg_size, uint8_t evt_size, uint8_t* txPackage);

COMMS_Messages_Struct DecodeAndSendPayload(uint8_t* UART_Buffer);