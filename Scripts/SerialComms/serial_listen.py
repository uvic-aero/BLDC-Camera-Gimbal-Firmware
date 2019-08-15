
import serial

com = 'COM16'
baud = 38400

#   OMMS_START = 0b10101010,
#	COMMS_STOP = 0b11011011,
#	COMMS_SysTime = 0b00000001,
#	COMMS_Curr_Pan = 0b00000010,
#	COMMS_Target_Pan = 0b00000100,
#	COMMS_Curr_Tilt = 0b00000011,
#	COMMS_Target_Tilt = 0b00000101,

headers = {0b10101010 : ('START', 0), 0b11011011 : ('STOP', 0), 0b00000001 : ('SYSTIME', 4), 0b000000010 : ('CURR_PAN',2), 0b00000011 : ('CURR_TILT',2), 0b00000100: ('TARG_PAN',2), 0b00000101 : ('TARG_TILT',2)}

try:
    with serial.Serial(com, baud, timeout = None) as ser:
        while True:
            x = ser.read()
            if x != 0b10101010.to_bytes(1, byteorder='big'):
                continue
            else:
                print(bin(ord(x)))
                break
        
        while True:
            x = ser.read()
            print(bin(ord(x)))
            if x == 0b11011011.to_bytes(1, byteorder='big'):
                break

except KeyboardInterrupt:
    ser.close()
