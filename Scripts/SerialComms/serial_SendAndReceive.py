import serial
import struct
import time

def splitBytes(byteStr):
  return [byteStr[i:i+1] for i in range(0, len(byteStr), 1)]

# ser = serial.Serial('/dev/ttyACM0', baudrate=38400, timeout=None)
ser = serial.Serial('COM9',
  baudrate=38400, 
  timeout=None,
  parity=serial.PARITY_NONE,
  stopbits=serial.STOPBITS_ONE,
  bytesize=serial.EIGHTBITS)

time.sleep(2)

# Get int representation of the bytes in big endian
start_byte = b'\xaa'
stop_byte = b'\xdb'
pan_header = b'\x04'
tilt_header = b'\x05'
systime_header = b'\x01'
systime_stamp = b'\x12\x34\x56\x79'
event_size_byte = b'\x00'

# def sendDelta(hAngle, vAngle):
data = []
message = []
event = []

total_data_bytes = ( 3 * (4) ) + 5 # 4 data messages (of 3 bytes each) - sysTime total bytes (5 bytes)
total_event_bytes = 2
# https://docs.python.org/2/library/struct.html
data_size_byte = struct.pack('>B', int(total_data_bytes))
event_size_byte = struct.pack('>B', int(total_event_bytes))


# Order of appends in important
message.append(start_byte)
message.append(data_size_byte)
message.append(event_size_byte)
message.append(systime_header)
message.append(systime_stamp)

hAngle = -1125.0
vAngle = -1456.0
hAngle2 = 999.9
vAngle2 = 2909.3

hAngle_b = struct.pack('>h', int(hAngle))
vAngle_b = struct.pack('>h', int(vAngle))
hAngle2_b = struct.pack('>h', int(hAngle2))
vAngle2_b = struct.pack('>h', int(vAngle2))

data.append(pan_header)
data.append(hAngle_b)
data.append(tilt_header)
data.append(vAngle_b)
data.append(pan_header)
data.append(hAngle2_b)
data.append(tilt_header)
data.append(vAngle2_b)

for d in data:
  message.append(d)

# Event Messages
#     COMMS_Switch_RC = 0b10000001,
#     COMMS_Switch_USB = 0b10000010,
event.append(b'\x81') # RC
event.append(b'\x82') # USB

# add event messages
for e in event:
  message.append(e)

# End of message byte
message.append(stop_byte)

# ========================== Formatting the message ================================

message_Tx = []
for m in message:
  message_Tx.append( splitBytes(m) )

# flatten the array to return an array of bytes
message_Tx = [y for x in message_Tx for y in x]
print(message_Tx)
print("\n")

# message MUST be an array of byte objects
# it will be transformed to an array of int (big endian) and transmitted over serial
# _message = [b'\xAA', b'\x0B', b'\x05', b'\x01', b'\x50', b'\x35', b'\x6E', b'\x3E', b'\x04', b'\x55', b'\x44', b'\x05', b'\xDC', b'\xCD', b'\x81', b'\x55', b'\x66', b'\x77', b'\x82', b'\xDB']


# Convert each byte to an int (in Big endian format)
packet = []
for h in message_Tx:
  packet.append( struct.unpack('>B', h)[0] )

# Send the packet over serial
ser.write(packet)
ser.flushInput()

# Sould recieve 6 messages (since 6 messages were sent in a single paylod to the arduino)
while True:
  if ser.in_waiting:
    # The total recieved bytes will ALWAYS be 13
    print(ser.read(13))

