import serial as s

#set connection parameters
port_num = 3
com_port = 'COM3'
baud_rate = 38400

#create serial connection
connection = s.Serial(com_port, baud_rate, timeout = None)

#define bytes used in message
start_byte = b'\xaa' #(0b10101010).to_bytes(1, byteorder = 'big')
stop_byte = b'\xdb' #(0b11011011).to_bytes(1, byteorder = 'big')
pan_header = b'\x04' #(0b00000100).to_bytes(1, byteorder = 'big')
tilt_header = b'\x05' #(0b00000101).to_bytes(1, byteorder = 'big')
systime_header = b'\x01'

pan_data1 = b'\xff'
pan_data2 = b'\x4c'

data_size_byte = b'\x08'
event_size_byte = b'\x00'

message = []
message.append(start_byte)
message.append(data_size_byte)
message.append(event_size_byte)
message.append(systime_header)
message.append(b'\xf0')
message.append(b'\xf1')
message.append(b'\xf2')
message.append(b'\xf3')
message.append(pan_header)
message.append(pan_data1)
message.append(pan_data2)
message.append(stop_byte)


for i in range(len(message)):
	connection.write(message[i])

connection.close()

'''
print("Enter the number of data packets to be sent: ")
num_data_packets = raw_input()
total_data_bytes = 3 * num_data_packets
for i in range(num_data_packets):
	print("Enter the type of data packet " + str(i) + ":")
	type_prompt = raw_input()
	if (type_prompt = "pan"):
		data_type[i] = pan_header
	elif (type_prompt = "tilt"):
		data_type[i] = tilt_header
	print("Enter the value of the data packet " + str(i) + ":")
	data_value[i] = bin(raw_input() & 0xffff).to_bytes(2, byteorder = 'big')

data_length_byte = (total_data_bytes).to_bytes(1, byteorder = 'big')

num_events = 0
event_length_byte = (num_events).to_bytes(1, byteorder = 'big')

data_value_1[0] = ()
data_value_1[1] = ()

data_value_2[0] = ()
data_value_2[1] = ()
'''