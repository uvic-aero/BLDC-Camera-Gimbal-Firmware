import serial as s

#set connection parameters
print("Enter the desired COM port number:")
port_num = input()
com_port = 'COM' + str(port_num)
print("Enter the desired baud rate:")
baud_rate = input()

#create serial connection
connection = s.Serial(com_port, baud_rate, timeout = None)

#define static bytes
start_byte = b'\xaa'
stop_byte = b'\xdb' 
pan_header = b'\x04'
tilt_header = b'\x05'
systime_header = b'\x01'
systime_stamp = b'\x00\x00\x00\x00'
event_size_byte = b'\x00'

while(True):
	#define bytes set by user
	data = []
	print("Enter the number of data packets to be sent: ")
	num_data_packets = int(input())
	total_data_bytes = 3 * num_data_packets
	for i in range(num_data_packets):
		print("Enter the type of data packet " + str(i) + ":")
		type_prompt = input()
		if (type_prompt == "pan"):
			data.append(pan_header)
		elif (type_prompt == "tilt"):
			data.append(tilt_header)
		print("Enter the value of the data packet " + str(i) + ":")
		data.append((int(input())).to_bytes(2, byteorder = 'big', signed = True))
	data_size_byte = (total_data_bytes).to_bytes(1, byteorder = 'big')

	#construct the message
	message = []
	message.append(start_byte)
	message.append(data_size_byte)
	message.append(event_size_byte)
	message.append(systime_header)
	message.append(systime_stamp)
	for j in range(len(data)):
		message.append(data[j])
	message.append(stop_byte)

	#send message over serial
	for i in range(len(message)):
		connection.write(message[i])
	print(message)

	#exit or continue sending
	print("Press enter to send a new message or any key to exit")
	resume = input()
	if (resume != ''):
		connection.close()
		break
