#include "grbl.h"


//used to keep the vesc watchdog alive without recalculating the buffer
uint8_t lastMessage[15];
uint8_t lastMessageDim;
bool lastMessageValid;

//called by a timer ISR
void VescRepeatLastMessage()
{
	if(lastMessageValid) { //dont write new message while it is being updated
    	for(int i = 0; i <= lastMessageDim; i++) {
		  	serial1_write(lastMessage[i]);
	  	}
  	}
}


int receiveUartMessage(uint8_t * payloadReceived) {

	// Messages <= 255 starts with "2", 2nd byte is length
	// Messages > 255 starts with "3" 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF

	uint16_t counter = 0;
	uint16_t endMessage = 256;
	bool messageRead = false;
	uint8_t messageReceived[256];
	uint16_t lenPayload = 0;
	
	// Defining the timestamp for timeout (100ms before timeout)
	uint8_t retryCounter = 0;
	uint8_t maxiterations = 10;

	while ( retryCounter < maxiterations && messageRead == false) {
		retryCounter++;

		while (serial1_get_rx_buffer_count() > 0) {

			messageReceived[counter++] = serial1_read();

			if (counter == 2) {

				switch (messageReceived[0])
				{
					case 2:
						endMessage = messageReceived[1] + 5; //Payload size + 2 for sice + 3 for SRC and End.
						lenPayload = messageReceived[1];
					break;

					case 3:
						// ToDo: Add Message Handling > 255 (starting with 3)
					break;

					default:
					break;
				}
			}

			if (counter >= sizeof(messageReceived)) {
				break;
			}

			if (counter == endMessage && messageReceived[endMessage - 1] == 3) {
				messageReceived[endMessage] = 0;
				messageRead = true;
				break; // Exit if end of message is reached, even if there is still more data in the buffer.
			}
		}
	}
	
	bool unpacked = false;

	if (messageRead) {
		unpacked = unpackPayload(messageReceived, endMessage, payloadReceived);
	}

	if (unpacked) {
		// Message was read
		return lenPayload; 
	}
	else {
		// No Message Read
		return 0;
	}
}


bool unpackPayload(uint8_t * message, int lenMes, uint8_t * payload) {

	uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;

	// Rebuild crc:
	crcMessage = message[lenMes - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += message[lenMes - 2];


	// Extract payload:
	memcpy(payload, &message[2], message[1]);

	crcPayload = crc16(payload, message[1]);

	if (crcPayload == crcMessage) {
		return true;
	} else {
		return false;
	}
}


int packSendPayload(uint8_t * payload, int lenPay) {
	
	uint16_t crcPayload = crc16(payload, lenPay);
	int count = 0;
	uint8_t messageSend[30];

	if (lenPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lenPay;
	}
	else
	{
		//bigger than serial1 tx buffer
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}

	memcpy(&messageSend[count], payload, lenPay);

	count += lenPay;
	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	messageSend[count] = '\0';

	//used to repeat the message and keep the watchdog alive 
	//without having to reporcess the packet from scratch
	//mark the packet as invalid to prevent the interrupt from 
	//accessing it while copying new data
	lastMessageValid = false;

	memcpy(&lastMessage, &messageSend, count);

	lastMessageDim = count;
	lastMessageValid = true;

	for(int i = 0; i <= count; i++) {
		serial1_write(messageSend[i]);
	}

	// Returns number of sent bytes
	return count;
}


bool processReadPacket(uint8_t * message, DataPacket * data) {

	COMM_PACKET_ID packetId;
	int32_t ind = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++; // Removes the packetId from the actual message (payload)

	switch (packetId){
		case COMM_GET_VALUES: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

			ind = 4; // Skip the first 4 bytes 
			data->avgMotorCurrent 	= buffer_get_float32(message, 100.0, &ind);
			data->avgInputCurrent 	= buffer_get_float32(message, 100.0, &ind);
			ind += 8; // Skip the next 8 bytes
			data->dutyCycleNow 		= buffer_get_float16(message, 1000.0, &ind);
			data->rpm 				= buffer_get_int32(message, &ind);
			data->inpVoltage 		= buffer_get_float16(message, 10.0, &ind);
			data->ampHours 			= buffer_get_float32(message, 10000.0, &ind);
			data->ampHoursCharged 	= buffer_get_float32(message, 10000.0, &ind);
			ind += 8; // Skip the next 8 bytes 
			data->tachometer 		= buffer_get_int32(message, &ind);
			data->tachometerAbs 		= buffer_get_int32(message, &ind);
			return true;

		break;

		default:
			return false;
		break;
	}
}

bool getVescValues(DataPacket * data) {

	uint8_t command[1] = { COMM_GET_VALUES };
	uint8_t payload[256];

	packSendPayload(command, 1);
	// delay(1); //needed, otherwise data is not read

	int lenPayload = receiveUartMessage(payload);

	if (lenPayload > 55) {
		bool read = processReadPacket(payload, data); //returns true if sucessful
		return read;
	}
	else
	{
		return false;
	}
}



void VescSetCurrent(float current) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT;
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);

	packSendPayload(payload, 5);
}

void VescSetBrakeCurrent(float brakeCurrent) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);

	packSendPayload(payload, 5);
}

void VescSetRPM(float rpm) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_RPM ;
	buffer_append_int32(payload, (int32_t)(rpm), &index);

	packSendPayload(payload, 5);
}

void VescSetDuty(float duty) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_DUTY;
	buffer_append_int32(payload, (int32_t)(duty * 100000), &index);

	packSendPayload(payload, 5);
}