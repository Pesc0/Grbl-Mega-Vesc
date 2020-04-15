#ifndef _VESCUART_h
#define _VESCUART_h

#include "datatypes.h"
#include "buffer.h"
#include "crc.h"

typedef struct {
	float avgMotorCurrent;
	float avgInputCurrent;
	float dutyCycleNow;
	int32_t rpm;
	float inpVoltage;
	float ampHours;
	float ampHoursCharged;
	int32_t tachometer;
	int32_t tachometerAbs;
} DataPacket;


/**
 * @brief      Sends a command to VESC and stores the returned data
 * @param	   data - reference to the read data
 * @return     True if successfull otherwise false
 */
bool getVescValues(DataPacket * data);
/**
 * @brief      Set the current to drive the motor
 * @param      current  - The current to apply
 */
void VescSetCurrent(float current);
/**
 * @brief      Set the current to brake the motor
 * @param      brakeCurrent  - The current to apply
 */
void VescSetBrakeCurrent(float brakeCurrent);
/**
 * @brief      Set the rpm of the motor
 * @param      rpm  - The desired RPM (actually eRPM = RPM * poles)
 */
void VescSetRPM(float rpm);
/**
 * @brief      Set the duty of the motor
 * @param      duty  - The desired duty (0.0-1.0)
 */
void VescSetDuty(float duty);
/**
 * @brief      Packs the payload and sends it over Serial
 *
 * @param      payload  - The payload as a unit8_t Array with length of int lenPayload
 * @param      lenPay   - Length of payload
 * @return     The number of bytes send
 */
int packSendPayload(uint8_t * payload, int lenPay);
/**
 * @brief      Receives the message over Serial
 *
 * @param      payloadReceived  - The received payload as a unit8_t Array
 * @return     The number of bytes receeived within the payload
 */
int receiveUartMessage(uint8_t * payloadReceived);
/**
 * @brief      Verifies the message (CRC-16) and extracts the payload
 *
 * @param      message  - The received UART message
 * @param      lenMes   - The lenght of the message
 * @param      payload  - The final payload ready to extract data from
 * @return     True if the process was a success
 */
bool unpackPayload(uint8_t * message, int lenMes, uint8_t * payload);
/**
 * @brief      Extracts the data from the received payload
 * @param      data - a reference to the processed data
 * @param      message  - The payload to extract data from
 * @return     True if the process was a success
 */
bool processReadPacket(uint8_t * message, DataPacket * data);

//sends again the last message (stored in a buffer) to keep the vesc watchdog alive, 
//otherwise the spindle will stop after 1 sec
void VescRepeatLastMessage();


#endif
