/*
 *                     ************************
 *                     *     serialDebug     *
 *                     ************************
 *
 * ============================================================================
 *
 *     File: 		serialDebug.cpp
 *   Author:		David Arnaiz
 *     Date:		28 abr. 2019	
 *    Brief: 		UART debugging module
 *   Github: 		TODO
 *  License: 		https://creativecommons.org/licenses/by-sa/4.0
 *
 * ============================================================================
 *
 *  Released under the Attribution-ShareAlike 4.0 International (CC BY-SA 4.0).
 *  
 *  This file is part of the MotorDriverCpp_test project. refer to "TODO" for further information.
 *
 *  --------------
 *  - Change Log -
 *  --------------
 *  V0.0: 28 abr. 2019 
 *		-)First issue.
 * 
 */ 


//            **************************
//            *        INCLUDES        *
//            **************************
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "serialDebug.h"
#include "serialDebugConfig.h"

#include "main.h"  // This is used to capture the USE_SERIAL_DEBUG definition

// -----------------------------------------
// - Only set this if debugging is enabled -
// -----------------------------------------
#ifdef USE_SERIAL_DEBUG

DebugSerial Serial;


//            **************************
//            *        FUNCTIONS       *
//            **************************

// =====================================================
// DebugSerial()
// Class initializer.
// ---
// Parameters:
//		None
// =====================================================
DebugSerial::DebugSerial()
{
	transmit_w = 0;
	buffer_busy = false;
	data_lost = false;
}


//--------------------------------------------------
// Control
//--------------------------------------------------

// =====================================================
// begin()
// Configures and initializes the UART port.
// ---
// Parameters:
//		@param baud_rate	--> (uint32_t) Serial baud rate (set to 115200bps)
// =====================================================
void DebugSerial::begin(uint32_t baud_rate = 115200)
{
	serialDebugUartConfiguration(baud_rate);
}


// =====================================================
// stop()
// Stops the debugging UART.
// ---
// Parameters:
//		None
// =====================================================
void DebugSerial::stop()
{
	serialDebugDisable();
}


// =====================================================
// start()
// Starts the debugging UART after it has been stopped
// ---
// Parameters:
//		None
// =====================================================
void DebugSerial::start()
{
	serialDebugEnable();
}


//--------------------------------------------------
// Transmission
//--------------------------------------------------

// =====================================================
// transmisionCompleted()
// This function is called once the UART transmission is completed.
// ---
// Parameters:
//		None
// =====================================================
void DebugSerial::transmisionCompleted()
{
	if (buffer_busy) transmit_w = 0;
	buffer_busy = false;
}


// =====================================================
// bufferStore()
// Saves the string to the transmit buffer. when the buffer is full it just ignores the rest.
// This function allows to copy strings to the buffer and send them all at once saving computation time.
// If the buffer is being transmitted or it is filled, this function will ignore the rest of the string and
// turn the DEBUG_LED on.
// 		+) The buffer size is set by "DEBUG_BUFFER_SIZE".
// 		+) The buffer is reset by using bufferReset() // it will reset after a transmission is completed
//		+) To transmit the buffer use bufferSend()
// ---
// Parameters:
//		@param *str		--> (const char) String
// =====================================================
void DebugSerial::bufferStore(const char *str)
{
	size_t temp_len = strlen(str);

	if (DEBUG_BUFFER_SIZE - transmit_w == 0 || buffer_busy) {
		LEDS_GPIO_PORT->BSRR = DEBUG_LED;
		return;  // the buffer is full
	} else if ((uint8_t)temp_len > DEBUG_BUFFER_SIZE - transmit_w) {
		// The string doesn't fit completely
		temp_len = DEBUG_BUFFER_SIZE - transmit_w;
		LEDS_GPIO_PORT->BSRR = DEBUG_LED;
		data_lost = true;
	}

	memcpy(serial_buffer + transmit_w, str, temp_len);
	transmit_w += temp_len;
}


// =====================================================
// bufferStoreln()
// bufferStore() adding \n\r at the end.
// ---
// Parameters:
//		@param *str		--> (const char*) String
// =====================================================
void DebugSerial::bufferStoreln(const char *str)
{
	bufferStore(str);
	bufferStore("\n\r");
}


// =====================================================
// bufferSend()
// Sends the transmit buffer.
// This function may block the CPU until the UART is empty.
// ---
// Parameters:
//		None
// =====================================================
void DebugSerial::bufferSend()
{
	// Check if the UART is busy and transmit if not
	if (HAL_UART_Transmit_IT(&serial_debug, serial_buffer, transmit_w) == HAL_BUSY) {

		// UART busy --> check the timeout
		uint32_t time_temp = millis();

		while (millis() - time_temp <= SERIAL_DEBUG_TIMEOUT) {
			if (HAL_UART_Transmit_IT(&serial_debug, serial_buffer, transmit_w) == HAL_OK) break;
			LEDS_GPIO_PORT->BSRR = DEBUG_LED;  // Turn DEBUG_LED ON
		}

		LEDS_GPIO_PORT->BRR = DEBUG_LED;  // Turn DEBUG_LED OFF
	}

	buffer_busy = true;
}


// =====================================================
// bufferReset()
// Reset the transmit buffer.
// If the buffer is busy it will not reset the buffer
// ---
// Parameters:
//		@return bool		--> (bool) If true the buffer is reset correctly.
// =====================================================
bool DebugSerial::bufferReset()
{
	if (!buffer_busy) {
		transmit_w = 0;
		return true;
	}else{
		return false;
	}
}


// =====================================================
// print()
// Sends string through the UART port.
// Note: the string should end with "/0"
// ---
// Parameters:
//		@param *str		--> (uint8_t*) Pointer to a string
// =====================================================
void DebugSerial::print(const char *str)
{

	size_t temp_len = strlen(str);
	uint8_t *p_str = (uint8_t*)str;

	// Check if the UART is busy and transmit if not
	if (HAL_UART_Transmit_IT(&serial_debug, p_str, temp_len) == HAL_BUSY) {

		// UART busy --> check the timeout
		uint32_t time_temp = millis();

		while (millis() - time_temp <= SERIAL_DEBUG_TIMEOUT) {
			if (HAL_UART_Transmit_IT(&serial_debug, p_str, temp_len) == HAL_OK) break;
			LEDS_GPIO_PORT->BSRR = DEBUG_LED;  // Turn DEBUG_LED ON
		}

		LEDS_GPIO_PORT->BRR = DEBUG_LED;  // Turn DEBUG_LED OFF
	}

	// Wait for the UART to be free :( this needs to be done to prevent the buffer from being overwritten
	uint32_t time_temp = millis();

	while (millis() - time_temp <= SERIAL_DEBUG_TIMEOUT) {
		if (serial_debug.gState == HAL_UART_STATE_READY) break;
		LEDS_GPIO_PORT->BSRR = DEBUG_LED;  // Turn DEBUG_LED ON
	}

	LEDS_GPIO_PORT->BRR = DEBUG_LED;  // Turn DEBUG_LED OFF
}


// =====================================================
// println()
// Sends string through the UART port and then the new line character.
// If "DEBUG_PRINTLN_BUFF" parameter is defined (check .h), it will;
//						Reset buffer --> copy to buffer --> send buffer
// If not:
// 						Write the string --> wait till is done --> write the \n\r characters
// Note: the string should end with "/0"
// ---
// Parameters:
//		@param *str		--> (*uint8_t) Pointer to a string
// =====================================================
void DebugSerial::println(const char *str)
{
	print(str);
	print("\n\r"); // new line + carriage return
}


// =====================================================
// bufferInt();
// Adds a sequence of 32 bit integer numbers to the buffer.
// This function is used to retrieve data through serial -_-
// Using 32bits as this allows to represent uint16_t too
// ---
// Parameters:
//		@param num		--> (int32_t) Double number to store in the buffer
// =====================================================
void DebugSerial::bufferInt(int32_t num)
{
	char temp_str[15];
	int temp_count;

	if (transmit_w == 0)  // Do not put comma for the fist
		temp_count = sprintf(temp_str, "%ld", num);
	else
		temp_count = sprintf(temp_str, ",%ld", num);

	if (temp_count > 0) bufferStore(temp_str);  // Add to buffer only if the conversion is successful
}


// =====================================================
// bufferDouble();
// Adds a sequence of double numbers to the buffer.
// This function is used to retrieve data through serial -_-
// Doesn't use the printf for doubles as this increases the code size. it sends data, then 4 decimals
// ---
// Parameters:
//		@param num		--> (double) Double number to store in the buffer
// =====================================================
void DebugSerial::bufferDouble(double num)
{

#ifdef PRINTF_FOR_FLOATS

	char temp_str[50];
	int temp_count;

	if (transmit_w == 0)
		temp_count = sprintf(temp_str, "%.10G", num);
	else
		temp_count = sprintf(temp_str, ",%.10G", num);

	if (temp_count > 0) bufferStore(temp_str);  // Add to buffer only if the conversion is successful

#else

	char temp_str[15];    // Result of the conversion
	char temp_str2[11];    // String for the conversion
	int temp_count;       // This is to indicate if the conversion is correct

	int32_t num_buffer;   // Holds the integer part
	int32_t num_buffer2;  // Holds the decimal part

	// --- Integer part ---
	num_buffer = (int16_t)num;
	num_buffer2 = (num - num_buffer) * 10000.0; // the 10000 should be changes if the precision is modified
	if (num_buffer2 < 0) num_buffer2 *= -1;

	if (num < 0 && num_buffer >= 0) {
		if (transmit_w == 0) {
			strcpy(temp_str2, "-%ld.%.4u");  // .4 precision
		} else {
			strcpy(temp_str2, ",-%ld.%.4u"); // .4 precision
		}
	} else {
		if (transmit_w == 0) {
			strcpy(temp_str2, "%d.%.4u");   // .4 precision
		} else {
			strcpy(temp_str2, ",%d.%.4u");  // .4 precision
		}
	}

	temp_count = snprintf(temp_str, 15, temp_str2, num_buffer, (uint16_t)num_buffer2);

	if (temp_count > 0) bufferStore(temp_str);  // Add to buffer only if the conversion is successful

#endif
}


//--------------------------------------------------
// Reception
//--------------------------------------------------

/*
// =====================================================
// bufferRecieve()
// Receives data to the serial buffer.
// If there is data on the buffer it will keep the data.
// If the buffer gets full the data will be ignored and the serial led turned on.
// ---
// Parameters:
//		@param bytes_num		--> (uint16_t) number of bytes to receive
// =====================================================
void DebugSerial::bufferRecieve(uint16_t bytes_num)
{
	if (!buffer_busy) {  // Check if the buffer is being used

		uint16_t temp_len = bytes_num;
		uint16_t temp_diff = DEBUG_BUFFER_SIZE - transmit_w;

		if (temp_diff < bytes_num) {  // Check if the data doesn't fit

			// Data doesn't fit :(
			LEDS_GPIO_PORT->BSRR = DEBUG_LED;
			data_lost = true;

			if (temp_diff == 0) return;  // The buffer is full
			temp_len = temp_diff;
		}

		// Data fits :)
		buffer_busy = true;
		HAL_UART_Receive_IT(&serial_debug, serial_buffer + transmit_w, temp_len);

	} else {

		// buffer busy
		LEDS_GPIO_PORT->BSRR = DEBUG_LED;
		data_lost = true;
	}
}
*/


// =====================================================
// bufferRecieve()
// Receives data until the "enter" key is pulsed. This function will hold the execution.
// The inputs are received one by one, stored in the serial buffer and send back through the UART.
// Once "enter" is received it will send "/n/r" and return.
// ---
// Parameters:
//		@param bytes_num		--> (uint16_t) number of bytes to receive
// =====================================================
uint8_t *DebugSerial::receiveString()
{

	while (serial_debug.RxState != HAL_UART_STATE_READY);  // Wait for the UART to be correct (no timeout)

	transmit_w = 0;

	uint8_t byte[2];
	byte[1] = 0;
	bool receiving = true;

	while (receiving) {

		// --- Read ---
		*byte = read();

		// --- Write ---
		switch (*byte) {
			case 127:  									// "/" Return (delete)
				if (transmit_w > 0) transmit_w --;
				print((const char*)byte);
				break;
			case 0: 									// "/0" Null pointer (timeout on read)
				break; // ignore null
			case 13: 									// "/r" Carriage return (enter)
				receiving = false;
				print("\n\r");
				break;
			default:
				if (transmit_w < DEBUG_BUFFER_SIZE - 1) {
					serial_buffer[transmit_w] = *byte;
					transmit_w ++;
					print((const char*)byte);
				}
				break;
		}
	}
	serial_buffer[transmit_w] = 0;  // This is the null character at the end
	transmit_w ++;
	return serial_buffer;
}


// =====================================================
// receiveInt()
// Does receiveString() and transforms it to an 32 bit integer.
// A 32 bit integer will allow coding a uint16_t value too :)
// ---
// Parameters:
//		None
// =====================================================
int32_t DebugSerial::receiveInt() {
	receiveString();
	return atol((const char*)serial_buffer);
}


// =====================================================
// receiveDouble()
// Does receiveString() and transforms it to double.
// ---
// Parameters:
//		None
// =====================================================
double DebugSerial::receiveDouble() {
	receiveString();
	return atof((const char*)serial_buffer);
}

// =====================================================
// readMultiple()
// Reads multiple bytes in blocking mode.
// ---
// Parameters:
//		@param *data			--> (uitn8_t *) Pointer to the uint8 array
//		@param bytes_num		--> (uint16_t) Number of bytes to receive
// =====================================================
void DebugSerial::readMultiple(uint8_t *data, uint16_t byte_num)
{
	HAL_UART_Receive(&serial_debug, data, byte_num, SERIAL_DEBUG_TIMEOUT);
}


// =====================================================
// read()
// Reads one byte from the UART.
// Compatible with the Arduino Serial.read()
// ---
// Parameters:
//		None
// =====================================================
uint8_t DebugSerial::read()
{
	uint8_t received;
	if (HAL_UART_Receive(&serial_debug, &received, 1, SERIAL_DEBUG_TIMEOUT) == HAL_OK)
		return received;
	else
		return 0x00;
}


//            **************************
//            *        Testing         *
//            **************************
// =====================================================
// serialDebug
// Dummy function to initialize the debug from the main code
// Called in the main.c file.
// ---
// Parameters:
//		None
// =====================================================
void serialDebug()
{
	#ifdef SERIAL_DEBUG_SEFL_CONFIG
		Serial.begin();
	#endif

	#ifdef SERIAL_DEBUG_QUIET
		Serial.bufferReset();
		Serial.println("\n\r\n\r\t\t\t\t#####################");
		Serial.println(        "\t\t\t\t#    BEEPY Debug    #");
		Serial.println(        "\t\t\t\t#####################\n\r");
		Serial.println("Welcome to the debug terminal!  :)");
	#endif

	/*
	 // --- Double receive and print example ---

	double test[10];
	for (uint8_t i = 0; i< 10; i++) {
		test[i] = Serial.receiveDouble();
	}

	Serial.bufferReset();
	for (uint8_t i = 0; i< 10; i++) {
		Serial.bufferDouble(test[i]);
	}
	Serial.bufferStoreln("");
	Serial.bufferSend();
	*/

	/*
	 // --- uint16_t receive and print example ---

	uint16_t test2[10];
	for (uint8_t i = 0; i< 10; i++) {
		test2[i] = (uint16_t)Serial.receiveInt();
	}

	Serial.bufferReset();
	for (uint8_t i = 0; i< 10; i++) {
		Serial.bufferInt((int32_t)test2[i]);
	}
	Serial.bufferStoreln("");
	Serial.bufferSend();
	*/

}

#endif
/* ---- END OF FILE ---- */
