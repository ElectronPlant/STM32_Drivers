/*
 *                     ************************
 *                     *     serialDebug     *
 *                     ************************
 *
 * ============================================================================
 *
 *     File: 		serialDebug.h
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
 *  ----------------
 *  - serialDebug -
 *  ----------------
 *  Serial Debugging module. This module is used to communicate the microcontroller with the PC
 *  to simplify the debugging.
 * 
 *  -------------------
 *  - Acknowledgments -
 *  -------------------
 *
 *
 *  --------------
 *  - Change Log -
 *  --------------
 *  V0.0: 28 abr. 2019 
 *		-)First issue.
 * 
 */ 

// Guard
#pragma once

//            **************************
//            *        INCLUDES        *
//            **************************
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "main.h"  	// This is used to capture the USE_SERIAL_DEBUG definition
					// main.h should have all the includes

//--------------------------------------------------
// Debug mode
//--------------------------------------------------
#ifndef USE_SERIAL_DEBUG

	#define USE_SERIAL_DEBUG						// Uncomment this to use the serial debug
													// Declare this in the specific file
	#ifdef USE_SERIAL_DEBUG
		#define SERIAL_DEBUG_SEFL_CONFIG			// Uncomment to test the serial itself
		//#define SERIAL_DEBUG_QUIET				// Uncomment to have initial menu display
	#endif

#endif

// -----------------------------------------
// - Only set this if debugging is enabled -
// -----------------------------------------
#ifdef USE_SERIAL_DEBUG


//            **************************
//            *       PARAMETERS       *
//            **************************

//--------------------------------------------------
// Printf()
//--------------------------------------------------
// To use this add "-u _printf_float" in:
//			Project -> Properties -> C/C++ Build -> Settings -> Tool Settings -> C++ Linker --> Command
// https://community.st.com/s/question/0D50X00009XkYaySAF/how-to-print-float-value-with-printf-in-truestudio-
#ifndef PRINTF_FOR_FLOATS
	#define PRINTF_FOR_FLOATS								// Uncomment this to use printf() for floats
#endif


#define SERIAL_DEBUG_TIMEOUT		200						// Serial timeout in [ms]


extern UART_HandleTypeDef serial_debug;						// Peripheral handler


//--------------------------------------------------
// Serial Buffer
//--------------------------------------------------
const uint8_t DEBUG_BUFFER_SIZE =	120;					// Maximum number if characters in the buffer

//--------------------------------------------------
// Serial LED
//--------------------------------------------------
// This LED is used to indicate if there is a problem with the serial buffer. The errors are:
//		+) The CPU has been blocked because of the serial buffer. It will be turned off when the CPU is free
//		+) The Buffer has been filled and text has been ignored.
#define DEBUG_LED					LD8_PIN					// LED used for the serial errors


//            **************************
//            *   DEBUG SERIAL CLASS   *
//            **************************

// The serial class has been implemented to support some of the basic Arduino Serial print
// (with no formating) codes, but anything beyond that will not be supported.

class DebugSerial{
	public:

		// --- variables ---
		uint8_t 			serial_buffer[DEBUG_BUFFER_SIZE];	// Buffer for the transmit data

		// --- Functions ---
		DebugSerial();											// Initializer;

		// -- Configuration --
		void begin(uint32_t baud_rate /* = 115200 */);			// Configures and starts the serial port
		void stop();											// Stops the serial port
		void start();											// Starts the serial port

		// -- Transmission --
		// - Buffer -
		void transmisionCompleted();
		void bufferStore(const char *str);						// Add string to the transmit buffer
		void bufferStoreln(const char *str);					// Buffer store with extra line
		void bufferSend();										// Send the transmit buffer
		bool bufferReset();										// Reset the transmit buffer
		void bufferInt(int32_t num);							// Adds 32bit integer to the buffer
		void bufferDouble(double num);							// Adds double value to the buffer
		// - "Arduino compatible" (more or less)
		void print(const char str[]);							// Serial print
		void println(const char str[]);							// Serial print with \n
		//void printf(uint8_t *str, const uitn8_t *format, ...);	// Serial print with

		// -- Receive --
		uint8_t read();											// Reads a byte from UART in blocking mode
		void readMultiple(uint8_t *data,
			  			  uint16_t byte_num);					// Reads multiple bytes from UART in
																// blocking mode
		uint8_t *receiveString();								// Receives string from UART
		int32_t receiveInt();									// Receives int from UART
		double receiveDouble();									// Receives double from UART


	private:
		volatile uint8_t 	transmit_w;							// Pointer to write the transmit buffer
		volatile bool 		buffer_busy;						// True if the buffer is being transmitted
		bool 				data_lost;							// True of data has been ignored
};

//            **************************
//            *       FUNCTIONS        *
//            **************************
void serialDebug();

//--------------------------------------------------
// Compatibility
//--------------------------------------------------
extern DebugSerial Serial;			// Predefine the class to make it compatible
inline const char* F(const char *str) 	// Flash strings still need to be implemented better
{
	return str;
}

#endif
/* ---- END OF FILE ---- */
