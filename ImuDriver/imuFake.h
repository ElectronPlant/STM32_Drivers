/*
 *                     *******************
 *                     *     imuFake     *
 *                     *******************
 *
 * ============================================================================
 *
 *     File: 		imuFake.h
 *   Author:		David Arnaiz
 *     Date:		29 jun. 2019	
 *    Brief: 		IMU fake target
 *   Github: 		TODO
 *  License: 		https://creativecommons.org/licenses/by-sa/4.0
 *
 * ============================================================================
 *
 *  Released under the Attribution-ShareAlike 4.0 International (CC BY-SA 4.0).
 *  
 *  This file is part of the MotorDriverCpp_test project. refer to "TODO" for further information.
 *
 *  -----------
 *  - imuFake -
 *  -----------
 *  The IMU fake target behaves as a IMU sensor, but just returns 0.
 *
 *  --------------
 *  - Change Log -
 *  --------------
 *  V0.0: 29 jun. 2019 
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

#include "imuInterface.h"
#include "imuConfig.h"

#include "serialDebug.h"


//            **************************
//            *       PARAMETERS       *
//            **************************
// The parameters are used to change the basic configurations easily

//--------------------------------------------------
// Debug mode
//--------------------------------------------------
#define DEBUG_MODE_IMU_FAKE								// Uncomment this to use the serial debug
#ifdef DEBUG_MODE_IMU_FAKE
	//#define SERIAL_INIT_MPU 							// Uncomment to initialize the serial port here
	//#define SERIAL_SPEED 					115200  	// Serial baud
#endif

//--------------------------------------------------
// Default Values
//--------------------------------------------------

// --- Temperature ---
#define IMU_FAKE_TEMPERATURE_DOUBLE			(double)25.0	// Fake temperature (°C)
#define IMU_FAKE_TEMPERATURE				(int16_t)2500	// Fake temperature (°C * 100)

// --- Measurements ---
#if defined(IMU_WORK_ORIENT_X)
	#define IMU_FAKE_GRAVITY 				0				// Set gravity in the X-axis
#elif defined(IMU_WORK_ORIENT_Y)
	#define IMU_FAKE_GRAVITY				1				// Set gravity in the Y-axis
#else
	#define IMU_FAKE_GRAVITY				2				// Set gravity in the z-axis
#endif

//            **************************
//            *    FAKE CLASS OBJECT   *
//            **************************

class ImuFake{

	public:

		// --- IMU state ---
		uint8_t imu_state;									// State of the IMU

		// --- Functions ---
		void imuFake();										// Constructor

		// -- Configuration --
		bool initialize_1();								// First initialization, configures the IMU
		bool initialize_2();								// Second initialization, calibrates the IMU

		void resetSignalPath();								// Resets the signal path
		void setSampleRate();								// Sets the sample rate for the IMU
		void setLowPowerMode(bool sleep_enabled);			// Sets the MPU to low power mode or wakes it up

		// -- Calibration --
		void getOffsetCorrection();							// Gets the offset correction values

		// -- Measurements --
		double getTemperatureDouble();						// Gets the temperature as a double (°C)
		int16_t getTemperature();							// Gets the temperature as int16_t (°C * 100)
		void getParameter6(int16_t *values_funct);			// Gets the measurements from the IMU
		void getRefinedValues(double *measurements_funct);	// Gets the refined measurements from the IMU

	private:
};


/* ---- END OF FILE ---- */
