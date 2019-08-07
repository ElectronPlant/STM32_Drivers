/*
 *                     *******************
 *                     *     imuFake     *
 *                     *******************
 *
 * ============================================================================
 *
 *     File: 		imuFake.cpp
 *   Author:		David Arnaiz
 *     Date:		29 jun. 2019	
 *    Brief: 		Fake target for the IMU
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

#ifndef UNUSED
	#define UNUSED(X) (void)X
#endif


//            **************************
//            *        INCLUDES        *
//            **************************
#include <stdbool.h>
#include <stdint.h>

#include "imuFake.h"

#include "imuInterface.h"
#include "serialDebug.h"
#include "imuConfig.h"


// -----------------------------------------------------------------------------------------
// -------------------------------  Fake IMU ClASS  ----------------------------------------
// -----------------------------------------------------------------------------------------

//            **************************
//            *      Configuration     *
//            **************************
// =====================================================
// imuFake()
// Class constructors.
// ---
// Parameters:
//		None
// =====================================================
void ImuFake::imuFake()
{
	imu_state = IMU_NOT_INITIALIZED;
}


// =====================================================
// initialize_1()
// Initialize the IMU 1.
// ---
// Parameters:
//		None
// =====================================================
bool ImuFake::initialize_1()
{
	#ifdef DEBUG_MODE_IMU_FAKE
		Serial.println("Fake IMU Initialized...");
	#endif

	imu_state = IMU_CORRECT;

	return true;
}


// =====================================================
// initialize_2()
// Initialize the IMU 2.
// ---
// Parameters:
//		None
// =====================================================
bool ImuFake::initialize_2()
{
	return true;
}


// =====================================================
// resetSignalPath()
// Reset IMU.
// ---
// Parameters:
//		None
// =====================================================
void ImuFake::resetSignalPath()
{
	return;
}


// =====================================================
// setSampleRate()
// Reset IMU.
// ---
// Parameters:
//		None
// =====================================================
void ImuFake::setSampleRate()
{
	return;
}


// =====================================================
// setLowPowerMode()
// ---
// Parameters:
//		None
// =====================================================
void ImuFake::setLowPowerMode(bool sleep_enabled)
{
	UNUSED(sleep_enabled);
	return;
}


//            **************************
//            *       CALIBRATION      *
//            **************************
// =====================================================
// getOffsetCorrection()
// ---
// Parameters:
//		None
// =====================================================
void ImuFake::getOffsetCorrection()
{
	return;
}


//            **************************
//            *      MEASUREMENTS      *
//            **************************

// =====================================================
// getTemperatureDouble()
// ---
// Parameters:
//		@return double			--> (double) temperature (°C)
// =====================================================
double ImuFake::getTemperatureDouble()
{
	return (double)25.0;
}

// =====================================================
// getTemperature()
// ---
// Parameters:
//		@return int16_t			--> (int16_t) temperature (°C * 100)
// =====================================================
int16_t ImuFake::getTemperature()
{
	return (int16_t)2500;
}

// =====================================================
// getParameter6()
// ---
// Parameters:
//		@param *values_funct	--> (*int16_t) Raw measurements from the IMU
// =====================================================
void ImuFake::getParameter6(int16_t *values_funct)
{
	for (uint8_t i = 0; i < 6; i++) {
		if (i == IMU_FAKE_GRAVITY) values_funct[i] = 16384;
		else values_funct[i] = 0;
	}
}


// =====================================================
// getRefinedValues()
// ---
// Parameters:
//		@param *measurements_funct	--> (*double) Calibrated measurements from the IMU
// =====================================================
void ImuFake::getRefinedValues(double *measurements_funct)
{
	for (uint8_t i = 0; i < 6; i++) {
		if (i == IMU_FAKE_GRAVITY) measurements_funct[i] = 1.0;
		else measurements_funct[i] = 0;
	}
}


/* ---- END OF FILE ---- */
