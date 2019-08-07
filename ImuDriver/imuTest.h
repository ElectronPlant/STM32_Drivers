/*
 *                     ************************
 *                     *     imuTest     *
 *                     ************************
 *
 * ============================================================================
 *
 *     File: 		imuTest.h
 *   Author:		David Arnaiz
 *     Date:		12 may. 2019	
 *    Brief: 		TODO
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
 *  - imuTest -
 *  ----------------
 *  TODO (Detailed description of the file)
 * 
 *  -------------------
 *  - Acknowledgments -
 *  -------------------
 *  TODO (Reference files here)
 *
 *  --------------
 *  - Change Log -
 *  --------------
 *  V0.0: 12 may. 2019 
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

#include <math.h>

#include "imuTest.h"

#include "imuConfig.h"
#include "i2c.h"
#include "exti.h"
#include "time.h"

#include "serialDebug.h"
#include "imuInterface.h"
#include "mpu6050.h"
#include "ahrs.h"

//            **************************
//            *       PARAMETERS       *
//            **************************
//--------------------------------------------------
// Debug mode
//--------------------------------------------------
//#define DEBUG_MODE_MPU									// Uncomment this to use the serial debug
#ifdef DEBUG_MODE_imuTest
	#define SERIAL_imuTest									// Comment if the serial interface is initialized in the main code
	#define SERIAL_SPEED				115200     			// Serial baud
#endif


//            **************************
//            *        FUNCTIONS       *
//            **************************

void imuTest();


/* ---- END OF FILE ---- */
