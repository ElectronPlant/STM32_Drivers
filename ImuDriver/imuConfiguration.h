/*
 *                     ****************************
 *                     *     imuConfiguration     *
 *                     ****************************
 *
 * ============================================================================
 *
 *     File: 		imuConfiguration.h
 *   Author:		David Arnaiz
 *     Date:		22 jun. 2019	
 *    Brief: 		Configuration file for the IMU sensor
 *   Github: 		TODO
 *  License: 		https://creativecommons.org/licenses/by-sa/4.0
 *
 * ============================================================================
 *
 *  Released under the Attribution-ShareAlike 4.0 International (CC BY-SA 4.0).
 *  
 *  This file is part of the MotorDriverCpp_test project. refer to "TODO" for further information.
 *
 *  --------------------
 *  - imuConfiguration -
 *  --------------------
 *  This file is used to configure the IMU sensor of the robot. It can select the sensor, communication
 *  interface orientation or similar.
 *
 *  --------------
 *  - Change Log -
 *  --------------
 *  V0.0: 22 jun. 2019 
 *		-)First issue. 
 * 
 */ 

// Guard
#pragma once


//            **************************
//            *    SENSOR SELECTION    *
//            **************************

// --- Test ---
//#define 	IMU_FAKE				// Uncomment to use the fake IMU target (Overwrites the rest)

// --- Gyro-Accel ---
#define 	IMU_USE_MPU6050			// Uncomment if using a MPU6050 sensor (8kHz gyro only supports I2C)
//#define 	IMU_USE_MPU6000			// Uncomment if using a MPU6000	sensor (8kHz gyro I2C and SPI)
//#define 	IMU_USE_MPU6500			// (TODO) Uncomment if using a MPU6500 sensor (32kHz gyro I2C and SPI)


//--- Gyro-Accel_Compass ---
//#define 	IMU_USE_MPU9250			// (TODO) Uncomment if using a MPU9250 sensor (MPU6500 + AK8963 compass)


// --- Error Check ---
// ===============================================================================================
#if defined(IMU_FAKE)
	// The fake target overwrites the rest
	#pragma message("[IMU COMFIGURATION]: Fake Target selected")
#elif defined(IMU_USE_MPU6050)
	#if defined(IMU_USE_MPU6000) || defined(IMU_USE_MPU6500) || defined(IMU_USE_MPU9250)
		#error --> [IMU Configuration]: More than one IMU sensor selected (1) -_-
	#elif defined(USE_IMU_SPI)
		#error --> [IMU Configuration]: the MPU6050 sensor only support SPI
	#endif
#elif defined(IMU_USE_MPU6000)
	#if defined(IMU_USE_MPU6050) || defined(IMU_USE_MPU6500) || defined(IMU_USE_MPU9250)
		#error --> [IMU Configuration]: More than one IMU sensor selected (2) -_-
	#endif
#elif defined(IMU_USE_MPU6500)
	#if defined(IMU_USE_MPU6050) || defined(IMU_USE_MPU6000) || defined(IMU_USE_MPU9250)
		#error --> [IMU Configuration]: More than one IMU sensor selected (3) -_-
	#endif
#elif defined(IMU_USE_MPU9250)
	#if defined(IMU_USE_MPU6050) || defined(IMU_USE_MPU6000) || defined(IMU_USE_MPU6500)
		#error --> [IMU Configuration]: More than one IMU sensor selected (4) -_-
	#endif
#else
	// No sensor selected
	#define IMU_FAKE
	#warning --> [IMU COMFIGURATION]: No sensor selected (fake target loaded)
#endif
// ===============================================================================================


//            **************************
//            *          FAKE          *
//            **************************
#if defined(IMU_FAKE)


#define USE_IMU_I2C  // Just to prevent compilation errors

#endif

//            **************************
//            *    MPU6050 & MPU6000   *
//            **************************
// The MPU6050 and MPU6000 sensors are joined as they have the same register addresses the only differences
// between the two is the SPI compatibility for the MPU6000.

#if !defined(IMU_FAKE) && (defined(IMU_USE_MPU6050) || defined(IMU_USE_MPU6000))

//--------------------------------------------------
// Configuration
//--------------------------------------------------

// --- Communication interface ---
#define USE_IMU_I2C					// Uncomment to select I2C interface
//#define USE_IMU_SPI				// Uncomment to select SPI interface (only MPU6000)

// --- I2C Address ---
#ifdef USE_IMU_I2C
	//#define MPU_AD0_ADDR_HIGH		// Uncomment if the AD0 pin is pulled-up
	#define MPU_AD0_ADDR_LOW		// Uncomment if the AD0 pin is pulled-down (default)
#endif

#endif


//            **************************
//            *   MPU6500 & MPU9250    *
//            **************************
// The MPU6500 and MPU9250 sensors are joined as they are the same sensor, only that the MPU9250
// has a compass.

#if !defined(IMU_FAKE) && (defined(IMU_USE_MPU6500) || defined(IMU_USE_MPU9250))


#endif


//            **************************
//            * GENERAL CONFIGURATION  *
//            **************************

//--------------------------------------------------
// Orientation
//--------------------------------------------------

// --- Calibration Orientation ---
// The calibration orientation of the MPU indicates the axis where the gravity acceleration will be
// measured during calibration.
#define IMU_CAL_ORIENT_X			// Uncomment if the gravity will be on the X-Axis during calibration
//#define IMU_CAL_ORIENT_Y			// Uncomment if the gravity will be on the Y-Axis during calibration
//#define IMU_CAL_ORIENT_Z			// Uncomment if the gravity will be on the Z_Axis during calibration

// --- Operation Orientation ---
// The operation orientation of the MPU indicates the axis where the gravity acceleration will be
// measured during normal operation
// - Gravity direction -
#define IMU_WORK_ORIENT_X			// Uncomment if the gravity will be on the X-Axis when working
//#define IMU_WORK_ORIENT_Y			// Uncomment if the gravity will be on the Y-Axis when working
//#define IMU_WORK_ORIENT_Z			// Uncomment if the gravity will be on the Z-Axis when working
// - Balance axis -
//#define IMU_WORK_BALANCE_X		// Uncomment if the balance angle is the X-Axis.
#define IMU_WORK_BALANCE_Y			// Uncomment if the balance angle is the Y-Axis.
//#define IMU_WORK_BALANCE_Z		// Uncomment if the balance angle is the Z-Axis.



// --- Error Check ---
// ===============================================================================================
// -- Calibration --
#if defined(IMU_CAL_ORIENT_X) && (defined(IMU_CAL_ORIENT_Y) || defined(IMU_CAL_ORIENT_Z))
	#warning --> [IMU CONFIGURATION]: Multiple orientations defined for the MPU calibration
#endif

#if defined(IMU_CAL_ORIENT_Y) && (defined(IMU_CAL_ORIENT_X) || defined(IMU_CAL_ORIENT_Z))
	#warning --> [IMU CONFIGURATION]: Multiple orientations defined for the MPU calibration
#endif

#if defined(IMU_CAL_ORIENT_Z) && (defined(IMU_CAL_ORIENT_Y) || defined(IMU_CAL_ORIENT_X))
	#warning --> [IMU CONFIGURATION]: Multiple orientations defined for the MPU calibration
#endif

#if !(defined(IMU_CAL_ORIENT_Z) || defined(IMU_CAL_ORIENT_Y) || defined(IMU_CAL_ORIENT_X))
	#error --> [IMU CONFIGURATION]: No Orientation defined for the MPU calibration
#endif

// -- Work --
// - Gravity angle -
#if defined(IMU_WORK_ORIENT_X) && (defined(IMU_WORK_ORIENT_Y) || defined(IMU_WORK_ORIENT_Z))
	#warning --> [IMU CONFIGURATION]: Multiple orientations defined for the MPU work angle
#endif

#if defined(IMU_WORK_ORIENT_Y) && (defined(IMU_WORK_ORIENT_X) || defined(IMU_WORK_ORIENT_Z))
	#warning --> [IMU CONFIGURATION]: Multiple orientations defined for the MPU work angle
#endif

#if defined(IMU_WORK_ORIENT_Z) && (defined(IMU_WORK_ORIENT_Y) || defined(IMU_WORK_ORIENT_X))
	#warning --> [IMU CONFIGURATION]: Multiple orientations defined for the MPU work angle
#endif

#if !(defined(IMU_WORK_ORIENT_Z) || defined(IMU_WORK_ORIENT_Y) || defined(IMU_WORK_ORIENT_X))
	#error --> [IMU CONFIGURATION]: No orientation defined for the work angle
#endif

//- Balance axis -
#if defined(IMU_WORK_BALANCE_X) && (defined(IMU_WORK_BALANCE_Y) || defined(IMU_WORK_BALANCE_Z))
	#warning --> [IMU CONFIGURATION]: Multiple orientations defined for the MPU balance axis
#endif

#if defined(IMU_WORK_BALANCE_Y) && (defined(IMU_WORK_BALANCE_X) || defined(IMU_WORK_BALANCE_Z))
	#warning --> [IMU CONFIGURATION]: Multiple orientations defined for the MPU balance axis
#endif

#if defined(IMU_WORK_BALANCE_Z) && (defined(IMU_WORK_BALANCE_Y) || defined(IMU_WORK_BALANCE_X))
	#warning --> [IMU CONFIGURATION]: Multiple orientations defined for the MPU balance axis
#endif

#if !(defined(IMU_WORK_BALANCE_Z) || defined(IMU_WORK_BALANCE_Y) || defined(IMU_WORK_BALANCE_X))
	#error --> [IMU CONFIGURATION]: No orientation defined for the MPU balance axis
#endif
// ===============================================================================================

/* ---- END OF FILE ---- */
