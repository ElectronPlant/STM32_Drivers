/*
 *                     ************************
 *                     *     imuInterface     *
 *                     ************************
 *
 * ============================================================================
 *
 *     File: 		imuInterface.h
 *   Author:		David Arnaiz
 *     Date:		19 may. 2019	
 *    Brief: 		Communication interface library for the IMU
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
 *  V0.0: 19 may. 2019 
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

#include "imuConfiguration.h"

#include "imuConfiguration.h"
#include "imuConfig.h"
#include "i2c.h"

// Devices
#include "imuFake.h"
#include "mpu6050.h"	// MPU6050 and MPU6000

#include "definitions.h"
#include "time.h"


// -----------------------------------------------------------------------------------------
// -------------------------------    INTERRUPT    -----------------------------------------
// -----------------------------------------------------------------------------------------

//            **************************
//            *      PARAMETERS        *
//            **************************

//--------------------------------------------------
// TIMEOUT
//--------------------------------------------------
#define IMU_INTERRUPT_TIMEOUT 		100			// Interrupt timeout. Number of counts.

//--------------------------------------------------
// External variables
//--------------------------------------------------
extern volatile bool overflow;  				// It will set to true if the measurements overflow
extern volatile bool imu_data_ready;   			// This variable is used as an interrupt
extern volatile uint32_t imu_time_buffer;   	// Time stamp of the measurements

//--------------------------------------------------
// LED
//--------------------------------------------------

// --- Interrupt LED ---
#define IMU_USE_LED								// Comment if the IMU LED is not used
#define IMU_LED 			LD9_PIN				// Pin for the IMU LED

// --- Calibration LED ---
#define IMU_USE_LED_CAL							// Comment if the IMU calibration LED is not used
#define IMU_LED_CAL			LD10_PIN			// Pin of the IMU LED for calibration

//--------------------------------------------------
// Serial debug
//--------------------------------------------------
#define IMU_INTERRUPT_DEBUG						// Uncomment to use the IMU interrupt Serial Debug

//            **************************
//            *        FUNCTION        *
//            **************************
void imuInterrupt(); 	// Interrupt function of the IMU
bool imuIntTimeout();	// Checks the timeout of the interrupt


// -----------------------------------------------------------------------------------------
// ------------------------------  DEVICE SELECTION  ---------------------------------------
// -----------------------------------------------------------------------------------------

// --- Forward declarations ---
class Mpu6050Dev;
class ImuFake;

// Selection
#if defined(IMU_FAKE)
	extern ImuFake imu;
#elif defined(IMU_USE_MPU6050) || defined(IMU_USE_MPU6000)
	extern Mpu6050Dev imu;
#elif defined(IMU_USE_MPU6500) || defined(IMU_USE_MPU9250)
	// TODO create libraries for more sensors
#endif

//            **************************
//            *      ERROR CHECK       *
//            **************************

//--------------------------------------------------
// Configuration Interface
//--------------------------------------------------
#if defined(IMU_FAKE)
	// With the fake target there is no need of communication :p
#elif defined(USE_IMU_SPI) && defined(USE_IMU_I2C)
	// Both communication interfaces enabled
	#error --> [IMU Configuration]: SPI and I2C interfaces enabled at the same time -_-
#elif !(defined (USE_IMU_SPI) || defined (USE_IMU_I2C))
	// No Communication interface defined for the IMU
	#warning --> [IMU Configuration]: No communication interface defined for the IMU :(
#endif


//            **************************
//            *     Configurations     *
//            **************************
// Only one IMU will be used.
// IMU_I2C_ADDR 		--> 7-bit I2C address of the IMU
// IMU_MEAS_REG_BASE 	--> Base register address for the IMU measurements
#if defined(USE_IMU_I2C)

	#if defined(IMU_USE_MPU6050)  || defined (IMU_FAKE)
		#define IMU_I2C_ADDR			I2C_ADDRESS_MPU			// MPU6050 or fake
		#define IMU_MEAS_REG_BASE		MPU_ACCEL_REG_BASE		// MPU6050 or fake
	#elif defined(IMU_USE_MPU9250)
		#define IMU_I2C_ADDR			I2C_ADDRESS_MPU			// MPU9250 same as MPU6050 o_0
	#endif

#endif


//            **************************
//            *        FUNCTIONS       *
//            **************************
// The functions should be the same regardless if it is I2C or SPI :p

bool readImuRegisters(uint8_t reg_addr, uint8_t *p_data, uint8_t data_size);
bool readImuRegister(uint8_t reg_addr, uint8_t *p_data);
bool readImuData(uint8_t reg_addr, int16_t *p_data);
bool readImuMeasurements(int16_t *p_data);
bool writeImuRegister(uint8_t reg_addr, uint8_t value, bool check_data = true);
bool updateImuRegister(uint8_t reg_addr, uint8_t value, uint8_t mask_funct, bool check_data = true);


/* ---- END OF FILE ---- */
