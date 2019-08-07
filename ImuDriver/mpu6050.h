/*
 *                     *******************
 *                     *     mpu6050     *
 *                     *******************
 *
 * ============================================================================
 *
 *     File: 		mpu6050.h
 *   Author:		David Arnaiz
 *     Date:		25 may. 2019	
 *    Brief: 		Driver for the MPU6050 gyroscope accelerometer
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
 *  V0.0: 25 may. 2019 
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
#include "serialDebug.h"
#include "memory.h"

#include "imuConfiguration.h"


//            *********************
//            *       STATE       *
//            *********************
// The state variable is used to detect if there has been an error so the system may react to it.
// The values are:
//		0			--> Not set.
//		1			--> MPU configured and working correctly.
//		2			--> Communication error (I2C or SPI)
//		3			--> Self-test error X-Accelerometer.
//		4			--> Self-test error Y-Accelerometer.
//		5			--> Self-test error Z-Accelerometer.
//		6			--> Self-test error X-Gyroscope.
//		7			--> Self-test error Y-Gyroscope.
//		8			--> Self-test error Z-Gyroscope.
//		9			--> The MPU needs to be calibrated.
//		10			--> The MPU couldn't be calibrated.
//		11			--> The interrupt has timed out


extern volatile bool imu_data_ready;  // Just for now TODO move were appropriate (and change name?)

// --- States of the MPU ---
#define IMU_NOT_INITIALIZED 				0		// MPU still not configured/tester/calibrated
#define IMU_CORRECT							1		// MPU working correctly
#define IMU_COMMS_ERROR						2		// Communication error
#define IMU_SELF_TEST_FAILED_BASE			3		// The MPU has failed one of the self-tests
													// 3 = A_X, 4 = A_Y, 5 = A_Z, 6 = G_X, 7 = G_Y, 8 = G_Z
#define IMU_NOT_CALIBRATED					9		// MPU needs to be calibrated
#define IMU_CALIBRATION_ERROR				10		// MPU couldn't be calibrated
#define IMU_INTERRUPT_ERROR					11		// MPU interrupt timeout


//            **************************
//            *       PARAMETERS       *
//            **************************
// The parameters are used to change the basic configurations easily

//--------------------------------------------------
// Debug mode
//--------------------------------------------------
//#define DEBUG_MODE_MPU								// Uncomment this to use the serial debug
#ifdef DEBUG_MODE_MPU
	//#define SERIAL_INIT_MPU 						// Uncomment to initialize the serial port here
	//#define SERIAL_SPEED 					115200  // Serial baud
#endif

//--------------------------------------------------
// I2C BUS
//--------------------------------------------------

// --- I2C instance ---
#define IMU_I2C_INSTANCE_NUM				2		// Number of the I2C instance (1 or 2) TODO move

// --- I2C Address ---

/*
 * Moved to the configuration file
//#define MPU_AD0_ADDR_HIGH 						// Uncomment if AD0 is set high
//#define MPU_AD0_ADDR_LOW							// Uncomment if AD0 is set low (default value)
*/

// --- I2C configuration ---
#define I2C_CONFIGURE_MPU  		    				// Comment if the I2C is defined in the main code

#ifdef I2C_CONFIGURE_MPU
	#define I2C_CLK_SPEED 					400000  // I2C CLK speed (set as 400k fast mode I2C)
#endif

#define I2C_TIMEOUT_CON            			100		// I2C timeout in ms
#define I2C_MPU_RETRIES						5		// Number of retries after a timeout

// --- Device ID ---
#define MPU_DEVICE_ID_REG					0x75	// Register address with the device ID
#define MPU_DEVICE_ID_VALUE					0x68	// ID of the device
													// (bits 7 and 0 are assumed to be 0)


//--------------------------------------------------
// Self-test and full-scale
//--------------------------------------------------

// --- Accelerometer ---
#define MPU_ACCELEROMETER_CONF_ADDR			0x1C	// Register address to configure the accelerometer
#define MPU_SELF_TEST_ACCEL_REG_VALUE		0xF0	// Register value to set the self-test on the accelerometer (+-8g and self-test bits set to '1')
#define MPU_DEFAULT_ACCEL_REG_VALUE			0x00	// Register value to set the accelerometer with the default configuration (+-2g) [for calibration]
#define MPU_ACCEL_CONFIG_MASK_VALUE			0xF8	// Mask for the accelerometer configuration register
// #define MPU_ACCEL_FS_2G							// Sets the working full-scale of the accel to 2g
// #define MPU_ACCEL_FS_4G    						// Sets the working full-scale of the accel to 4g
#define MPU_ACCEL_FS_8G		    					// Sets the working full-scale of the accel to 8g
// #define MPU_ACCEL_FS_16G				    		// Sets the working full-scale of the accel to 16g

#ifdef MPU_ACCEL_FS_16G
	#define MPU_ACCEL_CONFIG_VALUE 			0x18	// This will set the full-scale to 16g
	const double accel_1g_value = 2048;				// Sensitivity at 16g full-scale
#elif defined MPU_ACCEL_FS_8G
	#define MPU_ACCEL_CONFIG_VALUE 			0x10	// This will set the full-scale to 8g
	const double accel_1g_value = 4096;				// Sensitivity at 8g full-scale
#elif defined MPU_ACCEL_FS_4G
	#define MPU_ACCEL_CONFIG_VALUE 			0x08	// This will set the full-scale to 4g
	const double accel_1g_value = 8192;				// Sensitivity at 4g full-scale
#else
	#define MPU_ACCEL_CONFIG_VALUE 			0x00	// This will set the full-scale to 2g
	const double accel_1g_value = 16384;			// Sensitivity at 2g full-scale (default value)
#endif

// --- Gyroscope ---
#define MPU_GYRO_CONF_ADDR					0x1B	// Register address to configure the gyroscope
#define MPU_SELF_TEST_GYRO_REG_VALUE		0xE0	// Register value to set the self-test on the gyroscope (+-250dps and self-test bits set to '0')
#define MPU_DEFAULT_GYRO_REG_VALUE			0x00	// Register value to set the gyroscope with the default configuration (+-250dps) [defined below]
#define MPU_GYRO_CONFIG_MASK_VALUE			0xF8	// Mask for the gyroscope configuration register
// #define MPU_GYRO_FS_250DPS						// Sets the working full-scale of the gyro to 250dps
// #define MPU_GYRO_FS_500DPS		   				// Sets the working full-scale of the gyro to 500dps
#define MPU_GYRO_FS_1000DPS							// Sets the working full-scale of the gyro to 1000dps
// #define MPU_GYRO_FS_2000DPS						// Sets the working full-scale of the gyro to 2000dps

#ifdef MPU_GYRO_FS_2000DPS
	#define MPU_GYRO_CONFIG_VALUE			0x18	// This will set the full-scale to 2000dps
	const double gyro_1dps_value = 16.4;			// Sensitivity at 2000dps full-scale
#elif defined MPU_GYRO_FS_1000DPS
	#define MPU_GYRO_CONFIG_VALUE			0x10	// This will set the full-scale to 1000dps
	const double gyro_1dps_value = 32.8;			// Sensitivity at 1000dps full-scale
#elif defined MPU_GYRO_FS_500DPS
	#define MPU_GYRO_CONFIG_VALUE			0x08	// This will set the full-scale to 500dps
	const double gyro_1dps_value = 65.5;			// Sensitivity at 500dps full-scale
#else
	#define MPU_GYRO_CONFIG_VALUE			0x00	// This will set the full-scale to 250dps
	const double gyro_1dps_value = 131;				// Sensitivity at 250dps full-scale (default value)
#endif

// --- Configuration ---
#define MPU_SELF_TEST_WAIT_TIME				250 	// Time in ms that the program will wait while the self-test are performed
#define MPU_SELF_TEST_THRESHOLD				14		// Maximum percentage allowed for the self-test, 14% according to the datasheet
#define MPU_SELF_TEST_RESULT_ADDR_BASE		0x0D	// Base address for the Self-test results


//--------------------------------------------------
// Configuration
//--------------------------------------------------

// --- Clock reference ---
#define MPU_CLOCK_REF_ADDR					0x1B	// Address of the register to configure the clock reference of the MPU
#define MPU_CLOCK_REF_MASK					0x07	// Mask for the clock reference configuration
#define MPU_CLOCK_ZGYRO						0x00	// Sets the Gyro-Z PLL as reference (default used)
#define MPU_CLOCK_XGYRO						0x01	// Sets the Gyro_x PLL as reference

// --- Digital low-pass filter ---
#define MPU_DLPF_ADDR						0x1A	// Address of the register to configure the digital low-pass filter
													// (and disables the External Frame Synchronization)
#define MPU_DLPF_MASK						0x3F	// Mask for the DLPF configuration
#define MPU_DLPF_REG_VALUE_DEFAULT			0x00	// Default value for the digital low-pass filter configuration. Set to 260 Hz bandwidth
											 		// this is to get the full 8kHz oscillation frequency since the Gyro-Z PLL is being used
													// if not it will be 1kHz (which is the max sample rate for the accelerometer anyway)
#define MPU_DLPF_REG_VALUE_WORKING			0x02	// Value of the DLPF register for normal operation

// --- Interrupt ---
#define MPU_INTERRUPT_CONF_ADDR				0x38	// Address to the interrupt configuration register
#define MPU_INTERRUPT_CONF_MASK				0x19	// Mask for the interrupt configuration register
#define MPU_INTERRUPT_DEFAULT				0x01	// Default value for the Interrupt configuration register. This will set an interrupt when all
													// the measurements have been updated in the corresponding registers

// --- Sample rate ---
#define MPU_SAMPLE_RATE_ADDR				0x19	// Address of the register to configure the sample rate for the MPU
#define MPU_SAMPLE_RATE_DEFAULT				0x07	// Register value to set the sample rate, it will be:
													// Sample Rate = (PLL freq)/(MPU_SAMPLE_RATE_DEFAULT + 1)
																														// Default value sets it to 1kHz
#define MPU_SAMPLE_RATE_WORKING				0x1F	// Value of the sample rate register during normal operation. Set to 31.25Hz

// --- Signal path reset ---
#define MPU_RESET_SIGNAL_PATH_ADDR 			0x68	// Address for the signal path reset register
#define MPU_RESET_SIGNAL_PATH_MASK			0x07	// Mask for the signal path reset register
#define MPU_RESET_SIGNAL_PATH_RESET			0xFF 	// Value for to reset the signal path (it is assuming that the mask is correct -.-)
#define MPU_RESET_SIGNAL_PATH_DELAY			10		// Time in ms that the program will wait for the signal to be reset (It could be changed to receiving an interrupt)

// --- Low power mode ---
#define MPU_LOW_POWER_MODE_ADDR				0x6B	// Address for the power management register 1 in the MPU_GYRO_CONF_ADDR
#define MPU_LOW_POWER_MODE_MASK				0xE8	// Mask for the low power mode configuration
#define MPU_LOW_POWER_MODE_ENABLE			0x40	// Register value to set the MPU into sleep mode
#define MPU_LOW_POWER_MODE_DISABLE			0x00	// Register value to disable the sleep mode and wake the MPU


//--------------------------------------------------
// Calibration
//--------------------------------------------------

// --- Number of averages ---
#define CALIBRATION_AVERAGES           		10000	// The higher the better but it will be slower (int16_t value)
#define CALIBRATION_INITIAL_AVERAGES   		1000    // This value is to accelerate the offset location (int16_t value)
#define CALIBRATION_MAX_ITERATIONS     		100     // This is the limit of iterations to calibrate the MPU
#define CALIBRATION_INITIAL_ERROR	 		5		// Maximum error for which the number of calibrations should be increased to CALIBRATION_AVERAGES
#define CALIBRATION_MIN_ERROR         	 	1		// Maximum difference allowed between the high and low offset values to stop the calibration process

// --- Offset correction ---
#define CALIBRATION_CORRECTION_AVERAGES		1000	// Number of averages that will be made to calculate the offset correction array
#define FAST_CALIBRATION_CORRECTION					// Sets the calibration correction at 1kHz sample frequency if not commented

// --- Calibration adjustments ---
#define CALIBRATION_OFFSET_ADJUSTMENT 		1000	// Change done to the offset when finding the initial range

// --- Calibration targets ---

// -- Accelerometer --
// During calibration the accelerometer range is set to +-2g, the one 1g value
// is max value/2 as the scale is set to 2g.
#ifdef IMU_CAL_ORIENT_X
	#define X_ACCEL_TARGET		  		 	16384	// Target for the X accelerometer measurement
#elif IMU_CAL_ORIENT_Y
	#define Y_ACCEL_TARGET		  		 	16384	// Target for the Y accelerometer measurement
#elif IMU_CAL_ORIENT_Z
	#define Z_ACCEL_TARGET					16384	// Target for the Z accelerometer measurement
#endif

#ifndef IMU_CAL_ORIENT_X
	#define X_ACCEL_TARGET		  		 		0
#endif
#ifndef IMU_CAL_ORIENT_Y
	#define Y_ACCEL_TARGET		  		 		0
#endif
#ifndef IMU_CAL_ORIENT_Z
	#define Z_ACCEL_TARGET		  		 		0
#endif

// -- Gyroscope --
#define X_GYRO_TARGET						0		// Target for the X gyroscope measurement (0 by default)
#define Y_GYRO_TARGET						0		// Target for the Y gyroscope measurement (0 by default)
#define Z_GYRO_TARGET						0		// Target for the Z gyroscope measurement (0 by default)

// --- Temperature calibration threshold ---
#define CALIBRATION_MAX_TEMP_DIFF			1000	// Maximum temperature difference between the calibration temperature and the current one
													// It should be considered that the temperature stabilization won't be achieved when the
													// reading is done (temperature in Celsius * 100)

// --- Offsets registers ---
#define MPU_ACCEL_OFFSETS_BASE_ADDR			0x06	// Base register address for the accelerometer offset adjustment
#define MPU_GYRO_OFFSETS_BASE_ADDDR			0x13	// Base register address for the gyroscope offset adjustment


//--------------------------------------------------
// DATA
//--------------------------------------------------

// --- Temperature ---
#define MPU_TEMP_REG_BASE					0X41	// Base address of the registers with the temperature values

// --- Accelerometer ---
#define MPU_ACCEL_REG_BASE					0x3B	// Base address of the registers with the accelerometer values

// --- Gyroscope ---
#define MPU_GYRO_REG_BASE					0x43	// Base address of the registers with the gyroscope values

//--------------------------------------------------
// Memory
//--------------------------------------------------

#define MPU_MEMORY_CONTROL_VAL				0xDDAA	// Control value for the calibration parameters

#ifdef MEMORY_USE_FLASH

#define MPU_MEMORY_BASE_ADDR				0x0803F800	// Base address of the STM32 flash memory for the offsets
#define MPU_MEMORY_BASE_ADDR_TXT ".ARM.__at_0x0803F800"	// Base address of the STM32 flash memory for the offsets
// -- Initial values --
// The flash memory has two problems:
// 		1) To change a '1' to a '0' the complete flash page needs to be erased (2kB), but
//		   it can change a '0' for a '1' without any problem.
//		2) The flash memory is were the code is stored and every time the code is flashed,
//		   the complete flash memory is erased.
// There are two options for the calibration values:
//		1) Set the values to 0 so they may be written once the calibration has finished.
// 		   This can only be done once and the values will be lost if the code is re-written.
//		2) Hardcode the values so they aren't lost every time the micro is programmed.
// The two options are controlled with the "MPU_MEMORY_RESET_CAL" parameter.

//#define MPU_MEMORY_RESET_CAL						// Comment to set the flash offset values to 0

#ifndef MPU_MEMORY_RESET_CAL

	const int16_t offsets_flash[] __attribute__((section(".calibration"))) = {
			(int16_t) MPU_MEMORY_CONTROL_VAL,		// Control value to indicate there are values stored
			2822,									// Temperature value when the offsets have been taken
			-799, 									// X-accelerometer offset value
			-871,									// Y-accelerometer offset value
			1471,									// Z-accelerometer offset value
			127,									// X-gyro offset value
			20,										// Y-gyro offset value
			21										// Z-gyro offset value
	};

#else

	const int16_t offsets_flash[] __attribute__((section(".calibration"))) = {
			(int16_t) 0xFFFF,						// Control value to indicate there are values stored
			(int16_t) 0xFFFF,						// Temperature value when the offsets have been taken
			(int16_t) 0xFFFF, 						// X-accelerometer offset value
			(int16_t) 0xFFFF,						// Y-accelerometer offset value
			(int16_t) 0xFFFF,						// Z-accelerometer offset value
			(int16_t) 0xFFFF,						// X-gyro offset value
			(int16_t) 0xFFFF,						// Y-gyro offset value
			(int16_t) 0xFFFF						// Z-gyro offset value
	};

#endif

#else
	// TODO other memory
#endif

//--------------------------------------------------
// Communications
//--------------------------------------------------
#define MPU_CONFIGURE_COMMS


//            *********************
//            *    Definitions    *
//            *********************

//--------------------------------------------------
// I2C Bus
//--------------------------------------------------

// --- I2C Address ---

// By default the AD0 address has to be connected to GND
// The address is set as 7-bits and then corrected later on.
// It would be more efficient to work with the complete address directly
#ifdef MPU_AD0_ADDR_HIGH
	#define I2C_ADDRESS_MPU 				0b1101001  // I2C address with AD0 set to '1'
#else
	#define I2C_ADDRESS_MPU 				0b1101000  // I2C address with AD0 set to '0'
#endif

//            **************************
//            *    MPU CLASS OBJECT    *
//            **************************
class Mpu6050Dev{

	public:
		// --- Variables ---
		// -- MPU state --
		uint8_t imu_state;									// State of the MPU

		// --- Functions ---
		Mpu6050Dev();										// Constructor

		// -- configuration --
		bool initialize_1();								// First initialization, configures the MPU
		bool initialize_2();								// Second initialization, calibrates the MPU
		void resetSignalPath();								// Resets the signal path

		void setSampleRate();								// Sets the sample rate for the MPU
		void setLowPowerMode(bool sleep_enabled);			// Sets the MPU to low power mode or wakes it up

		// -- Calibration --
		void getOffsetCorrection();							// Gets the offset correction values

		// -- Measurements --
		double getTemperatureDouble();						// Gets the temperature as a double (°C)
		int16_t getTemperature();							// Gets the temperature as int16_t (°C * 100)
		void getParameter6(int16_t *values_funct);			// Gets the measurements from the MPU
		void getRefinedValues(double *measurements_funct);	// Gets the refined measurements from the MPU

	private:
		int16_t offset_correction[6];						// Offset correction array

		// --- Functions ---
		// -- Configuration --
		bool checkMpu();									// Checks that the MPU is not damaged
		void configureMpu();								// Configures the MPU for normal operation
		void changeFullScale(uint8_t accel_reg,
							 uint8_t gyro_reg);				// Sets the configuration registers for the accelerometer and gyroscope

		// -- Calibration --
		void setOffsets(int16_t *offsets); 					// Sets the offsets of the MPU
		void calculateAverages(int16_t *averages_funct,
							   uint16_t number_of_iterations,
							   int16_t *targets_funct);		// Calculates the averages with the given offset values

		bool calibrate(int16_t *offsets);					// Main function for the calibration process
		bool checkCalibration();							// Check if the calibration is needed or not
		bool performCalibration();							// Does the calibration

		// -- Memory --
		bool loadFromMemory(int16_t *temperature_mpu,
							int16_t *offsets_funct);		// Load the calibration data from the memory

		void saveOnMemory(int16_t *temperature_mpu,
						  int16_t *offsets_funct);			// Store the calibration data on the memory
};

/* ---- END OF FILE ---- */
