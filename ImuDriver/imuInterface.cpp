/*
 *                     ************************
 *                     *     imuInterface     *
 *                     ************************
 *
 * ============================================================================
 *
 *     File: 		imuInterface.cpp
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
 *  ----------------
 *  - imuInterface -
 *  ----------------
 *  This file contains the functions to interface with the IMU.
 *  This files is used to simplify the compatibility of the Arduino MPU 6050 library
 *  and to do the I2C/SPI selection independently from the device.
 *
 *  --------------
 *  - Change Log -
 *  --------------
 *  V0.0: 19 may. 2019 
 *		-)First issue.
 * 
 */ 


//            **************************
//            *        INCLUDES        *
//            **************************
#include <stdbool.h>
#include <stdint.h>

#include "imuConfiguration.h"
#include "imuConfig.h"
#include "i2c.h"

// Devices
#include "imuFake.h"
#include "mpu6050.h"	// MPU6050 and MPU6000

#include "definitions.h"
#include "time.h"

#ifdef IMU_INTERRUPT_DEBUG
	#include <stdio.h>
#endif

// -----------------------------------------------------------------------------------------
// -------------------------------    INTERRUPT    -----------------------------------------
// -----------------------------------------------------------------------------------------

volatile bool imu_overflow = false;  			// It will set to true if the measurements overflow
volatile bool imu_data_ready = false;   		// This variable is used as an interrupt
volatile uint32_t imu_time_buffer = 0;     		// Time stamp of the measurements
volatile uint32_t imu_timeout = 0;				// Counter for the interrupt timeout

//--------------------------------------------------
// Function
//--------------------------------------------------
// =====================================================
// imuInterrupt()
// Manages the external interrupts of the IMU.
// This IMU will be configured to set the interrupt every time new measurements are available.
// The function will log the time stamp of the interrupt and sets imu_data_ready as true. If
// imu_data_ready is true it means that the data has overflow and a LED will be turned on.
// ---
// Parameters:
//		None
// =====================================================
void imuInterrupt()
{
	// --- Time Stamp --
	// Needs to be done as soon as possible.

	//time_buffer = millis();		// Time captured in [ms]
	imu_time_buffer = microsIsr();  // Time captured in [us]

	// --- Overflow check ---
	if (imu_data_ready) {
		imu_overflow = true;
		// Turn LED on
		#ifdef IMU_USE_LED
			LEDS_GPIO_PORT->BSRR = IMU_LED;  // Turn IMU LED ON
		#endif
	}

	// Turn LED off
	#ifdef IMU_USE_LED
		if (!imu_data_ready) LEDS_GPIO_PORT->BRR = IMU_LED;  // Turn IMU LED OFF
	#endif

	// --- Data log ---
	imu_data_ready = true;

	// --- Reset Timeout ---
	imu_timeout = millis();
}


// =====================================================
// imuIntTimeout()
// Check the timeout of the interrupt.
// This is done through this function to have a consistent error function
// ---
// Parameters:
//		@return bool		--> (bool) If false, there is a timeout
// =====================================================
bool imuIntTimeout()
{

#ifdef IMU_FAKE

	// --- Fake target ---
	imu_data_ready = true;
	//delay(100);
	imu_time_buffer += 100;
	return true;

#else

	// --- Real target ---

	/*
	char debug_str[100];
	sprintf(debug_str, "Interrupt_count = %lu", imu_timeout);
	Serial.println(debug_str);
	*/

	if (millis() - imu_timeout > IMU_INTERRUPT_TIMEOUT) {
		// --- Timeout error ---
		#ifdef IMU_INTERRUPT_DEBUG
				Serial.println("IMU Interrupt Timeout");
		#endif

		// Turn LED on
		#ifdef IMU_USE_LED
			if (!imu_data_ready) LEDS_GPIO_PORT->BSRR = IMU_LED;  // Turn IMU LED On
		#endif

		imu.imu_state = IMU_INTERRUPT_ERROR;
		return false;
	}
	return true;

#endif
}


/*
// =====================================================
// imuIntTimeout()
// Check the timeout of the interrupt.
// This is done through this function to have a consistent error function
// ---
// Parameters:
//		@return bool		--> (bool) If false, there is a timeout
// =====================================================
bool imuIntTimeout()
{
#ifdef IMU_FAKE

	// --- Fake target ---
	imu_data_ready = true;
	//delay(100);
	time_buffer += 100;
	return true;

#else

	// --- Real target ---
	uint32_t buffer_time = time_buffer;
	if (micros() - buffer_time > (uint32_t)IMU_INTERRUPT_TIMEOUT) {

		// --- Timeout error ---
		#ifdef IMU_INTERRUPT_DEBUG
				Serial.println("IMU Interrupt Timeout");
		#endif
		// Turn LED on
		#ifdef IMU_USE_LED
			if (!imu_data_ready) LEDS_GPIO_PORT->BSRR = IMU_LED;  // Turn IMU LED On
		#endif

		imu.imu_state = IMU_INTERRUPT_ERROR;

		return false;
	}

	return true;

#endif
}
*/

// -----------------------------------------------------------------------------------------
// -------------------------------  COMMUNICATION  -----------------------------------------
// -----------------------------------------------------------------------------------------

//            **************************
//            *           I2C          *
//            **************************
#ifdef USE_IMU_I2C

//--------------------------------------------------
// Variables
//--------------------------------------------------
i2cDev imu_dev(&imu_i2c);		// IMU I2C object

//--------------------------------------------------
// Functions
//--------------------------------------------------

// =====================================================
// readImuRegisters()
// This function is kept for compatibility purposes.
// This function reads multiple consecutive registers of the IMU.
// ---
// Parameters:
//		@param reg_addr		--> (uint8_t) Address of the first register.
//		@param *p_data		--> (uint8_t*) Pointer to the data buffer.
//		@param data_size	--> (uint8_t) Number of registers to access.
//		@return bool		--> (bool) True if the communication is correct
// =====================================================
bool readImuRegisters(uint8_t reg_addr, uint8_t *p_data, uint8_t data_size)
{
	return imu_dev.readRegisters(IMU_I2C_ADDR, reg_addr, p_data, data_size);
}


// =====================================================
// readImuRegister()
// This function is kept for compatibility purposes.
// This function reads one register of the MPU.
// ---
// Parameters:
//		@param reg_addr		--> (uint8_t) Address of the register.
//		@param *p_data		--> (uint8_t*) Pointer to the data buffer.
//		@return bool		--> (bool) True if the communication is correct
// =====================================================
bool readImuRegister(uint8_t reg_addr, uint8_t *p_data)
{
	return imu_dev.readRegister(IMU_I2C_ADDR, reg_addr, p_data);
}


// =====================================================
// readImuData()
// This function is kept for compatibility purposes.
// This function reads a 16-bit value from the IMU.
// The goal of this function is to access a single gyroscope, accelerometer or temperature measurement.
// This will only work if the data is in consecutive registers with the high byte (most significant)
// in the first register.
// ---
// Parameters:
//		@param reg_addr		--> (uint8_t) Address of the first register (high-byte)
// 		@param *p_data		--> (int16_t*) Pointer for the data value.
//		@return bool		--> (bool) True if the communication has been successful.
// =====================================================
bool readImuData(uint8_t reg_addr, int16_t *p_data)
{

	uint8_t buffer_funct[2];         // Buffer for the raw data from the registers
	bool communication_successful;   // Temp value for the communication state

	// --- Read data form the device ---
	communication_successful = readImuRegisters(reg_addr, buffer_funct, 2);

	// --- Convert data ---
	*p_data = (buffer_funct[0] << 8) | (buffer_funct[1]);

	return communication_successful;
}

// =====================================================
// readImuMeasurements()
// Reads the gyroscope and accelerometer measurements of the IMU.
// At the moment this in only compatible with the MPU6050.
// ---
// Parameters:
//		@param p_data		--> (int16_t*) Pointer to the measurements array (6 in total)
//		@return bool 		--> (bool) True if the communication has been successful.
// =====================================================
bool readImuMeasurements(int16_t *p_data)
{

	uint8_t buffer_funct[14];         // Buffer for the raw data from the registers
	bool communication_successful;   // Temp value for the communication state

	// --- Read data form the device ---
	communication_successful = readImuRegisters(IMU_MEAS_REG_BASE, buffer_funct, 14);

	// --- Convert data ---
	p_data[0] = (buffer_funct[ 0] << 8) | (buffer_funct[ 1]);
	p_data[1] = (buffer_funct[ 2] << 8) | (buffer_funct[ 3]);
	p_data[2] = (buffer_funct[ 4] << 8) | (buffer_funct[ 5]);
	p_data[3] = (buffer_funct[ 8] << 8) | (buffer_funct[ 9]);
	p_data[4] = (buffer_funct[10] << 8) | (buffer_funct[11]);
	p_data[5] = (buffer_funct[12] << 8) | (buffer_funct[13]);

	return communication_successful;
}


// =====================================================
// writeImuRegister()
// This function is kept for compatibility purposes.
// This function writes one register of the IMU.
// ---
// Parameters:
//		@param reg_addr		--> (uint8_t) Address of the register.
//		@param value		--> (uint8_t) Register data.
//		@param check_data	--> (bool) If true the data is check one it is written (set to true by default)
//		@return bool		--> (bool) True if the communication is correct
// =====================================================
bool writeImuRegister(uint8_t reg_addr, uint8_t value, bool check_data /*= true*/)
{
	return imu_dev.writeRegister(IMU_I2C_ADDR, reg_addr, &value, check_data);
}

// =====================================================
// updateImuRegister()
// This function is kept for compatibility purposes.
// This function writes only the selected bits of the register.
// ---
// Parameters:
//		@param reg_addr		--> (uint8_t) Register address.
//		@param value		--> (uint8_t) New register value (only the bits set by the mask will be written).
//		@param mask_funct	--> (uint8_t) Only the bits set to '1' will be changed.
//		@param check_data	--> (bool) If set to true, the data will be checked.
//		@return bool		--> (bool) true if the communication is correct.
// =====================================================
bool updateImuRegister(uint8_t reg_addr, uint8_t value, uint8_t mask_funct, bool check_data /* = true */)
{
	return imu_dev.updateRegister(IMU_I2C_ADDR, reg_addr, value, mask_funct, check_data);
}

#endif


//            **************************
//            *           SPI          *
//            **************************
#ifdef USE_IMU_SPI
	// TODO implement SPI
#endif


// -----------------------------------------------------------------------------------------
// -----------------------------  DEVICE SELECTION  ----------------------------------------
// -----------------------------------------------------------------------------------------
#if defined(IMU_FAKE)
	ImuFake imu;
#elif defined(IMU_USE_MPU6050) || defined(IMU_USE_MPU6000)
	Mpu6050Dev imu;			// IMU object
#elif defined(IMU_USE_MPU6500) || defined(IMU_USE_MPU9250)
	// TODO create libraty for more sensors
#endif


/* ---- END OF FILE ---- */
