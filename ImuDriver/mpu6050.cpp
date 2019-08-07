/*
 *                     *******************
 *                     *     mpu6050     *
 *                     *******************
 *
 * ============================================================================
 *
 *     File: 		mpu6050.cpp
 *   Author:		David Arnaiz
 *     Date:		22 may. 2019	
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
 *  ----------------
 *  - mpu6050 -
 *  ----------------
 *  This file contains the STM32 compatible code for the MPU6050 device.
 * 
 *  -------------------
 *  - Acknowledgments -
 *  -------------------
 *  This library was inspired by the MPU6050 library implemented by "jrowberg" which is available on Github:
 *  			https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 *
 *  The self-test procedure was based on the one proposed by "kriswiner" which is also available on Github:
 *  			https://github.com/kriswiner/MPU6050/blob/master/MPU6050BasicExample.ino
 *
 *  --------------
 *  - Change Log -
 *  --------------
 *  V0.0: 22 may. 2019 
 *		-)First issue.
 * 
 */ 


//            **************************
//            *        INCLUDES        *
//            **************************
#include <stdbool.h>
#include <stdint.h>

#include <math.h>

#include "mpu6050.h"

#include "imuInterface.h"
#include "time.h"
#include "serialDebug.h"
#include "imuConfig.h"
#include "memory.h"

#ifdef DEBUG_MODE_MPU
	#include <stdio.h>
#endif


// -----------------------------------------------------------------------------------------
// -------------------------------  MPU6050 ClASS  -----------------------------------------
// -----------------------------------------------------------------------------------------

//            **************************
//            *     CONFIGURATION      *
//            **************************

// =====================================================
// Mpu6050Dev()
// Class initializer.
// ---
// Parameters:
//		None
// =====================================================
Mpu6050Dev::Mpu6050Dev()
{
	imu_state = IMU_NOT_INITIALIZED;
}

// The configuration is divided in two parts. This is required as the calibration should be done
// if the thermal stabilization of the device has been achieved (~5 minutes since it has been enabled)
//		+) initialize_1(): 	Enables the device and sets the sampling rate. starts the device so it
//							starts achieving thermal stabilization.
//		+) initialize2(): 	Calibrates the MPU. Full calibration will only be done if no calibration
//							data is found.


// =====================================================
// initialize1()
// This function starts the MPU initialization.
// It will initialize the communication, wake up the device, perform the self-test, configure
// the MPU and (TODO) load the data from memory if it is available.
// ---
// Parameters:
//		@return bool		--> (bool) Status of the MPU (true = correct, false = error).
// =====================================================
bool Mpu6050Dev::initialize_1()
{
	//--------------------------------------------------
	// System initialization
	//--------------------------------------------------
	// This is only needed if it is not being done elsewhere

	// -- Start Communication --
	#ifdef MPU_CONFIGURE_COMMS
		// Initialize the communication interface for the MPU
		imuConfig();
		imuCommStart();
	#endif

	// --- State serial debug mode ---
	#ifdef MPU_CONFIGURE_SERIAL  //------------------------------------------------------------------------- TBD
		//initialize serial interface
		Serial.begin(SERIAL_SPEED);
		Serial.println("Serial debug configured :)");
	#endif

	#ifdef DEBUG_MODE_MPU
		Serial.println("MPU Initialized...");
	#endif

	//--------------------------------------------------
	// Device Communication
	//--------------------------------------------------

	// --- Check communication ---
	// The communication is checked by reading the MPU ID

	uint8_t buffer_funct; // This is to hold the register value

	// -- Read ID --
	if (!readImuRegister(MPU_DEVICE_ID_REG, &buffer_funct)){

		// debug mode
		#ifdef DEBUG_MODE_MPU
			Serial.println("IMU communication failed.....");
		#endif

		imu_state = IMU_COMMS_ERROR;  // Set the error state
		return false;
	}

	// -- Check ID --
	if ((buffer_funct & 0x7E) != MPU_DEVICE_ID_VALUE) {

		// debug mode
		#ifdef DEBUG_MODE_MPU
			Serial.println("MPU ID incorrect -_-");
		#endif

		imu_state = IMU_COMMS_ERROR;  // Set the error state
		return false;
	}


	//--------------------------------------------------
	// Start MPU
	//--------------------------------------------------

	// --- Wakeup device ---
	setLowPowerMode(false);

	// --- Set the clock ---
	// The clock reference will be set the the gyro-Z
	updateImuRegister(MPU_CLOCK_REF_ADDR, MPU_CLOCK_ZGYRO, MPU_CLOCK_REF_MASK);

	//--------------------------------------------------
	// Self-test
	//--------------------------------------------------

	if (!checkMpu()){

		// Debug
		#ifdef DEBUG_MODE_MPU
			Serial.println("Self-test failed :(");
		#endif

		return false;
	}

	//--------------------------------------------------
	// Configure MPU
	//--------------------------------------------------

	configureMpu();
	if (imu_state == IMU_COMMS_ERROR) return false;

	//--------------------------------------------------
	// Check calibration
	//--------------------------------------------------
	// Checks if there is useful calibration data in the memory

	if (!checkCalibration()) {
		if (imu_state == IMU_COMMS_ERROR) return false;
	}

	// Debug
	#ifdef DEBUG_MODE_MPU
		Serial.println("Device Initialized successfully");
	#endif

	return true;
}


// =====================================================
// initialize2()
// This function completes the MPU initialization.
// This function makes the calibration if needed and calculates the offset_correction values.
// The MPU should have achieved the thermal stabilization state before calling this function.
// ---
// Parameters:
//		@return bool		--> (bool) state of the initialization, true = success.
// =====================================================
bool Mpu6050Dev::initialize_2()
{
	bool correct_funct = true;


	// --- Calibrate the MPU ---

	if (imu_state == IMU_NOT_CALIBRATED) {   // Check is calibration is required

		// -- Configure MPU for calibration --
		changeFullScale(MPU_DEFAULT_ACCEL_REG_VALUE, MPU_DEFAULT_GYRO_REG_VALUE);            // Sets the full-scale to the most sensitive one
		updateImuRegister(MPU_DLPF_ADDR, MPU_DLPF_REG_VALUE_DEFAULT, MPU_DLPF_MASK);         // Set the low pass filter. Check .h for more info
		writeImuRegister(MPU_SAMPLE_RATE_ADDR, MPU_SAMPLE_RATE_DEFAULT);   					 // Sets the sample rate. Check .h for more info
		resetSignalPath();                                                                   // Resets the signal path of the MPU
		if (imu_state == IMU_COMMS_ERROR) return false;

		// -- Calibrate the MPU --
		correct_funct = performCalibration();
		if (!correct_funct) return false;

		// -- Configure MPU --
		configureMpu();
	}


	// --- Get offset correction ---

	// -- Set the sampling rate --
	#ifdef FAST_CALIBRATION_CORRECTION
		updateImuRegister(MPU_DLPF_ADDR, MPU_DLPF_REG_VALUE_DEFAULT, MPU_DLPF_MASK);         // Set the low pass filter. Check .h for more info
																						 	 // (needs to be changed also to avoid issues)
		writeImuRegister(MPU_SAMPLE_RATE_ADDR, MPU_SAMPLE_RATE_DEFAULT);   					 // Sets the sample rate. Check .h for more info
	#endif

	// -- Reset the signal path --
	resetSignalPath();
	if (imu_state == IMU_COMMS_ERROR) return false;

	// -- Get values --
	getOffsetCorrection();

	// -- Re-configure the system --
	#ifdef FAST_CALIBRATION_CORRECTION
		configureMpu();
		// -- Reset the signal path --
		resetSignalPath();
	#endif

	// -- Check --
	if (imu_state != IMU_NOT_INITIALIZED) {

		#ifdef DEBUG_MODE_MPU
			Serial.println("The MPU couldn't be initialized correctly. ´:(");
		#endif

		return false;
	}

	// --- MPU Initialized correctly :p ---

	// Debug
	#ifdef DEBUG_MODE_MPU
		Serial.println("________ MPU CONFIGURED CORRECTLY ________");
	#endif

	imu_state = IMU_CORRECT;
	return true;
}


// =====================================================
// checkMpu()
// Checks if the gyro or the accelerometer are damaged.
// This procedure is inspired by:
// 			"https://github.com/kriswiner/MPU6050/blob/master/MPU6050BasicExample.ino".
// If this fails it means that the device is damaged and should be replaced :(
// ---
// Parameters:
//		@return bool		--> (bool) Status of the MPU (true = correct, false = error).
// =====================================================
bool Mpu6050Dev::checkMpu()
{

	// Variables
	uint8_t values_raw[3];		// Register values with the self-test results
	uint8_t self_test;      	// Self-test value (they are 5 bit unsigned integers)
	double factory_trimmed;   	// factory_trimmed value
	//float result;           	// Result of the test

	// Debug
	#ifdef DEBUG_MODE_MPU
		char debug_str[100];  // Used for the sprintf function
		Serial.println("::Self-Test::");
	#endif

	// --- Set the self-test of the MPU ---
	// This sets the self-test bits on the accel to '1' and the full-scale range to +-8g
	// This sets the self-test bits on the gyro to '1' and the full-scale range to 250dps
	changeFullScale(MPU_SELF_TEST_ACCEL_REG_VALUE, MPU_SELF_TEST_GYRO_REG_VALUE);

	// --- Wait for the self-test to be completed ---
	delay(MPU_SELF_TEST_WAIT_TIME);

	// --- Get the values ---
	if (!readImuRegisters(MPU_SELF_TEST_RESULT_ADDR_BASE, values_raw, 3)) {
		imu_state = IMU_COMMS_ERROR;
		return false;
	}

	// --- Restore the full-scale range ---
	// This is done now since the function will end if any error is detected
	// (No need to do this anymore, it will be done later :)
	// changeFullScale(MPU_DEFAULT_ACCEL_REG_VALUE, MPU_DEFAULT_GYRO_REG_VALUE);

	// --- Check accelerometer ---

	for (uint8_t i = 0; i < 3; i++) {

		// -- Get the self-test result --
		self_test = (values_raw[i] >> 3) | (values_raw[3] & (0x30 >> (2 * i)));

		// Debug
		#ifdef DEBUG_MODE_MPU
			sprintf(debug_str, "%u --> ", self_test);
			Serial.print(debug_str);
		#endif

		// -- Get the factory trimmed value
		factory_trimmed = (1392.64)*(pow( (0.92/0.34) , (((double)self_test - 1.0)/30.0)));

		// Debug
		#ifdef DEBUG_MODE_MPU
			sprintf(debug_str, "%g --> ", factory_trimmed);
			Serial.print(debug_str);
		#endif

		// -- Check self-test --
		factory_trimmed = 100 + 100 * ((double)self_test - factory_trimmed)/factory_trimmed;  // Calculate percentage
		factory_trimmed = abs(factory_trimmed);

		// Debug
		#ifdef DEBUG_MODE_MPU
			sprintf(debug_str, "%f%%\n\r", factory_trimmed);
			Serial.print(debug_str);
		#endif

		if (factory_trimmed > MPU_SELF_TEST_THRESHOLD) {

			// Debug
			#ifdef DEBUG_MODE_MPU
				Serial.print("Accelerometer damaged: ");
				if (i == 0) Serial.println("A_X");
				else if ( i == 2) Serial.println("A_Y");
				else Serial.println("A_Z");
			#endif

			imu_state = IMU_SELF_TEST_FAILED_BASE + i;  // Set the error state
			return false;
		}
	}

	// --- Check gyroscope ---

	for (uint8_t i = 0; i < 3; i++) {
		// -- Get the self-test result --
		self_test = values_raw[i] & 0x1F;  // Get the first 5 bits --> it is the 5 bit unsigned int result

		// Debug
		#ifdef DEBUG_MODE_MPU
			sprintf(debug_str, "%u --> ", self_test);
			Serial.print(debug_str);
		#endif

		// -- Get the factory trimmed value
		factory_trimmed = (3275.0)*(pow( 1.046 , ((double)self_test - 1.0) ));
		if (i == 1) factory_trimmed *= -1;

		// Debug
		#ifdef DEBUG_MODE_MPU
			sprintf(debug_str, "%E --> ", factory_trimmed);
			Serial.print(debug_str);
		#endif

		// -- Check self-test --
		factory_trimmed = 100 + 100 * ((double)self_test - factory_trimmed)/factory_trimmed;  // Calculate percentage
		factory_trimmed = abs(factory_trimmed);

		// Debug
		#ifdef DEBUG_MODE_MPU
			sprintf(debug_str, "%f%%\n\r", factory_trimmed);
			Serial.print(debug_str);
		#endif

		if (factory_trimmed > MPU_SELF_TEST_THRESHOLD) {

			// Debug
			#ifdef DEBUG_MODE_MPU
			Serial.print("Gyroscope damaged: ");
				if (i == 0)	Serial.println("G_X");
				else if ( i == 2) Serial.println("G_Y");
				else Serial.println("G_Z");
			#endif

			imu_state = IMU_SELF_TEST_FAILED_BASE + 3 + i;  // Set the error state
			return false;
		}
	}

	// Debug
	#ifdef DEBUG_MODE_MPU
		Serial.println("Self-Test successful");
	#endif

	return true;
}


// =====================================================
// configureMpu()
// This function updates the registers of the MPU to set the:
//		-) Full-scale values for the gyro and accel.
//      -) Digital low-pass filter.
//		-) Interrupt output.
//		-) Sample rate.
// Then resets the signal path to avoid any issues with the data.
// ---
// Parameters:
//		None
// =====================================================
void Mpu6050Dev::configureMpu()
{

	// --- Configure the full-scale ---
	changeFullScale(MPU_ACCEL_CONFIG_VALUE, MPU_GYRO_CONFIG_VALUE);  // Sets the full-scale to the working one (It will be changed for calibration)

	// --- Set the low pass filter ---
	// The digital high-pass filter will limit the maximum sample frequency, check .h and the documentation of the MPU for more info. ----------------------------------------------------------- Needs adjustment
	updateImuRegister(MPU_DLPF_ADDR, MPU_DLPF_REG_VALUE_WORKING, MPU_DLPF_MASK); // Set the low pass filter. Check .h for more info

	// --- Configure the interrupt ---
	// The interrupt will be set as DATA_RDY, so it will be set as high when all the measurement registers are updated (including temp, turn off?)
	updateImuRegister(MPU_INTERRUPT_CONF_ADDR, MPU_INTERRUPT_DEFAULT, MPU_INTERRUPT_CONF_MASK);  // Check .h for more info

	// --- Set the sample rate ---
	// The maximum sample rate is 1kHz because of the accelerometer (it could go higher, but the accelerometer measurements will be repeated)
	// The sample rate will be set by MPU_SAMPLE_RATE, so the sample rate will be:
	//                          Sample rate = (Oscillation frequency)/(MPU_SAMPLE_RATE_DAFAULT + 1)
	writeImuRegister(MPU_SAMPLE_RATE_ADDR, MPU_SAMPLE_RATE_WORKING);   // Check .h for more info

	//(Moved)
	//resetSignalPath();
}


// =====================================================
// resetSignalPath()
// Resets the signal path of the MPU.
// This is used to prevent having any problem when the configuration is changed
// ---
// Parameters:
//		None
// =====================================================
void Mpu6050Dev::resetSignalPath()
{
  // --- Reset signal path ---
  updateImuRegister(MPU_RESET_SIGNAL_PATH_ADDR, MPU_RESET_SIGNAL_PATH_RESET, MPU_RESET_SIGNAL_PATH_MASK, false);

  delay(MPU_RESET_SIGNAL_PATH_DELAY);  // wait
}


// =====================================================
// changeFullScale()
// This function configures the gyroscope and the accelerometer registers.
// These registers are used to set:
// 		+) The self-test (three most significant bits).
//		+) Set the full-scale range (bit 4 and 3).
// The rest of the bits should be set as '0' (they are really undefined).
// The values of the full-scale range are:
//
//      Accelerometer		       Gyroscope               input
//      ------------               ---------               ------
//       0 --> +-2g              0 --> +-250dps             0x00
//       1 --> +-4g              1 --> +-500dps             0x08
//       2 --> +-8g              2 --> +-1000dps            0x10
//		 3 --> +-16g             3 --> +-2000dps  	    	0x18
// ---
// Parameters:
//		@param accel_reg      --> (uint8_t) Desired value for the accelerometer configuration register.
//		@param gyro_reg       --> (uint8_t) Desired value for the gyroscope configuration register.
// =====================================================
void Mpu6050Dev::changeFullScale(uint8_t accel_reg, uint8_t gyro_reg)
{
	// --- Set the accelerometer ---
	updateImuRegister(MPU_ACCELEROMETER_CONF_ADDR, accel_reg, MPU_ACCEL_CONFIG_MASK_VALUE);

	// --- Set the gyroscope ---
	updateImuRegister(MPU_GYRO_CONF_ADDR, gyro_reg, MPU_GYRO_CONFIG_MASK_VALUE);
}


// =====================================================
// setLowPowerMode()
// This function sets the MPU in low power mode.
// It will only set the MPU into sleep mode. It can be configured to wake up at a certain
// rate to get accelerometer values or to disable one of the accelerometers or gyroscopes to save energy.
// ---
// Parameters:
//		@param sleep_enabled      --> (bool) true = set sleep mode
// =====================================================
void Mpu6050Dev::setLowPowerMode(bool sleep_enabled)
{
	// --- Set the register ---
	if (sleep_enabled)
		updateImuRegister(MPU_LOW_POWER_MODE_ADDR, MPU_LOW_POWER_MODE_ENABLE, MPU_LOW_POWER_MODE_MASK);
	else
		updateImuRegister(MPU_LOW_POWER_MODE_ADDR, MPU_LOW_POWER_MODE_DISABLE, MPU_LOW_POWER_MODE_MASK);
}



//            **************************
//            *      CALIBRATION       *
//            **************************

// =====================================================
// setOffsets()
// Write the values in the offset array to the MPU offset registers
// ---
// Parameters:
//		@param *offsets_funct2		--> (int16_t*) Pointer to the offset array.
// =====================================================
void Mpu6050Dev::setOffsets(int16_t *offsets_funct2)
{
	uint8_t buffer_funct[2];                                // This array will hold the two bytes that form the int16_t
	uint8_t address_funct = MPU_ACCEL_OFFSETS_BASE_ADDR;    // Set the address to the accelerometer first

	for (uint8_t i = 0; i < 6; i++) {

		// --- Split the data into bytes ---
		*(int16_t *) buffer_funct = *(offsets_funct2 + i);

		// --- Change address ---
		// Change the register base address once the accelerometers are done
		if (i == 3) address_funct = MPU_GYRO_OFFSETS_BASE_ADDDR;

		// --- Write the register ---
		writeImuRegister(address_funct, buffer_funct[1]);
		address_funct++;
		writeImuRegister(address_funct, buffer_funct[0]);
		address_funct++;
	}
}


// =====================================================
// calculateAverages()
// Calculate the measurement averages.
// The averages will be written in the averages array, so it will overwrote the previous ones.
// The target value will be subtracted to the average as only the error is needed for the calibration.
// ---
// Parameters:
//		@param *averages_funct		--> (int16_t*) Pointer to the averages array.
// 		@param number_of_iterations	--> (uint16_t) Number of samples used to calculate the averages.
//		@param *targets_funct		--> (int16_t*) Pointer to the target array (expected measurements)
// =====================================================
void Mpu6050Dev::calculateAverages(int16_t *averages_funct, uint16_t number_of_iterations,
							   int16_t *targets_funct)
{
	int32_t sum_average[6] = {0, 0, 0, 0, 0, 0};  // 32-bits to avoid overflow :(

	// Zero the average values
	for (uint8_t index = 0; index < 6; index++) *(averages_funct + index) = 0;  // Maybe is better just to create a local new array as zero? :p

	// -------------------------------
	// --- Obtain the measurements ---
	// -------------------------------
	for (int16_t index = 0; index < number_of_iterations; index++) {

		int16_t values_raw[6];  // Values read from the device

		// Interrupt timeout
		while (!imu_data_ready) {
			if(imuIntTimeout()) {
				return;
			}
		}

		imu_data_ready = false;

		// -- Measure --
		getParameter6(values_raw);

		// -- Reset --
		imu_data_ready = false;

		// -- Sum --
		for (uint8_t index = 0; index < 6; index++) sum_average[index] += values_raw[index];
	}

	// ------------------------------
	// --- Calculate the averages ---
	// ------------------------------
	for (uint8_t index = 0; index < 6; index++) {
		*(averages_funct + index) = (sum_average[index] / number_of_iterations) - *(targets_funct + index);
	}

	/*
	// Debug
	#ifdef DEBUG_MODE_MPU
		char debug_str[100];  // Used for the sprintf function
		sprintf(debug_str, "averages : %d, %d, %d, %d, %d, %d",
				*(averages_funct + 0),
				*(averages_funct + 1),
				*(averages_funct + 2),
				*(averages_funct + 3),
				*(averages_funct + 4),
				*(averages_funct + 5));
		Serial.println(debug_str);
	#endif
	*/
}


// =====================================================
// calibrate()
// This is the main function to calibrate the MPU. The calibration process goes as follows:
//	1)  The fist step is to find an offset value that produces a negative average (low offset) and
//  	other value that produces a positive one (high offset). This values will be the initial range
// 		for the approximation process.
//	2)  The next step is to follow a successive approximation process to decrease the offset
//  	range found during (1). This is done by calculating the average offset between the low
//		and high offsets and evaluating the average value. If positive --> that offset will be
//		the new high offset, or if negative, the value will be the low offset. This will be repeated
// 		until the maximum number of iterations is done or the range is closed.
//	3)  The offsets register values (values which will be written to the registers) and the external
//		offset values (error even after the offset register has been adjusted) are returned.
// ---
// Parameters:
//		@param *offsets_funct	--> (int16_t*) Pointer to the offset array (it will be modified)
//		@return bool			--> (bool) Calibration status (true = success)
// =====================================================
bool Mpu6050Dev::calibrate(int16_t *offsets_funct)
{
	// Variables
	int16_t low_offsets[6], high_offsets[6];       // These arrays store the low and high offset values respectively
	int16_t low_values[6], high_values[6];
	int16_t averages[6];                           // Average array
	int16_t targets[] = {X_ACCEL_TARGET, Y_ACCEL_TARGET, Z_ACCEL_TARGET,
			   	   	   	 X_GYRO_TARGET, Y_GYRO_TARGET, Z_GYRO_TARGET};   // Target array, stores the expected values
	uint16_t count = 0;                            // This is to count the number of iterations
	bool use_max_iterations = false;               // This is to set the maximum number of iterations for the average
	int16_t max_difference = 0;

	// Debug
	#ifdef DEBUG_MODE_MPU
		Serial.println("MPU Calibration initialized :)");
		char debug_str[100];  // Used for the sprintf function
	#endif

	#ifdef IMU_USE_LED_CAL
		LEDS_GPIO_PORT->BSRR = IMU_LED_CAL;  	// Turn IMU calibration LED ON to show the calibration process is ongoing
	#endif

	//--------------------------------------------------
	// Initialize
	//--------------------------------------------------
	for (uint8_t index = 0; index < 6; index++) {
		low_offsets[index]  = offsets_funct[index];
		high_offsets[index] = offsets_funct[index];
	}

	bool done = false;  // This is to know if the process has been completed

	//--------------------------------------------------
	// Locate initial range
	//--------------------------------------------------

	// Debug
	#ifdef DEBUG_MODE_MPU
		Serial.println("Locating the initial range...");
	#endif

	// Loop
	while (!done) {
		done = true;  // Set the process as done, then set it as not done if not

		// -- Locate the low offset value --
		setOffsets(low_offsets);                                            // Set the offsets
		calculateAverages(averages, CALIBRATION_INITIAL_AVERAGES, targets); // Calculate averages

		// Check that there aren't any issues
		if (imu_state == IMU_COMMS_ERROR || imu_state == IMU_INTERRUPT_ERROR) return false;

		for (uint8_t index = 0; index < 6; index++) {
			low_values[index] = averages[index];
			if (averages[index] >= 0) {
				done = false;
				low_offsets[index] -= CALIBRATION_OFFSET_ADJUSTMENT;
			}
		}

		// -- Locate the high offset value --
		setOffsets(high_offsets);                                           // Set the offsets
		calculateAverages(averages, CALIBRATION_INITIAL_AVERAGES, targets); // Calculate averages

		// Check that there aren't any issues
		if (imu_state == IMU_COMMS_ERROR || imu_state == IMU_INTERRUPT_ERROR) return false;

		for (uint8_t index = 0; index < 6; index++) {
			high_values[index] = averages[index];
			if (averages[index] <= 0) {
				done = false;
				high_offsets[index] += CALIBRATION_OFFSET_ADJUSTMENT;
			}
		}

		// Check if it is done or not
		if (count++ > CALIBRATION_MAX_ITERATIONS) done = true;  // stop for over iterations

		// Debug
		#ifdef DEBUG_MODE_MPU
			sprintf(debug_str, "Iterations: %u", count);
			Serial.println(debug_str);
			for (uint8_t debug_i = 0; debug_i < 6; debug_i ++) {
				sprintf(debug_str, "[%d, %d] --> [%d, %d]", low_offsets[debug_i], high_offsets[debug_i],
															low_values[debug_i], high_values[debug_i]);
				Serial.println(debug_str);
			}
		#endif
	}  // Range found

	// Debug
	#ifdef DEBUG_MODE_MPU
		sprintf(debug_str, "Done locating the initial range :p. It took %u iterations", count);
		Serial.println(debug_str);
		Serial.println("Reducing the range....");
	#endif

	if (count < CALIBRATION_MAX_ITERATIONS) done = false;

	// -- calculate new offsets --
	for(uint8_t index = 0; index < 6; index++) {
		// Set the new offset to the middle point
		*(offsets_funct + index) = (low_offsets[index] + high_offsets[index])/2;
	}

	//--------------------------------------------------
	// Decrease the initial range
	//--------------------------------------------------
	while (!done) {

		max_difference = 0;   // This is to store the maximum difference between the high and low offsets of all the components
		done = true;

		// -- Average with the new offset values --
		setOffsets(offsets_funct);
		if (use_max_iterations) calculateAverages(averages, CALIBRATION_AVERAGES, targets);
		else calculateAverages(averages, CALIBRATION_INITIAL_AVERAGES, targets);

		// Loop all the components
		for (uint8_t index = 0; index < 6; index++) {

			int16_t difference;  // This is to hold the current difference between the high and low offset for each components

			// -- Check if the new offsets are low or high --
			if (averages[index] <= 0) {  // Low offset
				low_offsets[index] = *(offsets_funct + index);
				low_values[index] = averages[index];
			} else {                      // High offset
				high_offsets[index] = *(offsets_funct + index);
				high_values[index] = averages[index];
			}

			// -- Evaluate --
			difference = high_offsets[index] - low_offsets[index];
			if (max_difference < difference) max_difference = difference;

			// -- Set the new offsets --
			*(offsets_funct + index) = (low_offsets[index] + high_offsets[index])/2;  // Set the new offset to the middle point for the next iteration
		}

		// Check the progress
		if (max_difference > CALIBRATION_MIN_ERROR) done = false;                     // Check if the calibration is done
		if (max_difference <= CALIBRATION_INITIAL_ERROR) use_max_iterations = true;   // Change the iterations value
		if (count++ > CALIBRATION_MAX_ITERATIONS) done = true;                        // stop for over iterations
		// Check that there aren't any issues
		if (imu_state == IMU_COMMS_ERROR || imu_state == IMU_INTERRUPT_ERROR) return false;

		// Debug
		#ifdef DEBUG_MODE_MPU
			sprintf(debug_str, "Iterations: %u", count);
			Serial.println(debug_str);
			sprintf(debug_str, "Maximum Difference: %d", max_difference);
			Serial.println(debug_str);

			for (uint8_t debug_i = 0; debug_i < 6; debug_i ++) {
				sprintf(debug_str, "[%d, %d] --> [%d, %d]", low_offsets[debug_i], high_offsets[debug_i],
															low_values[debug_i], high_values[debug_i]);
				Serial.println(debug_str);
			}
		#endif
	}

	//--------------------------------------------------
	// Done
	//--------------------------------------------------

	// -- Select the best results --
	// The best will be the ones with the lowest absolute value average
	for (uint8_t index = 0; index < 6; index++) {
		if ((-1 * low_values[index]) <= high_values[index]) {
			*(offsets_funct + index) = low_offsets[index];
			// *(offset_correction + index) = low_values[index];
		} else {
			*(offsets_funct + index) = high_offsets[index];
			// *(offset_correction + index) = high_values[index];
		}
	}

	// Debug
	#ifdef DEBUG_MODE_MPU
		sprintf(debug_str, "Done locating the offsets XD\n\rNumber of iterations: %u", count);
		Serial.println(debug_str);

		sprintf(debug_str, "%d, %d, %d,\n\r%d, %d, %d",
				offsets_funct[0], offsets_funct[1], offsets_funct[2],
				offsets_funct[3], offsets_funct[4], offsets_funct[5]);
		Serial.println(debug_str);

		/*
		sprintf(debug_str, "%d, %d, %d\n\r", offset_correction[0], offset_correction[1],
											 offset_correction[2], offset_correction[3],
											 offset_correction[4], offset_correction[5]);
		Serial.println(debug_str);
		*/
	#endif

	// The calibration is done, return true if correct and false is it stopped for over iterations
	if (count > CALIBRATION_MAX_ITERATIONS) {
		imu_state = IMU_CALIBRATION_ERROR;
		return false;
	}

	#ifdef IMU_USE_LED_CAL
		LEDS_GPIO_PORT->BRR = IMU_LED_CAL;  // Turn IMU calibration LED OFF to show the calibration process is done
	#endif

	return true;
}


// =====================================================
// getOffsetCorrection()
// This function obtains the offset correction values.
// The offset correction values are used for two purposes:
//		+) Eliminate the offset that exists once the offset registers have been adjusted.
//		+) Compensate the small temperature differences when the calibration data is loaded from memory.
// This function should only be called once the thermal stabilization has been reached.
// ---
// Parameters:
//		None
// =====================================================
void Mpu6050Dev::getOffsetCorrection()
{
	// Target array, stores the expected values. It needs to be modified to match the
	// full-scale values. The modification is done assuming all the targets are zero
	// except the 1g gravity component, which is aligned with one of the axis.
	int16_t targets_funct[] = {X_ACCEL_TARGET, Y_ACCEL_TARGET, Z_ACCEL_TARGET,
							   X_GYRO_TARGET, Y_GYRO_TARGET, Z_GYRO_TARGET};

	if (targets_funct[2] != 0) targets_funct[2] = accel_1g_value;			// Z-axis
	else if (targets_funct[1] != 0) targets_funct[1] = accel_1g_value;		// Y-axis
	else if (targets_funct[0] != 0) targets_funct[0] = accel_1g_value;		// X-axis

	// Debug Mode
	#ifdef DEBUG_MODE_MPU
		Serial.print("Locating the offset correction values.....");
	#endif

	// --- Calculate the averages ---
	calculateAverages(offset_correction, CALIBRATION_CORRECTION_AVERAGES, targets_funct);

	// Check that there aren't any issues
	if (imu_state == IMU_COMMS_ERROR || imu_state == IMU_INTERRUPT_TIMEOUT) {
		// There is an error set the offset correction to 0
		for (uint8_t i = 0; i < 6; i++) {
			offset_correction[i] = 0;
		}
	}

	// Debug Mode
	#ifdef DEBUG_MODE_MPU
		Serial.println(F("Completed!"));
		char debug_str[100];  // Used for the sprintf function
		sprintf(debug_str, "%d, %d, %d,\n\r%d, %d, %d",
				offset_correction[0], offset_correction[1],
				offset_correction[2], offset_correction[3],
				offset_correction[4], offset_correction[5]);
		Serial.println(debug_str);
	#endif

	// Done, the values are already stored in offset_correction
}


// =====================================================
// checkCalibration
// Checks if there are any calibration values in memory and loads them.
// This function check the memory for calibration data. If there is no calibration data or the temperature
// for the existing calibration is too different it will return false so the MPU can be calibrated after
// thermal stabilization. If there is valid calibration data in the memory, it will load the values to
// the offset registers of the MPU.
// ---
// Parameters:
//		@return bool	--> (bool) If false the MPU will need to be calibrated.
// =====================================================
bool Mpu6050Dev::checkCalibration()
{
	// Variables
	int16_t mpu_offsets[6];               // Initial offset values
	int16_t mpu_current_temperature;       // Temperature of the MPU
	int16_t mpu_previous_temperature = 0;  // Temperature of the MPU
	bool calibrated;                      // Flag to know if there is calibration data in the memory

	// --- Get current temperature ---
	mpu_current_temperature = getTemperature();

	// --- Check if there is data in the memory ---
	calibrated = loadFromMemory(&mpu_previous_temperature, mpu_offsets);
	// TODO --> Non volatile memory in STM32 ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	//calibrated = false;  // Just for testing :(

	if (calibrated) {  // Calibration data exists
		int16_t temp_difference = mpu_current_temperature - mpu_previous_temperature;  // Now this stores the temperature difference
		if (abs(temp_difference) < CALIBRATION_MAX_TEMP_DIFF) {  // The temperature is in range
			// No calibration needed, set the offsets and quit
			setOffsets(mpu_offsets);
			// Check communications
			if (imu_state == IMU_COMMS_ERROR) return false;

			// Debug
			#ifdef DEBUG_MODE_MPU
				Serial.println("Calibration data found. Loading data from memory");
			#endif

			return true;
		}
	}

	// Debug
	#ifdef DEBUG_MODE_MPU
		Serial.println("No calibration data found. The MPU would need to be calibrated :(");
		char debug_str[50];
		sprintf(debug_str, "Temperature: %d.%d C", mpu_current_temperature/100, mpu_current_temperature%100);
		Serial.println(debug_str);

		/*
		// >>>Just testing<<<
		Serial.println(">>> Testing Temperature <<<");
		double test_temperature_double;
		int16_t test_temperature_int;
		for (uint8_t i = 0; i < 100; i++) {
			test_temperature_double = getTemperatureDouble();
			test_temperature_int = getTemperature();
			sprintf(debug_str, "Temperature: %d.%d C --> %.2f", test_temperature_int/100,
					test_temperature_int%100, test_temperature_double);
			Serial.println(debug_str);
		}
		*/
	#endif

	// -- Set state --
	imu_state = IMU_NOT_CALIBRATED;
	return false;  // leave the MPU running until while temperature stabilization is reached
}


// =====================================================
// performCalibration()
// This function manages the calibration and stores the result in the memory.
// The main code for the calibration is in the calibrate() function.
// The function should only be called if the checkCalibration() function has returned false and the
// MPU has reached thermal stabilization.
// ---
// Parameters:
//		@return bool		--> (bool) Calibration status (true = calibration successful).
// =====================================================
bool Mpu6050Dev::performCalibration()
{
	// Variables
	int16_t mpu_offsets[6] = {0, 0, 0, 0, 0, 0};   // Initial offset values
	int16_t mpu_current_temperature;                // Temperature of the MPU
	bool correct;                                 // Flag to know if the calibration is correct or not

	// --- Start calibration ---
	correct = calibrate(mpu_offsets);  // Calibrate
	if (!correct) return false;  // calibration error

	imu_state = IMU_NOT_INITIALIZED;

	// -- Get the calibration temperature --
	mpu_current_temperature = getTemperature();

	// -- Set the offset values --
	setOffsets(mpu_offsets);
	// Check
	if (imu_state == IMU_COMMS_ERROR) return false;

	// Debug
	#ifdef DEBUG_MODE_MPU
		Serial.println("Calibration done. Loading data to the memory");
	#endif

	// --- save calibration on the memory ---
	saveOnMemory(&mpu_current_temperature, mpu_offsets);

	/*
	// The memory errors will be managed by the memory functions
	if (!correct) {
		// The data could not be loaded
		// TODO define error procedure. Not critical, just as a warning
	}
	// Debug
	#ifdef DEBUG_MODE_MPU
		if (correct) Serial.println("MPU calibrated and data loaded in the memory");
		else Serial.println("Memory Error, the calibration data could not be loaded");
	#endif
	*/

	// Completed :)
	return true;
}



//            **************************
//            *        Memory          *
//            **************************

// The original functions saved the data in the internal EERPOM of the Arduino MCU.
// The STM32 doesn't have an internal EEPROM, so the values are stored in the internal flash.
// check .h for more info on how this will be managed.


// =====================================================
// loadFromMemory()
// This function retrieves the calibration values from the memory

// ---
// Parameters:
//		@param *temperature_mpu		--> (int16_t*) Pointer to the temperature value.
//		#param *offsets_funct		--> (int16_t*) Pointer to the offsets array.
//		@return bool				--> (bool) If true the loading has been successful.
// =====================================================
bool Mpu6050Dev::loadFromMemory(int16_t *temperature_mpu, int16_t *offsets_funct)
{
#ifdef MEMORY_USE_FLASH

	const int16_t *p = offsets_flash;

	// --- Check if there is Data ---
	if (*(int16_t*)(p) != (int16_t)MPU_MEMORY_CONTROL_VAL) return false; // The values haven't been stored

	// --- Read ---
	*temperature_mpu = *(int16_t*)(p + 1);
	for (uint i = 0; i < 6; i++) {
		offsets_funct[i] = *(int16_t*)(p + i + 2);
	}

	return true;

#else

	return false;

#endif
}


// =====================================================
// saveOnMemory()
// This function stores the calibration data in the memory.
// To ensure that the data isn't corrupted due to voltage spikes in the writing process,
// the data will be written, then read and compared. So it will be a relatively slow process,
// but it only needs to be done once.
// ---
// Parameters:
//		@param *temperature_mpu		--> (int16_t*) Pointer to the temperature value.
//		#param *offsets_funct		--> (int16_t*) Pointer to the offsets array.
// =====================================================
void Mpu6050Dev::saveOnMemory(int16_t *temperature_mpu, int16_t *offsets_funct)
{
#ifdef MEMORY_USE_FLASH

	const int16_t *p = offsets_flash;
	const int16_t **ptr = &p;

	// --- Write ---
	if (!memoryWrite(*(uint32_t*)ptr + 2, (uint16_t*)temperature_mpu, 1)) return;
	if (!memoryWrite(*(uint32_t*)ptr + 4, (uint16_t*)offsets_funct, 6)) return;

	// --- Check ---
	if (*temperature_mpu != *(int16_t*)(p + 1)) return;

	for (uint i = 0; i < 6; i++) {
		if(offsets_funct[i] != *(int16_t*)(p + 2 + i)) return;
	}

	// --- Correct ---
	uint16_t control_value = MPU_MEMORY_CONTROL_VAL;
	memoryWrite(*(uint32_t*)ptr, &control_value, 1);

#else

	return;

#endif
}


//            **************************
//            *     Measurements       *
//            **************************


// =====================================================
// getTemperatureDouble()
// Gets the temperature measurement from the MPU as a double precision floating point value.
// ---
// Parameters:
//		@return double		--> (double) temperature measurement
// =====================================================
double Mpu6050Dev::getTemperatureDouble()
{
	int16_t temperature_temp;

	// --- Get data form the MPU ---
	readImuData(MPU_TEMP_REG_BASE, &temperature_temp);

	// --- Convert value ---
	double foo_temp = ((double)(temperature_temp)/340) + 36.53;

	return foo_temp;
}


// =====================================================
// getTemperature()
// Gets the temperature measurement from the MPU as a 16-bit value.
// This function is used to reduce the memory usage of for the temperature.
// To reduce the truncation errors it will give the temperature in degrees Celsius * 100.
// ---
// Parameters:
//		@return int16_t		--> (int16_t) temperature (Celsius * 100)
// =====================================================
int16_t Mpu6050Dev::getTemperature()
{
	int16_t temperature_temp;

	// --- Get data form the MPU ---
	readImuData(MPU_TEMP_REG_BASE, &temperature_temp);

	// --- Convert value ---
	temperature_temp = (100* (temperature_temp)/340) + 3653;

	return temperature_temp;
}


// =====================================================
// getParameter6()
// Gets the raw accelerometer and gyroscope measurements.
// The accelerometer units and scaling of the values will depend on the configuration of the MPU.
// The measurements are read in the following order: A_X, A_Y, A_Z, G_X, G_Y, G_Z.
// ---
// Parameters:
//		@param *values_funct	--> (int16_t*) Pointer to the measurements array.
// =====================================================
void Mpu6050Dev::getParameter6(int16_t *values_funct)
{
	// --- Read the values ---
	readImuMeasurements(values_funct);

	/*
	// Debug
	#ifdef DEBUG_MODE_MPU
		char debug_str[100];
		sprintf(debug_str, "Raw values:\n\r  %d, %d, %d,\n\r  %d, %d, %d, values_funct[0], values_funct[1],
																	      values_funct[2], values_funct[3],
																	      values_funct[4], values_funct[5]);
		Serial.println(debug_str);
		if ((values_funct[0] == 0X7FFF) || (values_funct[0] == 0X8000)) Serial.println("Accel-X overflow!--------------------------------------------------------------");
		if ((values_funct[1] == 0X7FFF) || (values_funct[1] == 0X8000)) Serial.println("Accel-Y overflow!--------------------------------------------------------------");
		if ((values_funct[2] == 0X7FFF) || (values_funct[2] == 0X8000)) Serial.println("Accel-Z overflow!--------------------------------------------------------------");
		if ((values_funct[3] == 0X7FFF) || (values_funct[3] == 0X8000)) Serial.println("Gyro-X overflow!---------------------------------------------------------------");
		if ((values_funct[4] == 0X7FFF) || (values_funct[4] == 0X8000)) Serial.println("Gyro-Y overflow!---------------------------------------------------------------");
		if ((values_funct[5] == 0X7FFF) || (values_funct[5] == 0X8000)) Serial.println("Gyro-Z overflow!---------------------------------------------------------------");
	#endif
	*/
}


// =====================================================
// getRefinedValues()
// Gets the raw accelerometer and gyroscope measurements.
// The accelerometer values are in [g] and the angular speeds in [rad/s].
// The measurements are read in the following order: A_X, A_Y, A_Z, G_X, G_Y, G_Z.
// ---
// Parameters:
//		@param *values_funct	--> (int16_t*) Pointer to the measurements array.
// =====================================================
void Mpu6050Dev::getRefinedValues(double *measurements_funct)
{
	int16_t raw_values[6];

	// --- Get the raw values ---
	getParameter6(raw_values);

	// --- Refine values ---
	for(uint8_t i = 0; i < 6; i++){

		// -- Offset correction --
		*(measurements_funct + i) = ((double) *(raw_values + i) - *(offset_correction + i));

		// -- Obtain magnitudes --
		if ( i < 3) *(measurements_funct + i) /=  accel_1g_value;  			 // Accelerations
		else *(measurements_funct + i) *= (M_PI / (180 * gyro_1dps_value));  // Gyroscope

		/* The angles are correct
		// -- Correct Y angles --
		// The Y direction on the datasheet is the opposite as the assumed one
		if ((i == 2) ||( i == 4)) *(measurements_funct + i) *= -1;
		*/
	}

	/*
	// Debug
	#ifdef DEBUG_MODE_MPU
		char debug_str[50];
		Serial.println("Raw VS refined values:");
		for (uint8_t debug_t; debug_i < 6; debug_i ++) {
			sprintf(debug_str, "%d --> %d", raw_vales[debug_i], measuremetns_funct[debug_i]);
			Serial.println(debug_str);
		}
	#endif
	*/
}

/* ---- END OF FILE ---- */
