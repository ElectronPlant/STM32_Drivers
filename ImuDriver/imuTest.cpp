/*
 *                     ************************
 *                     *     imuTest     *
 *                     ************************
 *
 * ============================================================================
 *
 *     File: 		imuTest.cpp
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
#include "filter.h"
#include "commonMath.h"

#include <stdio.h>

//            **************************
//            *       PARAMETERS       *
//            **************************


//            **************************
//            *        VARIABLES       *
//            **************************

void imuTest()
{
	// Init IMU
	if (!imu.initialize_1()) Serial.println("error...error 1");
	delay(500);
	if (!imu.initialize_2()) Serial.println("error...error 2");

	//debugIntegralDerivative();

	/*
	//--------------------------------------------------
	// Test Init
	//--------------------------------------------------
	 // Start measurements
	if(!resetAngle()) Serial.println("error...error 3");

	//int16_t raw_values[6] = {0, 0, 0, 0, 0, 0};
	//double refined_values[6] = {0, 0, 0, 0, 0, 0};
	double imu_measurements_test[6] 	= {0, 0, 0, 0, 0, 0};
	double test[6]						= {0, 0, 0, 0, 0, 0};
	uint32_t time_test 					= 0;

	char debug_str[500];
	//double filtered_output = 0;
	while (1) {
		imuIntTimeout();

		if (imu_data_ready) {
			//imu.getParameter6(raw_values);
			imu.getRefinedValues(imu_measurements_test);

			//filtered_output = accelFilter(refined_values[2]);

			testSimplifiedKf(imu_measurements_test, time_test, test);
			imu_data_ready = false;

			if(Serial.bufferReset()) {

				// Refined values
				sprintf(debug_str, "%f, %f, %f",
						test[0], test[1], test[2]);

				Serial.bufferStore(debug_str);

				Serial.bufferStoreln("");

				Serial.bufferSend();
			}
		}
	}
	//--------------------------------------------------
	// Test End
	//--------------------------------------------------

	// Start measurements
	if(!resetAngle()) Serial.println("error...error 3");

//	while (1) {
//		imuIntTimeout();
//
//		if (imu_data_ready){
//			simplifiedKf(imu_time_buffer);
//			imu_data_ready = false;
//
//			if(Serial.bufferReset()) {
//				Serial.bufferDouble(state_kf[0] * 180.0f/PI);
//				Serial.bufferDouble(state_kf[1] * 180.0f/PI);
//				Serial.bufferStoreln("");
//				Serial.bufferSend();
//			}
//		}
//	}
	*/

	double imu_measurements[6] 			= {0, 0, 0, 0, 0, 0};
	double imu_measurements_test[6] 	= {0, 0, 0, 0, 0, 0};
	double test[6]						= {0, 0, 0, 0, 0, 0};
	uint32_t time_test 					= 0;

	double filtered_output = 0;


	while (1) {
		imuIntTimeout();

		if (imu_data_ready) {

			// --- Get measurements ---
			imu.getRefinedValues(imu_measurements);
			time_test = imu_time_buffer;
			if (imu.imu_state != IMU_CORRECT) {
				Serial.print("IMU Error");
				while (1);
			}//return;

			for (uint8_t i = 0; i < 6; i++) {
				imu_measurements_test[i] = imu_measurements[i];
			}

			simplifiedKf(imu_measurements, time_test);
			//imu_measurements_test[1] = accelFilter(imu_measurements_test[1]);
			//testSimplifiedKf(imu_measurements_test, time_test, test);
			imu_data_ready = false;

			//filtered_output = accelFilter(test[2]);

			if(Serial.bufferReset()) {
				Serial.bufferDouble(state_kf[0] * 180.0f/PI);
				Serial.bufferDouble(state_kf[1] * 180.0f/PI);
				//Serial.bufferStoreln("Gyro estimation:");
				//Serial.bufferDouble(test[0] * 180.0f/PI);
				//Serial.bufferDouble(test[1] * 180.0f/PI);
				//Serial.bufferStoreln("\n\rAcceleration estimation:");
				Serial.bufferDouble(test[2] * 180.0f/PI);
				Serial.bufferDouble(test[3] * 180.0f/PI);
				Serial.bufferDouble(filtered_output * 180.0f/PI);
				Serial.bufferStoreln("");
				Serial.bufferSend();
			}
		}
	}
	//

//	uint8_t buffer[4];
//	uint8_t address = 0x68;
//	int32_t test = 0x00000004;
//
//	Serial.println("Testing IMU :)\n\r");
//
//	Serial.println("--- Reading ---");
//	*(uint16_t *)(buffer) = 0xDACB;
//	*buffer = test;
//	if (!test_imu.readRegister(address, 0x75, buffer)) Serial.println("Error: 1");
//	Serial.bufferReset();
//	Serial.bufferInt((int32_t)*buffer);
//	Serial.bufferStoreln("");
//	Serial.bufferSend();
//
//	delay(100);
//	Serial.println("--- Writing ---");
//	if (!test_imu.readRegister(address, 0x6B, buffer)) Serial.println("Error: 4");
//	//if (!test_imu.readRegister(address, 0x07, buffer)) Serial.println("Error: 5");
//	Serial.bufferReset();
//	Serial.bufferInt(buffer[0]);
//	Serial.bufferStoreln("");
////	Serial.bufferInt(buffer[1]);
////	Serial.bufferStoreln("");
//	Serial.bufferSend();
//
//	buffer[1] = (buffer[0] & ~0xE8) | (0x00 & 0xE8);
//	if (!test_imu.writeRegister(address, 0x6B, buffer + 1, true)) Serial.println("Error: 2");
//	//if (!test_imu.writeRegister(address, 0x07, buffer + 1)) Serial.println("Error: 3");
//	for (uint8_t i = 0; i < 4; i++) {
//		buffer[i] = 0;
//	}
//	if (!test_imu.readRegister(address, 0x6B, buffer)) Serial.println("Error: 4");
//	//if (!test_imu.readRegister(address, 0x07, buffer)) Serial.println("Error: 5");
//	Serial.bufferReset();
//	Serial.bufferInt(buffer[0]);
//	Serial.bufferStoreln("");
//	//Serial.bufferInt(buffer[1]);
//	//Serial.bufferStoreln("");
//	Serial.bufferSend();
//
//	delay(100);
//	Serial.println("--- Writing ---");
//	if (!test_imu.readRegister(address, 0x19, buffer)) Serial.println("Error: 4");
//	//if (!test_imu.readRegister(address, 0x07, buffer)) Serial.println("Error: 5");
//	Serial.bufferReset();
//	Serial.bufferInt(buffer[0]);
////	Serial.bufferInt(buffer[1]);
//	Serial.bufferStoreln("");
//	Serial.bufferSend();
//
//	buffer[1] = 0x44;
//	if (!test_imu.writeRegister(address, 0x19, buffer + 1)) Serial.println("Error: 2");
//	//if (!test_imu.writeRegister(address, 0x07, buffer + 1)) Serial.println("Error: 3");
////	for (uint8_t i = 0; i < 4; i++) {
////		buffer[i] = 0;
////	}
//	if (!test_imu.readRegister(address, 0x19, buffer)) Serial.println("Error: 4");
//	//if (!test_imu.readRegister(address, 0x07, buffer)) Serial.println("Error: 5");
//	Serial.bufferReset();
//	Serial.bufferInt(buffer[0]);
//	Serial.bufferInt(buffer[1]);
//	Serial.bufferStoreln("");
//	Serial.bufferSend();
}




/* ---- END OF FILE ---- */
