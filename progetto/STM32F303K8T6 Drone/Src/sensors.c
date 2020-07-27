/*
 * sensors.c
 *
 *  Created on: 25 mag 2019
 *      Author: edoardo
 */

#include "common.h"
#include "config.h"
#include "main.h"
#include "stdlib.h"
#include "math.h"

void initialize_imu(){
	  HAL_I2C_Mem_Read (&hi2c1, MPU_6050,WHO_AM_I,1, &check, 1, 1000);

	if (check == 104){  // 0x68 will be returned by the sensor if everything goes well
	  //Wake-up the sensor
	  i2c_buffer[0] = WAKE_UP_REG;
	  i2c_buffer[1] = 0x00;
	  HAL_I2C_Master_Transmit(&hi2c1, MPU_6050, i2c_buffer, 2, 1000);

	  i2c_buffer[0] = PWR_MNG_1;
	  i2c_buffer[1] = 0x01;
	  HAL_I2C_Master_Transmit(&hi2c1, MPU_6050, i2c_buffer, 2, 1000);

	  //Set DATA RATE of 1KHz by writing SMPLRT_DIV register
	  i2c_buffer[0] = SMPLRT_DIV_REG;
	  i2c_buffer[1] = 0x07;
	  HAL_I2C_Master_Transmit(&hi2c1, MPU_6050, i2c_buffer, 2, 1000);

	  //Setting gyro sensitivity DPS scale
	  i2c_buffer[0] = GYRO_CONFIG_REG;
	  i2c_buffer[1] = DPS_FULL_SCALE_500;
	  HAL_I2C_Master_Transmit(&hi2c1, MPU_6050, i2c_buffer, 2, 1000);

	  //Setting accelerometer scale
	  i2c_buffer[0] = ACCEL_CONFIG_REG;
	  i2c_buffer[1] = ACCEL_8G_RANGE;
	  HAL_I2C_Master_Transmit(&hi2c1, MPU_6050, i2c_buffer, 2, 1000);

	  //Setting digital low-pass filter at ~43MHz
	  i2c_buffer[0] = LOW_PASS_FILTER_REG;
	  i2c_buffer[1] = 0x03;
	  HAL_I2C_Master_Transmit(&hi2c1, MPU_6050, i2c_buffer, 2, 1000);
	}
}





void calibrate_imu() {

	HAL_GPIO_WritePin(GPIOB, CAL_PIN, SET);

	HAL_Delay(500);

	//first ~150 values are discarded
	for (i = 0; i < 150; i++) {
		HAL_I2C_Mem_Read(&hi2c1,MPU_6050,GYRO_DATA_REG ,I2C_MEMADD_SIZE_8BIT, i2c_buffer, 6, 1000);
		HAL_Delay(3);
	}


	for (i = 0; i < CALIBRATION_SAMPLES; i++) {
		//ACCEL ANGLE CORRECTION CALCULATION CODE
		HAL_I2C_Mem_Read(&hi2c1,MPU_6050,GYRO_DATA_REG ,I2C_MEMADD_SIZE_8BIT, i2c_buffer, 6, 1000);			  //asking for a 6 register burst for gyro

		//GYRO OFFSET CALCULATION CODE
	    gyro_raw_x  =   i2c_buffer[0]  << 8 | i2c_buffer[1];                         //gyro 16 bit X axis value
	    gyro_raw_y  =   i2c_buffer[2]  << 8 | i2c_buffer[3];                         //gyro 16 bit Y axis value
	    gyro_raw_z  =   i2c_buffer[4]  << 8 | i2c_buffer[5];                         //gyro 16 bit Z axis value

	    gyro_raw_error_x += gyro_raw_x;        //cumulative value for the error on X axis
	    gyro_raw_error_y += gyro_raw_y;        //cumulative value for the error on Y axis
	    gyro_raw_error_z += gyro_raw_z;        //cumulative value for the error on Z axis


	    if (i == CALIBRATION_SAMPLES - 1) {
			gyro_raw_error_x  /= CALIBRATION_SAMPLES;                 //final gyro X axis correcting offset value
			gyro_raw_error_y  /= CALIBRATION_SAMPLES;                 //final gyro Y axis correcting offset value
			gyro_raw_error_z  /= CALIBRATION_SAMPLES;                 //final gyro Z axis correcting offset value
	    }

	    HAL_Delay(3);
	}

	HAL_GPIO_WritePin(GPIOB, CAL_PIN, RESET);

}





void read_imu(){

	if(flight_mode == STABILIZED){

		HAL_I2C_Mem_Read(&hi2c1,MPU_6050,ACCEL_DATA_REG,I2C_MEMADD_SIZE_8BIT,i2c_buffer,14,10);

		accel_raw_x =   i2c_buffer[0]  << 8 | i2c_buffer[1];                         					   //accel 16 bit X axis value
		accel_raw_y =   i2c_buffer[2]  << 8 | i2c_buffer[3];                         					   //accel 16 bit Y axis value
		accel_raw_z =   i2c_buffer[4]  << 8 | i2c_buffer[5];                         					   //accel 16 bit Z axis value

		gyro_raw_x  = ( ( i2c_buffer[8]  << 8 | i2c_buffer[9]  ) - gyro_raw_error_x ) * FLIP_GYRO_X_DATA_SIGN;	 //gyro  16 bit X axis value
		gyro_raw_y  = ( ( i2c_buffer[10] << 8 | i2c_buffer[11] ) - gyro_raw_error_y ) * FLIP_GYRO_Y_DATA_SIGN;   //gyro  16 bit Y axis value
		gyro_raw_z  = ( ( i2c_buffer[12] << 8 | i2c_buffer[13] ) - gyro_raw_error_z ) * FLIP_GYRO_Z_DATA_SIGN; 	 //gyro  16 bit Z axis value

		//Integration of gyro angle given by the known sampling period of the sensor and the sampled value added to the global value
		// (deg/sec) * sec = deg
		//Remember traveled_angle_multiplier = 1 / control_speed_in_hz  so we have grater multiplier for slower control loops
		gyro_stab_angle_x += ( gyro_raw_x / GYRO_PRESCALER ) * traveled_angle_multiplier;
	    gyro_stab_angle_y += ( gyro_raw_y / GYRO_PRESCALER ) * traveled_angle_multiplier;

	    //Now it is very very very important as a flying object to manage the transfer load over not spirit level when yaw is not null
	    //in order to pass correctly values over roll and pitch axis and viceversa
	    //Remember yaw_load_transfer_multiplier = traveled_angle_multiplier * DEG_TO_RAD;
	    gyro_stab_angle_x = gyro_stab_angle_x - gyro_stab_angle_y * sin( ( gyro_raw_z / GYRO_PRESCALER ) * yaw_load_transfer_multiplier); //If load transfer roll  angle to the pitch angle
	    gyro_stab_angle_y = gyro_stab_angle_y + gyro_stab_angle_x * sin( ( gyro_raw_z / GYRO_PRESCALER ) * yaw_load_transfer_multiplier); //If load transfer pitch angle to the roll  angle


	    //Gyro component is ready, now we calculate accelerometer component
		acceleration_total_vector = sqrt( (accel_raw_x * accel_raw_x) + (accel_raw_y * accel_raw_y) + (accel_raw_z * accel_raw_z) );

		if(abs(accel_raw_y) < acceleration_total_vector){	//Avoid asin function to output NaN ("Not a Number")
			accel_stab_angle_x = asin( (float)accel_raw_y / acceleration_total_vector) * RAD_TO_DEG * FLIP_ACCEL_X_DATA_SIGN;         //Calculate the pitch angle.
		}
		if(abs(accel_raw_x) < acceleration_total_vector){	//Avoid asin function to output NaN ("Not a Number")
			accel_stab_angle_y = asin( (float)accel_raw_x / acceleration_total_vector) * RAD_TO_DEG * FLIP_ACCEL_Y_DATA_SIGN;        //Calculate the roll angle.
		}

		//Accelerometer as opposite to gyro is reliable on long period so make sense to calibrate it observing stable offset w.r.t zero over time
		accel_stab_angle_x -= accel_angle_error_x;                //Acceleration adjusted value for pitch angle ( X axis)
		accel_stab_angle_y -= accel_angle_error_y;                //Acceleration adjusted value for roll  angle ( Y axis)

		//Finally complementary filter is applied to obtain total angles taking advantage of gyro integrated angles and removing gyro drift
		//using accelerometer angle (in a very little portion to avoid as much as possible accelerometer measure noise)
		gyro_stab_angle_x = gyro_stab_angle_x * ALPHA_VALUE + accel_stab_angle_x * (1 - ALPHA_VALUE);
		gyro_stab_angle_y = gyro_stab_angle_y * ALPHA_VALUE + accel_stab_angle_y * (1 - ALPHA_VALUE);

	}
	else{
		HAL_I2C_Mem_Read(&hi2c1, MPU_6050, GYRO_DATA_REG, I2C_MEMADD_SIZE_8BIT, i2c_buffer, 6, 1000);		 		   //asking for a 6 register burst for gyro

		gyro_raw_x  = ( ( i2c_buffer[0] << 8 | i2c_buffer[1] ) - gyro_raw_error_x ) * FLIP_GYRO_X_DATA_SIGN;	 //gyro  16 bit X axis value
		gyro_raw_y  = ( ( i2c_buffer[2] << 8 | i2c_buffer[3] ) - gyro_raw_error_y ) * FLIP_GYRO_Y_DATA_SIGN;     //gyro  16 bit Y axis value
		gyro_raw_z  = ( ( i2c_buffer[4] << 8 | i2c_buffer[5] ) - gyro_raw_error_z ) * FLIP_GYRO_Z_DATA_SIGN; 	 //gyro  16 bit Z axis value

		gyro_raw_x /= GYRO_PRESCALER;
		gyro_raw_y /= GYRO_PRESCALER;
		gyro_raw_z /= GYRO_PRESCALER;

	}

		//Using previous value as first component of the complementary (LP) filter due to avoid gyro position drift (percentages may change on sensors !?)
		if ( gyro_raw_x >= MIN_GYRO_VALUE && gyro_raw_x <= MAX_GYRO_VALUE ) {
			gyro_pitch_input = ( gyro_pitch_input * 0.8 ) + ( gyro_raw_x * 0.2 );		//EWMA filter only for gyro (reliable and coherent data)
		}
		if ( gyro_raw_y >= MIN_GYRO_VALUE && gyro_raw_y <= MAX_GYRO_VALUE ) {
			gyro_roll_input  = ( gyro_roll_input  * 0.8 ) + ( gyro_raw_y * 0.2 );       //EWMA filter only for gyro (reliable and coherent data)
		}
		if ( gyro_raw_z >= MIN_GYRO_VALUE && gyro_raw_z <= MAX_GYRO_VALUE ) {
			gyro_yaw_input   = ( gyro_yaw_input   * 0.8 ) + ( gyro_raw_z * 0.2 );       //EWMA filter only for gyro (reliable and coherent data)
		}


}

