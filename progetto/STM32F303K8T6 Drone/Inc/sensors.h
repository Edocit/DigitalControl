/*
 * sensors.h
 *
 *  Created on: 25 mag 2019
 *      Author: edoardo
 */

#ifndef SENSORS_H_
#define SENSORS_H_

//FUNCTIONS PROTOTYPES
void initialize_imu(void);	//initial setup of the sensor
void calibrate_imu(void);	//calibration is needed in order to minimize sensor error
void read_gyro(void);		//read only gyro for acro mode and full speed controller
void read_accel(void);		//read only accelerometer, just use for splitted tests
void read_imu(void);   	//retrieves all data from the IMU (Inertial Measurement Unit)


#endif /* SENSORS_H_ */
