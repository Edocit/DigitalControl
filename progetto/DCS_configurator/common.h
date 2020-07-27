
#ifndef COMMON_H_
#define COMMON_H_

#include <stm32f3xx_it.h>
#include "stm32f3xx_hal.h"
#include <sys/_stdint.h>

extern uint8_t check;

	enum commands         {PITCH, ROLL, THROTTLE, YAW};										        //just to make the code more readable
	enum modes            {ACRO, STABILIZED};                                                       //flights modes switchable only when NOT armed
	enum motors           {MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, MOTOR_5, MOTOR_6, MOTOR_7, MOTOR_8}; //motors positions w.r.t the documentation
	enum droneType        {QUADCOPTER=4, HEXACOPTER=6, OCTACOPTER=8};                               //flight configurations

	typedef int bool;
	#define false 0
	#define true  1

    #define HIGH                              1 //just for readability
    #define LOW                               0 //just for readability

	#define MOTOR_N							  4 //number of motors in the desired architecture (choice {4,6,8})
	#define MAX_MOTOR_N                       8 //DO NOT MODIFY max number of motors possible
    #define RADIO_CH_N						  4 //number of the radio channels that will be involved in the process loop

    #define ACRO_LB_SWITCH				   1180 //Rotating fixed position potenziometer for flight mode switch
    #define ACRO_UB_SWITCH				   1200 //Rotating fixed position potenziometer for flight mode switch
	#define STABILIZED_LB_SWITCH		   1280 //Rotating fixed position potenziometer for flight mode switch
	#define STABILIZED_UB_SWITCH    	   1300 //Rotating fixed position potenziometer for flight mode switch

	#define MAX_PWM_OUTPUT_OS_125   		250
	#define MAX_PWM_OUTPUT_OS_42   		     84
	#define MAX_PWM_OUTPUT           	   2000
	#define MAX_TIM1_VALUE                 2038	//DO NOT MODIFY AutoReloadRegister (ARR) value for TIM2 that is a free-running counter with step 1us

	#define MPU_6050                       0xD0 //DO NOT MODIFY slave address of IMU MPU_6050 over I2C
	#define READ_ENABLE					   0x01 //DO NOT MODIFY enables slave IMU MPU_6050 read over I2C
	#define WAKE_UP_REG                    0x6B //DO NOT MODIFY wake-up register of the unit
    #define PWR_MNG_1                      0x6B //DO NOT MODIFY the same as the previous just for readability
	#define TEMPERATURE_REG                0x41 //DO NOT MODIFY temperature value first register
	#define SMPLRT_DIV_REG				   0x19 //DO NOT MODIFY data output sample rate


	#define ACCEL_CONFIG_REG        	   0x1C //DO NOT MODIFY register that allows to modify accelerometer configuration
	#define ACCEL_DATA_REG				   0x3B //DO NOT MODIFY first data register of the requested burst
	#define ACCEL_8G_RANGE				   0x10 //DO NOT MODIFY +- 8g acceleration measure scale
	#define ACCEL_PRESCALER				 4096.0 //DO NOT MODIFY divisor to obtain correct accel data (is important the ".0" because it must be float)
	#define FLIP_ACCEL_X_DATA_SIGN             1 //inverter of accel motion (it must agree with the model given in the doc)
	#define FLIP_ACCEL_Y_DATA_SIGN            -1 //inverter of accel motion (it must agree with the model given in the doc)
	#define FLIP_ACCEL_Z_DATA_SIGN             1 //inverter of accel motion (it must agree with the model given in the doc)

	#define GYRO_CONFIG_REG                0x1B //DO NOT MODIFY register that allows to modify gyro configuration
	#define DPS_FULL_SCALE_500             0x08 //DO NOT MODIFY gyro dps mask
    #define GYRO_PRESCALER                 65.5 //DO NOT MODIFY divisor to obtain the true data from raw data for a gyro configuration of 500 dps
    #define GYRO_DATA_REG                  0x43 //DO NOT MODIFY first data register of the requested burst
    #define WHO_AM_I		    		   0x75


	#define LOW_PASS_FILTER_REG            0x1A //DO NOT MODIFY register used to enable am hardware low-pass filter buil in the MPU-6050
	#define CALIBRATION_SAMPLES            2000 //number of iterations that will be performed in order to find calibration values
	#define FLIP_GYRO_X_DATA_SIGN             1 //inverter of gyro motion (it must agree with the model given in the doc)
	#define FLIP_GYRO_Y_DATA_SIGN             1 //inverter of gyro motion (it must agree with the model given in the doc)
	#define FLIP_GYRO_Z_DATA_SIGN            -1 //inverter of gyro motion (it must agree with the model given in the doc)
	#define MAX_GYRO_VALUE                  500 //valued expressed in degrees per second... from datasheet the possible values are { +- 250, 500, 1000, 2000 }
	#define MIN_GYRO_VALUE                 -500 //valued expressed in degrees per second... from datasheet the possible values are { +- 250, 500, 1000, 2000 }

	#define ANGLE_CORRECTION_10				 50 //DO NOT MODIFY this gain bounds angles to [-10;+10] --> 2000 - (50      * 10) = 1500.0000 aka zero angular motion at 10 deg
	#define ANGLE_CORRECTION_15			 33.333 //DO NOT MODIFY this gain bounds angles to [-15;+15] --> 2000 - (33.333  * 15) = 1500.0050 aka zero angular motion at 15 deg
	#define ANGLE_CORRECTION_35			  14.28 //DO NOT MODIFY this gain bounds angles to [-35;+35] --> 2000 - (14.28   * 35) = 1500.2000 aka zero angular motion at 35 deg
	#define ANGLE_CORRECTION_45			11.1111 //DO NOT MODIFY this gain bounds angles to [-45;+45] --> 2000 - (11.1111 * 45) = 1500.0005 aka zero angular motion at 45 deg

	#define PI 						 3.14159265
    #define DEG_TO_RAD             0.0174532925  // ( PI / 180 )
	#define RAD_TO_DEG     		   57.295779579  // ( 1 / (PI / 180) )

	#define SW_PITCH_REVERSE                  1 //channel inverter (for example if pitching forward the quad goes backward)
	#define SW_ROLL_REVERSE                   1 //channel inverter
	#define SW_YAW_REVERSE                    1 //channel inverter

	#define THROTTLE_LOWER_THD_OS_125 		130 //DO NOT MODIFY under this value arm/disarm commands are enabled
	#define THROTTLE_LOWER_THD_OS_42 		 45 //DO NOT MODIFY under this value arm/disarm commands are enabled
	#define MAX_THROTTLE	 			   1800 //Throttle taken from stick is limited in order to leave a certain ceil to the PID also at full throttle
	#define THROTTLE_LOWER_THD      	   1010 //DO NOT MODIFY under this value arm/disarm commands are enabled

	#define MAX_PWM_RADIO				   2000 //DO NOT MODIFY maximum width of 50Hz radio pulse
	#define DEADBAND_UPPER_BOUND           1508 //DO NOT MODIFY upper bound of the middle position where the radio signal is evaluated as 0 dps
	#define DEADBAND_LOWER_BOUND           1492 //DO NOT MODIFY lower bound of the middle position where the radio signal is evaluated as 0 dps

	#define PID_UPPER_BOUND_OS_125			 50 //upper bound for calculated PID value
	#define PID_LOWER_BOUND_OS_125 		    -50 //lower bound for calculated PID value
	#define PID_UPPER_BOUND	    			400 //upper bound for calculated PID value
	#define PID_LOWER_BOUND     		   -400 //lower bound for calculated PID value

	#define SETPOINT_PITCH_SPEED_RATE       3.0 //ex for value 3   -->   (500 - 8) / 3.0 = 164 deg/sec
	#define SETPOINT_ROLL_SPEED_RATE        3.0 //ex for value 3   -->   (500 - 8) / 3.0 = 164 deg/sec
	#define SETPOINT_YAW_SPEED_RATE         3.0 //ex for value 3   -->   (500 - 8) / 3.0 = 164 deg/sec

	#define LOOP_8KHZ	125   //Timing for acro mode (only using gyro F4 and F7)
    #define LOOP_4KHZ   250	  //Timing for acro mode (only using gyro F3, F4, F7)
	#define LOOP_2_5KHZ 400	  //Timing for acro mode (old devices like F1 (maybe F0) without FPU) if needed control speed (only) basic flight loop
    #define LOOP_2KHZ   500	  //Timing for acro mode (old devices like F1 (maybe F0) without FPU)
	#define LOOP_1KHZ   1000  //Timing for stabilized mode (accelerometer limits 1KHz control loop for MPU-6050)
    #define LOOP_490HZ  2038  //Timing for slower flight controllers or intense back-control activity
    #define LOOP_250HZ  4000  //Timing for very slow flight controllers or very intense back-control activity

	#define MIN_SPIN_OS_125     125  //lowest  PWM value usable to make motors not spin without loss of signal on ONE_SHOT_125
	#define MAX_SPIN_OS_125     250  //highest PWM value usable to make motors spin at max throttle (100% PWM pulse width)
	#define MIN_SPIN            1100 //lowest  PWM value usable to make motors not spin without loss of signal on classical PWM (until 500Hz)

	#define WRN_PIN       GPIO_PIN_5
	#define CAL_PIN       GPIO_PIN_4


	/////////////////////////////////////////VARIABLES DECLARATION AND INTERFACE///////////////////////////////////////////////////

	extern TIM_HandleTypeDef  htim1;    			  			//PWM radio handler -- it's free running with 1us step
	extern TIM_HandleTypeDef  htim3;    			  			//Ctrl loop handler -- it's free running with 1us step
	extern I2C_HandleTypeDef  hi2c1;				  			//Handler for IMU I2C communication

	extern bool      armed;							  			//Check if motors are enabled to spin
	extern uint8_t   check;
	extern uint16_t  throttle;						  			//Raw radio channel 3 (CH3) value
	extern uint8_t   drone_config;								//Vehicle configuration {QUADCOPTER,HEXACOPTER,OCTACOPTER}
	extern uint8_t   flight_mode;					  			//Flight controller mode (see enum modes at the beginning of the file)
	extern int       i;								  			//General purpose variable used for loops
	extern float     cpu_load;									//Used to know how much of time is left in control loop stay inside requested KHZ

	extern volatile uint16_t counter_rising_edge;	 			//Used to take the beginning of the radio pulse
 	extern volatile uint16_t counter_falling_edge;	  			//Used to take the end of the radio pulse
	extern volatile uint16_t global_input_count;      			//Used to take the timer value at the beginning of each radio interrupt

	extern volatile uint8_t  CH_1_PREV_STATE;					//Take trace of the pulse edge for the channel
	extern volatile uint8_t  CH_2_PREV_STATE;					//Take trace of the pulse edge for the channel
	extern volatile uint8_t  CH_3_PREV_STATE;					//Take trace of the pulse edge for the channel
	extern volatile uint8_t  CH_4_PREV_STATE;					//Take trace of the pulse edge for the channel

	extern volatile uint16_t radio_channel_pulse[RADIO_CH_N];	//Radio pulse array

	extern uint16_t controller_speed; 				            //Choosen speed for control loop
	extern uint16_t default_controller_speed;					//Used to know when switching back from stabilized to acro if increase control loop rate or not

	extern float traveled_angle_multiplier;						//Variable used for gyro integration in stabilized mode
	extern float yaw_load_transfer_multiplier;					//Variable used for load tranfer from pitch to roll (and viceversa)
	extern uint8_t i2c_buffer[14];								//Buffer used to read/write data from MPU-6050 communication over I2C bus

	extern uint8_t angle_correction;							//variable to store the gain value for runtime error correction

	extern uint16_t speedVec[MAX_MOTOR_N];				        //Motors speed array (min 4 motors - max 8 motors)

	extern float gyro_pitch_input;							    //X axis angle
	extern float gyro_roll_input;								//Y axis angle
	extern float gyro_yaw_input;								//Z axis angle

	extern int16_t gyro_raw_x;									//X axis gyro raw value
	extern int16_t gyro_raw_y;									//Y axis gyro raw value
	extern int16_t gyro_raw_z;									//Z axis gyro raw value

	extern int32_t gyro_raw_error_x;							//Specific sensor X axis error mitigation
	extern int32_t gyro_raw_error_y;							//Specific sensor Y axis error mitigation
	extern int32_t gyro_raw_error_z;							//Specific sensor Z axis error mitigation

	extern float   gyro_angle_x;								//X axis gyro value
	extern float   gyro_angle_y;								//Y axis gyro value
	extern float   gyro_angle_z;								//Z axis gyro value

	extern int16_t accel_raw_x;									//X axis accelerometer raw value
	extern int16_t accel_raw_y;									//Y axis accelerometer raw value
	extern int16_t accel_raw_z;									//Z axis accelerometer raw value

	extern float accel_angle_error_x;							//Specific sensor X axis error mitigation
	extern float accel_angle_error_y;							//Specific sensor Y axis error mitigation
	extern float accel_angle_error_z;							//Specific sensor Z axis error mitigation (not used in 6DOF but disposed for 9DOF future IMU)

	extern float   gyro_stab_angle_x;								//X axis gyro value for stabilized flight mode
	extern float   gyro_stab_angle_y;								//Y axis gyro value for stabilized flight mode

	extern float   acceleration_total_vector;						//Total acceleration vector from all accelerometer components

	extern float   accel_stab_angle_x;								//X axis accelerometer value for stabilized flight mode
	extern float   accel_stab_angle_y;								//Y axis accelerometer value for stabilized flight mode
	extern float   accel_stab_angle_z;								//Z axis accelerometer value for stabilized flight mode (not used in 6DOF but disposed for 9DOF future IMU)

	extern float   total_stab_angle_x;
	extern float   total_stab_angle_y;

	extern float   kp_pitch;									//Proportional multiplicative constant for pitch
	extern float   ki_pitch;									//Integral     multiplicative constant for pitch
	extern float   kd_pitch;									//Derivative   multiplicative constant for pitch

	extern float   kp_roll;									    //Proportional multiplicative constant for roll
	extern float   ki_roll;									    //Integral     multiplicative constant for roll
	extern float   kd_roll;									    //Derivative   multiplicative constant for roll

	extern float   kp_yaw;									    //Proportional multiplicative constant for yaw
	extern float   ki_yaw;									    //Integral     multiplicative constant for yaw
	extern float   kd_yaw;									    //Derivative   multiplicative constant for yaw

	extern float error_x;										//Current error for X axis
	extern float error_y;										//Current error for Y axis
	extern float error_z;										//Current error for Z axis

	extern float previous_error_x;						    //Current error for X axis
	extern float previous_error_y;							//Current error for X axis
	extern float previous_error_z;							//Current error for X axis

	extern float pid_p_x;										//Proportional controller response for X axis
	extern float pid_i_x;										//Integral     controller response for X axis
	extern float pid_d_x;										//Derivative   controller response for X axis

	extern float pid_p_y;										//Proportional controller response for Y axis
	extern float pid_i_y;										//Integral     controller response for Y axis
	extern float pid_d_y;										//Derivative   controller response for Y axis

	extern float pid_p_z;										//Proportional controller response for Z axis
	extern float pid_i_z;										//Integral     controller response for Z axis
	extern float pid_d_z;										//Derivative   controller response for Z axis

	extern float total_pid_x;									//Total PID controller output for X axis
	extern float total_pid_y;									//Total PID controller output for Y axis
	extern float total_pid_z;									//Total PID controller output for Z axis

	extern float desired_angle_rate_x;						//Filtered input signal from user radio for pitch
	extern float desired_angle_rate_y;						//Filtered input signal from user radio for roll
	extern float desired_angle_rate_z;						//Filtered input signal from user radio for yaw


#endif /* COMMON_H_ */
