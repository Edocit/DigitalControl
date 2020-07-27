
#include "main.h"
#include "setup.h"
#include "pid.h"
#include "common.h"
#include "config.h"






void configurable_pars_initialization(){

	HAL_GPIO_WritePin(GPIOB,WRN_PIN,SET);
	HAL_GPIO_WritePin(GPIOB,CAL_PIN,SET);

////////////////////////////////////////////////QUADCOPTER SPECIFIC PARS//////////////////////////////////////////////////
	drone_config     = QUADCOPTER;																						//
	flight_mode      = ACRO;																						    //
	controller_speed = LOOP_490HZ; //set this as primal loop speed, if lower than 1KHz will be kept in all flight modes //
	angle_correction = ANGLE_CORRECTION_35;										    									//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	if(flight_mode == STABILIZED && controller_speed < LOOP_1KHZ){ controller_speed = LOOP_1KHZ; }

//////////////PID PARAMETERS///////////////
	kp_pitch = KP_PITCH;                 //
	ki_pitch = KI_PITCH;                 //
	kd_pitch = KD_PITCH;                 //
									     //
	kp_roll  = KP_ROLL;                  //
	ki_roll  = KI_ROLL;                  //
	kd_roll  = KD_ROLL;                  //
									     //
	kp_yaw  = KP_YAW;                    //
	ki_yaw  = KI_YAW;                    //
	kd_yaw  = KD_YAW;                    //
///////////////////////////////////////////


}




void not_configurable_pars_initialization(){

	  float control_speed_in_hz = 0.0;

	  HAL_TIM_Base_Start(&htim1);				//Inizialization of TIM1 as free-running counter
	  HAL_TIM_Base_Start(&htim3);				//Inizialization of TIM3 as free-running counter

      HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	  TIM1->CCR1 = 1000;
	  TIM1->CCR2 = 1000;
	  TIM1->CCR3 = 1000;
	  TIM1->CCR4 = 1000;
	  HAL_Delay(3000);

	  CH_1_PREV_STATE = 0;
	  CH_2_PREV_STATE = 0;
	  CH_3_PREV_STATE = 0;
	  CH_4_PREV_STATE = 0;

	  counter_rising_edge = 0;
	  counter_falling_edge = 0;
	  global_input_count = 0;

	  cpu_load = 0;

      accel_raw_x = 0;
	  accel_raw_y = 0;
	  accel_raw_z = 0;

	  accel_angle_error_x = -1.8;
	  accel_angle_error_y = -2.0;
	  accel_angle_error_z = 0.0;

	  gyro_stab_angle_x = 0.0;
	  gyro_stab_angle_y = 0.0;

	  acceleration_total_vector = 0.0;

	  accel_stab_angle_x = 0.0;
	  accel_stab_angle_y = 0.0;

	  gyro_angle_x = 0.0;
	  gyro_angle_y = 0.0;
	  gyro_angle_z = 0.0;

	  gyro_raw_x = 0;
	  gyro_raw_y = 0;
	  gyro_raw_z = 0;

	  gyro_raw_error_x = 0;
	  gyro_raw_error_y = 0;
	  gyro_raw_error_z = 0;

	  total_stab_angle_x = 0.0;
	  total_stab_angle_y = 0.0;

	  error_x = 0;
	  error_y = 0;
	  error_z = 0;

	  previous_error_x = 0;
	  previous_error_y = 0;
	  previous_error_z = 0;

	  pid_p_x = 0.0;
	  pid_i_x = 0.0;
	  pid_d_x = 0.0;

	  pid_p_y = 0.0;
	  pid_i_y = 0.0;
	  pid_d_y = 0.0;

	  pid_p_z = 0.0;
	  pid_i_z = 0.0;
	  pid_d_z = 0.0;

	  total_pid_x = 0.0;
	  total_pid_y = 0.0;
	  total_pid_z = 0.0;

	  gyro_pitch_input = 0;
	  gyro_roll_input  = 0;
	  gyro_yaw_input   = 0;

	  desired_angle_rate_x = 0;
	  desired_angle_rate_y = 0;
	  desired_angle_rate_z = 0;

	  i = 0;

	  armed                = false;
	  throttle             = 0;

	  default_controller_speed = controller_speed;  //just to simplify and make more readable the formula below

	  if(default_controller_speed <= LOOP_1KHZ){	//if controller speed is faster or equal to the max MPU-6050 full gyro and accel speed (1 KHz)
		  traveled_angle_multiplier = 1.0 / 1000.0;
		  yaw_load_transfer_multiplier = traveled_angle_multiplier * DEG_TO_RAD;
	  }
	  else{	//if speed is slower than MPU-6050 full register operation keep that speed for integration
		  control_speed_in_hz = 1.0 / ((float)default_controller_speed / 1000000.0); // <-- division by 1000000.0 to convert us in s
		  traveled_angle_multiplier = 1.0 / control_speed_in_hz;
		  yaw_load_transfer_multiplier = traveled_angle_multiplier * DEG_TO_RAD;
	  }

		HAL_GPIO_WritePin(GPIOB,WRN_PIN,RESET);
		HAL_GPIO_WritePin(GPIOB,CAL_PIN,RESET);
}





void check_for_motor_arm(){

	if( radio_channel_pulse[THROTTLE] <= 1050 && radio_channel_pulse[YAW] >= 1990 ){
	  //add small delay
	  if( radio_channel_pulse[THROTTLE] <= 1050 && radio_channel_pulse[YAW] >= 1990 ){
		reset_pid();
		armed = true;
	  }
	}

	if( radio_channel_pulse[THROTTLE] <= 1050 && radio_channel_pulse[THROTTLE] >= 955 && radio_channel_pulse[YAW] <= 1010 ){
		//add small delay
	  if( radio_channel_pulse[THROTTLE] <= 1050 && radio_channel_pulse[YAW] <= 1010 ){
		armed = false;
	  }
	}

}




void check_for_flight_mode(){

	if(armed == false ){

		int mode = 0; //da togliere poi

		if(mode >= ACRO_LB_SWITCH && mode <= ACRO_UB_SWITCH){
			flight_mode = ACRO;
			if(controller_speed == LOOP_1KHZ && default_controller_speed != LOOP_1KHZ){	//if control speed is lower than 1KHz that speed is preserved for all modes
				controller_speed = default_controller_speed;			//need to know previous speed to bring back the control loop as the same of original config
			}
		}

		if(mode >= STABILIZED_LB_SWITCH && mode <= STABILIZED_UB_SWITCH){
			flight_mode = STABILIZED;
			if(controller_speed < LOOP_1KHZ ){  //if control speed is lower than 1KHz that speed is preserved for all modes
				controller_speed = LOOP_1KHZ;	//this speed is given by MPU-6050 datasheet... when add accelerometer we can have new data at max 1KHz
			}
		}

	}

}


