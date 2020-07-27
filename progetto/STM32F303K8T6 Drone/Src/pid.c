/*
 * controller.c
 *
 *  Created on: 25 mag 2019
 *      Author: edoardo
 */


#include "main.h"
#include "pid.h"
#include "common.h"


 void reset_pid(){

        pid_i_x = 0;				//integral component refresh because it is an over time value
        pid_i_y = 0;				//integral component refresh because it is an over time value
        pid_i_z = 0;				//integral component refresh because it is an over time value

        previous_error_x = 0;		//at the begin of a flight the derivative behavior is zero
        previous_error_y = 0;		//at the begin of a flight the derivative behavior is zero
        previous_error_z = 0;		//at the begin of a flight the derivative behavior is zero
 }




  void calculate_pid(){

  throttle = radio_channel_pulse[THROTTLE]; //assign throttle value to a variable in order to not disable interrupts

  error_x = gyro_pitch_input - desired_angle_rate_x;                  	   //input for rate/acro mode is the difference between gyro and radio normalized pulse
  error_y = gyro_roll_input  - desired_angle_rate_y;                       //input for rate/acro mode is the difference between gyro and radio normalized pulse
  error_z = gyro_yaw_input   - desired_angle_rate_z;                       //input for rate/acro mode is the difference between gyro and radio normalized pulse

  pid_p_x = kp_pitch * error_x;                                            //proportional value of PID controller for X
  pid_p_y = kp_roll  * error_y;                                            //proportional value of PID controller for Y
  pid_p_z = kp_yaw   * error_z;                                            //proportional value of PID controller for Z

  if(throttle > 1080){														   //avoid to integrate when on ground
	  pid_i_x = pid_i_x + (ki_pitch * error_x);                                //integral value of PID controller for X
	  pid_i_y = pid_i_y + (ki_roll  * error_y);                                //integral value of PID controller for Y
	  pid_i_z = pid_i_z + (ki_yaw   * error_z);                                //integral value of PID controller for Z
  }

  if(pid_i_x > PID_UPPER_BOUND){ pid_i_x = PID_UPPER_BOUND; }              //avoid extreme output of integral component
  if(pid_i_x < PID_LOWER_BOUND){ pid_i_x = PID_LOWER_BOUND; }              //avoid extreme output of integral component
  if(pid_i_y > PID_UPPER_BOUND){ pid_i_y = PID_UPPER_BOUND; }              //avoid extreme output of integral component
  if(pid_i_y < PID_LOWER_BOUND){ pid_i_y = PID_LOWER_BOUND; }              //avoid extreme output of integral component
  if(pid_i_z > PID_UPPER_BOUND){ pid_i_z = PID_UPPER_BOUND; }              //avoid extreme output of integral component
  if(pid_i_z < PID_LOWER_BOUND){ pid_i_z = PID_LOWER_BOUND; }              //avoid extreme output of integral component

  pid_d_x = kd_pitch * (error_x - previous_error_x);                       //derivative component acts on the speed of the error
  pid_d_y = kd_roll  * (error_y - previous_error_y);                       //derivative component acts on the speed of the error
  pid_d_z = kd_yaw   * (error_z - previous_error_z);                       //derivative component acts on the speed of the error

  total_pid_x = pid_p_x + pid_i_x + pid_d_x;                               //final PID for X axis
  total_pid_y = pid_p_y + pid_i_y + pid_d_y;                               //final PID for Y axis
  total_pid_z = pid_p_z + pid_i_z + pid_d_z;                               //final PID for Z axis

  if( total_pid_x < PID_LOWER_BOUND ){ total_pid_x = PID_LOWER_BOUND; }    //value normalization
  if( total_pid_x > PID_UPPER_BOUND ){ total_pid_x = PID_UPPER_BOUND; }    //value normalization
  if( total_pid_y < PID_LOWER_BOUND ){ total_pid_y = PID_LOWER_BOUND; }    //value normalization
  if( total_pid_y > PID_UPPER_BOUND ){ total_pid_y = PID_UPPER_BOUND; }    //value normalization
  if( total_pid_z < PID_LOWER_BOUND ){ total_pid_z = PID_LOWER_BOUND; }    //value normalization
  if( total_pid_z > PID_UPPER_BOUND ){ total_pid_z = PID_UPPER_BOUND; }    //value normalization

  if(    throttle > MAX_THROTTLE    ){ throttle = MAX_THROTTLE;  }         //ceil for PID controller at maximum throttle

  switch(drone_config){
	  case QUADCOPTER:
		  speedVec[MOTOR_1] = throttle + total_pid_y + total_pid_x + total_pid_z;  //rear  right motor PWM
		  speedVec[MOTOR_2] = throttle + total_pid_y - total_pid_x - total_pid_z;  //front right motor PWM
		  speedVec[MOTOR_3] = throttle - total_pid_y + total_pid_x - total_pid_z;  //rear   left motor PWM
		  speedVec[MOTOR_4] = throttle - total_pid_y - total_pid_x + total_pid_z;  //front  left motor PWM
		  break;
	  case HEXACOPTER:
		  //TODO add mix for hex and use a board with 2 timers or at least 6 channels
		  break;
	  case OCTACOPTER:
		  //TODO add mix for oct and use a board with 2 timers or at least 8 channels
		  break;
	  default:
		  break;
  }


  for(i = 0; i < MOTOR_N; i++){                                            //speed array normalization
    if(speedVec[i] < MIN_SPIN){ speedVec[i] = MIN_SPIN; }                  //if the value is less than the minimum
    if(speedVec[i] > 2000){ speedVec[i] = 2000; }                  //if the value is greater than the maximum
  }


  previous_error_x = error_x;                                              //actual error become previous error for the next cycle
  previous_error_y = error_y;                                              //actual error become previous error for the next cycle
  previous_error_z = error_z;                                              //actual error become previous error for the next cycle

}





//Lower bound for the controller initialized at the beginning of the control loop
void synchr(){
	__HAL_TIM_SET_COUNTER(&htim3, 0);
}


//Get how much of the control loop is used (it is good practice to leave room as well as we don't know the WCET)
void get_cpu_load(void){
	cpu_load = ( __HAL_TIM_GET_COUNTER(&htim3) * 100 ) / controller_speed;

	if(__HAL_TIM_GET_COUNTER(&htim3) > controller_speed){
		HAL_GPIO_WritePin(GPIOB, WRN_PIN, SET);
	}
}



//Upper bound for the controller handled at the end of the control loop and waits for the defined period to pass
void wait_for_next_activation(){
	while(__HAL_TIM_GET_COUNTER(&htim3) < controller_speed ){};
}






