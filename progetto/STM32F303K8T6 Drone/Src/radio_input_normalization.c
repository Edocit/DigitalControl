/*
 * radio_input_normalization.c
 *
 *  Created on: 30 mag 2019
 *      Author: edoardo
 */

#include "main.h"
#include "pid.h"
#include "common.h"
#include "config.h"


void map_radio_channels_to_angle_rates(){

    desired_angle_rate_x = 0;  //default value for value included in the deadband

    if( radio_channel_pulse[PITCH] > DEADBAND_UPPER_BOUND ){ desired_angle_rate_x = ( radio_channel_pulse[PITCH] - DEADBAND_UPPER_BOUND ) * SW_PITCH_REVERSE; }         //X axis value -- pitch
    if( radio_channel_pulse[PITCH] < DEADBAND_LOWER_BOUND ){ desired_angle_rate_x = ( radio_channel_pulse[PITCH] - DEADBAND_LOWER_BOUND ) * SW_PITCH_REVERSE; }         //X axis value -- pitch

    if( flight_mode == STABILIZED ){
    	desired_angle_rate_x = desired_angle_rate_x - (gyro_stab_angle_x * angle_correction);
    	if( desired_angle_rate_x > DEADBAND_LOWER_BOUND && desired_angle_rate_x < DEADBAND_UPPER_BOUND){ desired_angle_rate_x = 0; }
    }

    desired_angle_rate_x = desired_angle_rate_x / SETPOINT_PITCH_SPEED_RATE;



    desired_angle_rate_y = 0;  //default value for value included in the deadband

    if( radio_channel_pulse[ROLL] > DEADBAND_UPPER_BOUND ){ desired_angle_rate_y = ( radio_channel_pulse[ROLL] - DEADBAND_UPPER_BOUND ) * SW_ROLL_REVERSE; }            //Y axis value -- roll
    if( radio_channel_pulse[ROLL] < DEADBAND_LOWER_BOUND ){ desired_angle_rate_y = ( radio_channel_pulse[ROLL] - DEADBAND_LOWER_BOUND ) * SW_ROLL_REVERSE; }            //Y axis value -- roll

    if( flight_mode == STABILIZED ){
    	desired_angle_rate_y = desired_angle_rate_y - (gyro_stab_angle_y * angle_correction);
    	if( desired_angle_rate_y > DEADBAND_LOWER_BOUND && desired_angle_rate_y < DEADBAND_UPPER_BOUND){ desired_angle_rate_y = 0; }
    }

    desired_angle_rate_y = desired_angle_rate_y / SETPOINT_ROLL_SPEED_RATE;



    desired_angle_rate_z = 0;  //default value for value included in the deadband

    if(radio_channel_pulse[THROTTLE] > THROTTLE_LOWER_THD){
      if( radio_channel_pulse[YAW] > DEADBAND_UPPER_BOUND ){ desired_angle_rate_z = ( radio_channel_pulse[YAW] - DEADBAND_UPPER_BOUND ) * SW_YAW_REVERSE; }            //Z axis value -- yaw
      if( radio_channel_pulse[YAW] < DEADBAND_LOWER_BOUND ){ desired_angle_rate_z = ( radio_channel_pulse[YAW] - DEADBAND_LOWER_BOUND ) * SW_YAW_REVERSE; }            //Z axis value -- yaw
    }

    desired_angle_rate_z = desired_angle_rate_z / SETPOINT_YAW_SPEED_RATE;


  }








