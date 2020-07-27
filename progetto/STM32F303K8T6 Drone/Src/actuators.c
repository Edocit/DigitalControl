
#include "main.h"
#include "common.h"
#include "config.h"


/*To map
[A, B] --> [a, b] or [1000;2000] --> [130;250] or [1000;2000] --> [45;84]
the formula is this one
(val - A)*(b-a)/(B-A) + a */

void map_os_125(void){

	speedVec[MOTOR_1] = ( ( (speedVec[MOTOR_1] - 1000) * 130 ) / 1000 ) + 130; //it is a linear mapping
	speedVec[MOTOR_2] = ( ( (speedVec[MOTOR_2] - 1000) * 130 ) / 1000 ) + 130; //it is a linear mapping
	speedVec[MOTOR_3] = ( ( (speedVec[MOTOR_3] - 1000) * 130 ) / 1000 ) + 130; //it is a linear mapping
	speedVec[MOTOR_4] = ( ( (speedVec[MOTOR_4] - 1000) * 130 ) / 1000 ) + 130; //it is a linear mapping

}


void map_os_42(void){

	speedVec[MOTOR_1] = ( ( (speedVec[MOTOR_1] - 1000) * 45 ) / 1000 ) + 45; //it is a linear mapping
	speedVec[MOTOR_2] = ( ( (speedVec[MOTOR_2] - 1000) * 45 ) / 1000 ) + 45; //it is a linear mapping
	speedVec[MOTOR_3] = ( ( (speedVec[MOTOR_3] - 1000) * 45 ) / 1000 ) + 45; //it is a linear mapping
	speedVec[MOTOR_4] = ( ( (speedVec[MOTOR_4] - 1000) * 45 ) / 1000 ) + 45; //it is a linear mapping

}





void write_motors(void){

	TIM1->CCR1 = speedVec[MOTOR_3];
	TIM1->CCR2 = speedVec[MOTOR_1];
	TIM1->CCR3 = speedVec[MOTOR_2];
	TIM1->CCR4 = speedVec[MOTOR_4];

}


void write_base_pulse(){

	TIM1->CCR1 = THROTTLE_LOWER_THD;
	TIM1->CCR2 = THROTTLE_LOWER_THD;
	TIM1->CCR3 = THROTTLE_LOWER_THD;
	TIM1->CCR4 = THROTTLE_LOWER_THD;

}




void write_motors_pulse(){

    if(armed == true){
    	write_motors();
    }else{
    	write_base_pulse();
    }

}
