#ifndef CONFIG_H_
#define CONFIG_H_

	#include "common.h"

	#define KP_PITCH 1.00
	#define KI_PITCH 0.002
	#define KD_PITCH 0.00

	#define KP_ROLL 1.00
	#define KI_ROLL 0.002
	#define KD_ROLL 0.00

	#define KP_YAW 2.00
	#define KI_YAW 0.02
	#define KD_YAW 0.00

	#define DEADBAND_UPPER_BOUND 1508
	#define DEADBAND_LOWER_BOUND 1492

	#define ALPHA_VALUE 0.998

	#define BOUNDING_ANGLE ANGLE_CORRECTION_10
	#define CONTROLLER_SPEED LOOP_250HZ
	#define SETPOINT_PITCH_SPEED_RATE 3.0
	#define SETPOINT_ROLL_SPEED_RATE 3.0
	#define SETPOINT_YAW_SPEED_RATE 3.0



#endif