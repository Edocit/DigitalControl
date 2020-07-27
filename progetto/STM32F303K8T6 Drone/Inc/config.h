
#ifndef CONFIG_H_
#define CONFIG_H_



#define	KP_PITCH  0.9//0.55
#define KI_PITCH  0.002//0.01
#define	KD_PITCH  0.0

#define	KP_ROLL  0.9//0.55
#define KI_ROLL  0.002//0.01
#define	KD_ROLL  0.0

#define	KP_YAW   4.0
#define	KI_YAW   0.002
#define	KD_YAW   0.0

#define DEADBAND_UPPER_BOUND           1508 //DO NOT MODIFY upper bound of the middle position where the radio signal is evaluated as 0 dps
#define DEADBAND_LOWER_BOUND           1492 //DO NOT MODIFY lower bound of the middle position where the radio signal is evaluated as 0 dps

#define THROTTLE_LOWER_THD      	   1011 //DO NOT MODIFY under this value arm/disarm commands are enabled

#define BOUNDING_ANGLE ANGLE_CORRECTION_35

#define ALPHA_VALUE 	   0.9996 //gyro gain on complementary filter for stabilized mode

#define SETPOINT_PITCH_SPEED_RATE       3.0 //ex for value 3   -->   (500 - 8) / 3.0 = 164 deg/sec
#define SETPOINT_ROLL_SPEED_RATE        3.0 //ex for value 3   -->   (500 - 8) / 3.0 = 164 deg/sec
#define SETPOINT_YAW_SPEED_RATE         3.0 //ex for value 3   -->   (500 - 8) / 3.0 = 164 deg/sec

#endif
