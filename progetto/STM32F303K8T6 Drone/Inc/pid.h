/*
 * controller.h
 *
 *  Created on: 25 mag 2019
 *      Author: edoardo
 */

#ifndef PID_H_
#define PID_H_

void reset_pid(void);
void calculate_pid(void);
void synchr(void);
void get_cpu_load(void);
void wait_for_next_activation(void);


#endif /* PID_H_ */
