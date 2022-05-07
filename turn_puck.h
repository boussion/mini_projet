/*
 * turn_puck.h
 *
 *  Created on: 7 May 2022
 *      Author: louis
 */

#ifndef TURN_PUCK_H_
#define TURN_PUCK_H_

#define POSITION_NOT_REACHED	0
#define POSITION_REACHED       	1

void motor_set_position(float position_r, float position_l, float speed_r, float speed_l);
void turn_puck(float deg);
uint8_t motor_position_reached(void);




#endif /* TURN_PUCK_H_ */
