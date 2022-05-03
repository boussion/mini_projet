/*
 * mouvements_robot.h
 *
 *  Created on: 23 avr. 2022
 *      Author: tbous
 */

#ifndef MOUVEMENTS_ROBOT_H_
#define MOUVEMENTS_ROBOT_H_

int32_t p_regulator(uint16_t Kp);

void p_regulator_start(void);

bool detection_rotation(void);

void rotation(void);

bool range_angle_rotation(void);



#endif /* MOUVEMENTS_ROBOT_H_ */
