/*
 * mouvements_robot.h
 *
 *  Created on: 23 avr. 2022
 *      Author: tbous
 */

#ifndef MOUVEMENTS_ROBOT_H_
#define MOUVEMENTS_ROBOT_H_


/*
 *  pi_regulator_long: allows you to determine a forward or backward speed of the robot
 */
int32_t pi_regulator(void);

/*
 * movements_start : allows the activation of the PRegulator thread
 */
void movements_start(void);



#endif /* MOUVEMENTS_ROBOT_H_ */
