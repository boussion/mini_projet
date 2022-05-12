/*
 * mouvements_robot.h
 *
 *  Created on: 23 avr. 2022
 *      Author: tbous
 */

#ifndef MOUVEMENTS_ROBOT_H_
#define MOUVEMENTS_ROBOT_H_


/*
 * @brief Allows you to determine a forward or backward speed of the robot
 *
 * @return int value that represents the speed
 */
int32_t p_regulator(void);

/*
 * @brief Allows the coordination of all movements performed by the robot
 */
void movements_start(void);

/*
 * @brief Allows the correction of the robot's position when it returns after detecting a line so that it is centred
 */
void correction(void);

/*
 * @brief Updating the static variable center_position to 0 if the robot was at proximity form the edges
 */
void center(void);



#endif /* MOUVEMENTS_ROBOT_H_ */
