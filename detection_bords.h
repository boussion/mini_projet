/*
 * detection_bords.h
 *
 *  Created on: 22 avr. 2022
 *      Author: tbous
*/

#ifndef DETECTION_BORDS_H_
#define DETECTION_BORDS_H_

/**
 * @brief  Set front leds
 */
void set_front_leds(void);

/**
 * @brief  Adjustement_dist: Used to return the adjusted distance given by sensor
 *
 * @return Unsigned int number which gives the corrected Tof distance
 */

uint16_t adjustement_dist(void);

/**
 * @brief  Edge_distance: Update the remaining distance to the edges and return it
 *
 * @return Unsigned int number which gives the distance to the edges
 */
uint16_t edge_distance(void);

/**
 * @brief  Center_distance: Update the remaining distance to the centre and return it
 *
 * @return  Int number which gives the distance to the center
 */
int16_t centre_distance(void);

/**
 * @brief  Update_distance: Updates the remaining distance based on the sound detection and returns it
 *
 * @return  Int number which gives the distance to travel in function of the sound
*/
int16_t update_distance(void);

/**
 * @brief  Indicates whether centrage is 1 or 0
 *
 * @return  Boolean variable worth 1 if centring = 1 or 0 if centring = 0
*/
int get_centrage(void);

/**
 * @brief  Reset the value of centrage to 0
 */
void reset_centrage(void);


#endif /* DETECTION_BORDS_H_ */
