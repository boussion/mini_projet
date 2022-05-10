/*
 * detection_bords.h
 *
 *  Created on: 22 avr. 2022
 *      Author: tbous
*/

#ifndef DETECTION_BORDS_H_
#define DETECTION_BORDS_H_

//Sets LEDS to ON
void set_front_leds(void);

/*
 * adjustement_dist: Used to return the adjusted distance given by sensor
*/
uint16_t adjustement_dist(void);

/*
 * edge_distance: Update the remaining distance to the edges and return it
*/
uint16_t edge_distance(void);

/*
 *  centre_distance: Update the distance to the centre and return it
*/
int16_t centre_distance(void);

/*
 * update_distance: Updates the remaining distance based on the sound detection and returns it
*/
int16_t update_distance(void);



#endif /* DETECTION_BORDS_H_ */
