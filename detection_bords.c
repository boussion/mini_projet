/*
 * detection_bords.c
 *
 *  Created on: 21 avr. 2022
 *      Author: tbous
 */

#include "sensors/VL53L0X/VL53L0X.h"

#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <leds.h>

#include <audio_processing.h>
#include <arm_math.h>

#define LIMITE_DETECTION 65536 // maximum uint16_t => 65536
#define RAYON_CERCLE 200 // we consider a circle of radius 235 mm => 35 mm between the front and the center of the e-puck
#define LIMITE_DISTANCE 10 //
#define ERREUR_POSSIBLE 20 // error threshold
#define ERROR_EDGE      50
#define CORRECTION_FACTOR_1 0.8818
#define CORRECTION_FACTOR_2 37.559
#define LED_INTENSITY   	100

// Static values to represent the different distances:
static int16_t distance_to_travel;
static int16_t distance_to_edges;
static int16_t distance_from_center;

static uint8_t centrage=0;



//Sets leds
void set_front_leds(void){
		set_led(LED1, LED_INTENSITY);
        set_led(LED3, LED_INTENSITY);
        set_led(LED7, LED_INTENSITY);
}


/*
 * adjustement_dist: Used to return the adjusted distance given by sensor
*/
uint16_t adjustement_dist(void) {

	uint16_t dist_mm =(uint16_t)(VL53L0X_get_dist_mm()*CORRECTION_FACTOR_1 - CORRECTION_FACTOR_2) ; //correction of the error

	// avoid distance overflow due to sensor instability
	if( (dist_mm>=(LIMITE_DETECTION-6))){
		dist_mm=0;
	}
	return dist_mm;
}

/*
 * edge_distance: Update the remaining distance to the edges and return it
*/
int16_t edge_distance(void){

	int16_t dist = adjustement_dist();

	//if the distance is less than the stopping distance in front of the edges + error range taken into account => distance to travel =0
	if((dist <= (LIMITE_DISTANCE+ERROR_EDGE))){
		distance_to_edges = 0;
		set_front_leds(); //sets leds when the e puck is close to the wall
		centrage=1;

	}else{
		distance_to_edges = dist;
	}
	return distance_to_edges;
}

/*
 *  centre_distance: Update the distance to the centre and return it
*/
int16_t centre_distance(void){

	int16_t dist = adjustement_dist();

	//if the sensor is in the centre at more or less POSSIBLE_ERROR
	if((dist >= (RAYON_CERCLE-ERREUR_POSSIBLE)) && (dist <= (RAYON_CERCLE+ERREUR_POSSIBLE))){
		distance_from_center = 0;

	}else{
		distance_from_center = (dist-RAYON_CERCLE);
	}
	return distance_from_center;
}

/*
 * update_distance: Updates the remaining distance based on the sound detection and returns it
*/
int16_t update_distance(void){

	//update static values
	centre_distance();
	edge_distance();

	//if sound detected => robot reachs the edges
	if(sound_detection()==1){
		distance_to_travel = distance_to_edges;

	//if sound not detected => robot stays in the centre
	}else{
		distance_to_travel = distance_from_center;

	}
	return distance_to_travel;
}


bool get_centrage(void){

	return centrage;
}

void reset_centrage(void){
	centrage = 0;
}











