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

#define LIMITE_DETECTION 65536 // Maximum uint16_t => 65536
#define RAYON_CERCLE 200 // We consider a circle of radius 235 mm => 35 mm between the front and the center of the e-puck
#define LIMITE_DISTANCE 10 // Stop at 1 cm to the edge
#define ERREUR_POSSIBLE 20 // Error threshold
#define ERROR_EDGE      30 // Correction because of the problem with the Tof at small distance in ordre to stop the robot before 1 cm
#define CORRECTION_FACTOR_1 0.8818 // Correction factor nb1 of the Tof sensor values
#define CORRECTION_FACTOR_2 37.559 // Correction factor nb2 of the Tof sensor values
#define LED_INTENSITY   	100 // Intensity of the LEDS

static int16_t distance_to_travel; // store the final distance to travel in function of the sound
static int16_t distance_to_edges; // store the distance to the edge
static int16_t distance_from_center; // store the distance to the center

static uint8_t centrage=0;// value to indicates the proximity with the edges



void set_front_leds(void){
		set_led(LED1, LED_INTENSITY);
        set_led(LED3, LED_INTENSITY);
        set_led(LED7, LED_INTENSITY);
}


uint16_t adjustement_dist(void) {

	// adjustment of the distance given by the Tof
	uint16_t dist_mm =(uint16_t)(VL53L0X_get_dist_mm()*CORRECTION_FACTOR_1 - CORRECTION_FACTOR_2) ; //correction of the error

	// avoid distance overflow due to sensor instability
	if( (dist_mm>=(LIMITE_DETECTION-6))){
		dist_mm=0;
	}
	return dist_mm;
}


int16_t edge_distance(void){

	// recovery of the corrected value of the sensor
	int16_t dist = adjustement_dist();

	// if the distance is less than the stopping distance in front of the edges + error range taken into account => distance to travel =0
	if((dist <= (LIMITE_DISTANCE+ERROR_EDGE))){
		distance_to_edges = 0;
		set_front_leds(); // sets leds when the e puck is close to the wall
		centrage=1; // set the value of centrage

	}else{
		distance_to_edges = dist;
	}
	return distance_to_edges;
}


int16_t centre_distance(void){

	// recovery of the corrected value of the sensor
	int16_t dist = adjustement_dist();

	// if the sensor is in the centre at more or less POSSIBLE_ERROR
	if((dist >= (RAYON_CERCLE-ERREUR_POSSIBLE)) && (dist <= (RAYON_CERCLE+ERREUR_POSSIBLE))){
		distance_from_center = 0;

	}else{
		// calculation of the distance to the centre
		distance_from_center = (dist-RAYON_CERCLE);
	}
	return distance_from_center;
}


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


int get_centrage(void){
	return centrage;
}


void reset_centrage(void){
	centrage = 0;
}










