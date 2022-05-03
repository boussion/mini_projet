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

#include <audio_processing.h>
#include <communications.h>
#include <arm_math.h>

// Constantes :
#define LIMITE_DETECTION 65536 // Maximum de uint16_t => 65536
#define RAYON_CERCLE 200 // On considère un cercle de 40 cm de rayon
#define LIMITE_DISTANCE 20 // On veut que le robot s'arrête à 5 mm du bord du cerle
#define ERREUR_POSSIBLE 20


static int16_t distance_a_parcourir; // Distance à parcourir
static int16_t distance_aux_bords; // Distance à parcourir jusqu'aux bords
static int16_t distance_au_centre; // Distance à parcourir pour retourner au centre


/* ajustement_dist: Permet d'ajuster les valeurs données par le capteur de distance
 */

uint16_t ajustement_dist(void) {

	uint16_t dist_mm = VL53L0X_get_dist_mm() - 48; // Correction erreur

	if( (dist_mm>=(LIMITE_DETECTION-6)) && (dist_mm<=LIMITE_DETECTION)){ // Le detecteur a une erreur de + ou -2 mm
		dist_mm=0;
	}

	return dist_mm;

}


/* distance_son: Mets à jour la distance qui reste à parcourir pour atteindre les bords et la retourne
 */
uint16_t distance_bords(void){

	uint16_t dist = ajustement_dist();

		if((dist >= (LIMITE_DISTANCE-ERREUR_POSSIBLE)) && (dist <= (LIMITE_DISTANCE+ERREUR_POSSIBLE))){

			distance_aux_bords = 0;

		}else{

		distance_aux_bords = dist;

	}

	return distance_aux_bords;

}

/* distance_centre: Mets à jour la distance qui reste à parcourir pour atteindre le centre et la retourne
 */
uint16_t distance_centre(void){

	uint16_t dist = ajustement_dist();

		//if((dist <= (RAYON_CERCLE+ERREUR_POSSIBLE))){
		if((dist >= (RAYON_CERCLE-ERREUR_POSSIBLE)) && (dist <= (RAYON_CERCLE+ERREUR_POSSIBLE))){
			distance_au_centre = 0;

		}else{
		distance_au_centre = (dist-RAYON_CERCLE);
		//if (distance_au_centre<=0){
		//	distance_au_centre=0;
		//}

		}


	return distance_au_centre;

}

/* mise_a_jour_distance_actuelle: Mets à jour la distance qui reste à parcourir en fonction de la détection du son et la retourne
 */
int16_t mise_a_jour_distance_actuelle(void){

	distance_centre();

	distance_bords();

	if(detection_son()==1){

		distance_a_parcourir = distance_aux_bords;

	}else{

		distance_a_parcourir = distance_au_centre;

	}

	return distance_a_parcourir;

}












