/*
 * mouvements_robots.c
 *
 *  Created on: 23 avr. 2022
 *      Author: tbous
 */

#include <motors.h>
#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <detection_bords.h>
#include <detection_ligne.h>
#include <audio_processing.h>

#define RAYON_CERCLE 400 // On considère un cercle de 40 cm de rayon
#define LIMITE_DISTANCE 5 // On veut que le robot s'arrête à 5 mm du bord du cerle
#define ANGLE_OPPOSITION_BRUIT 360 //angle de 360 degres
#define VARIATIONS_ANGLE 20 // on veut un nagle à +ou- 20 degres près
#define VITESSE_ROTATION 50
static uint16_t KP_centre=10;

static uint16_t KP_bords=10;

static uint16_t Ki=0.7;

static int16_t angle_bruit=0;

static uint16_t activation_rotation=0; //dépend d'une fonction qui donnera l'angle avec le bruit qui devra etre dans une range de 180 avec le bruit + si bruti detecté


/* pi_regulator: Permet de détermienr la vitesse du robot e-puck
 Paramètres :
 *	float *data	  Buffer contenant 1024 échantillons symétriques, soit 2*512 échantillons. Il correspond à un des 4 microphones.
 */
int32_t p_regulator(uint16_t Kp){

 int16_t error = 0;
 int32_t speed = 0;

 static int16_t somme_erreur = 0;

 error = mise_a_jour_distance_actuelle();
 somme_erreur=error+somme_erreur;
 speed = Kp*error + Ki*somme_erreur;

 return speed;

}


int32_t p_regulator_line(uint16_t Kp){

	 int16_t error = 0;
	 int32_t speed = 0;

	 static int16_t somme_erreur = 0;


	 error = get_line_position_to_center_mm();

	 //introduire une range pour éviter les beugs sur la rotation :

	 somme_erreur=error+somme_erreur;
	 speed = Kp*error + Ki*somme_erreur;
	 //chprintf((BaseSequentialStream*)&SD3,"erreur = %d\n", somme_erreur);

	 return speed;



}



static THD_WORKING_AREA(waPRegulator, 256);
static THD_FUNCTION(PRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    int32_t speed = 0;
    systime_t time;


    while(1){

    	time = chVTGetSystemTime();
        int16_t distance_a_parcourir = mise_a_jour_distance_actuelle();

        if(detection_son()==1){

        	if(distance_a_parcourir==0){
        		speed=0;

        	}else{
        		speed = p_regulator(KP_bords);
        	}
        }else{
        	if(distance_a_parcourir==0){
        		speed=0;

        	}else{
        		speed = p_regulator(KP_centre);

        	}
        }

        //applies the speed from the PI regulator
     //right_motor_set_speed(speed);
	 //left_motor_set_speed(speed);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));

    }
}

void p_regulator_start(void){
	chThdCreateStatic(waPRegulator, sizeof(waPRegulator), NORMALPRIO, PRegulator, NULL);
}

/*
 * range_angle_de_rotation :
 */
bool range_angle_rotation(void){

	uint16_t inside_range=0;

	if( (angle_bruit>= (ANGLE_OPPOSITION_BRUIT-VARIATIONS_ANGLE)) && (angle_bruit<=(ANGLE_OPPOSITION_BRUIT+VARIATIONS_ANGLE)) ){
		inside_range=1;
	}else{
		inside_range=0;
	}

	return inside_range;
}


/*
 * detection_rotation : donner un signal qui permet d'activer ou desactiver la rotation
 */

bool detection_rotation(void){

	if(detection_son()!=0){
		activation_rotation=0;

	}else{

		if( range_angle_rotation()==1 ){
				 activation_rotation=0;

		}else{
			 	 activation_rotation=1;

		}

		return activation_rotation;
	}
	return activation_rotation;
}

/*
 * rotation : active les moteur en rotation vers la droite pour linstant jusqu'à ce que l'angle avec le bruit soit de 180 degrés
 * + à rajouetr dans la thread avce une condition sur le signal d'activation pour bloquer le Kp
 */
void rotation(void){

	if( detection_rotation()==1 ){

		while (range_angle_rotation()!=0){

			left_motor_set_speed(VITESSE_ROTATION); // si positif > 0 rotation gauche / si negatif < 0 rotation droite
			right_motor_set_speed(VITESSE_ROTATION);
		}
	}
}

