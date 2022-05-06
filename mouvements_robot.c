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

#define RAYON_CERCLE 470 // On considère un cercle de 40 cm de rayon
#define LIMITE_DISTANCE 5 // On veut que le robot s'arrête à 5 mm du bord du cerle
#define VARIATIONS_ANGLE 20 // on veut un nagle à +ou- 20 degres près
#define ROTATION_THRESHOLD 50
#define ERROR_THRESHOLD  10
#define ROTATION_COEFF 0.25
#define IMAGE_BUFFER_SIZE 640
#define MOTOR_SPEED_LIMIT 1100 // [step/s]
#define KP					5
#define KI 					0.5
#define MAX_SUM_ERROR 	(MOTOR_SPEED_LIMIT/KI)
#define MAX_ERROR		(MOTOR_SPEED_LIMIT/KP)




/*
 *  pi_regulator_long: allows you to determine a forward or backward speed of the robot
 */
int32_t pi_regulator_long(void){

	//Enter the value of the error that depends on the sound detection
	int16_t error = mise_a_jour_distance_actuelle();
	float speed = 0;
	static float sum_error = 0;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if(abs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

	return (int16_t)speed;
}





static THD_WORKING_AREA(waPRegulator, 256);
static THD_FUNCTION(PRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    	    systime_t time;

    	    int16_t speed = 0;
    	    int16_t speed_correction = 0;


    	    while(1){
    	        time = chVTGetSystemTime();



    	        //computes the speed to give to the motors
    	        //distance_cm is modified by the image processing thread
    	        speed = pi_regulator_long();


    	        //if(detection_son()==1){
    	        	speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));
    	       // }else{
    	        //	speed_correction=0;
    	       // }

    	        //computes a correction factor to let the robot rotate to be in front of the line


    	        //if the line is nearly in front of the camera, don't rotate
    	        if(abs(speed_correction) < ROTATION_THRESHOLD){
    	        	speed_correction = 0;
    	        }


    	       // chprintf((BaseSequentialStream*)&SD3,"correction = %ld", test);
    	        //chprintf((BaseSequentialStream*)&SD3,"position ligne = %ld", get_line_position());

				//applies the speed from the PI regulator and the correction for the rotation
				right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
				left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);


    		//chprintf((BaseSequentialStream*)&SD3,"error = %ld", distance_centre());
    		//chprintf((BaseSequentialStream*)&SD3,"capture = %ld", capture());


        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void movements_start(void){
	chThdCreateStatic(waPRegulator, sizeof(waPRegulator), NORMALPRIO, PRegulator, NULL);
}


