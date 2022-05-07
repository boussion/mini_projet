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


#define ROTATION_THRESHOLD 50
#define ERROR_THRESHOLD  10
#define ROTATION_COEFF 0.25
#define IMAGE_BUFFER_SIZE 640
#define MOTOR_SPEED_LIMIT 1100 // [step/s]
#define KP					5
//#define KI 					0.5
#define MAX_SUM_ERROR 	(MOTOR_SPEED_LIMIT/KI)
#define MAX_ERROR		(MOTOR_SPEED_LIMIT/KP)


/*
 *  pi_regulator_long: allows to determine a forward or backward speed of the robot
 */
int32_t pi_regulator(void){

	//The error takes the value of update_distance()
	int16_t error = 0;
	float speed = 0;
	static int16_t sum_error = 0;

	/*
	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if(abs(error) < ERROR_THRESHOLD){
		return 0;
	}
	*/
	error = update_distance();
	sum_error += error;

	/*
	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}*/

	//speed calculation
	speed = KP * error;
			//+ KI * sum_error;
	if(speed > MAX_ERROR){
			sum_error = MAX_ERROR;
		}else if(sum_error < -MAX_ERROR){
			sum_error = -MAX_ERROR;
		}

	//chprintf((BaseSequentialStream*)&SD3,"somme = %ld", sum_error);
	//chprintf((BaseSequentialStream*)&SD3,"error = %ld", error);

	return (int16_t)speed;
}

/*
 * PRegulator : allows the update of the speed of the motors with the pi regulator
 */

static THD_WORKING_AREA(waPRegulator, 256);
static THD_FUNCTION(PRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;


    	    systime_t time;
    	    int16_t speed = 0;
    	    int16_t speed_correction = 0;

    	    while(1){

    	    	//We recover the time of the system
    	        time = chVTGetSystemTime();

    	        //computes the speed to give to the motors using the pi-regulator
    	        speed = pi_regulator();

    	        chprintf((BaseSequentialStream*)&SD3,"distance= %u", adjustement_dist());


    	        //speed correction according to the position of the line to let the robot rotate to be in front of the line
    	        speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));
    	        //speed_correction=0;

    	        //if the line is nearly in front of the camera, don't rotate
    	        //need to correct this value to allow the good recognition of the line
    	        if(abs(speed_correction) < ROTATION_THRESHOLD || adjustement_dist() <= 45 || adjustement_dist()>=180){
    	        	speed_correction = 0;
    	        }

				//applies the speed from the PI regulator and the correction for the rotation on the motors
				right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
				left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);

				//chprintf((BaseSequentialStream*)&SD3,"error = %ld", distance_centre());
				//chprintf((BaseSequentialStream*)&SD3,"capture = %ld", capture());

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

/*
 * movements_start : allows the activation of the PRegulator thread
 */
void movements_start(void){
	chThdCreateStatic(waPRegulator, sizeof(waPRegulator), NORMALPRIO, PRegulator, NULL);
}


