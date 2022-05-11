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
#include <locate_sound.h>
#include "sensors/VL53L0X/VL53L0X.h"



#define ROTATION_THRESHOLD 50
#define ERROR_THRESHOLD  10
#define ERROR_THRESHOLD_SOUND 2
#define ROTATION_COEFF_LINE 0.5
#define ROTATION_COEFF_SOUND 11
#define IMAGE_BUFFER_SIZE 640
#define MOTOR_SPEED_LIMIT 1100 // [step/s]
#define KP					3
#define KP_RETURN 			6
#define MAX_SUM_ERROR 	(MOTOR_SPEED_LIMIT/KI)
#define MAX_ERROR		(MOTOR_SPEED_LIMIT/KP)
#define DIST_ROTATION	1300

static uint16_t center_position=0;// equal to 1 when the center position has been actuated

/*
 *  p_regulator: allows to determine a forward or backward speed of the robot
 * 	modified to only work with update_distance
 */
int32_t p_regulator(void){

	//The error takes the value of update_distance()
	int16_t error = 0;
	float speed = 0;
	static int16_t sum_error = 0;
	int8_t Kp =0;

	if(sound_detection()==1){
		Kp=KP;
	}else{
		Kp=KP_RETURN;
	}

	error = update_distance();
	sum_error += error;

	//speed calculation


	speed = Kp * error;
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
 * correction: alignment of the robot with the smallest distance
 */
void correction(void){

	//not 0 because otherwise adjustment_dist will always be greater than distance_min
	uint16_t distance_min=400;

	//The robot is rotated continuously until it makes DIST_ROTATION corresponding to one revolution.
	do{
		// update of the minimum distance if the one captured is smaller
		if(adjustement_dist()<=distance_min){
			distance_min=adjustement_dist();
		}
		left_motor_set_speed(-100);
		right_motor_set_speed(100);

	}while(abs(left_motor_get_pos())<=DIST_ROTATION);

	//The robot is rotated continously until it find the min distance fin previously
	do{
		left_motor_set_speed(100);
		right_motor_set_speed(-100);
	}while(adjustement_dist()>=distance_min+5 || adjustement_dist()<=distance_min-5);
	right_motor_set_speed(0);
	left_motor_set_speed(0);

	//update the static variable center_position
	if((adjustement_dist()>=distance_min+5 || adjustement_dist()<=distance_min-5)){
		center_position=1;
	}else{
		center_position=0;
	}
	*/
}

/*
 * center: set the static variable center_position to 0 if one sound is detected
 */
void center(void){

	if(sound_detection()==1){
		center_position=0;
	}
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
    	    int16_t speed_correction_line = 0;
    	    int16_t speed_correction_sound = 0;

    	    while(1){

    	    	//update of the static variable center_position
    	    	center();

    	    	//We recover the time of the system
    	        time = chVTGetSystemTime();

    	       // chprintf((BaseSequentialStream*)&SD3,"Distance  = %ld", left_motor_get_pos());

    	        //set_front_led(1);
    	        //computes the speed to give to the motors using the pi-regulator
    	        speed = p_regulator();

    	        //speed correction according to the sound direction : sound must be detected + adjustement in proximity of the center

    	        if(sound_detection()==1  && (adjustement_dist()>=120)){
    	        	speed_correction_sound = (int16_t)get_last_direction();
    	        }else{
    	        	speed_correction_sound = 0;
    	        }
    	        //range for the sound speed correction
    	        if((speed_correction_sound>=-ERROR_THRESHOLD_SOUND) && (speed_correction_sound<=ERROR_THRESHOLD_SOUND)){
    	        	speed_correction_sound=0;
    	        }

    	        //speed correction according to the position of the line
    	       // speed_correction_line = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

    	        //line speed correction : correction miuts be in a certain range + no correction in center (image is not enough precise)
    	        if((abs(speed_correction_line) < ROTATION_THRESHOLD) || (adjustement_dist()>=160) || (update_distance()<=0)){

    	        		//(adjustement_dist()<=0)){
    	        	speed_correction_line = 0;
    	        }
    	       //The correction is put in place: when the robot is at 150 mm minimum from the boards +position in the center not already updated
    	        if(sound_detection()==0 && (adjustement_dist()>=150) && center_position==0){
    	        	correction();

    	        }

    	        //corrections + speed => in the motors
    	     right_motor_set_speed( speed - ROTATION_COEFF_LINE * speed_correction_line+ ROTATION_COEFF_SOUND * speed_correction_sound);
    	     left_motor_set_speed( speed + ROTATION_COEFF_LINE * speed_correction_line - ROTATION_COEFF_SOUND * speed_correction_sound);

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


