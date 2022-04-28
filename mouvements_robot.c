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
#include <audio_processing.h>

#define RAYON_CERCLE 400 // On considère un cercle de 40 cm de rayon
#define LIMITE_DISTANCE 5 // On veut que le robot s'arrête à 5 mm du bord du cerle

static uint16_t KP_centre=10;
static uint16_t KI_centre=4;
static uint16_t KP_bords=10;
static uint16_t KI_bords=4;

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}


int32_t pi_regulator(uint16_t Kp, uint16_t Ki ){

 int16_t error = 0;
 int32_t speed = 0;

 static float sum_error = 0;

 error = mise_a_jour_distance_actuelle();
 //chprintf((BaseSequentialStream*)&SD3," erreur= %d\n", error);

 sum_error += error;
 speed = Kp*error;
 //chprintf((BaseSequentialStream*)&SD3," vitessepi= %d\n", speed);
 //speed = Kp * error + Ki * sum_error;

 return speed;

}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

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
        		speed = pi_regulator(KP_bords, KI_bords);
        		//speed = 0;

        	}
        }else{
        	if(distance_a_parcourir==0){
        		speed=0;

        	}else{
        		speed = pi_regulator(KP_centre, KI_centre);
        		//speed=0;

        	}
        }

        //chprintf((BaseSequentialStream*)&SD3,"vitesse = %d\n", speed);
        //chprintf((BaseSequentialStream*)&SD3," dist= %d\n", distance_a_parcourir);

        //applies the speed from the PI regulator
       //right_motor_set_speed(speed);
	   //left_motor_set_speed(speed);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}






