/*
 * shortest_edge.c
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

static bool turned;
static uint16_t form_dist;
static uint16_t new_dist;
static int16_t current_angle;


void set_turned(bool new_turn){
    turned = new_turn;
}


bool get_turned(void){
    return turned;
}

void turn_to(int angle, int dir){

    left_motor_set_pos(0);
    right_motor_set_pos(0);

    // The e-puck will pivot on itself
    left_motor_set_speed(dir*MOTOR_SPEED_LIMIT/2);
    right_motor_set_speed(-dir*MOTOR_SPEED_LIMIT/2);

    // Turns until the desired angle is reached
    while ((abs(left_motor_get_pos()) < abs((angle/FULL_PERIMETER_DEG)*NSTEP_ONE_TURN*CORRECTION_FACTOR))
    	&& (abs(right_motor_get_pos()) < abs((angle/FULL_PERIMETER_DEG)*NSTEP_ONE_TURN*CORRECTION_FACTOR))) {
	}
    halt();
}

void halt(void){

    left_motor_set_speed(0);
    right_motor_set_speed(0);
}

void find_shortest_edge(void){
    form_dist = adjustement_dist();
    current_angle = 0;
    while(current_angle<100){
    	turn_to(current_angle+5, 1);
    	new_dist=adjustement_dist();
    	if(form_dist>new_dist){
    		halt();
    		return;
    	}
    	else{
    		form_dist = new_dist;
    		current_angle = current_angle +5;
    	}
    }
    turn_to(100, -1);
    current_angle=0;
    while(current_angle<100){
    	turn_to(current_angle+5, -1);
    	new_dist=adjustement_dist();
    	if(form_dist>new_dist){
    		return;
    	}
    	else{
    		form_dist = new_dist;
    		current_angle = current_angle +5;
    	}
    }
    return;
}












