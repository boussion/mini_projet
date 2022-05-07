#include <motors.h>
#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <detection_bords.h>
#include <audio_processing.h>
#include <turn_puck.h>

#include <stdlib.h>
#include <stdint.h>
#include <stm32f4xx.h>
#define TIMER_CLOCK 84000000
#define TIMER_FREQ 100000 // [Hz]
#define NSTEP_ONE_TURN 1000 // number of step for 1 turn of the motor
#define NSTEP_ONE_EL_TURN 4 //number of steps to do 1 electrical turn
#define NB_OF_PHASES 4 //number of phases of the motors
#define WHEEL_PERIMETER 13 // [cm]
#define MOTOR_RIGHT_A GPIOE, 13
#define MOTOR_RIGHT_B GPIOE, 12
#define MOTOR_RIGHT_C GPIOE, 14
#define MOTOR_RIGHT_D GPIOE, 15
#define MOTOR_LEFT_A GPIOE, 9
#define MOTOR_LEFT_B GPIOE, 8
#define MOTOR_LEFT_C GPIOE, 11
#define MOTOR_LEFT_D GPIOE, 10
#define MOTOR_RIGHT_TIMER TIM6
#define MOTOR_RIGHT_TIMER_EN RCC_APB1ENR_TIM6EN
#define MOTOR_RIGHT_IRQHandler TIM6_DAC_IRQHandler
#define MOTOR_RIGHT_IRQ TIM6_DAC_IRQn
#define MOTOR_LEFT_TIMER TIM7
#define MOTOR_LEFT_TIMER_EN RCC_APB1ENR_TIM7EN
#define MOTOR_LEFT_IRQ TIM7_IRQn
#define MOTOR_LEFT_IRQHandler TIM7_IRQHandler
#define SPEED_CONTROL 0
#define POSITION_CONTROL 1
#define PI                  3.1415926536f
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)

//some static global variables
static int16_t right_speed = 0; // in [step/s]
static int16_t left_speed = 0; // in [step/s]
static int16_t counter_step_right = 0; // in [step]
static int16_t counter_step_left = 0; // in [step]
static int16_t position_to_reach_right = 0; // in [step]
static int16_t position_to_reach_left = 0; // in [step]
static uint8_t position_right_reached = 0;
static uint8_t position_left_reached = 0;
static uint8_t state_motor = 0;
//tables containing the steps for the motors
static const uint8_t step_halt[NB_OF_PHASES] = {0, 0, 0, 0};

static const uint8_t step_table[NSTEP_ONE_EL_TURN][NB_OF_PHASES] = {
{1, 0, 1, 0},
{0, 1, 1, 0},
{0, 1, 0, 1},
{1, 0, 0, 1},
};


void turn_puck(float deg){
	motor_set_position(PERIMETER_EPUCK*deg/360, PERIMETER_EPUCK*deg/360, -5, 5);
	while(motor_position_reached() != POSITION_REACHED);
}

uint8_t motor_position_reached(void)
{
    if(state_motor == POSITION_CONTROL && position_right_reached && position_left_reached){
        return POSITION_REACHED;
    }else{
        return POSITION_NOT_REACHED;
    }
}

void motor_set_position(float position_r, float position_l, float speed_r, float speed_l)
{
	//reinit global variable
	counter_step_left = 0;
	counter_step_right = 0;

    position_right_reached = 0;
    position_left_reached = 0;

	//Set global variable with position to reach in step
	position_to_reach_left = position_l * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	position_to_reach_right = -position_r * NSTEP_ONE_TURN / WHEEL_PERIMETER;

	motor_set_speed(speed_r, speed_l);

	//flag for position control, will erase flag for speed control only
	state_motor = POSITION_CONTROL;
}














