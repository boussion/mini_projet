#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <leds.h>

#include <motors.h>
#include <audio/microphone.h>
#include <epuck1x/a_d/advance_ad_scan/e_micro.h>
#include <arm_math.h>

#include <audio_processing.h>

#include <locate_sound.h>

#define TABLE_TO_FREQ	15.625	//Value to switch from table length to actual frequency
#define NB_MIC 3				//Numbers of Mics on the e puck



#define MAXIMUM_DELTA_T1 0.06 	//Distance between the front mics
#define MAXIMUM_DELTA_T2 0.05 	//Distance between the Front Mics and the back one
#define SPEED_SOUND 343			//Speed of Sound

static int max_freq; //Highest frequency, determined by audio_processing.c

static int bufferOutput[NB_MIC][2];	//Array to store the mic values

void reassign_table(struct Mic_Record stored_mic){
	bufferOutput[0][0]=stored_mic.Mic0real;
	bufferOutput[0][1]=stored_mic.Mic0cplx;
	bufferOutput[1][0]=stored_mic.Mic1real;
	bufferOutput[1][1]=stored_mic.Mic1cplx;
	bufferOutput[2][0]=stored_mic.Mic2real;
	bufferOutput[2][1]=stored_mic.Mic2cplx;
}

float rad_to_deg(float rad){
	rad = (rad/PI)*180;
	return rad;
}

float adjust_deg(float deg){
	deg = deg - 90;
	return deg;
}

float get_arg(float real, float complex){
	float arg;
	arg = atan2f(complex,real);
	return arg;
}

float get_sound_direction(struct Mic_Record stored_mic, int freq_max)
{
	float direction;
	max_freq=freq_max;
	reassign_table(stored_mic); //reassigns the stored values in an array of arrays so that it's easier to analyse

	direction = calculate_direction();   // do all the calculations where the sound is coming from

	return direction;
}

float calculate_direction(void)
{
	float delta_t1;
	float direction;

	delta_t1 = find_delta_t_phase(0,1);

	float d_esp1; //spatial_distance

	d_esp1 = delta_t1*SPEED_SOUND; //spatial_distance = time_difference * speed_of_sound

	direction = acosf(d_esp1/MAXIMUM_DELTA_T1);
	// We want an angle strictly between [0,2*PI]
	if (direction < 0){
		direction = 2*PI + direction;
	}
	
	if(!((direction<2.45)&&(direction>0.698))){//Gets rid of absurd values and stays in the range of +- 50 deg
		direction=PI/2;
	}
	return direction;
}

float find_delta_t_phase(int mic1_nb, int mic2_nb){
	float delta_t = 0;
	float arg1, arg2;
	arg1=get_arg(bufferOutput[mic1_nb][0],bufferOutput[mic1_nb][1]); //Gets the argument of Mic1
	arg2=get_arg(bufferOutput[mic2_nb][0],bufferOutput[mic2_nb][1]); //Gets the argument of Mic2
	delta_t = ((arg1-arg2))/(2*PI*max_freq*TABLE_TO_FREQ); //delta  = (dif_arg)/(2*PI*FREQ)
	return delta_t;
}


