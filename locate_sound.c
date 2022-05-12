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

/* defines used in main.c                          */
#define MEAN_MAX 		50		// Length of the mean_table
#define MEAN_MAX_INV 	0.01	// Inversed value of MEAN_MAX
#define PERCENT 	0.1			// Defines range to distinguish between noise an signal
#define NB_SAMPLES	100
#define TABLE_TO_FREQ	15.625	//Value to switch from table length to actual frequency
#define NB_MIC 3

//counters
//static int ooo;
//static int iii;

/* defines used in ad_conv_int.c					*/
//#define MIC_SAMP_NB 100		// number of microphone samples to store

/* defines used in find_direction					*/
//maximum_delta_t =
//sampling_frequency[Hz] * distance_between_microphones[m] / speed_of_sound[m/s];
#define MAXIMUM_DELTA_T1 0.06
#define MAXIMUM_DELTA_T2 0.05
#define SPEED_SOUND 343
#define DIST_MIC1 0.05

/* defines used in find_delta_t.c					*/
#define TAU_RANGE 14		// Needs to be a pair number

static int max_freq;


float mean_table[3][MEAN_MAX];
int mean_nb;
float mean[3];
float signal_max[3], signal_min[3];

//static int freq_real;

//WE NEED TO INITIATE BEFORE
static float bufferOutput[NB_MIC][2];				//Array to store the mic values


void reassign_table(struct Mic_Record stored_mic){
	bufferOutput[0][0]=stored_mic.Mic0real;
	bufferOutput[0][1]=stored_mic.Mic0cplx;
	bufferOutput[1][0]=stored_mic.Mic1real;
	bufferOutput[1][1]=stored_mic.Mic1cplx;
	bufferOutput[2][0]=stored_mic.Mic2real;
	bufferOutput[2][1]=stored_mic.Mic2cplx;
}


//converts angles in rads to degrees
float rad_to_deg(float rad){
	rad = (rad/PI)*180;
	return rad;
}

//adjusts values of output to have a 0 at the origin
float adjust_deg(float deg){
	deg = deg - 90;
	return deg;
}

float get_arg(float real, float complex){
	float arg;
	arg = atan2f(complex,real);
	return arg;
}

//main location function
float get_sound_direction(struct Mic_Record stored_mic, int freq_max)
{
	float direction;
	max_freq=freq_max;
	reassign_table(stored_mic); //reassigns the stored values in an array of arrays so that it's easier to analyse

	//filter_signal();        // filters the signals

	direction = calculate_direction();   // do all the calculations where the sound is coming from

	//show_led(direction);    // indicate where the sound is coming from

	return direction;
}

//uses trigonometry and the phase difference to determine the angle of incidence of the noise
//it's only effective for +-45 degs of the front of the e puck
float calculate_direction(void)
{
	float delta_t1, delta_t2;
	float direction, angle1, angle2;

	delta_t1 = find_delta_t_phase(0,1);

	float d_esp1; //spatial_distance

	d_esp1 = delta_t1*SPEED_SOUND; //spatial_distance = time_difference * speed_of_sound

	direction = acosf(d_esp1/MAXIMUM_DELTA_T1);
	// We want an angle strictly between [0,2*PI]
	if (direction < 0){
		direction = 2*PI + direction;
	}
	
	if(!((direction<2.433)&&(direction>0.698))){
		//chprintf((BaseSequentialStream*)&SD3,"WRONG NUMBER", direction);
		direction=PI/2;
	}
	//chprintf((BaseSequentialStream*)&SD3,"direction direct: %f\r\n", direction);
	return direction;
}

//determine the phase shift between the signals
float find_delta_t_phase(int mic1_nb, int mic2_nb){
	float delta_t = 0;
	float arg1, arg2;
	arg1=get_arg(bufferOutput[mic1_nb][0],bufferOutput[mic1_nb][1]);
	arg2=get_arg(bufferOutput[mic2_nb][0],bufferOutput[mic2_nb][1]);
	delta_t = ((arg1-arg2))/(2*PI*max_freq*TABLE_TO_FREQ);
	return delta_t;
}

//gets the absolute value of a float
float absolute_float(float enter){
	if(enter<0){
		enter=-enter;
	}
	return enter;
}


