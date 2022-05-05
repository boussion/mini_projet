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

#include <locate_sound.h>

/* defines used in main.c                          */
#define MEAN_MAX 50		// Length of the mean_table
#define MEAN_MAX_INV 0.01	// Inversed value of MEAN_MAX
#define PERCENT 0.1			// Defines range to distinguish between noise an signal
#define NB_SAMPLES 100


/* defines used in ad_conv_int.c					*/
//#define MIC_SAMP_NB 100		// number of microphone samples to store

/* defines used in find_direction					*/
//maximum_delta_t =
//sampling_frequency[Hz] * distance_between_microphones[m] / speed_of_sound[m/s];
#define MAXIMUM_DELTA_T1 6.
#define MAXIMUM_DELTA_T2 5.

/* defines used in find_delta_t.c					*/
#define TAU_RANGE 14		// Needs to be a pair number

/* defines used in turn_to_direction.c */
#define TURN_SPEED 1000
#define STEPS_FOR_2PI 1300.
#define MIC_SAMP_NB 100


//int new_sample;
float mean_table[3][MEAN_MAX];
int mean_nb;
float mean[3];
float signal_max[3], signal_min[3];

//WE NEED TO INITIATE BEFORE
static int bufferOutput[MIC_SAMP_NB][3];				//Array to store the mic values


void reassign_table(struct Mic_Record* stored_mic){
	for (int i = 0; i < NB_SAMPLES; i++)			// for the whole signal
	{
		bufferOutput[i][0]=(int)stored_mic[i].Mic0;
		bufferOutput[i][1]=(int)stored_mic[i].Mic1;
		bufferOutput[i][2]=(int)stored_mic[i].Mic2;
	}
}

float get_sound_direction(struct Mic_Record* stored_mic)
{

	float direction;

	reassign_table(stored_mic);

	//filter_signal();        // filters the signals

	direction = calculate_direction();   // do all the calculations where the sound is coming from

	show_led(direction);    // indicate where the sound is coming from

	return direction;
}



void filter_signal(void){
	for (int j=0; j<3; j++)  							// for all three mics
	{
		for (int i = 0; i < NB_SAMPLES; i++)			// for the whole signal
		{
			bufferOutput[i][j] -= mean[j];				// shift the signal down to around 0
			if (bufferOutput[i][j] < 0)			        // take the absolute value
				bufferOutput[i][j] = -bufferOutput[i][j];   // --> gives better values in the cross-correlation
		}
	}
}

float calculate_direction(void)
{
	int delta_t1, delta_t2;
	float direction, angle1, angle2;

	// first get the phase-shift between the right and the left microphone
	delta_t1 = find_delta_t(0,1);
	chprintf((BaseSequentialStream*)&SD3,"delta t1: %d\n", delta_t1, "%d\n");
	// calculate the angle (between -90deg and +90deg where the sound is coming from)
	if (delta_t1 >= MAXIMUM_DELTA_T1){
			angle1 = PI * 0.5; 			// to avoid NaN of asin
	}
	else if (delta_t1 <= -MAXIMUM_DELTA_T1){
			angle1 = -PI * 0.5; 		// to avoid NaN of asin
	}
	else{
		angle1 = asin( (float)(delta_t1)/MAXIMUM_DELTA_T1 );
	}

	// now if the signal is coming from the right, we check the phase-shift between the
	// left and the rear microphone in order to find out if the direction is
	// angle1 or (180deg - angle1)
	// if the signal coming from the left, we make the same test with the right and the
	// rear microphone
	if(angle1 > 0)
	{
		delta_t2 = find_delta_t(2,1);   // phase shift between left and rear microphone
		chprintf((BaseSequentialStream*)&SD3,"2,1 delta t2: %d\n", delta_t2, "%d\n");
		if (delta_t2 >= MAXIMUM_DELTA_T2)
			angle2 = PI * 0.5; 			// to avoid NAN of asin
		else if (delta_t2 <= -MAXIMUM_DELTA_T2)
			angle2 =-PI * 0.5; 			// to avoid NAN of asin
		else
			angle2 = asin( (float)delta_t2/MAXIMUM_DELTA_T2 );

		if(angle2 > PI/6.)  			// if the second angle is bigger than +30deg
			direction = PI - angle1;   	// the direction = 90deg -angle1
		else direction = angle1;
	}
	else
	{
		delta_t2 = find_delta_t(0,2); 	// phase shift between right and rear microphone
		chprintf((BaseSequentialStream*)&SD3,"0,2 delta t2: %d\n", delta_t2, "%d\n");
		if (delta_t2 >= MAXIMUM_DELTA_T2)
			angle2 = PI * 0.5; 			// to avoid NAN of asin
		else if (delta_t2 <= -MAXIMUM_DELTA_T2)
			angle2 = -PI * 0.5; 		// to avoid NAN of asin
		else
			angle2 = asin( (float)delta_t2/MAXIMUM_DELTA_T2 );

		if(angle2 < -PI/6.)				// if the second angle is smaller than -30deg
			direction = PI - angle1;	// the direction = 90deg -angle1
		else direction = angle1;
	}

	// We want an angle strictly between [0,2*PI]
	if (direction < 0){
		direction = 2*PI + direction;
	}
	return direction;
}


//Find the time separation delta_t between the two signals based on the correlation between the signals.
int find_delta_t(int mic1_nb,int mic2_nb)
{
	int delta_t = 0;
	int tau = 0;
	int k = 0;
	long int correlation, max;

	int tau_min = -TAU_RANGE / 2;
	int tau_max = TAU_RANGE / 2;

	int save_sound_start = TAU_RANGE / 2 + 1;
	int save_sound_end = NB_SAMPLES - TAU_RANGE / 2 - 1;


	max = 0;

	for (tau = tau_min; tau < tau_max; tau++)
	{
		//reset the the correlation value
		correlation = 0;

	    // For each tau calculate the correlation between the two signals
	    for (k = save_sound_start; k < save_sound_end; k++)
	    {
		    correlation += (long int)(bufferOutput[k][mic1_nb]) * (long int)(bufferOutput[k+tau][mic2_nb] );
		}

		// find out if this correlation is the biggest one so far. --> If yes,
    	// save the value of tau --> This gives us the phase shift between the
	    // signals
        if (correlation > max)
        {
	        max = correlation;
	        delta_t = tau;
	    }

	}

	return delta_t;
}

void show_led(float angle)
{
	clear_leds();
	int led_nb;
	if ( angle > (PI/8) )
		led_nb = LED1;
	if ( angle > (3*PI/8) )
		led_nb = LED2;
	if ( angle > (5*PI/8) )
		led_nb = LED3;
	if ( angle > (7*PI/8) )
		led_nb = LED4;
	if ( angle > (9*PI/8) )
		led_nb = LED5;
	if ( angle > (11*PI/8) )
		led_nb = LED6;
	if ( angle > (13*PI/8) )
		led_nb = LED7;
	if ( angle > (15*PI/8) )
		led_nb = LED8;
	set_led(led_nb, 1);


}


