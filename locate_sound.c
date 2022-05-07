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
#define MEAN_MAX 50		// Length of the mean_table
#define MEAN_MAX_INV 0.01	// Inversed value of MEAN_MAX
#define PERCENT 0.1			// Defines range to distinguish between noise an signal
#define NB_SAMPLES 100

//counters
static int ooo;
static int iii;

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

/* defines used in turn_to_direction.c */
#define TURN_SPEED 1000
#define STEPS_FOR_2PI 1300.
#define MIC_SAMP_NB 100
#define NB_MIC 3
#define FREQ_REF 2000
#define FREQ_REF_INDEX 128


//int new_sample;
float mean_table[3][MEAN_MAX];
int mean_nb;
float mean[3];
float signal_max[3], signal_min[3];

static int freq_real;

//WE NEED TO INITIATE BEFORE
static int bufferOutput[NB_MIC][2];				//Array to store the mic values


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

	reassign_table(stored_mic); //reassigns the stored values in an array of arrays so that it's easier to analyse

	//filter_signal();        // filters the signals

	direction = calculate_direction();   // do all the calculations where the sound is coming from

	show_led(direction);    // indicate where the sound is coming from

	return direction;
}


//not used
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

//uses trigonometry and the phase difference to determine the angle of incidence of the noise
//it's only effective for +-45 degs of the front of the e puck
float calculate_direction(void)
{
	float delta_t1, delta_t2;
	float direction, angle1, angle2;

	// first get the phase-shift between the right and the left microphone

	/*//UNCOMMENT IF CORRELATION
	delta_t1 = find_delta_t(0,1);
	chprintf((BaseSequentialStream*)&SD3,"delta t1: %d\n", delta_t1, "%d\n");
	*/
	delta_t1 = find_delta_t_phase(0,1);

	// calculate the angle (between -90deg and +90deg where the sound is coming from)
	/*
	if (delta_t1 >= MAXIMUM_DELTA_T1){
			angle1 = PI * 0.5; 			// to avoid NaN of asin
	}
	else if (delta_t1 <= -MAXIMUM_DELTA_T1){
			angle1 = -PI * 0.5; 		// to avoid NaN of asin
	}
	else{
		angle1 = atan((float)(delta_t1)/MAXIMUM_DELTA_T1 );
	}

	// now if the signal is coming from the right, we check the phase-shift between the
	// left and the rear microphone in order to find out if the direction is
	// angle1 or (180deg - angle1)
	// if the signal coming from the left, we make the same test with the right and the
	// rear microphone
	if(angle1 > 0)
	{
		chprintf((BaseSequentialStream*)&SD3,"delta t2: %d\n", delta_t2, "%d\n");
		*//*
		delta_t2 = find_delta_t_phase(2,1);
		chprintf((BaseSequentialStream*)&SD3,"delta t2: %f\n", delta_t2);

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
		/*UNCOMMENT IF CORRELATION
		delta_t1 = find_delta_t(0,2);
		chprintf((BaseSequentialStream*)&SD3,"delta t2: %d\n", delta_t2, "%d\n");

		delta_t2 = find_delta_t_phase(0,2);
		chprintf((BaseSequentialStream*)&SD3,"delta t2: %f\n", delta_t2);
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
*/
	float d_esp1; //spatial_distance
	d_esp1 = delta_t1*SPEED_SOUND; //spatial_distance = time_difference * speed_of_sound
	/*
	if(ooo>10){
			chprintf((BaseSequentialStream*)&SD3,"desp: %f\r\n", d_esp1);
		ooo=0;
		}
	++ooo;
	*/
	direction = acosf(d_esp1);
	// We want an angle strictly between [0,2*PI]
	if (direction < 0){
		direction = 2*PI + direction;
	}
	if(!((direction<2.433)&&(direction>0.698))){
		//chprintf((BaseSequentialStream*)&SD3,"WRONG NUMBER", direction);
		direction=PI/2;
	}
	/*
	if(direction>2.443){
		direction = 2.443;
	}
	if(direction<0.698){
		direction = 0.698;
	}
	*/
	//chprintf((BaseSequentialStream*)&SD3,"direction direct: %f\r\n", direction);
	return direction;
}

//determine the phase shift between the signals
float find_delta_t_phase(int mic1_nb, int mic2_nb){
	float delta_t = 0;
	float arg1, arg2;
	arg1=get_arg(bufferOutput[mic1_nb][0],bufferOutput[mic1_nb][1]);
	arg2=get_arg(bufferOutput[mic2_nb][0],bufferOutput[mic2_nb][1]);
	delta_t = (1024*0.001*(arg1-arg2))/(2*PI*FREQ_REF_INDEX);
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

//gets the absolute value of a float
float absolute_float(float enter){
	if(enter<0){
		enter=-enter;
	}
	return enter;
}

//second method to determine the direction of the sound made
float calculate_direction2(void)
{
	/*
	if(iii>5){
		chprintf((BaseSequentialStream*)&SD3,"HELLO %f\r\n");
		iii=0;
	}
	++iii;
	*/
	float direction, angle1, angle11;
	float d_esp1, d_esp2, delta_t1, delta_t2;

	delta_t1 = find_delta_t_phase(0,1);

	if(delta_t1<0){
		delta_t2 = find_delta_t_phase(0,2);
	}
	else{
		delta_t2 = find_delta_t_phase(1,2);
	}

	d_esp1 = SPEED_SOUND*delta_t1;

	//angle1 = absolute_float(acosf(absolute_float(d_esp1)/0.06));

	angle11 = acosf((d_esp1));
/*
	if(absolute_float(angle1)<(PI/10)){
		direction = PI/2;
	}
	else if((angle1<(0.41*PI))&&(d_esp2>0.04)){
		direction = (PI/2)-angle1;
	}
	else if((angle1>=(0.41*PI))&&(angle1<(PI/2))&&(d_esp2>0)){
		direction=(PI/2)-angle1;
	}
	else if(d_esp2<0.04){
		direction = (PI/2)+angle1;
	}
	else{
	}
*/
		direction = angle11;
/*
	if(d_esp1>0){
		direction =- direction;
	}
*/
	/*if(ooo>5){
			chprintf((BaseSequentialStream*)&SD3,"angle1: %f\r\n", 180*angle1/PI);
			chprintf((BaseSequentialStream*)&SD3,"angle11: %f\r\n", 180*angle11/PI);
			chprintf((BaseSequentialStream*)&SD3,"delta: %f\r\n", delta_t1);
			chprintf((BaseSequentialStream*)&SD3,"direction: %f\r\n", 180*direction/PI);
			ooo=0;

	}
	++ooo;
	*/
		if(direction>2.443){
			direction = 2.443;
		}
		if(direction<0.698){
			direction = 0.698;
		}
		chprintf((BaseSequentialStream*)&SD3,"direction direct: %f\r\n", direction);

	return direction;
}

