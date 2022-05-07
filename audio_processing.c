#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>
#include <leds.h>
#include <locate_sound.h>


//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE]; // FFT_SIZE =1024
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

//Different methods used to store the values of the angles
static float stored_dir[5];
static float sum_dir;

//Stores the value of the last average direction calculated
static float last_direction;

//used to store the frequency in the analysed range with the largest amplitude
static int freq_max;

//used to count different iterations
static int ooo;


#define MIN_VALUE_THRESHOLD	10000 


// Bandpass filter:
#define MIN_FREQ	115	// no sound detected before 1 796,875 Hz
#define MAX_FREQ	140 // no sound detected after 2 187,5 Hz

// Frequencies studied:
#define FREQ_REF	128   // 2 000 Hz
#define FREQ_MVT_MIN	(FREQ_REF-2) // 1 984,375 Hz
#define FREQ_MVT_MAX 	(FREQ_REF+2) // 2 015.625 Hz
#define FREQ_MVT_RANGE (FREQ_MVT_MAX-FREQ_MVT_MIN)	//MAX_FREQ-MIN_FREQ

#define NB_SAMPLES 100 //Nb de samples enregistr�s pour localiser le son

#define MIC_FRONT_RIGHT 1
#define MIC_FRONT_LEFT 2
#define MIC_LEFT_BACK 3
#define MIC_RIGHT_BACK 4
#define NO_MIC 0

// To set the amplitude at which a frequency is considered to be detected:
#define AMPLITUDE_MIN 30000 // avoids detection of shocks to the structure

static struct Mic_Record stored_mic[NB_SAMPLES];


// Sound detection:
static	int  son_detection = 0; // Signal that indicates whether noise is detected
//static uint16_t micro_a_proximite=0;

/* sound_analysis: Detects a sound frequency between 1984.375 Hz and 2015.625 Hz and updates the static variable
 Param�tres :
*/
//float *data			Buffer containing 1024 symmetrical samples, that is 2*512 samples. It corresponds to one of the 4 microphones.

void sound_analysis(float* data){
	int detection = 0; // use a local variable to update a static variable

	// scans the frequency range
	for(uint8_t i = MIN_FREQ; i <= MAX_FREQ ; i++){

		// search for an amplitude above a threshold
		if(data[i] > AMPLITUDE_MIN){

			if((i>=FREQ_MVT_MIN) && (i<= FREQ_MVT_MAX)){
				detection = 1; // sound_detetected
				break;

			}else{
				detection = 0; // sound_not_detected
			}
		}
	}

	// update static variable
	son_detection = detection;

	// turn on the body leds if frequency detected
	if(son_detection==1){
		set_body_led(1);

	}else{
		set_body_led(0);

	}
}

/* sound_detection : allows you to indicate the detection of a sound in another file
 */
bool sound_detection (void){

	return son_detection;

}

//A VIRER SI RECHECK

bool detection_son(void){

	return son_detection;
}

//function used to determine the average sound [NOT USED]
float mean_sound(float* mic_nb){
	float average = 0;
	for(uint8_t i = FREQ_MVT_MIN; i <= FREQ_MVT_MAX; ++i){
		average = mic_nb[i] + average;
	}
	average = average/(FREQ_MVT_RANGE+1);
	return average;
}

//Enregistre le son des micros
void record_sound(void){
	chprintf((BaseSequentialStream*)&SD3,"sound in %d\n");

	//for(uint16_t i=0; i<NB_SAMPLES; ++i){
	//	recorded_sound_1[i]=mean_sound(micRight_output);
	//	recorded_sound_2[i]=mean_sound(micLeft_output);
	//	recorded_sound_3[i]=mean_sound(micBack_output);
	//}
	chprintf((BaseSequentialStream*)&SD3,"sound done %d\n");

}


/*
 *	Callback called when the demodulation of the four microphones is done.
 *	We get 160 samples per mic every 10ms (16kHz)
 *
 *	params :
 *	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
 *							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
 *	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/


//Calculates the average direction over a period of 5 recordings
float mean_dir(float* stored_dir){
	float sum_dir=0;
	for(int i=0; i<5; ++i){
		sum_dir =+ stored_dir[i];
	}
	sum_dir=sum_dir/5;
	return sum_dir;
}



void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	 *
	 *	We get 160 samples per mic every 10ms
	 *	So we fill the samples buffers to reach
	 *	1024 samples, then we compute the FFTs.
	 *
	 */

	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;
	static uint16_t nb_record = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT processing
		 *
		 *	This FFT function stores the results in the input buffer given.
		 *	This is an "In Place" function.
		 */

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		/*	Magnitude processing
		 *
		 *	Computes the magnitude of the complex numbers and
		 *	stores them in a buffer of FFT_SIZE because it only contains
		 *	real numbers.
		 *
		 */
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);
		//deux_microphones_a_proximite();

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3

		if(mustSend > 8){
			//signals to send the result to the computer
			//chBSemSignal(&sendToComputer_sem);
			mustSend = 0;
		}
		nb_samples = 0;
		mustSend++;
		store_sound(nb_record);
		sound_analysis(micFront_output);
		if(detection_son()){
			//turn_puck(60);
			float direction;
			process_direction(); //process the direction of the detected sound
			//chprintf((BaseSequentialStream*)&SD3,"direction mean: %f\r\n", get_last_direction());

		}
	}
}

//function used to determine the location of the sound, as well as the necessary adjustments to get a readable value
void process_direction (void){	
	float direction;
	direction = get_sound_direction(stored_mic[0], freq_max);
	float direction2;
	direction2 = rad_to_deg(direction); //converts the radians to degrees

	if(direction2 < 50){
		//chprintf((BaseSequentialStream*)&SD3,"NUMBER TOOOOO LOOOOOWWW %d\r\n");
		direction2=50;
	}
	if(direction2>140){
		//chprintf((BaseSequentialStream*)&SD3,"NUMBER TOOOOO HIGGGHHHH %d\r\n");
		direction2 = 140;
	}

	direction2 = adjust_deg(direction2);
	sum_dir = sum_dir + direction2; //Sums the values before dividing them
	//chprintf((BaseSequentialStream*)&SD3,"direction 2: %f\r\n", direction2);
	//chprintf((BaseSequentialStream*)&SD3,"sum_dir: %f\r\n", sum_dir);


	++ooo;
	if(ooo > 4){
		//chprintf((BaseSequentialStream*)&SD3,"ooo: %d\r\n", ooo);
		//chprintf((BaseSequentialStream*)&SD3,"direction mean: %f\r\n", sum_dir/5);
		last_direction = sum_dir/5;
		ooo = 0;
		sum_dir = 0;
	}
}

float get_last_direction(void){
	return last_direction;
}

void move_round(float direction){

}



void store_sound(uint16_t nb_record){
	nb_record = 0; //COMMENT IF NEED MORE THAN 1 SAMPLE
	int freq_max = FREQ_REF;
	float val_max = micRight_output[FREQ_REF-1];
	/*
	if(ooo>5){
		chprintf((BaseSequentialStream*)&SD3,"val0: %f\r\n", micRight_output[FREQ_REF-2]);
		chprintf((BaseSequentialStream*)&SD3,"val1: %f\r\n", micRight_output[FREQ_REF]);
		chprintf((BaseSequentialStream*)&SD3,"val2: %f\r\n", micRight_output[FREQ_REF+2]);
	}
	*/
	/*
	for(int i=0; i<1; ++i){
		if (micRight_output[FREQ_REF+i]>val_max){
			val_max=micRight_output[FREQ_REF+i];
			freq_max = FREQ_REF+i;
		}
	}
	*/
	stored_mic[nb_record].Mic0real = micRight_cmplx_input[2*freq_max];
	stored_mic[nb_record].Mic1real = micLeft_cmplx_input[2*freq_max];
	stored_mic[nb_record].Mic2real = micBack_cmplx_input[2*freq_max];
	stored_mic[nb_record].Mic0cplx = micRight_cmplx_input[2*freq_max+1];
	stored_mic[nb_record].Mic1cplx = micLeft_cmplx_input[2*freq_max+1];
	stored_mic[nb_record].Mic2cplx = micBack_cmplx_input[2*freq_max+1];
}

void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}



/*
 *	deux_microphones_a_proximite: permet de d�terminer les deux microphones les plus proches de la provenance du bruit en mettant � jour la variable statique micro_a_proximite
 */

/*
void deux_microphones_a_proximite(void){

	uint16_t moyenne_front = 0;
	uint16_t moyenne_back = 0;
	uint16_t moyenne_right = 0;
	uint16_t moyenne_left = 0;

	for(int i=FREQ_MVT_MIN; i<FREQ_MVT_MAX; i++){

		moyenne_front = micFront_output[i] + moyenne_front;
		moyenne_back = micBack_output[i] + moyenne_back;
		moyenne_right = micRight_output[i] + moyenne_right;
		moyenne_left = micLeft_output[i] + moyenne_left;

		moyenne_front = moyenne_front / 3;
		moyenne_back = moyenne_back / 3;
		moyenne_right = moyenne_right / 3;
		moyenne_left = moyenne_left / 3;

	}

	if(detection_son()==1){

		if((moyenne_front > moyenne_back) && (moyenne_front > moyenne_right) && (moyenne_front > moyenne_left) && (moyenne_right > moyenne_left)){
			micro_a_proximite = MIC_FRONT_RIGHT;

		}else{
			if((moyenne_front > moyenne_back) && (moyenne_front > moyenne_right) && (moyenne_front > moyenne_left)){
				micro_a_proximite = MIC_FRONT_LEFT;

			}else{
				if((moyenne_left > moyenne_right) && (moyenne_left > moyenne_back)){
					micro_a_proximite = MIC_LEFT_BACK;

				}else{
					micro_a_proximite = MIC_RIGHT_BACK;
				}
			}
		}
	}else{
		micro_a_proximite=0;
	}
}
*/

