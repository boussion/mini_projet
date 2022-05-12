#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
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
static float sum_dir;

//Stores the value of the last average direction calculated
static float last_direction;

//used to store the frequency in the analysed range with the largest amplitude
static int freq_max;

//used to count different iterations
static int counter;

#define MIN_VALUE_THRESHOLD	10000 

// Frequencies studied:
#define FREQ_CORRECTION 	3 //Frequency corrector (About 50 Hz)
#define FREQ_REF	30 + FREQ_CORRECTION  // 468.75 Hz
#define FREQ_MVT_MIN	(FREQ_REF-2) // FREQ_REF - 30 Hz
#define FREQ_MVT_MAX 	(FREQ_REF+2) // FREQ_REF + 30 Hz
#define FREQ_MVT_RANGE (FREQ_MVT_MAX-FREQ_MVT_MIN)	//MAX_FREQ-MIN_FREQ

// Bandpass filter:
#define MIN_FREQ	FREQ_REF-12 // FREQ_REF - 187.5 Hz
#define MAX_FREQ	FREQ_REF+12 // FREQ_REF + 187.5 Hz

#define MIC_FRONT_RIGHT 1
#define MIC_FRONT_LEFT 2
#define MIC_LEFT_BACK 3
#define MIC_RIGHT_BACK 4
#define NO_MIC 0

// To set the amplitude at which a frequency is considered to be detected:
#define AMPLITUDE_MIN 30000 // avoids detection of shocks to the structure

static struct Mic_Record stored_mic;


// Sound detection:
static	int  son_detection = 0; // Signal that indicates whether noise is detected
//static uint16_t micro_a_proximite=0;


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


void select_freq(void){
	freq_max=FREQ_REF-2;
	float  val_max=micFront_output[FREQ_REF-2]; //Stores the amplitude of the lowest checked value as a reference
	for(int i=0;i<4;++i){
		if(val_max<micFront_output[FREQ_REF-1+i]){ 		//Compares to all other analyzed values to store if they're
			val_max = micFront_output[FREQ_REF-1+i];	//Higher
			freq_max = FREQ_REF-1+i;
		}
	}
}

bool sound_detection (void){
	return son_detection;
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

		nb_samples = 0;
		store_sound();
		sound_analysis(micFront_output);
		if(sound_detection()){
			process_direction(); //process the direction of the detected sound
		}
		else{
			clear_leds();
		}
	}
}

void process_direction (void){	
	float direction;
	select_freq();

	direction = get_sound_direction(stored_mic, freq_max);

	float direction2;
	direction2 = rad_to_deg(direction); //converts the radians to degrees

	if(direction2 < 50){
		direction2=50;
	}
	if(direction2>140){
		direction2 = 140;
	}

	direction2 = adjust_deg(direction2);
	sum_dir = sum_dir + direction2; //Sums the values before dividing them

	++counter; //counter to make sure we get to 5 values before averaging the directions
	if(counter > 4){
		last_direction = sum_dir/5;
		counter = 0;
		sum_dir = 0;
	}
}

float get_last_direction(void){
	return last_direction;
}

void store_sound(void){

	stored_mic.Mic0real = micRight_cmplx_input[2*freq_max];
	stored_mic.Mic1real = micLeft_cmplx_input[2*freq_max];
	stored_mic.Mic2real = micBack_cmplx_input[2*freq_max];
	stored_mic.Mic0cplx = micRight_cmplx_input[2*freq_max+1];
	stored_mic.Mic1cplx = micLeft_cmplx_input[2*freq_max+1];
	stored_mic.Mic2cplx = micBack_cmplx_input[2*freq_max+1];
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



