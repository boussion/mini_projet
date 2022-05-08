#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;

/* sound_analysis: Detects a sound frequency between 1984.375 Hz and 2015.625 Hz and updates the static variable
 */
void sound_analysis(float* data);

void process_direction (void);	


float get_last_direction(void);

void select_freq(void);

/* sound_detection : allows you to indicate the detection of a sound in another file
 */

//DOUBLON, TO CLEAN
bool sound_detection (void);

void move_round(float direction);

void processAudioData(int16_t *data, uint16_t num_samples);

uint16_t correction_detection_son(void);

float mean_sound(float* mic_nb);
void record_sound(void);
void store_sound(uint16_t nb_record);


/*
*	put the invoking thread into sleep until it can process the audio datas
*/
void wait_send_to_computer(void);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

//void frequency_detection_start(void);

//void deux_microphones_a_proximite(void);

//int16_t determination_angle_rotation(void);

#endif /* AUDIO_PROCESSING_H */
