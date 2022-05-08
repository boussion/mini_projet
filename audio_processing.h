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

/* 
	sound_analysis: Detects a sound frequency between 1984.375 Hz and 2015.625 Hz and updates the static variable
*/
void sound_analysis(float* data);

//function used to determine the location of the sound, as well as the necessary adjustments to get a readable value
void process_direction (void);	

//Exports the last direction in memory
float get_last_direction(void);

//Selects the frequency with the highest amplitude between FREQ_REF-2 and FREQ_REF+2
void select_freq(void);

/* 
	sound_detection : allows you to indicate the detection of a sound in another file
*/
bool sound_detection (void);


void processAudioData(int16_t *data, uint16_t num_samples);


//Stores the sound in a structure in order to send them to the location function
void store_sound(void);


/*
*	put the invoking thread into sleep until it can process the audio datas
*/
void wait_send_to_computer(void);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

#endif /* AUDIO_PROCESSING_H */
