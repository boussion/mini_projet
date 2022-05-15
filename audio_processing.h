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

/**
 * @brief  Detects a sound frequency between FREQ_REF - 30 Hz and FREQ_REF + 30 Hz and updates the static variable
 * 
 * @param data Buffer containing 1024 symmetrical samples, that is 2*512 samples. It corresponds to one of the 4 microphones.
 */
void sound_analysis(float* data);


/**
 * @brief //Selects the frequency with the highest amplitude between FREQ_REF-2 and FREQ_REF+2
 * 
 */
void select_freq(void);


/**
 * @brief  allows you to indicate the detection of a sound in another file
 * 
 * @return if a sound is being heard

 */
bool sound_detection (void);


void processAudioData(int16_t *data, uint16_t num_samples);


/**
 * @brief Used to determine the location of the sound, as well as the necessary adjustments to get a readable value
 * 
 */
void process_direction (void);	


/**
 * @brief Get the last direction in memory
 * 
 * @return float direction
 */
float get_last_direction(void);


/**
 * @brief Stores the sound in a structure in order to send them to the location function
 * 
 */
void store_sound(void);


/**
 * @brief Get the audio buffer ptr
 * 
 * @param name Which buffer is wanted
 * @return float* the numer of the wanted buffer
 */
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

#endif /* AUDIO_PROCESSING_H */
