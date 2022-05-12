#ifndef LOCATE_SOUND_H_
#define LOCATE_SOUND_H_

/**
 * @brief Structure to store mic values
 * 
 */
struct Mic_Record{
	float Mic0real;
	float Mic1real;
	float Mic2real;
	float Mic0cplx;
	float Mic1cplx;
	float Mic2cplx;
};

/**
 * @brief Creates a buffer and locally stored the values of stored_mic
 * 
 * @param stored_mic the array in which the values of the mic are stored
 */
void reassign_table(struct Mic_Record stored_mic); //Stores the mic values as a buffer


/**
 * @brief Converts Radians to Degrees
 * 
 * @param rad The angle in radians
 * @return float the angle in degrees
 */
float rad_to_deg(float rad); //converts angles in rads to degrees


/**
 * @brief Centers the degrees around 0 rather than 90 degrees
 * 
 * @param deg the angle centered around 0
 * @return float 
 */
float adjust_deg(float deg); //adjusts values of output to have a 0 at the origin


/**
 * @brief Get the arg of a complex number
 * 
 * @param real 		The real part of a complex number
 * @param complex 	The complex part of a complex number
 * @return float The argument of the complex number
 */
float get_arg(float real, float complex); //gets the argument of the complex number


/**
 * @brief Get the sound direction
 * 
 * @param stored_mic the recorded values in the mic
 * @param freq_max the value where the frequency amplitude is the highest
 * @return float direction
 */
float get_sound_direction(struct Mic_Record stored_mic, int freq_max); 


/**
 * @brief Uses trigonometry and the phase difference to determine the angle of incidence of the noise
 *		The effective range is +-45 degrees.
 * 
 * @return float direction in radians
 */
float calculate_direction(void);


/**
 * @brief determine the phase shift between the signals
 * 
 * @param mic1_nb The first mic's number
 * @param mic2_nb The second mic's number
 * @return float the difference in the phase
 */
float find_delta_t_phase(int mic1_nb, int mic2_nb);


#endif /* LOCATE_SOUND_H_ */
