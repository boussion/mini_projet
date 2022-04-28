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

#define MIN_VALUE_THRESHOLD	10000 

// Constantes:
//pour fixer la fréquence d'utilisation du son dans le projet
#define MIN_FREQ	115	// On n'analyse pas le bruit avant 1 796,875 Hz
#define MAX_FREQ	140 	// On n'analyse pas le bruit après 2 187,5 Hz

#define FREQ_REF	128   // 2 000 Hz
#define FREQ_MVT_MIN	(FREQ_REF-1) // 1 984,375 Hz
#define FREQ_MVT_MAX 	(FREQ_REF+1) // 2 015.625 Hz

#define NB_ECHANTILLONS 10
#define NB_ECHANTILLONS_DETECTES 5


#define MIC_FRONT_RIGHT 1
#define MIC_FRONT_LEFT 2
#define MIC_LEFT_BACK 3
#define MIC_RIGHT_BACK 4
#define NO_MIC 0


// pour fixer l'amplitude à partir de laquelle on considère détecter une fréquence:
#define AMPLITUDE_MIN 10000 // On pose amplitude min = 10 000  pour l'instant

// Constantes neccessaires au fonctionnement du programme:
static	int  son_detection = 0; // Signal qui indique si un bruit est détecté

static uint16_t micro_a_proximite=0;





static void serial_start(void)
{
	static SerialConfig ser_cfg = {
			115200,
			0,
			0,
			0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}


/* analyse_son: Détecte un son de fréquence comprise entre 984.375 Hz et 1015.625 Hz. Si le son est détecté, il met à jour
 Paramètres :
 *	float *data			Buffer contenant 1024 échantillons symétriques, soit 2*512 échantillons. Il correspond à un des 4 microphones.
 */
void analyse_son(float* data){

	int detection = 0; // On utilise une variables locale pour mettre à jour a variable statique
	uint16_t amplitude_min = AMPLITUDE_MIN; // On initialise l'amplitude minimum à AMPLITUDE_MIN

	for(uint8_t i = MIN_FREQ; i <= MAX_FREQ ; i++){ // On parcourt la gamme de fréquence

		if(data[i] > amplitude_min){ // Amplitude supérieure à 10 000 dans [864.5 Hz-1280.125 Hz]?
			if((i>=FREQ_MVT_MIN) && (i<= FREQ_MVT_MAX)){ // Localisée dans la plage de fréquence : [984.375 Hz-1015.625 Hz]?
				detection = 1; // Si on capte on met à 1 la valeur de Detection_son
				break;

			}else{
				detection = 0; // Doit être dans [984.375 Hz-1015.625 Hz]
			}
		}
	}

	son_detection = detection;

	if(son_detection==1){ // Si le son est détecté
		set_body_led(1);

	}else{
		set_body_led(0);
		}

	}

/* detection_son: sort 1 si son détecté
 */
bool detection_son (void){

	return son_detection;

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
		/*	FFT proccessing
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

		deux_microphones_a_proximite();


		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3
		if(mustSend > 8){
			//signals to send the result to the computer
			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;
		}
		nb_samples = 0;
		mustSend++;

		analyse_son(micLeft_output);
	}
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
 *	deux_microphones_a_proximite: permet de déterminer les deux microphones les plus proches de la provenance du bruit en mettant à jour la variable statique micro_a_proximite
 */

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

		chprintf((BaseSequentialStream*)&SD3,"micro = %d\n", micro_a_proximite);

	}


/*int16_t determination_angle_rotation(void){

	int16_t angle_de_rotation = 0;

	if (micro_a_proximite==1){
		angle_de_rotation=-225;
	}

	if (micro_a_proximite==2){
		angle_de_rotation=225;
	}

	if (micro_a_proximite==3){
		angle_de_rotation=135;
		}

	if (micro_a_proximite==4){
		angle_de_rotation=-135;
		}

	if (micro_a_proximite==0){
		angle_de_rotation=à;
			}
	return angle_de_rotation;
}

*/










