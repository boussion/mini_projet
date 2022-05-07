#ifndef LOCATE_SOUND_H_
#define LOCATE_SOUND_H_

struct Mic_Record{
	float Mic0real;
	float Mic1real;
	float Mic2real;
	float Mic0cplx;
	float Mic1cplx;
	float Mic2cplx;
};


float get_sound_direction(struct Mic_Record stored_mic, int freq_max);

void record_sound(void);

void filter_signal(void);

float calculate_direction(void);

int find_delta_t(int mic1_nb,int mic2_nb);

float find_delta_t_phase(int mic1_nb, int mic2_nb);

void show_led(float angle);




#endif /* LOCATE_SOUND_H_ */
