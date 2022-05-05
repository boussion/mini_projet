/*
 * detection_ligne.h
 *
 *  Created on: 5 mai 2022
 *      Author: tbous
 */

#ifndef DETECTION_LIGNE_H_
#define DETECTION_LIGNE_H_


/*
 * extract_line1_position : update the static variable line1_position with the position of line 1
 */
void extract_line1_position(uint8_t *buffer);

/*
 * extract_line2_position : update the static variable line2_position with the position of line 2
 */
void extract_line2_position(uint8_t *buffer);

/*
 * middle_distance_lines : update the static variable middle_distance_btw_lines with the position of the centre between the two lines
 */
void middle_distance_lines(void);

/*
 * error_center_vs_middle : gives the error between the centre of the camera and the centre of the two lines as output
 */
int16_t error_center_vs_middle(void);

/*
 * capture : exit true if two lines have been detected otherwise exit false
 */
bool capture(void);

/*
 * process_image_start : starts the two threads
 */
void process_image_start(void);


#endif /* DETECTION_LIGNE_H_ */
