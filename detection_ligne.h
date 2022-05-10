/*
 * detection_ligne.h
 *
 *  Created on: 5 mai 2022
 *      Author: tbous
 */

#ifndef DETECTION_LIGNE_H_
#define DETECTION_LIGNE_H_

/*
 * extract_line_position : update the static variable line1_position with the position of line 1
 */
void extract_line_position(uint8_t *buffer);
/*
 * get_line_position : returns the value in the static variable line_position for use in another file
 */
int16_t get_line_position(void);
/*
 * process_image_start : starts the two threads
 */
void process_image_start(void);

/*
 * detection_ligne : returns 1 if a line is detected by the camera otherwise returns 0
 */
bool detection_line(void);

/*
 * front_leds : set front led when the line is detected
 */
void front_leds(void);


#endif /* DETECTION_LIGNE_H_ */
