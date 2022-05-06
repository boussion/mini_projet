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

int16_t get_line_position(void);

/*
 * process_image_start : starts the two threads
 */
void process_image_start(void);


#endif /* DETECTION_LIGNE_H_ */
