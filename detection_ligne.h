/*
 * detection_ligne.h
 *
 *  Created on: 5 mai 2022
 *      Author: tbous
 */

#ifndef DETECTION_LIGNE_H_
#define DETECTION_LIGNE_H_



//float get_distance_cm(void);
//uint16_t get_line_position(void);
void process_image_start(void);
//uint16_t extract_line_width(uint8_t *buffer);
int16_t extract_line_position_to_center(uint8_t *buffer);
int16_t get_line_position_to_center_mm(void);



#endif /* DETECTION_LIGNE_H_ */
