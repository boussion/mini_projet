/*
 * detection_ligne.h
 *
 *  Created on: 5 mai 2022
 *      Author: tbous
 */

#ifndef DETECTION_LIGNE_H_
#define DETECTION_LIGNE_H_

/*
 * @brief Update the static variable line1_position with the position of line 1
 *
 * @param Unsigned int buffer filled with the last image in RGB565
 */
void extract_line_position(uint8_t *buffer);

/*
 * @brief Returns the value in the static variable line_position for use in another file
 *
 * @return Int variable that gives us the position of the line
 */
int16_t get_line_position(void);

/*
 * @brief Starts the two threads
 */
void process_image_start(void);

#endif /* DETECTION_LIGNE_H_ */
