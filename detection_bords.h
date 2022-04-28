/*
 * detection_bords.h
 *
 *  Created on: 22 avr. 2022
 *      Author: tbous
 */

#ifndef DETECTION_BORDS_H_
#define DETECTION_BORDS_H_


/* Ajustement_dist_mm: Permet d'ajuster les valeurs trouv�es
 */
uint16_t ajustement_dist(void);

/* distance_bords: Mets � jour la distance qui reste � parcourir pour atteindre les bords et la retourne
 */
uint16_t distance_bords(void);

/* distance_centre: Mets � jour la distance qui reste � parcourir pour atteindre le centre et la retourne
 */
uint16_t distance_centre(void);

/* mise_a_jour_distance_actuelle: Mets � jour la distance qui reste � parcourir en fonction de la d�tection du son et la retourne
 */
int16_t mise_a_jour_distance_actuelle(void);


#endif /* DETECTION_BORDS_H_ */
