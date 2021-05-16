
/**
 * Fichier: proximity_detection.h
 * auteurs: Nicolas Nouel et Alicia Mauroux
 * Created on: Apr 29, 2021
 *
 * @brief
 * Ce fichier gère l'évitement d'obstacle
 */

#ifndef PROXIMITY_DETECTION_H_
#define PROXIMITY_DETECTION_H_

//@brief
//Activer proximity --> appel du thread
void proxi_start(void);

//@brief
//fonction pour retourner les valeurs
uint8_t get_avoid(void);
uint8_t get_game_over_proxi(void);

#endif /* PROXIMITY_DETECTION_H_ */
