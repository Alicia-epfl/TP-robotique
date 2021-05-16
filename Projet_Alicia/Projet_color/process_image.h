/**
 * Fichier: process_image.h
 * auteurs: TP4, modifié par Alicia Mauroux
 *
 * @brief
 * Fichier qui gère la capture des images et leur traitement.
 * Il indique également les couleurs perçues
 */

#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

//fonction pour indiquer si la caméra est active
uint8_t get_record(void);
/*Fonction pour vérifier si le robot est en train de tourner à gauche
 * Si left = false --> tourne à gauche, tout le monde s'arrête*/
uint8_t get_left(void);
//fonctions pour indiquer quelle couleur afficher sur les leds RGB
uint8_t get_red(void);
uint8_t get_green(void);
uint8_t get_blue(void);
//fonctions de fin de parties
uint8_t get_game_over(void);
uint8_t get_win(void);

#endif /* PROCESS_IMAGE_H */
