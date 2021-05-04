#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H


void find_color(uint8_t *buffer);

/*Fonction pour vérifier si le robot est en train de tourner à droite
 * Si right = true --> normal
 * Si right = false --> tourne à droite, tout le monde s'arrête*/
uint8_t get_right(void);

#endif /* PROCESS_IMAGE_H */
