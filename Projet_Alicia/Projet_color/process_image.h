#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H


/*Fonction pour vérifier si le robot est en train de tourner à droite
 * Si right = true --> normal
 * Si right = false --> tourne à droite, tout le monde s'arrête*/
uint8_t get_right(void);
uint8_t get_left(void);
uint8_t get_red(void);
uint8_t get_green(void);
uint8_t get_blue(void);




#endif /* PROCESS_IMAGE_H */
