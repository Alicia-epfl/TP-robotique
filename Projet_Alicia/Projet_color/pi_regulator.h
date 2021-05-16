/**
 * Fichier: pi_regulator.c
 * auteurs: TP4, modifié par Alicia Mauroux
 *
 * @brief
 * Fichier qui gère la locomotion du robot e-puck2 à l'aide de régulateurs PI.
 */

#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

// @brief
/* fonction pour faire tourner le robot d'angle alpha
 *
 * @param alpha     alpha est en radian*/
void turn(float alpha);

// @brief
//Démarre le thread PI regulator
void pi_regulator_start(void);

#endif /* PI_REGULATOR_H */
