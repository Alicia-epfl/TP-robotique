#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include "sensors/proximity.h"
#include <main.h>
#include <proximity_detection.h>

#include <sensors/proximity.h>
#include <msgbus/messagebus.h>
//besoin de lui pour avoir accès au run
#include "noise_detection.h"
//besoin de lui pour la fonction de tourner
#include "pi_regulator.h"

#include <leds.h>
#include <motors.h>

//static pour indiquer si on est en train d'éviter un obstacle --> si oui on met le "run" en pause dans le pi
static uint8_t avoid = false;
static uint8_t game_over = false;
/*
 * THREADS
 */
/*Thread pour proximity*/
static THD_WORKING_AREA(waProximity, 1024);
static THD_FUNCTION(Proximity, arg) {

	 chRegSetThreadName(__FUNCTION__);
	  (void)arg;

	  uint8_t avoid_allowed;

	while (1){
		avoid_allowed = get_avoid_allowed_fsm();

		if(avoid_allowed){
			/*Fonction d'évitement*/

			//OBSTACLE DEVANT
			if(((get_prox(0))>IR_THRESHOLD) || ((get_prox(7))>IR_THRESHOLD)){
				//indique qu'on est en processus d'évitement
				avoid = true;


				//OBSTACLE DEVANT + A DROITE
				if(get_prox(2)>IR_TRES_SIDE){

				//OBSTACLE DEVANT + A DROITE + A GAUCHE
					if(get_prox(5)>IR_TRES_SIDE){

				//OBSTACLE PARTOUT --> ENCERCLE
						if((get_prox(3)>IR_TRES_SIDE) || (get_prox(4)>IR_TRES_SIDE)){
							//Le jeu est perdu --> game over
							game_over = true;
						}
						//OBSTACLE DEVANT + A DROITE + A GAUCHE
						else{
							//effectue un demi-tour
							turn(PI);
							//libère run
							avoid = false;
						}

					//OBSTACLE DEVANT + A DROITE
					}else{
						//tourne à gauche
						turn(PI/2);
						//libère run
						avoid = false;
					}
				//OBSTACLE DEVANT
				}else{
					//tourne à droite
					turn(-PI/2);
					//libère run
					avoid = false;

				}//if obstacle devant
			}//if obstacle
			/*===========================*/

			/* Delai de 250 millisecondes.*/
			chThdSleepMilliseconds(250);
		}//while(run)
	}//while(1)
}//thread
/*Fin du thread Proximity*/

void proxi_start(void){
	 //Activer proximity --> appel du thread
	 chThdCreateStatic(waProximity, sizeof(waProximity), NORMALPRIO, Proximity, NULL);
}
uint8_t get_avoid(void){
	return avoid;
}

