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

		while(avoid_allowed){
			/*Fonction d'évitement*/

			//OBSTACLE DEVANT
			if((get_prox(0)>AXIS_THRESHOLD) || (get_prox(7)>AXIS_THRESHOLD)){
				//indique qu'on est en processus d'évitement
				avoid = true;


				//OBSTACLE DEVANT + A DROITE
				if(get_prox(2)>AXIS_THRESHOLD){

				//OBSTACLE DEVANT + A DROITE + A GAUCHE
					if(get_prox(5)>AXIS_THRESHOLD){

				//OBSTACLE PARTOUT --> ENCERCLE
						if((get_prox(3)>AXIS_THRESHOLD) || (get_prox(4)>AXIS_THRESHOLD)){
							//Le jeu est perdu --> game over
							game_over = true;
						}
						//OBSTACLE DEVANT + A DROITE + A GAUCHE
						else{
							//effectue un demi-tour
							turn(PI);
							//libère run
//							avoid = false;
							chThdSleepMilliseconds(100);
						}

					//OBSTACLE DEVANT + A DROITE
					}else{
						//tourne à gauche
						turn(PI/2);
						//libère run
//						avoid = false;
						chThdSleepMilliseconds(100);
					}
				//OBSTACLE DEVANT
				}else{
					//tourne à droite
					turn(-PI/2);

					//libère run
					chThdSleepMilliseconds(100);
//					avoid = false;
				}
			}else{
//				avoid = false;
				set_body_led(ON);
			}//if obstacle devant
			/*===========================*/

			/* Delai de 500 millisecondes.*/
			chThdSleepMilliseconds(500);
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

