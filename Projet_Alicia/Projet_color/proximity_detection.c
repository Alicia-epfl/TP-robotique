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
/*
 * THREADS
 */
/*Thread pour proximity*/
static THD_WORKING_AREA(waProximity, 4096);
static THD_FUNCTION(Proximity, arg) {

	 chRegSetThreadName(__FUNCTION__);
	  (void)arg;

/* Proximity detection variables*/
	  uint8_t run = 0;
//	  uint16_t rightspeed = 0, leftspeed = 0;//Nico
	  /*===========================*/

	while (1){
		run = get_run();

		while(run){
			/*Fonction d'évitement*/

			//OBSTACLE DEVANT
			if((get_prox(0)>AXIS_THRESHOLD) || (get_prox(7)>AXIS_THRESHOLD)){
				//le robot s'arrete avant de chercher son chemin
				right_motor_set_speed(0);
				left_motor_set_speed(0);

				//indique qu'on est en processus d'évitement
				avoid = true;

				//OBSTACLE DEVANT + A DROITE
				if(get_prox(2)>AXIS_THRESHOLD){

				//OBSTACLE DEVANT + A DROITE + A GAUCHE
					if(get_prox(5)>AXIS_THRESHOLD){

				//OBSTACLE PARTOUT --> ENCERCLE
						if((get_prox(3)>AXIS_THRESHOLD) || (get_prox(4)>AXIS_THRESHOLD)){
							run = 0;	//RETURN RUN A FALSE											WARNING
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
					turn(-PI/4);
					//libère run
					avoid = false;
				}
			}
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

