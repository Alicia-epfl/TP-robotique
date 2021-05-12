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
	//		//What is that?
	//		leftspeed = MOTOR_SPEED_LIMIT - get_prox(0)*2 - get_prox(1);
	//		rightspeed = MOTOR_SPEED_LIMIT - get_prox(7)*2 - get_prox(6);
	//		right_motor_set_speed(rightspeed);
	//		left_motor_set_speed(leftspeed);
	//		//Fin de What is that?

			/*===========================*/
			/*Fonction d'évitement*/

			//OBSTACLE DEVANT
			if((get_prox(0)>AXIS_THRESHOLD) || (get_prox(7)>AXIS_THRESHOLD)){
				//le robot s'arrete avant de chercher son chemin
				right_motor_set_speed(0);
				left_motor_set_speed(0);

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
						}

					//OBSTACLE DEVANT + A DROITE
					}else{
						//tourne à gauche
						turn(PI/2);
					}
				//OBSTACLE DEVANT
				}else{
					//tourne à droite
					turn(-PI/4);
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

