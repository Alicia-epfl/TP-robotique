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

#include <leds.h>

static int turn = false; //pour garder en mémoire les tours faits --> MISE EN ATTENTE DE CETTE IDEE

/*
 * THREADS
 */
/*Thread pour proximity*/
static THD_WORKING_AREA(waProximity, 4096);
static THD_FUNCTION(Proximity, arg) {

	 chRegSetThreadName(__FUNCTION__);
	  (void)arg;

/* Proximity detection variables*/
	  bool prox_detected = false;
	  int16_t proxi_value = 0;
	  uint8_t i=0;
	  uint16_t rightspeed = 0, leftspeed = 0;//Nico
	  /*===========================*/

while (!chThdShouldTerminateX()) {
	//What is that?
	leftspeed = MOTOR_SPEED_LIMIT - get_prox(0)*2 - get_prox(1);
	rightspeed = MOTOR_SPEED_LIMIT - get_prox(7)*2 - get_prox(6);
	right_motor_set_speed(rightspeed);
	left_motor_set_speed(leftspeed);
	//Fin de What is that?

	/*Fonction d'évitement*/
	if((get_prox(0)>AXIS_THRESHOLD) || (get_prox(7)>AXIS_THRESHOLD) /* || blue*/){
				prox_front = true;  // finalement, n'est pas très utile je crois rip
				right_motor_set_speed(0); //le robot s'arrete avant de chercher son chemin
				left_motor_set_speed(0);
				set_led(LED1, ON);// n'est pas destiné à rester là je pense


				//pour l'instant je me limite aux axes principaux pour plus de façilité

				if(get_prox(2)>AXIS_THRESHOLD)//obstacle à droite
				{
					prox_right_full = true; //useless pour l'instant
					set_led(LED3, ON);

					if(get_prox(5)>AXIS_THRESHOLD)//obstacle à gauche
					{
						prox_left_full = true; //useless pour l'instant
						set_led(LED7, ON);

						if((get_prox(3)>AXIS_THRESHOLD) || (get_prox(4)>AXIS_THRESHOLD))//obstacle derrière == robot encerclé
						{
							prox_back = true; //useless pour l'instant
							set_led(LED5, ON);
						}
						else // la voie est libre derrière le robot, il se retourne pour progresser dans le parcours
						{
	//						right_motor_set_speed(-100); //le robot se retourne pour trouver une sortie
	//						left_motor_set_speed(100);   // tourne de 180° vers la droite == 2 virages à droite pour l'instant
							back = false;

							// manque la gestion du demi tour parfait

	//						right_motor_set_speed(100);
	//						left_motor_set_speed(100);
							turn++;//enregistre un quart de tour vers la droite
							turn++;
						}
					}
					else //voie est libre à gauche, pas besoin de vérifier derrière, tourne à gauche
					{
	//					right_motor_set_speed(100);
	//					left_motor_set_speed(-100);
						left=false;
						//ajouter gestion durée quart de tour
	//					right_motor_set_speed(100);
	//					left_motor_set_speed(100);
						turn--;
					}
				}
				else if(turn>0) //turn > 0 == virage à droite non compensé
				{
					if(get_prox(5)<AXIS_THRESHOLD) // si y'a pas d'obstacle à gauche, tourner à gauche
						// peut être finalement se mettre à utiliser mes ptits booleens, appeler une fonction à chaque fois ça pèse un peu plus lourd qu'appeler un booleen non ?
					{
	//					right_motor_set_speed(100);
	//					left_motor_set_speed(-100);
						left=false;
						//ajouter gestion durée quart de tour
						//right_motor_set_speed(100);
						//left_motor_set_speed(100);
						turn--;
					}
				}
				else
				{
	//				right_motor_set_speed(-100);
	//				left_motor_set_speed(100);
					right=false;
					//ajouter gestion durée quart de tour
					//right_motor_set_speed(100);
					//left_motor_set_speed(100);
					turn++;
				}



			}
			else if(((get_prox(2) < AXIS_THRESHOLD) || get_prox(5)<AXIS_THRESHOLD) &&(turn != 0)) // si la voie est libre devant et sur au moins un côté et qu'il y a des virages à compenser
			{
				if(turn > 0) // s'il y a des virages a droite à compenser
				{
	//				right_motor_set_speed(100);
	//				left_motor_set_speed(-100);
					right=false;
					//ajouter gestion durée quart de tour
					//right_motor_set_speed(100);
					//left_motor_set_speed(100);
					turn--;
				}
				else if(turn < 0) // s'il y a des virages à gauche à compenser
				{
	//				right_motor_set_speed(-100);
	//				left_motor_set_speed(100);
					left=false;
					//ajouter gestion durée quart de tour
					//right_motor_set_speed(100);
					//left_motor_set_speed(100);
					turn++;
				}
			}
			else
			{
				set_led(LED1, OFF);
				prox_front = false;
			}




			/*===========================*/

			/* Delay of 250 milliseconds.*/
			chThdSleepMilliseconds(500);

			prox_detected = false;

	}
}
/*Fin du thread Proximity*/

void proxi_start(void){
	 //Activer proximity --> appel du thread
	 chThdCreateStatic(waProximity, sizeof(waProximity), NORMALPRIO, Proximity, NULL);
}

