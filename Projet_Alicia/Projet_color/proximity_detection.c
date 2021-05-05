#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>
#include <proximity_detection.h>

#include <sensors/proximity.h>
#include <msgbus/messagebus.h>
#include <motors.h>
#include <leds.h>

#include <stdbool.h>

/*
 * lignes où ajouter la gestion de quarts de tours: 193, 203, 216, 226, 241, 250
 *
 * */

static bool prox_detected = false;
static bool prox_front = false;
static bool prox_right_half = false; // Capteur de droite 45°
static bool prox_right_full = false; // capteur de droite 90°
static bool prox_left_half = false;  // capteur gauche 45°
static bool prox_left_full = false;  // capteur gauche 90°
static bool prox_back = false;

static int turn = 0;

//static int simple_dodge = 0; // faire des defines associés. Sert a enregistrer que le robot a tourné à cause d'un obstacle et le faire retourner dans sa direction initiale dès que possible
/*
 * THREADS
 */
/*Thread pour proximity*/
static THD_WORKING_AREA(waProximity, 4096);
static THD_FUNCTION(Proximity, arg) {

	 chRegSetThreadName(__FUNCTION__);
	  (void)arg;

/* Proximity detection variables*/
//	  bool prox_detected = false;
//	  bool prox_front = false;
//	  bool prox_right_half = false; // Capteur de droite 45°
//	  bool prox_right_full = false; // capteur de droite 90°
//	  bool prox_left_half = false;  // capteur gauche 45°
//	  bool prox_left_full = false;  // capteur gauche 90°
//	  bool prox_back = false;
//	  int simple_dodge = 0; // faire des defines associés. Sert a enregistrer que le robot a tourné à cause d'un obstacle et le faire retourner dans sa direction initiale dès que possible
	  int16_t temp = 0;  // je sais pas encore comment l'appeler/ si elle est vraiment utile. Sert à prendre la valeur absolue de la différence de valeurs entre les deux capteurs avant, afin de vérifier que les deux voient bien le meme obstacle
	  int16_t proxi_value = 0;
	  /*===========================*/

	while (!chThdShouldTerminateX())
	{
		for(int i=0; i<7; i++)
		{
			proxi_value= get_prox(i);
			//chprintf((BaseSequentialStream *)&SDU1, "green=%d \n", proxi_value);
			if(proxi_value>THRESHOLD)
			{
				prox_detected=true;
			}
		}

		if(prox_detected)
		{
				set_front_led(ON); // J'utilise front led pour vérifier que la detection fonctionne toujours au cas ou ce que je fais marche pas de son coté
		}
		else
		{
				set_front_led(OFF);
		}

		   //temp: Need to see if turns off when nothing is close, later, each sensor will likely have their own boolean and a case/switch
			 //to toggle different leds or different actions such as obstacle avoidance
			 /*======================*/


		/*
		 * SI c'est tout cassé, c'est très certainement ici que ça se passe
		 * */

//		if((get_prox(0)>AXIS_THRESHOLD) || (get_prox(7)>AXIS_THRESHOLD))
//		{
//			prox_front = true;
//			set_led(LED1, ON);    // n'est pas destiné à rester là, tout les bool auront leur propre fonction get_bool_prox
//		}
//		else
//		{
//			set_led(LED1, OFF);
//			prox_front = false;
//		}
//
//		if(get_prox(1)>THRESHOLD)
//		{
//			prox_right_half = true;
//			set_led(LED1, ON);
//			set_led(LED3, ON);
//		}
//		else
//		{
//			set_led(LED1, OFF);
//			set_led(LED3, OFF);
//			prox_right_half = false;
//		}
//
//		if(get_prox(2)>AXIS_THRESHOLD)
//		{
//			prox_right_full = true;
//			set_led(LED3, ON);
//		}
//		else
//		{
//			set_led(LED3, OFF);
//			prox_right_full = false;
//		}
//
//		if((get_prox(3)>AXIS_THRESHOLD) || (get_prox(4)>AXIS_THRESHOLD))
//		{
//			prox_back = true;
//			set_led(LED5, ON);
//		}
//		else
//		{
//			prox_back = false;
//			set_led(LED5, OFF);
//		}
//
//		if(get_prox(6)>THRESHOLD)
//		{
//			prox_left_half = true;
//			set_led(LED1, ON);
//			set_led(LED7, ON); // c'est peut etre plutot la led 5, je sais plus si elle sont dans le sens horaire ou antihoraire, tant que y'en a deux qui s'allume c'est que ça marche donc osef
//		}
//		else
//		{
//			set_led(LED1, OFF);
//			set_led(LED7, OFF);
//			prox_left_half = false;
//		}
//		if(get_prox(5)>AXIS_THRESHOLD)
//		{
//			prox_left_full = true;
//			set_led(LED7, ON);
//		}
//		else
//		{
//			set_led(LED7, OFF);
//			prox_left_full = false;
//		}


		/*Fin de si c'est tout cassé*/



		/*SI c'est tout cassé le retour*/
		if((get_prox(0)>AXIS_THRESHOLD) || (get_prox(7)>AXIS_THRESHOLD) /* || blue*/)
		{
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
						right_motor_set_speed(-100); //le robot se retourne pour trouver une sortie
						left_motor_set_speed(100);   // tourne de 180° vers la droite == 2 virages à droite pour l'instant

						// manque la gestion du demi tour parfait

//						right_motor_set_speed(100);
//						left_motor_set_speed(100);
						turn++;//enregistre un quart de tour vers la droite
						turn++;
					}
				}
				else //voie est libre à gauche, pas besoin de vérifier derrière, tourne à gauche
				{
					right_motor_set_speed(100);
					left_motor_set_speed(-100);
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
					right_motor_set_speed(100);
					left_motor_set_speed(-100);
					//ajouter gestion durée quart de tour
					//right_motor_set_speed(100);
					//left_motor_set_speed(100);
					turn--;
				}
			}
			else
			{
				right_motor_set_speed(-100);
				left_motor_set_speed(100);
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
				right_motor_set_speed(100);
				left_motor_set_speed(-100);
				//ajouter gestion durée quart de tour
				//right_motor_set_speed(100);
				//left_motor_set_speed(100);
				turn--;
			}
			else if(turn < 0) // s'il y a des virages à gauche à compenser
			{
				right_motor_set_speed(-100);
				left_motor_set_speed(100);
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


int get_prox_front(void)
{
	return prox_front;
}

int get_prox_right_half(void)
{
	return prox_right_half;
}

int get_prox_right_full(void)
{
	return prox_right_full;
}

int get_prox_left_full(void)
{
	return prox_left_full;
}

int get_prox_left_half(void)
{
	return prox_left_half;
}

int get_prox_back(void)
{
	return prox_back;
}

