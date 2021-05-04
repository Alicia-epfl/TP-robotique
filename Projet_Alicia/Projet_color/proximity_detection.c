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
	  bool prox_front = false;
	  bool prox_right_half = false; // Capteur de droite 45°
	  bool prox_right_full = false; // capteur de droite 90°
	  bool prox_left_half = false;  // capteur gauche 45°
	  bool prox_left_full = false;  // capteur gauche 90°
	  bool prox_back = false;
	  int simple_dodge = 0; // faire des defines associés. Sert a enregistrer que le robot a tourné à cause d'un obstacle et le faire retourner dans sa direction initiale dès que possible
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
				set_front_led(ON); // J'utilise body led pour vérifier que la detection fonctionne toujours au cas ou ce que je fais marche pas de son coté
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

		if((get_prox(0)>THRESHOLD) || (get_prox(7)>THRESHOLD))
		{
			prox_front = true;
			set_led(LED7, ON);    // n'est pas destiné à rester là, tout les bool auront leur propre fonction get_bool_prox
		}
		else
		{
			set_led(LED7, OFF);
			prox_front = false;
		}
		if(get_prox(1)>THRESHOLD)
		{
			prox_right_half = true;
			set_led(LED7, ON);
			set_led(LED1, ON); // c'est peut etre plutot la led 5, je sais plus si elle sont dans le sens horaire ou antihoraire, tant que y'en a deux qui s'allume c'est que ça marche donc osef
		}
		else
		{
			set_led(LED7, OFF);
			set_led(LED1, OFF);
			prox_right_half = false;
		}
		if(get_prox(2)>THRESHOLD)
		{
			prox_right_full = true;
			set_led(LED1, ON);
		}
		else
		{
			set_led(LED1, OFF);
			prox_right_full = false;
		}

		if((get_prox(3)>THRESHOLD) || (get_prox(4)>THRESHOLD))
		{
			prox_back = true;
			set_led(LED3, ON);
		}
		else
		{
			prox_back = false;
			set_led(LED3, OFF);
		}

		if(get_prox(6)>THRESHOLD)
		{
			prox_left_half = true;
			set_led(LED3, ON);
			set_led(LED5, ON); // c'est peut etre plutot la led 5, je sais plus si elle sont dans le sens horaire ou antihoraire, tant que y'en a deux qui s'allume c'est que ça marche donc osef
		}
		else
		{
			set_led(LED3, OFF);
			set_led(LED5, OFF);
			prox_left_half = false;
		}
		if(get_prox(5)>THRESHOLD)
		{
			prox_left_full = true;
			set_led(LED5, ON);
		}
		else
		{
			set_led(LED5, OFF);
			prox_left_full = false;
		}


		/*Fin de si c'est tout cassé*/

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

