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
	  int16_t proxi_value = 0;
	  uint8_t i=0;
	  /*===========================*/

while (!chThdShouldTerminateX()) {
  	  for(i=0; i<7; i++)
  	  	  {
     	   		proxi_value= get_prox(i);
//     	   		chprintf((BaseSequentialStream *)&SDU1, "green=%d \n", proxi_value);
     	    	 	if(proxi_value>THRESHOLD)
     	    	 	{
     	    	 		prox_detected=true;
     	    	 	}
     	     }

     	     if(prox_detected){
     	    	 	 set_led(LED7, ON);
//     	    	 	left_motor_set_speed(0);
//     	    	 	right_motor_set_speed(0);
     	     }else{
     	    	 	 set_led(LED7, OFF);
     	     }

     	   //temp: Need to see if turns off when nothing is close, later, each sensor will likely have their own boolean and a case/switch
     	     //to toggle different leds or different actions such as obstacle avoidance
     	     /*======================*/

     	    /* Delay of 250 milliseconds.*/
//     	    chThdSleepMilliseconds(100);
     	   prox_detected = false;

	}
}
/*Fin du thread Proximity*/

void proxi_start(void){
	 //Activer proximity --> appel du thread
	 chThdCreateStatic(waProximity, sizeof(waProximity), NORMALPRIO, Proximity, NULL);
}

