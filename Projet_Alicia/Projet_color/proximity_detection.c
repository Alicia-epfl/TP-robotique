#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "sensors/proximity.h"
#include <main.h>
#include <proximity_detection.h>

#include <leds.h>


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

parameter_namespace_t parameter_root, aseba_ns;

messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;

int threshold = 1000;

void proximity_detection(void){

	void proximity_start(void); //Démarre le module de detection de proximité

	void calibrate_ir(void); // Calibre les capteurs
	while(1){
		if((prox_values.delta[0]<=threshold) || (prox_values.delta[1]<=threshold) || (prox_values.delta[2]<=threshold) || (prox_values.delta[3]<=threshold) || (prox_values.delta[4]<=threshold) || (prox_values.delta[5]<=threshold) || (prox_values.delta[6]<=threshold) || (prox_values.delta[7]<=threshold) ){
			set_led(LED7, ON);
		}
		else{
			set_led(LED7, OFF);
		}
	}

}

