//Rappel: "" --> fichiers dans notre projets; <> --> fichiers dans le dossier (librairies par exemple)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
/*includes pour noise detection*/
#include <audio/microphone.h>
#include <noise_detection.h>
#include <fft.h>
#include <arm_math.h>
/*===============*/
#include <pi_regulator.h>
#include <process_image.h>
#include <leds.h>
/*Includes for proximity sensors*/
#include <sensors/proximity.h>
#include <msgbus/messagebus.h>
#include "proximity_detection.h"
/*============================*/
#include <audio/play_melody.h>
/*============================*/
#include "sensors/VL53L0X/VL53L0X.h"
//#include <i2c_bus.h>

#define SEND_FROM_MIC

/*BUS pour proximity*/
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

/*
 * THREADS
 */
/*Thread pour le clignotement de BODY LED*/
static THD_WORKING_AREA(waBlinker, 256);//128 mais 256 juste pour le printf
static THD_FUNCTION(Blinker, arg) {

	 chRegSetThreadName(__FUNCTION__);
	  (void)arg;
uint8_t toggle = 0, right=0, left=0;
uint8_t red_rgb =0, green_rgb=0, blue_rgb=0;

  while (!chThdShouldTerminateX()) {
    /* Toggling LEDs while the main thread is busy   .*/
//    set_body_led(TOGGLE);
	  right = get_right();
	  left=get_left();
	  red_rgb=get_red();
	  green_rgb=get_green();
	  blue_rgb=get_blue();

//	  chprintf((BaseSequentialStream *)&SDU1, "R=%3d\r", right);
	  if((!right)||(!left)){
		  set_rgb_led(LED6, red_rgb, green_rgb, blue_rgb);
		  set_rgb_led(LED2, red_rgb, green_rgb, blue_rgb);
		  set_rgb_led(LED4, red_rgb, green_rgb, blue_rgb);
		  set_rgb_led(LED8, red_rgb, green_rgb, blue_rgb);

	  }else{
		if(toggle){
			set_rgb_led(LED6, RED_CYAN, GREEN_CYAN, BLUE_CYAN);
			set_rgb_led(LED2, RED_CYAN, GREEN_CYAN, BLUE_CYAN);
			set_rgb_led(LED4, RED_MAUVE, GREEN_MAUVE, BLUE_MAUVE);
			set_rgb_led(LED8, RED_MAUVE, GREEN_MAUVE, BLUE_MAUVE);
			toggle =!toggle;
		}else{
				set_rgb_led(LED4, RED_CYAN, GREEN_CYAN, BLUE_CYAN);
				set_rgb_led(LED8, RED_CYAN, GREEN_CYAN, BLUE_CYAN);
				set_rgb_led(LED2, RED_MAUVE, GREEN_MAUVE, BLUE_MAUVE);
				set_rgb_led(LED6, RED_MAUVE, GREEN_MAUVE, BLUE_MAUVE);
				toggle =!toggle;
		}
	  }

    /* Delay of 250 milliseconds.*/
    chThdSleepMilliseconds(250);
  }
}
/*Fin du thread de BODY LED*/


/*
 * FUNCTIONS
 */
// Init function required by __libc_init_array
void _init(void) {}

/*Permet la communication USB et Bluetooth*/
static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

/*Permet les "printf" via le terminal
 * Version pour uint8_t:*/
void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}
/*Version pour les floats:*/
void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size)
{
	chSequentialStreamWrite(out, (uint8_t*)"START", 5);
	chSequentialStreamWrite(out, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite(out, (uint8_t*)data, sizeof(float) * size);
}
// @brief
/*Sequence d'animation de body led au démarrage du programme */
void readyAnimation(void) {

	set_body_led(ON);
	chThdSleepMilliseconds(100);
	set_body_led(OFF);
	chThdSleepMilliseconds(50);
	set_body_led(ON);
	chThdSleepMilliseconds(100);
	set_body_led(OFF);
	chThdSleepMilliseconds(50);
	set_body_led(ON);
	chThdSleepMilliseconds(100);
	set_body_led(OFF);
	chThdSleepMilliseconds(50);
	set_body_led(ON);

//attendre avant de lancer
	chThdSleepMilliseconds(500);
	set_body_led(OFF);
}

/*
 * MAIN
 */
int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //Démarre le bus
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    //Démarre la serial communication
    serial_start();
    //Démarre la communication USB
    usb_start();
    //pour la musique
    dac_start();
    //enclenche la connection SPI pour pouvoir utiliser les LEDS RGB
    	spi_comm_start();
    //Démarre la camera
    dcmi_start();
	po8030_start();


	/*Démarre les capteurs IR*/
	proximity_start();
	calibrate_ir();

	//initialise les moteurs
	motors_init();

	readyAnimation();

	//Démarre VL53L0X
	VL53L0X_start();



	//Démarre le thread des melodies
	playMelodyStart();

	//Démarre les threads pour le régulateur pi et le image processing
	pi_regulator_start();
	process_image_start();

	//Démarre les micro
	mic_start(&processAudioData);

	//Clignotement BODY LED --> appel du thread
	 chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, Blinker, NULL);

	 //Activer proximity --> appel du thread
	 proxi_start();


    /* Infinite loop. */
    while (1) {
    	//Attend 1 seconde
    	chThdSleepMilliseconds(1000);
    }
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
