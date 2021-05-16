/**
 * Fichier: main.c
 * auteurs: Nicolas Nouel et Alicia Mauroux
 *
 * @brief
 * Ce fichier est le coeur de notre programme. C'est ici que se fait toutes les initialisations.
 */

//Convention rappel: "" --> fichiers dans notre projets; <> --> fichiers dans le dossier (librairies par exemple)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>

#include "main.h"

#include <leds.h>
#include <audio/play_melody.h>

/*includes pour noise detection*/
#include <audio/microphone.h>
#include <noise_detection.h>

#include "pi_regulator.h"
#include "process_image.h"

/*Includes for proximity sensors*/
#include <sensors/proximity.h>
#include <msgbus/messagebus.h>
#include "proximity_detection.h"
#include "sensors/VL53L0X/VL53L0X.h"


#define SEND_FROM_MIC

/*Définition des statiques pour la FSM*/
static uint8_t stop =false;		//définit si le robot doit avancer ou non
static uint8_t record_allowed = true;	//définit si on peut faire du traitement d'image ou non
static uint8_t avoid_allowed = true;	//définit si on peut faire une manoeuvre d'évitement
static uint8_t run =false; //variable qui va déterminer si notre e-puck lance le jeu ou est en "pause"
//cette variable s'active grâce au son!


/*BUS pour proximity*/
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

/*
 * :::::::::THREADS::::::::::::::::
 */
/*Thread pour la gestion de la FSM
 * Thread de gestion générale de nos états*/
static THD_WORKING_AREA(wafsm, 256);
static THD_FUNCTION(fsm, arg) {

	 chRegSetThreadName(__FUNCTION__);
	  (void)arg;

volatile uint8_t left=0, avoid = 0;
uint8_t game_over = 0, win = 0;

  while (1){
/*Gestion des moteurs*/
	  avoid = get_avoid();
	  left = get_left();

	  if(run){
		  //s'il a vu du bleu --> pas d'évitement
		  if(!left){
			  avoid_allowed = false;
		  }else{
			  stop = !run;
			  avoid_allowed = true;
		 }//left

		  //si il est en évitement --> pas de check des couleurs
		  if(avoid){
			  record_allowed = false;
		  }else{
			  record_allowed = true;
		  }//avoid

	//si run = 0
	  }else{
		  stop = true;
		  avoid_allowed = false;
		  record_allowed = false;
	  }//run
/*Fin de gestion des moteurs*/

 /*Game over ou Win*/
	  game_over = get_game_over();
	  win = get_win();

	  //dans le cas où la partie est terminée
	  //tout mettre en pause jusqu'à ce que le joueur puisse recommencer
	  if(win || game_over){
		  run =0;
		  stop=true;
		  avoid_allowed = false;
		  record_allowed = false;

	  }else if(run){
		  stop = !run;
		  avoid_allowed = true;
		  record_allowed = true;
	  }
	  chThdSleepMilliseconds(250);
  }//while(1)
}//thread
/*Fin du thread de fsm*/

/*Thread pour la gestion des leds*/
static THD_WORKING_AREA(waBlinker, 256);
static THD_FUNCTION(Blinker, arg) {

	 chRegSetThreadName(__FUNCTION__);
	  (void)arg;
	 //variables qui vont déterminer l'état des LEDs
	uint8_t toggle = 0, left=0, avoid = 0;
	uint8_t red_rgb =0, green_rgb=0, blue_rgb=0;
	uint8_t game_over = 0, win = 0;

  while (1){
	  left=get_left();
	  avoid = get_avoid();

	  game_over = get_game_over();
	  win = get_win();

/*variables permettant d'afficher sur les LEDs
 * la couleur bleu que le robot a repéré*/
	  red_rgb=get_red();
	  green_rgb=get_green();
	  blue_rgb=get_blue();

	  if(!left){
		  set_rgb_led(LED6, red_rgb, green_rgb, blue_rgb);
		  set_rgb_led(LED2, red_rgb, green_rgb, blue_rgb);
		  set_rgb_led(LED4, red_rgb, green_rgb, blue_rgb);
		  set_rgb_led(LED8, red_rgb, green_rgb, blue_rgb);

	  }else if(avoid){
			set_rgb_led(LED6, RED_ORANGE, GREEN_ORANGE, BLUE_ORANGE);
			set_rgb_led(LED2, RED_ORANGE, GREEN_ORANGE, BLUE_ORANGE);
			set_rgb_led(LED4, RED_ORANGE, GREEN_ORANGE, BLUE_ORANGE);
			set_rgb_led(LED8, RED_ORANGE, GREEN_ORANGE, BLUE_ORANGE);

	  }else if(win){
			set_rgb_led(LED6, NO_COL, GREEN, NO_COL);
			set_rgb_led(LED2, NO_COL, GREEN, NO_COL);
			set_rgb_led(LED4, NO_COL, GREEN, NO_COL);
			set_rgb_led(LED8, NO_COL, GREEN, NO_COL);

	  }else if(game_over){
			set_rgb_led(LED6, RED, NO_COL, NO_COL);
			set_rgb_led(LED2, RED, NO_COL, NO_COL);
			set_rgb_led(LED4, RED, NO_COL, NO_COL);
			set_rgb_led(LED8, RED, NO_COL, NO_COL);

	  }else if(run){
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
		}//toggle
	  }else{
			set_rgb_led(LED6, RED_CYAN, GREEN_CYAN, BLUE_CYAN);
			set_rgb_led(LED2, RED_CYAN, GREEN_CYAN, BLUE_CYAN);
			set_rgb_led(LED4, RED_CYAN, GREEN_CYAN, BLUE_CYAN);
			set_rgb_led(LED8, RED_CYAN, GREEN_CYAN, BLUE_CYAN);
	  }//gestion de leds

    /* Délai de 250 millisecondes.*/
    chThdSleepMilliseconds(250);
  }//while(1)
}//thread
/*Fin du thread de BODY LED*/

/*
 * ::::::::::::FUNCTIONS::::::::::::::::::::
 */
// @brief
// Initialise les fonctions requisent par __libc_init_array
void _init(void) {}

// @brief
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
// @brief
/*Permet les "printf" via le terminal
 * Version pour uint8_t:*/
void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}
// @brief
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
// @brief
/*fonctions qui renvoient aux fichiers.c les valeurs dictées par la fsm
 * Permet le contrôle des différents segments*/
uint8_t get_stop_fsm(void){
	return stop;
}
uint8_t get_record_allowed_fsm(void){
	return record_allowed;
}
uint8_t get_avoid_allowed_fsm(void){
	return avoid_allowed;
}
// @brief
/*fonction de la gestion de run
 * si run est activé, le jeu est lancé
 * si run est à zéro, le robot est en "standbye"*/
void toggle_run(void){
		run = !run;
}

/*
 * ::::::::MAIN::::::::::
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

	//Démarre les micro
	mic_start(&processAudioData);

	//Démarre les threads pour le régulateur pi et le image processing
	pi_regulator_start();
	process_image_start();

	 //Activer proximity --> appel du thread
	 proxi_start();

	//Clignotement BODY LED --> appel du thread
	 chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, Blinker, NULL);

	 //Lancement du thread fsm
	 chThdCreateStatic(wafsm, sizeof(wafsm), NORMALPRIO, fsm, NULL);

	//Démarre le thread des melodies
	playMelodyStart();

    /* Boucle INFINIE. */
    while (1){
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
