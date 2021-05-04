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


//#include <audio/play_melody.h> //COMMENT DEMARRER UNE MELODIE???


/*defines pour NOISE detection
 * !!!!!!!!!!IMPORTANT!!!!!!!!!
 * SEND_FROM_MIC permet d'utiliser les micro de e-puck alors que
 * DOUBLE_BUFFERING prends les infos de l'ordinateur!
 * ==================================================================
 * Visiblement il faut garder DOUBLE_BUFFERING quand même? --> Je comprends pas pourquoi, askip on a vu en TP*/
#define SEND_FROM_MIC
#define DOUBLE_BUFFERING
/*end of defines pour noise detection*/

/*BUS pour proximity?*/
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

/*
 * THREADS
 */
/*Thread pour le clignotement de BODY LED*/
static THD_WORKING_AREA(waBlinker, 128);
static THD_FUNCTION(Blinker, arg) {

	 chRegSetThreadName(__FUNCTION__);
	  (void)arg;
uint8_t toggle = 0, right=0;

  while (!chThdShouldTerminateX()) {
    /* Toggling LEDs while the main thread is busy   .*/
//    set_body_led(TOGGLE);
	  right = get_right();
	  if(right){
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
/*Thread pour PROXIMITY*/
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
/*
*	Sends floats numbers to the computer
*/
void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size)
{
	chSequentialStreamWrite(out, (uint8_t*)"START", 5);
	chSequentialStreamWrite(out, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite(out, (uint8_t*)data, sizeof(float) * size);
}
// @brief
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

    //start bus
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //pour la musique
    dac_start();
    //enclenche la connection SPI pour pouvoir utiliser les LEDS RGB
    	spi_comm_start();
    //starts the camera
    dcmi_start();
	po8030_start();

	/*Starts the IR sensors*/
	proximity_start();
	calibrate_ir();

	//inits the motors
	motors_init();

	readyAnimation();

	//stars the threads for the pi regulator and the processing of the image
	//	pi_regulator_start();
	process_image_start();

	//start the melody
	playMelodyStart();



	//Clignotement BODY LED --> appel du thread
	 chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, Blinker, NULL);


	 //Activer proximity --> appel du thread
	 proxi_start();

//
	 /*===============================================FROM TP5 AUDIO PROCESSING =============================================*/
	     //send_tab is used to save the state of the buffer to send (double buffering)
	     //to avoid modifications of the buffer while sending it
	     static float send_tab[FFT_SIZE];


	 #ifdef SEND_FROM_MIC
	     //starts the microphones processing thread.
	     //it calls the callback given in parameter when samples are ready
	     mic_start(&processAudioData);
	 #endif  /* SEND_FROM_MIC */

	     /* Infinite loop. */
	     while (1) {
	 #ifdef SEND_FROM_MIC
	         //waits until a result must be sent to the computer
//	         wait_send_to_computer();
	 #ifdef DOUBLE_BUFFERING
	         //we copy the buffer to avoid conflicts
	         arm_copy_f32(get_audio_buffer_ptr(LEFT_OUTPUT), send_tab, FFT_SIZE);

	 #else
	         SendFloatToComputer((BaseSequentialStream *) &SD3, get_audio_buffer_ptr(LEFT_OUTPUT), FFT_SIZE);
	 #endif  /* DOUBLE_BUFFERING */
	 #else
	         float* bufferCmplxInput = get_audio_buffer_ptr(LEFT_CMPLX_INPUT);
	         float* bufferOutput = get_audio_buffer_ptr(LEFT_OUTPUT);

	         uint16_t size = ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, bufferCmplxInput, FFT_SIZE);

	         if(size == FFT_SIZE){
	             /*
	             *   Optimized FFT
	             */

	             doFFT_optimized(FFT_SIZE, bufferCmplxInput);

	             /*
	             *   End of optimized FFT
	             */

	             arm_cmplx_mag_f32(bufferCmplxInput, bufferOutput, FFT_SIZE);

	             SendFloatToComputer((BaseSequentialStream *) &SD3, bufferOutput, FFT_SIZE);

	         }
	 #endif  /* SEND_FROM_MIC */
	     }
	     /*=======================================END OF TP5 IMPORTATION================================*/

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
//    	playMelody(WE_ARE_THE_CHAMPIONS, ML_SIMPLE_PLAY, NULL);
    	chThdSleepMilliseconds(1000);
    }
}


//Pas encore trop compris sa fonction? A noter si tu sais?
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
