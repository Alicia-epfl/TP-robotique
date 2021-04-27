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


/*defines pour noise detection*/
//#define SEND_FROM_MIC
#define DOUBLE_BUFFERING
/*end of defines pour noise detection*/
/*
 * THREADS
 */
/*Thread pour le clignotement de BODY LED*/
static THD_WORKING_AREA(waBlinker, 128);
static THD_FUNCTION(Blinker, arg) {

	 chRegSetThreadName(__FUNCTION__);
	  (void)arg;

  while (!chThdShouldTerminateX()) {
    /* Toggling BODY LED while the main thread is busy.*/
    set_body_led(TOGGLE);

    /* Delay of 250 milliseconds.*/
    chThdSleepMilliseconds(500);
  }
}
/*Fin du thread de BODY LED*/


/*
 * FUNCTIONS
 */
// Init function required by __libc_init_array
void _init(void) {}

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


// Simple delay function
void delay(unsigned int n)
{
    while (n--) {
        __asm__ volatile ("nop");
    }
}


void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}


/*
 * MAIN
 */
int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();

	//inits the motors
	motors_init();

//	spi_comm_start();//RGB LED

	//stars the threads for the pi regulator and the processing of the image
//	pi_regulator_start();
	process_image_start();

	//Clignotement BODY LED --> appel du thread
	 chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, Blinker, NULL);


	 /*===============================================FROM TP5 AUDIO PROCESSING =============================================*/
	 //temp tab used to store values in complex_float format
	     //needed bx doFFT_c
	     static complex_float temp_tab[FFT_SIZE];
	     //send_tab is used to save the state of the buffer to send (double buffering)
	     //to avoid modifications of the buffer while sending it
	     static float send_tab[FFT_SIZE];

//NOTE==========IMPORTANT===================
//==========Comme on définit SEND_FROM_MIC à priori, on peut juste mettre la ligne mic_start!!!
	     //pareil pour en bas
//===============================================================================================
	 #ifdef SEND_FROM_MIC
	     //starts the microphones processing thread.
	     //it calls the callback given in parameter when samples are ready
	     mic_start(&processAudioData);
	 #endif  /* SEND_FROM_MIC */

	     /* Infinite loop. */
	     while (1) {
	 #ifdef SEND_FROM_MIC
	         //waits until a result must be sent to the computer
	         wait_send_to_computer();
	 #ifdef DOUBLE_BUFFERING
	         //we copy the buffer to avoid conflicts
	         arm_copy_f32(get_audio_buffer_ptr(LEFT_OUTPUT), send_tab, FFT_SIZE);
	         SendFloatToComputer((BaseSequentialStream *) &SD3, send_tab, FFT_SIZE);
	 #else
	         SendFloatToComputer((BaseSequentialStream *) &SD3, get_audio_buffer_ptr(LEFT_OUTPUT), FFT_SIZE);
	 #endif  /* DOUBLE_BUFFERING */
	 #else
	         //time measurement variables
	         volatile uint16_t time_fft = 0;
	         volatile uint16_t time_mag  = 0;

	         float* bufferCmplxInput = get_audio_buffer_ptr(LEFT_CMPLX_INPUT);
	         float* bufferOutput = get_audio_buffer_ptr(LEFT_OUTPUT);

	         uint16_t size = ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, bufferCmplxInput, FFT_SIZE);

	         if(size == FFT_SIZE){
	             /*
	             *   Optimized FFT
	             */

	             chSysLock();


	             doFFT_optimized(FFT_SIZE, bufferCmplxInput);


	             chSysUnlock();

	             /*
	             *   End of optimized FFT
	             */



	             chSysLock();


	             arm_cmplx_mag_f32(bufferCmplxInput, bufferOutput, FFT_SIZE);


	             chSysUnlock();

	             SendFloatToComputer((BaseSequentialStream *) &SD3, bufferOutput, FFT_SIZE);
	             //chprintf((BaseSequentialStream *) &SDU1, "time fft = %d us, time magnitude = %d us\n",time_fft, time_mag);

	         }
	 #endif  /* SEND_FROM_MIC */
	     }
	     /*=======================================END OF TP5 IMPORTATION================================*/

    /* Infinite loop. */
    while (1) {
    	//waits 1 second

    	chThdSleepMilliseconds(1000);
    }
}




#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
