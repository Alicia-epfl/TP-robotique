#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <leds.h>


#define TOGGLE 2

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
/*
 * Blinker thread.
 */
static THD_WORKING_AREA(waBlinker, 128);
static THD_FUNCTION(Blinker, arg) {

	 chRegSetThreadName(__FUNCTION__);
	  (void)arg;

  while (!chThdShouldTerminateX()) {
    /* Toggling a LED while the main thread is busy.*/
    set_body_led(TOGGLE);

    /* Delay of 250 milliseconds.*/
    chThdSleepMilliseconds(500);
  }
}
/*
 * Main application.
 */
int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();

//    thread_t *blink = chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO + 1, Blinker, NULL);
    chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, Blinker, NULL);

    while(true){
    		//set_body_led(TOGGLE);
    		chThdSleepMilliseconds(500);


//    		  /* Stopping the blinker thread and retrieving the number of times
//    		     the LED toggled.*/
//    		  chThdTerminate(blink);
    }


}
