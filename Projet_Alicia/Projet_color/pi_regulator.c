#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include "noise_detection.h"

#include "sensors/VL53L0X/VL53L0X.h"

static float sum_error = 0;

//simple PI regulator implementation
int16_t pi_regulator(uint16_t distance, uint16_t goal){

	float error = 0;
	float speed = 0;



	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
			return 0;
	}

	sum_error += error;
	sum_error = 0.5*sum_error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

	return (int16_t)speed;

}

static THD_WORKING_AREA(waPiRegulator,1024);//256 pas suffisant, 2048 non plus
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    volatile int16_t speed = 0;
    int16_t measure = 0;
    uint8_t run = 0;


    while(1){
//    	 	run = get_run();
//    	run = true;
    		time = chVTGetSystemTime();
    	 	if(1){
			measure	= VL53L0X_get_dist_mm();
			//computes the speed to give to the motors
			//distance_cm is modified by the image processing thread
			speed = pi_regulator(measure, GOAL_DISTANCE);
//			if(speed<0){speed = 0;}


			//applies the speed from the PI regulator and the correction for the rotation
			right_motor_set_speed(speed);
			left_motor_set_speed(speed);
    	 	}else{
    	 		right_motor_set_speed(0);//enlever les Magic numbers
    	 		left_motor_set_speed(0);
    	 	}
        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));


    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
