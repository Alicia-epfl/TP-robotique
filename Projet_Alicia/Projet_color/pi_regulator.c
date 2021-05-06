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

//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(error < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < 0){
		sum_error = 0;
	}

	speed = KP * error + KI * sum_error;

if(speed <0){
	return (int16_t)0;
}else{
    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator,8192);//256 pas suffisant, 2048 non plus
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t measure = 0;
    uint8_t run = 0;


    while(1){
//    	 	run = get_run();
    	run=true;
    	 	if(run){
			measure	= VL53L0X_get_dist_mm();
			//computes the speed to give to the motors
			//distance_cm is modified by the image processing thread
			speed = pi_regulator(measure, GOAL_DISTANCE);


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
