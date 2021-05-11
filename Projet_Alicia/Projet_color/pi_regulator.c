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
/*Define pour les demi-tours*/
#define NSTEP_ONE_TURN      1000								// number of step for 1 turn of the motor
#define WHEEL_PERIMETER    	13//12.5663706144f					// [cm] -->4*pi --> 4cm est la mesure du diamètre des roues
#define WHEEL_OFFSET			2.65								//[cm] --> de combien la roue est décalé par rapport au centre du robot

static float sum_error = 0;
static uint8_t done_l = false;

//simple PI regulator implementation
int16_t pi_regulator(uint16_t distance){

	float error = 0;
	float speed = 0;



	error = distance - GOAL_DISTANCE;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(error < ERROR_THRESHOLD){
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
    uint8_t right =0, left=0;


    while(1){
//    	 	run = get_run();
//    	run = true;
    		time = chVTGetSystemTime();
    		right=get_right(); //si !right --> alors il tourne à droite donc ne doit pas avancer ou être arrêté
    		left=get_left(); //si !left --> alors il tourne à gauche donc ne doit pas avancer ou être arrêté
    	 	if(run && left){
			measure	= VL53L0X_get_dist_mm();
			//computes the speed to give to the motors
			//distance_cm is modified by the image processing thread
			speed = pi_regulator(measure);
//			if(speed<0){speed = 0;}


			//applies the speed from the PI regulator and the correction for the rotation
			right_motor_set_speed(speed);
			left_motor_set_speed(speed);
    	 	}else if(left){
    	 		right_motor_set_speed(0);//enlever les Magic numbers
    	 		left_motor_set_speed(0);

    	 	}else if(!left){//run && !right				WARNING
    	 		turn(PI/2);
    	 		chThdSleepMilliseconds(500);
    	 		done_l = false;
    	 	}

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void turn(float alpha){
	int32_t nstep = 0;
	volatile int32_t right_pos=0, left_pos=0;//Finalement prendre les 2 pour plus de précisions

	nstep = (int32_t)((alpha*WHEEL_OFFSET*NSTEP_ONE_TURN)/WHEEL_PERIMETER);

	/*Option avec SET_POS*/
//	left_motor_set_pos(nstep);
//	right_motor_set_pos(nstep);
//
//	if(alpha>0){//gauche
//		right_motor_set_speed(600);
//		left_motor_set_speed(-600);
//	}else{//droite
//		right_motor_set_speed(-600);
//		left_motor_set_speed(600);
//	}

/*Option inspirée du TP2*/
	left_motor_set_pos(0);
	right_motor_set_pos(0);

	while((abs(right_pos) < abs(nstep)) && (abs(left_pos) < abs(nstep))){

		right_pos = right_motor_get_pos();
		left_pos = left_motor_get_pos();

		if(alpha > 0){//gauche
			right_motor_set_speed(100);
			left_motor_set_speed(-100);
		}else{//droite
			right_motor_set_speed(-100);
			left_motor_set_speed(100);
		}

	}
	done_l = true;
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
uint8_t get_done_left(void){

	return done_l;
}
