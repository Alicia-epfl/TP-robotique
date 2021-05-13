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
/*pour obtenir le avoid*/
#include "proximity_detection.h"

#include <leds.h>



/*Define pour les demi-tours*/
#define NSTEP_ONE_TURN      1000								// number of step for 1 turn of the motor
#define WHEEL_PERIMETER    	13//12.5663706144f					// [cm] -->4*pi --> 4cm est la mesure du diamètre des roues
#define WHEEL_OFFSET			2.65								//[cm] --> de combien la roue est décalé par rapport au centre du robot

/*Initialisation des statics*/
static float sum_error = 0;//pour le pi
static float sum_error_rot = 0;//pour le pi de rotation
static uint8_t done_l = false;//pour indiquer à process image que la rotation a bien été effectuée

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

//simple PI regulator for rotation implementation
int16_t pi_rotator(uint16_t position, int32_t nstep){

	float error = 0;
	float speed = 0;



	error = nstep - position;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if((error) < ERROR_THRE_ROT){
			return 200;//faible vitesse pour finir correctement le tour
	}

	sum_error_rot += error;
	sum_error_rot = 0.5*sum_error_rot;//division par 2 dans l'idée d'une moyenne

	//on pose un maximum et un minimum à la somme pour éviter une croissance non contrôlée
	if(sum_error_rot > MAX_SUM_ERROR){
		sum_error_rot = MAX_SUM_ERROR;
	}else if(sum_error_rot < -MAX_SUM_ERROR){
		sum_error_rot = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error_rot;



	return (int16_t)speed;


}

static THD_WORKING_AREA(waPiRegulator,1024);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    volatile int16_t speed = 0;
    int16_t measure = 0;
    uint8_t stop = 0, left= 0;
    volatile uint8_t avoid=0;


    while(1){
		avoid = get_avoid();
		//si il est en manoeuvre d'évitement, pas besoin d'entrer dans ce thread
    		if(!avoid){
				stop = get_stop_fsm();

				time = chVTGetSystemTime();//pour le sleep Window d'en dessous

				left=get_left(); //si !left --> alors il tourne à gauche donc ne doit pas avancer ou être arrêté

				//si il est en mouvement est n'a pas vu de bleu
				if(!stop){
					//mesure de la distance à un obstacle pour réguler la vitesse grâce au PI
				measure	= VL53L0X_get_dist_mm();
				speed = pi_regulator(measure);

				//applique la vitesse calculée aux moteurs
				right_motor_set_speed(speed);
				left_motor_set_speed(speed);
				set_front_led(OFF);

			//s'il doit être arrêté --> on remet les conditions avec left par sécurité même si déjà contrôlées dans la fsm
				}else if(stop && left){
					right_motor_set_speed(0);
					left_motor_set_speed(0);
					set_front_led(ON);

				//s'il a vu du bleu
				}else if(stop && !left){
					turn(PI/2);
					chThdSleepMilliseconds(500);
					done_l = false;
					set_front_led(OFF);
				}

			//100Hz
			chThdSleepUntilWindowed(time, time + MS2ST(10));
		}
    }
}

void turn(float alpha){
	int32_t nstep = 0;
	volatile float speed = 0;
	volatile int32_t right_pos=0, left_pos=0;//Finalement prendre les 2 pour plus de précisions

	nstep = (int32_t)((alpha*WHEEL_OFFSET*NSTEP_ONE_TURN)/WHEEL_PERIMETER);

/*Option inspirée du TP2*/
	left_motor_set_pos(0);
	right_motor_set_pos(0);

	while((abs(right_pos) < abs(nstep)) && (abs(left_pos) < abs(nstep))){

		right_pos = right_motor_get_pos();
		left_pos = left_motor_get_pos();

		speed = pi_rotator(right_pos, nstep);

		if(alpha > 0){//gauche
			right_motor_set_speed(speed);
			left_motor_set_speed(-speed);
		}else{//droite
			right_motor_set_speed(-speed);
			left_motor_set_speed(speed);
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
