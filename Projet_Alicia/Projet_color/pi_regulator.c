/**
 * Fichier: pi_regulator.c
 * auteurs: TP4, modifié par Alicia Mauroux
 *
 * @brief
 * Fichier qui gère la locomotion du robot e-puck2 à l'aide de régulateurs PI.
 */

#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>

#include "main.h"
#include "pi_regulator.h"

//pour vérifier les autorisations
#include "process_image.h"
#include "proximity_detection.h"

//capteurs de proximité
#include "sensors/VL53L0X/VL53L0X.h"
#include <sensors/proximity.h>


/*Define pour les demi-tours*/
#define NSTEP_ONE_TURN      1000								// number of step for 1 turn of the motor
#define WHEEL_PERIMETER    	13								// [cm] --> ≈ 4*pi --> 4cm est la mesure du diamètre des roues
#define WHEEL_OFFSET			2.65								//[cm] --> de combien la roue est décalé par rapport au centre du robot

/*Initialisation des statics*/
static float sum_error = 0;//pour le pi
static float sum_error_rot = 0;//pour le pi de rotation
static float sum_error_al = 0;//pour le pi d'alignement
static float sum_error_diag = 0;//pour le pi qui évite les collisions en diagonale


/*::::::::::PI REGULATOR:::::::::::::*/
//@brief
//une simple implémentation PI
/*Donne la vitesse pour aller en ligne droite*/
int16_t pi_regulator(uint16_t distance){

	float error = 0;
	float speed = 0;

	error = distance - GOAL_DISTANCE;

	//désactive le régulateur PI si l'erreur est trop faible
	//ça évite de toujours bouger étant donné qu'on ne peut pas être parfaitement précis
	if(error < ERROR_THRESHOLD){//pas de valeur absolue car on n'a pas besoin de le faire reculer pour être parfaitement précis
			return NO_SPEED;
	}

	sum_error += error;
	sum_error = 0.5*sum_error;//on prend un moyenne du passé

	//On pose un maximum et un minimum à la somme pour éviter une ascension incontrolée
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

	if(speed>MAX_SPEED){speed = MAX_SPEED;}
	if(speed<(-MAX_SPEED)){speed = -MAX_SPEED;}

	return (int16_t)speed;

}

/*::::::::::PI ALIGNEMENT:::::::::::::*/
//@brief
//implémentation d'un régulateur PI pour l'alignement
//donne une vitesse de rotation
int16_t pi_alignment(int right, int left){

	float error = 0;
	float speed = 0;

	error = right - left;

	//désactive le régulateur PI si l'erreur est trop faible
	//ça évite de toujours bouger étant donné qu'onne peut pas être parfaitement précis
	if(abs(error) < ERROR_THRE_AL){
			return NO_SPEED;
	}

	sum_error_al += error;
	sum_error_al = 0.5*sum_error_al;

	//On pose un maximum et un minimum à la somme pour éviter une ascension incontrolée
	if(sum_error_al > MAX_SUM_ERROR){
		sum_error_al = MAX_SUM_ERROR;
	}else if(sum_error_al < -MAX_SUM_ERROR){
		sum_error_al = -MAX_SUM_ERROR;
	}

	speed = KP_AL * error + KI_AL * sum_error_al;

	//on ne veut pas une vitesse trop rapide car on souhaite
	//un mouvement qui reste fluide
	if(speed > MID_SPEED){speed = MID_SPEED;}
	if(speed < (-MID_SPEED)){speed = -MID_SPEED;}

	return (int16_t)speed;
}

/*::::::::::PI DIAGONALES:::::::::::::*/
//@brief
//implémentation d'un régulateur PI pour éviter les obstacles
//apparaissant dans les capteurs diagonaux IR2 et IR7
//donne une vitesse de rotation
int16_t pi_diagonal(int position){

	float error = 0;
	float speed = 0;

	error = position - GOAL_DIAG;

	//désactive le régulateur PI si l'erreur est trop faible
	//ça évite de toujours bouger étant donné qu'onne peut pas être parfaitement précis
	if(error < ERROR_THRE_DIAG){
			return NO_SPEED;
	}

	sum_error_diag += error;
	sum_error_diag = 0.5*sum_error;

	//On pose un maximum et un minimum à la somme pour éviter une ascension incontrolée
	if(sum_error_diag > MAX_SUM_ERROR){
		sum_error_diag = MAX_SUM_ERROR;
	}else if(sum_error_diag < -MAX_SUM_ERROR){
		sum_error_diag = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error_diag;

	//on ne veut pas une vitesse trop rapide car on souhaite
	//un mouvement qui reste fluide
	if(speed>MID_SPEED){speed = MID_SPEED;}
	if(speed<(-MID_SPEED)){speed = -MID_SPEED;}

	return (int16_t)speed;
}

/*:::::::::::ROTATION::::::::::::::::::::::::*/
//@brief
//fonction qui détermine la vitesse de rotation lors des 1/4 et 1/2 tours
int16_t pi_rotator(uint16_t position, int32_t nstep){

	float error = 0;
	float speed = 0;

	error = nstep - position;

	//désactive le régulateur PI si l'erreur est trop faible
	//ça évite de toujours bouger étant donné qu'onne peut pas être parfaitement précis
	if(abs(error) < ERROR_THRE_ROT){
		//faible vitesse pour finir correctement le tour
		if(error>0){
			return MID_SPEED;
		}else{
			return -MID_SPEED;
		}
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

	if(speed>MAX_SPEED){speed = MAX_SPEED;}
	if(speed<(-MAX_SPEED)){speed = -MAX_SPEED;}

	return (int16_t)speed;
}

/*:::::::::::::TOURNER::::::::::::::*/
void turn(float alpha){
	int32_t nstep = 0;
	volatile float speed = 0;
	volatile int32_t right_pos=0, left_pos=0;

	nstep = (int32_t)((alpha*WHEEL_OFFSET*NSTEP_ONE_TURN)/WHEEL_PERIMETER);

/*Option inspirée du TP2*/
	left_motor_set_pos(0);
	right_motor_set_pos(0);

	while((abs(right_pos) < abs(nstep)) && (abs(left_pos) < abs(nstep))){

		right_pos = right_motor_get_pos();
		left_pos = left_motor_get_pos();

		speed = pi_rotator(right_pos, nstep);

			right_motor_set_speed(speed);
			left_motor_set_speed(-speed);
	}
}
//====================================
/*:::::::::THREAD:::::::::::*/

static THD_WORKING_AREA(waPiRegulator,512);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    //cor_speed : vitesse à corriger pour s'aligner (dans la même idée qu'une Tesla sur la route)
    //diaf_speed: vitesse pour éviter d'entre en collision si le robot n'est pas perpendiculaire au mur
    int16_t speed = 0, cor_speed = 0, diag_speed = 0;
    uint16_t measure = 0;
    uint8_t stop = 0, left= 0, win = 0, game_over = 0;
    volatile uint8_t avoid=0;

    while(1){
		avoid = get_avoid();
		left=get_left(); //si !left --> alors il tourne à gauche donc ne doit pas avancer ou être arrêté
		//si il est en manoeuvre d'évitement, pas besoin d'entrer dans ce thread
		game_over = get_game_over();
		win = get_win();

		//Vérifier qu'on passe bien dans la boucle si on a fini le jeu pour que les moteurs s'arrêtent bien
		if(win || game_over){
			avoid = false;
			left = true;
			stop = true;
		}

    		if(!avoid && left){
				stop = get_stop_fsm();
				time = chVTGetSystemTime();//pour le sleep Window d'en dessous

				if(!stop){
				//mesure de la distance à un obstacle pour réguler la vitesse grâce au PI
				measure	= VL53L0X_get_dist_mm();

				//Vitesse "de route"
				if(measure < MAX_DISTANCE){
					speed = pi_regulator(measure);
				}else{
					speed = MAX_SPEED;
				}

				//Modifications éventuelles de la vitesse "de route"
				//dans le cas où le robot détecte un mur des 2 côtés --> il s'aligne
				if((get_prox(IR3)>IR_TRES_SIDE) && (get_prox(IR6)>IR_TRES_SIDE)){

					cor_speed = pi_alignment(get_prox(IR3), get_prox(IR6));
				}else{
					cor_speed = NO_SPEED;

					//dans le cas où le robot arrive de diagonale contre un objet (1 seul mur)
					if((get_prox(IR2)>DIAG_DETECT) || (get_prox(IR7)>DIAG_DETECT)){
						if(get_prox(IR2) > get_prox(IR7)){ //celui de droite plus proche d'un mur que celui de gauche
							diag_speed = pi_diagonal(get_prox(IR2));
						}else{
							diag_speed = -pi_diagonal(get_prox(IR7));
						}
					}else{
							diag_speed = NO_SPEED;
					}
				}

				//applique la vitesse calculée aux moteurs
				right_motor_set_speed(speed + ROT_COEF*cor_speed + diag_speed);
				left_motor_set_speed(speed - ROT_COEF*cor_speed - diag_speed);

				}else{
					right_motor_set_speed(NO_SPEED);
					left_motor_set_speed(NO_SPEED);
				}

			//100Hz
			chThdSleepUntilWindowed(time, time + MS2ST(10));
		}
    }
}

//@brief
//démarre le thread
void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}

