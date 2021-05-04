#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>
#include <proximity_detection.h>
#include <sensors/proximity.h>
#include <msgbus/messagebus.h>
#include <leds.h>
#include <motors.h>
#include <stdbool.h>




static THD_WORKING_AREA(waControl, 2048);
static THD_FUNCTION(Control, arg)
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	while (!chThdShouldTerminateX())
	{

		if(!get_prox_front) // si pas d'obstacle, continuer tout droit
		{
			right_motor_set_speed(100);
			left_motor_set_speed(100);
		}

		if(get_prox_front && get_prox_left && !get_prox_right) // si obstacle + ouverture à droite, tourner à droite
		{
			right_motor_set_speed(-100);
			left_motor_set_speed(100);
		}

		if(get_prox_front && get_prox_right && !get_prox_left) // si obstacle + ouverture à gauche, tourner à gauche
		{
			set_led(LED7, ON);
			right_motor_set_speed(100);
			left_motor_set_speed(-100);
		}







		if(get_prox_front && get_prox_right && get_prox_left && get_prox_back) // si encerclé, tout allumer
		{
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			set_led(LED1, ON);
			set_led(LED3, ON);
			set_led(LED5, ON);
			set_led(LED7, ON);
			set_front_led(ON);
		}








	}
}






//systime_t time;
//
//    int16_t speed = 0;
//    int16_t speed_correction = 0;
//
//    while(1){
//        time = chVTGetSystemTime();
//
//        //computes the speed to give to the motors
//        //distance_cm is modified by the image processing thread
//        speed = pi_regulator(get_distance_cm(), GOAL_DISTANCE);
//        //computes a correction factor to let the robot rotate to be in front of the line
//        speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));
//
//        //if the line is nearly in front of the camera, don't rotate
//        if(abs(speed_correction) < ROTATION_THRESHOLD){
//        	speed_correction = 0;
//        }
//
//        //applies the speed from the PI regulator and the correction for the rotation
//		right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
//		left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
//
//        //100Hz
//        chThdSleepUntilWindowed(time, time + MS2ST(10));
//    }
