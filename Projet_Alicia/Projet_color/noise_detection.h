/*
 * noise_detection.h
 *
 *  Created on: Apr 27, 2021
 *      Author: Nico
 */
#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

typedef enum {
	//2 fois FFT_SIZE car ces tableaux contiennent des nombres complexes (réels + imaginaires)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Tableaux contenant la magnitude calculée des nombres complexes
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;


void processAudioData(int16_t *data, uint16_t num_samples);

/*
*	mettre le thread invoqué en veille jusqu'à ce qu'il puisse traiter les données audio
*/
void wait_send_to_computer(void);

/*
*	Retourne le pointeur à BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

/*Fonction pour return si le robot doit avancer ou s'arrêter*/
uint8_t get_run(void);

#endif /* AUDIO_PROCESSING_H */

