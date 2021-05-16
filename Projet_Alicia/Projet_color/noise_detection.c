/**
 * Fichier: noise_detection.c
 * auteurs: TP5, modifié par Nicolas Nouel
 *
 * @brief
 * Fichier qui gère les micro: indique si le son attendu a été détecté
 */

#include "ch.h"
#include "hal.h"
#include "main.h"
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <noise_detection.h>
#include <fft.h>
#include <arm_math.h>

#include <leds.h>

#include <pi_regulator.h>
#include "process_image.h"

//pour avoir accès à la statique "avoid" qui "contrôle" l'état de run
#include "proximity_detection.h"


/*======================== Repris du TP5, en enlevant les analyses sur les micros inutilisés =====================*/

//2 fois FFT_SIZE car ce tableau contient des nombres complexes (Réel + Imaginaire)
static float micLeft_cmplx_input[2 * FFT_SIZE];

//Tableau contenant le module des nombres complexes
static float micLeft_output[FFT_SIZE];

#define MIN_VALUE_THRESHOLD	10000
#define MAX_MIC_INPUT_COUNTER 10


#define L_FREQ 240 //1400Hz
#define M_FREQ 270 //1700Hz

/*======================== Fin des int/defines du TP5 ================================================*/

static uint8_t successive_freq_counter = 0; // compte le nombre de detections successive de la fréquence cible


/*
*	Fonction détectant la valeur maximale dans un buffer
*	et incrémentant un compteur en conséquence
*	Si le compteur dépasse une valeur seuil, on effectue une commande
*
*/
void sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;

	//Cherche le pic le plus haut
	for(uint16_t i = L_FREQ ; i <= M_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	if(successive_freq_counter > MAX_MIC_INPUT_COUNTER) //si la fréquence choisie est détectée à répétition, effectue une commande
	{
		toggle_run();//fonction qui change la variable run dans le main
		successive_freq_counter=0;
	}
	else if(max_norm_index >= L_FREQ && max_norm_index <= M_FREQ){
		successive_freq_counter ++;
	}
	else
	{
		successive_freq_counter = 0;
	}
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*
*	params :
*	int16_t *data			Buffer contenant 4 fois 160 échantillons. Les échantillons sont triés par micro
*							on a donc [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Donne le nombre d'échantillons total que nous avons (devrait toujours être à 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*	On obtiens 160 échantillons par micro toutes les 10ms
	*	Donc on remplis les buffers d'échantillons pour atteindre 1024 échantillons
	*	Puis on calcule les FFT
	*
	*/

	static uint16_t nb_samples = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part

		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];

		nb_samples++;

		micLeft_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function.
		*/

		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);


		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/

		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);

		nb_samples = 0;
//		chprintf((BaseSequentialStream *)&SDU1, "compteur=%3d\r",successive_freq_counter);
		sound_remote(micLeft_output);

	}
}


float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}

	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}

	else{
		return NULL;
	}
}

