#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <noise_detection.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>
#include <leds.h>

#include <pi_regulator.h>
#include "process_image.h"



//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];

static uint8_t freq_found = false;
static int8_t led_on = false;
static int16_t successive_freq_counter = 0;//pourquoi un int? NICO


#define MIN_VALUE_THRESHOLD	10000
#define MAX_MIC_INPUT_COUNTER 10


#define L_FREQ 90
#define M_FREQ 110

/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/
void sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;
	//uint8_t right =0, left=0;

	//search for the highest peak
	for(uint16_t i = L_FREQ ; i <= M_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	if(max_norm_index >= L_FREQ && max_norm_index <= M_FREQ){
		freq_found = true;
	}
	else
	{
		freq_found = false;
	}

}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
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
		if(successive_freq_counter > MAX_MIC_INPUT_COUNTER) //environs 1/2 secondes
		{
			if(led_on)
			{
				set_led(LED3, OFF);
				led_on = false;
			}
			else if(!led_on)
			{
				set_led(LED3, ON);
				led_on = true;
			}
			successive_freq_counter=0;
		}
		else if(freq_found)
		{
			successive_freq_counter++;
			freq_found = false;
		}
		else
		{
			successive_freq_counter = 0;
		}
	}
}

//void wait_send_to_computer(void){
//	chBSemWait(&sendToComputer_sem);
//}

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

uint8_t get_run(void){
	return freq_found;
}
