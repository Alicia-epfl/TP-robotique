/**
 * Fichier: fft.c :  venant du TP5, quelques fonctions enlevées par Alicia Mauroux
 */

#include "ch.h"
#include "hal.h"
#include <main.h>
#include <math.h>
#include <fft.h>

#include <arm_math.h>
#include <arm_const_structs.h>

/* Définit les multiplication complexes et ses conjugués */
#define  rmul(x,y)      (x.real * y.real - x.imag * y.imag)
#define  imul(x,y)      (x.imag * y.real + x.real * y.imag)
#define rcmul(x,y)      (x.real * y.real + x.imag * y.imag)
#define icmul(x,y)      (x.imag * y.real - x.real * y.imag)


/*
*	Fonction fft très optimisée fournie par ARM
*/
void doFFT_optimized(uint16_t size, float* complex_buffer){
	if(size == 1024)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);

}


