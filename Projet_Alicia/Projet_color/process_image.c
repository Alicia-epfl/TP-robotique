/**
 * Fichier: process_image.c
 * auteurs: TP4, modifié par Alicia Mauroux
 *
 * @brief
 * Fichier qui gère la capture des images et leur traitement.
 * Il indique également les couleurs perçues
 */

#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include "main.h"
#include "process_image.h"

#include <camera/po8030.h>
#include <motors.h>


//pour activer/désactiver le traitement d'image
#include "sensors/VL53L0X/VL53L0X.h"
#include <sensors/proximity.h>
//pour tourner
#include "pi_regulator.h"
//pour jouer la mélodie
#include <audio/play_melody.h>

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

//commande de la caméra et du quart de tour pour le bleu
static uint8_t record = true;
static uint8_t left=true;//si left true --> ne tourne pas, si left false --> tourne

//variables pour afficher sur les leds la couleur que le robot a détecté sur la caméra
static uint8_t red_rgb=false;
static uint8_t green_rgb=false;
static uint8_t blue_rgb=false;

//variables de game_over et win
static uint8_t game_over = false;
static uint8_t win = false;

//===================
//=====THREADS======

//@brief
//Thread qui gère les conditions pour vérifier si on active/désactive
//le traitement d'image
static THD_WORKING_AREA(waRecord, 128);
static THD_FUNCTION(Record, arg){
	uint16_t measure=0, record_allowed=0;

	while(1){
		//mesure de la distance à une paroie via ToF
		measure = VL53L0X_get_dist_mm();//unité en [mm]

		//vérification de si on fait le traitement d'image ou non
		record_allowed = get_record_allowed_fsm();

		if (measure<TOF_RECORD){
			record = true;
		}else{
			record = false;
		}
		//vérifier que le record est autorisé par la fsm
		if(!record_allowed){
			record = false;
		}
		//protection pour éviter que le robot détecte du bleu en même temps qu'il veuille éviter un obstacle
		if((get_prox(IR1)>RECORD_THRES) || (get_prox(IR8)>RECORD_THRES)){
					record = false;
		}
		//fin de la vérification

		//sleep de 250 milisecondes
		chThdSleepMilliseconds(250);
	}
}

/*Thread CAPTURE*/
static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    //Prends les pixels de 0 à IMAGE_BUFFER_SIZE de la ligne ligne 150 + 151 (minimum 2 lignes)
	po8030_advanced_config(FORMAT_RGB565, 0, 150, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();


    while(1){
			//démarre une capture
			dcmi_capture_start();
			//attends que la capture soit faite
			wait_image_ready();
			//signale que l'image a été capturée
			chBSemSignal(&image_ready_sem);
    }
}

//@brief
//Thread de traitement d'image
static THD_WORKING_AREA(waProcessImage, 1024); //une taille de 512 nous semblait limite alors on a pris une marge supplémentaire
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t red_image = 0, green_image=0, blue_image=0;
	//moyenne des pixels de l'image
	uint16_t mean_red = 0, mean_blue=0, mean_green=0;
	//moyennes filtrèes par un filtre passe-bas
	uint16_t mean_red_filtered = 0, mean_blue_filtered=0, mean_green_filtered=0;

    while(1){

			//attend jusqu'à ce qu'un image soit capturée
			chBSemWait(&image_ready_sem);
			img_buff_ptr = dcmi_get_last_image_ptr();

			//reset les valeurs moyennes
			mean_red = 0;
			mean_blue=0;
			mean_green=0;

		if(record){
			//insertion de l'image capturée dans le tableau "image"
			// +4 pour essayer de prendre moins de pixels et ainsi éviter un dépassement
			for(uint16_t i = 0 ; i < (2*IMAGE_BUFFER_SIZE) ; i+=4){

				//séparation des couleurs
				red_image = ((uint8_t)img_buff_ptr[i]&0xF8)>>3;
				green_image = (((uint8_t)img_buff_ptr[i]&0x07)<<3) | (((uint8_t)img_buff_ptr[i+1]&0xE0)>>5);
				blue_image = ((uint8_t)img_buff_ptr[i+1]&0x1F);

				//moyennes
				mean_red += red_image;
				mean_green += green_image;
				mean_blue += blue_image;
			}
			mean_red /= (IMAGE_BUFFER_SIZE/2);
			mean_green /= (IMAGE_BUFFER_SIZE);
			mean_blue /= (IMAGE_BUFFER_SIZE/2);

			/*Filtre passe-bas*/
			mean_red_filtered = 0.5*mean_red+0.5*mean_red_filtered;
			mean_green_filtered = 0.5*mean_green+0.5*mean_green_filtered;
			mean_blue_filtered = 0.5*mean_blue+0.5*mean_blue_filtered;


			/*BLUE
			 * Tourner à gauche*/
 			if((mean_blue_filtered > FACT_B_R*mean_red_filtered) && (mean_blue_filtered > FACT_B_G*mean_green_filtered) && (mean_blue_filtered > TRES_BLUE)){

				/*Pour afficher ce qu'il voit comme couleur*/
				red_rgb=0.8*mean_red_filtered;

				/*Pour éclaircir un peu le bleu des LEDs*/
				if(LIGHT_BLUE_FACT*mean_green_filtered < GREEN_TRES){
					green_rgb=LIGHT_BLUE_FACT*mean_green_filtered;
				}else{
					green_rgb=mean_green_filtered;
				}
				blue_rgb = mean_blue_filtered;

				/*Lancer la rotation*/
				left = false;
				turn(PI/2);
				left = true;

			/*RED
			 * Game Over*/
			}else if((mean_red_filtered >FACT_R_B*mean_blue_filtered) && (mean_red_filtered > FACT_R_G*mean_green_filtered)){
				playMelody(MARIO_DEATH, ML_SIMPLE_PLAY, NULL);
				game_over = true;
				//attendre 20 secondes avant de relancer une partie
				chThdSleepMilliseconds(20000);
				game_over = false;

			/*GREEN
			 * Gagnée*/
			}else if((mean_green_filtered > FACT_G_B*mean_blue_filtered) && (mean_green_filtered > FACT_G_R*mean_red_filtered)){
				playMelody(SEVEN_NATION_ARMY, ML_SIMPLE_PLAY, NULL);
				win = true;
				//attendre 20 secondes avant de relancer une partie
				chThdSleepMilliseconds(20000);
				win = false;
			}//color
    		}//record
		//sleep pendant 250 millisecondes
		chThdSleepMilliseconds(250);
    }//while
}//thread


//@brief
//démarre les thread de process_image
void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
	//thread contrôlant la distance grace à VL
	chThdCreateStatic(waRecord, sizeof(waRecord),NORMALPRIO+1, Record, NULL);
}

//@brief
//fonctions qui retournent les valeurs définies pour passer les information en évitant
//des variables globales

//Si left = false --> tourne à gauche
//si left = true --> tout fonctionne normalement
uint8_t get_left(void){
	return left;
}

//fonctions pour transmettre les couleurs à afficher sur les leds RGB
uint8_t get_red(void){
	return red_rgb;
}
uint8_t get_green(void){
	return green_rgb;
}
uint8_t get_blue(void){
	return blue_rgb;
}

//fonctions qui indiquent la fin de la partie
uint8_t get_game_over(void){
	return game_over;
}

uint8_t get_win(void){
	return win;
}
