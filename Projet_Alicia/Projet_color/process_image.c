#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include "main.h"
#include <camera/po8030.h>
#include <leds.h>
#include <motors.h>
#include "pi_regulator.h"

#include <process_image.h>
#include <audio/play_melody.h>
#include "sensors/VL53L0X/VL53L0X.h"

//MEMO
//			r4 r3 r2 r1  r0 g5 g4 g3         g2 g1 g0 b4  b3 b2 b1 b0
		//			image_r[i/2] = ((uint8_t)img_buff_ptr[i]&0xF8)>>3;//rouge
		//			image_g[i/2] = ((uint8_t)img_buff_ptr[i]&0x07)<<3 | ((uint8_t)img_buff_ptr[i+1]&0xE0)>>5 ;//vert
		//			image_b[i/2] = ((uint8_t)img_buff_ptr[i+1]&0x1F);//blue

/*NE PAS OUBLIER:
 * De prendre les valeurs left et right pour indiquer au robot qui rencontre un obstacle qu'il va devoir donner
 * la priotité aux couleurs et donc skip la détection d'obstacle*/

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

//commande de la caméra et du quart de tour pour le bleu
static uint8_t record = true;
static uint8_t left=true;

//variables pour afficher sur les leds la couleur que le robot a détecté sur la caméra
static uint8_t red_rgb=false;
static uint8_t green_rgb=false;
static uint8_t blue_rgb=false;

//variables de game_over et win
static uint8_t game_over = false;
static uint8_t win = false;

//===================
//=====THREADS======

/*Thread CHECKER LA CAMERA*/
static THD_WORKING_AREA(waRecord, 2046);//tentative de 256 à 1024
static THD_FUNCTION(Record, arg){
	uint16_t measure=0, record_allowed=0;



	while(1){
/*L'idée ensuite c'est de faire une fonction qui récuoère les mesures dans pi et qui return 1 ou 0 directement pour ici!*/
		measure = VL53L0X_get_dist_mm();
//		chprintf((BaseSequentialStream *)&SDU1, "R=%3d\r", measure);
		record_allowed = get_record_allowed_fsm();

		if (measure<130){
			record = true;
		}else{
			record = false;
		}
		//vérifier que le record est autorisé par la fsm
		if(!record_allowed){
			record = false;
		}
		chThdSleepMilliseconds(500);
	}
}

/*Thread CAPTURE*/
static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 100, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();


    while(1){
//    		if(record){
			//starts a capture
			dcmi_capture_start();
			//waits for the capture to be done
			wait_image_ready();
			//signals an image has been captured
			chBSemSignal(&image_ready_sem);
//		}
    }
}



static THD_WORKING_AREA(waProcessImage, 1024); //Je suis montée de 1024 à 4096 pour voir si ça réglait le problème
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint16_t mean_red = 0, mean_blue=0, mean_green=0;// mean_red_old=0;
	uint16_t mean_red_filtered = 0, mean_blue_filtered=0, mean_green_filtered=0;
	uint8_t red_image = 0, green_image=0, blue_image=0;

	/*Méthode "RGB"*/
    while(1){

			//waits until an image has been captured
			chBSemWait(&image_ready_sem);
			//gets the pointer to the array filled with the last image in RGB565
			img_buff_ptr = dcmi_get_last_image_ptr();
			mean_red = 0;
			mean_blue=0;
			mean_green=0;

	//		uint32_t mean_red = 0, mean_blue=0, mean_green=0;
		if(record){
			//insertion de l'image capturée dans le tableau "image"
			for(uint16_t i = 0 ; i < (2*IMAGE_BUFFER_SIZE) ; i+=4){// +4 pour essayer de prendre moins de pixels pour éviter un dépassement mais aucun effet
				//séparation des couleurs
				red_image = ((uint8_t)img_buff_ptr[i]&0xF8)>>3;
				green_image = (((uint8_t)img_buff_ptr[i]&0x07)<<3) | (((uint8_t)img_buff_ptr[i+1]&0xE0)>>5);
				blue_image = ((uint8_t)img_buff_ptr[i+1]&0x1F);

	//			chprintf((BaseSequentialStream *)&SDU1, "R=%3d, G=%3d, B=%3d\r\n\n", red_image, green_image, blue_image);

				//moyennes
				mean_red += red_image;
				mean_green += green_image;
				mean_blue += blue_image;

	//			set_led(LED1, ON);//vérifier qu'on passe bien dans cette boucle
			}
			mean_red /= (IMAGE_BUFFER_SIZE/2);
			mean_green /= (IMAGE_BUFFER_SIZE);
			mean_blue /= (IMAGE_BUFFER_SIZE/2);

	//		chprintf((BaseSequentialStream *)&SDU1, "R=%3d, G=%3d, B=%3d\r\n\n", mean_red, mean_green, mean_blue);//valeurs RGB moyennes
	//		chprintf((BaseSequentialStream *)&SDU1, "R=%3d, R_old=%3d\r\n", mean_red, mean_red_old);

			/*Filtre passe-bas*/
			mean_red_filtered = 0.5*mean_red+0.5*mean_red_filtered;
			mean_green_filtered = 0.5*mean_green+0.5*mean_green_filtered;
			mean_blue_filtered = 0.5*mean_blue+0.5*mean_blue_filtered;


			/*RED*/
			if((mean_red_filtered > 1.5*mean_blue_filtered) && (mean_red_filtered > 1.5*mean_green_filtered)){// RED --> GAME OVER
				playMelody(MARIO_DEATH, ML_SIMPLE_PLAY, NULL);
				game_over = true;
			}

			/*GREEN*/
			if((mean_green_filtered > mean_blue_filtered) && (mean_green_filtered > 1.5*mean_red_filtered)){// GREEN --> SUCESS
				playMelody(SEVEN_NATION_ARMY, ML_SIMPLE_PLAY, NULL);
				win = true;
			}

			/*BLUE*/
			if((mean_blue_filtered > 1.5*mean_red_filtered) && (mean_blue_filtered > mean_green_filtered)){//--> pas le green car il est très élevé pour le bleu

				/*Pour afficher ce qu'il voit comme couleur*/
				red_rgb=0.8*mean_red_filtered;

				/*Pour éclaircir un peu le bleu des LEDs*/
				if(1.2*mean_green_filtered<63){
					green_rgb=1.2*mean_green_filtered;
				}else{
					green_rgb=mean_green_filtered;
				}
				blue_rgb = mean_blue_filtered;

				/*Lancer la rotation*/
				left = false;
				turn(PI/2);
				left = true;
			}//color
    		}//record
		chThdSleepMilliseconds(250);
    }//while
}//thread


void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
	//thread contrôlant la distance grace à VL
	chThdCreateStatic(waRecord, sizeof(waRecord),NORMALPRIO+1, Record, NULL);
}

//Si left = false --> tourne à droite, tout le monde s'arrête
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
//fonction qui indique si on utilise la caméra
uint8_t get_record(void){
	return record;
}

//fonctions qui indiquent la fin de la partie
uint8_t get_game_over(void){
	return game_over;
}

uint8_t get_win(void){
	return win;
}
