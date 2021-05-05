#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include "main.h"
#include <camera/po8030.h>
#include <leds.h>
#include <motors.h>

#include <process_image.h>
#include <audio/play_melody.h>

//MEMO
//			r4 r3 r2 r1  r0 g5 g4 g3         g2 g1 g0 b4  b3 b2 b1 b0
		//			image_r[i/2] = ((uint8_t)img_buff_ptr[i]&0xF8)>>3;//rouge
		//			image_g[i/2] = ((uint8_t)img_buff_ptr[i]&0x07)<<3 | ((uint8_t)img_buff_ptr[i+1]&0xE0)>>5 ;//vert
		//			image_b[i/2] = ((uint8_t)img_buff_ptr[i+1]&0x1F);//blue

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);
static uint8_t  right = true;
static uint8_t left=true;
static uint8_t red_rgb=false;
static uint8_t green_rgb=false;
static uint8_t blue_rgb=false;

//===================
//=====THREADS======

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
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);

    }
}

/*Thread TOURNER A DROITE*/
static THD_WORKING_AREA(waRight, 128);
static THD_FUNCTION(Right, arg){
	while(1){
		if (!right){
			right_motor_set_speed(-600);
			left_motor_set_speed(600);



			chThdSleepMilliseconds(550);
			//on remet à 0 avant de laisser la décision au thread du son pour plus de précisions
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			right = true;

		}
		chThdSleepMilliseconds(500);
	}
}
/*Thread TOURNER A GAUCHE*/
static THD_WORKING_AREA(waLeft, 128);
static THD_FUNCTION(Left, arg){
	while(1){
		if (!left){
			right_motor_set_speed(600);
			left_motor_set_speed(-600);



			chThdSleepMilliseconds(550);
			//on remet à 0 avant de laisser la décision au thread du son pour plus de précisions
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			left = true;

		}
		chThdSleepMilliseconds(500);
	}
}
static THD_WORKING_AREA(waProcessImage, 8192); //Je suis montée de 1024 à 4096 pour voir si ça réglait le problème
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint16_t mean_red = 0, mean_blue=0, mean_green=0;// mean_red_old=0;
	uint16_t mean_red_filtered = 0, mean_blue_filtered=0, mean_green_filtered=0;
	uint8_t red = 0, green=0, blue=0;
	uint8_t red_image = 0, green_image=0, blue_image=0;




	/*Méthode "RGB"*/
    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

/*re-initialiser les valeurs --> j'ai voulu les remettre à 0 ici mais après mon chprintf dans la boucle imprime que des 0?
*donc je les ai intialisées juste avant la boucle pour contrer ce problème*/
//		mean_red_old = mean_red; //tentative de plus tard faire un filtre passe-bas mais étape en attente pour le moment


		mean_red = 0;
		mean_blue=0;
		mean_green=0;
		red =0, green =0;

//		uint32_t mean_red = 0, mean_blue=0, mean_green=0;

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

		chprintf((BaseSequentialStream *)&SDU1, "R=%3d, G=%3d, B=%3d\r\n\n", mean_red, mean_green, mean_blue);//valeurs RGB moyennes
//		chprintf((BaseSequentialStream *)&SDU1, "R=%3d, R_old=%3d\r\n", mean_red, mean_red_old);

		/*Filtre passe-bas*/
		mean_red_filtered = 0.5*mean_red+0.5*mean_red_filtered;
		mean_green_filtered = 0.5*mean_green+0.5*mean_green_filtered;
		mean_blue_filtered = 0.5*mean_blue+0.5*mean_blue_filtered;

		if(mean_red_filtered > RED_VALUE){red = true;}
		if(mean_green_filtered > GREEN_VALUE){green = true;}
		if(mean_blue_filtered > BLUE_VALUE){blue = true;}

		if((mean_red_filtered > 1.5*mean_blue_filtered) && (mean_red_filtered > 1.5*mean_green_filtered)){// RED
						set_led(LED1, ON);
						red++;

				}else{
						set_led(LED1, OFF);
						red=0;
				/*GREEN*/					}
				if((mean_green_filtered > mean_blue_filtered) && (mean_green_filtered > 1.5*mean_red_filtered)){// GREEN --> SUCESS
						set_led(LED3, ON);
						playMelody(WE_ARE_THE_CHAMPIONS, ML_SIMPLE_PLAY, NULL);
				}else{
						set_led(LED3, OFF);
				}
				/*BLUE*/
				if((mean_blue_filtered > 1.5*mean_red_filtered) && (mean_blue_filtered > mean_green_filtered)){//--> pas le green car il est très élevé pour le bleu
//						set_led(LED5, ON);
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
						right = false;

						chThdSleepMilliseconds(500);
				}else{
						set_led(LED5, OFF);
				}
				/*YELLOW*/
				if((mean_red_filtered > 1.5*mean_blue_filtered) && (mean_green_filtered > mean_blue_filtered)){//YELLOW
					set_led(LED5, ON);
					/*Pour afficher ce qu'il voit comme couleur*/
					red_rgb=mean_red_filtered;
					green_rgb=mean_green_filtered;
					blue_rgb = 0.5*mean_blue_filtered;

					/*Lancer la rotation*/
					left = false;

					chThdSleepMilliseconds(500);
				}else{
					set_led(LED5, OFF);
				}
    }
}


void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
	//lance le thread de tourner --> efficacité?
	chThdCreateStatic(waRight, sizeof(waRight),NORMALPRIO+1, Right, NULL);
	chThdCreateStatic(waLeft, sizeof(waLeft),NORMALPRIO+1, Left, NULL);
}

//Si right = true --> normal
//Si right = false --> tourne à droite, tout le monde s'arrête
uint8_t get_right(void){

	return right;
}
uint8_t get_left(void){

	return left;
}
uint8_t get_red(void){

	return red_rgb;
}
uint8_t get_green(void){

	return green_rgb;
}
uint8_t get_blue(void){

	return blue_rgb;
}
