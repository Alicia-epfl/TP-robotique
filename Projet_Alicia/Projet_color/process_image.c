#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <leds.h>

#include <process_image.h>

//MEMO
//			r4 r3 r2 r1  r0 g5 g4 g3         g2 g1 g0 b4  b3 b2 b1 b0
		//			image_r[i/2] = ((uint8_t)img_buff_ptr[i]&0xF8)>>3;//rouge
		//			image_g[i/2] = ((uint8_t)img_buff_ptr[i]&0x07)<<3 | ((uint8_t)img_buff_ptr[i+1]&0xE0)>>5 ;//vert
		//			image_b[i/2] = ((uint8_t)img_buff_ptr[i+1]&0x1F);//blue

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */


/*
 * Alternative 2 pour trouver les couleurs :'(
 * FONCTION POUR TROUVER LES FLANC MONTANT ET DESCENDANT DES INTENSITé DE COULEUR
 */

uint16_t extract_color(uint8_t *buffer){

		uint16_t i = 0, begin = 0, end = 0; //width = 0;
		uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
		uint32_t mean = 0;
//		static uint32_t  mean_filtered = 0;

//		static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

		//performs an average
		for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
			mean += buffer[i];
		}
		mean /= IMAGE_BUFFER_SIZE;
//		mean_filtered = 0.5*mean+0.5*mean_filtered;


		do{
			wrong_line = 0;
			//search for a begin
			while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
			{
				//the slope must at least be WIDTH_SLOPE wide and is compared
			    //to the mean of the image
			    if(buffer[i] > (mean+WALL) && buffer[i+WIDTH_SLOPE] < (mean+WALL))
			    {
			        begin = i;
			        stop = 1;
			    }
			    i++;
			}
			//if a begin was found, search for an end
			if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
			{
			    stop = 0;

			    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
			    {
			        if(buffer[i] > (mean+WALL) && buffer[i-WIDTH_SLOPE] < (mean+WALL))
			        {
			            end = i;
			            stop = 1;
			        }
			        i++;
			    }
			    //if an end was not found
			    if (i > IMAGE_BUFFER_SIZE || !end)
			    {
			        line_not_found = 1;
			    }
			}
			else//if no begin was found
			{
			    line_not_found = 1;
			}

			//if a line too small has been detected, continues the search
			if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
				i = end;
				begin = 0;
				end = 0;
				stop = 0;
				wrong_line = 1;
			}
		}while(wrong_line);

		if(line_not_found){
			begin = 0;
			end = 0;
//			width = last_width;
			return 0;
		}else{
//			last_width = width = (end - begin);
//			line_position = (begin + end)/2; //gives the line position.
			return 1;
		}

}


static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
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

static THD_WORKING_AREA(waProcessImage, 1024); //Je suis montée de 1024 à 2048 car il n'y avait pas assez de place pour les 3 couleurs
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;//BOMBE
//	uint8_t image[IMAGE_BUFFER_SIZE] = {0};//BOMBE
//	uint8_t image_r[IMAGE_BUFFER_SIZE] = {0}, image_g[IMAGE_BUFFER_SIZE] = {0}, image_b[IMAGE_BUFFER_SIZE] = {0}; //OPTION 2
//	uint8_t red =0, green=0, blue=0; //OPTION 2
	uint16_t mean_red = 0, mean_blue=0, mean_green=0;
	uint8_t red = 0, green=0, blue=0;
	uint16_t red_image = 0, green_image=0, blue_image=0;

	/*Méthode "RGB"*/
    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		//insertion de l'image capturée dans le tableau "image"
		for(uint16_t i = 0 ; i < (2*IMAGE_BUFFER_SIZE) ; i+=4){
			//séparation des couleurs
			red_image = ((uint16_t)img_buff_ptr[i]&0xF8)>>3;
			green_image = (((uint16_t)img_buff_ptr[i]&0x07)<<3) | (((uint16_t)img_buff_ptr[i+1]&0xE0)>>5);
			blue_image = ((uint16_t)img_buff_ptr[i+1]&0x1F);

			//moyennes
			mean_red += red_image;
			mean_green += green_image;
			mean_blue += blue_image;
		}
		mean_red /= (IMAGE_BUFFER_SIZE/2);
		mean_green /= (IMAGE_BUFFER_SIZE/2);
		mean_blue /= (IMAGE_BUFFER_SIZE/2);

		chprintf((BaseSequentialStream *)&SDU1, "R=%3d, G=%3d, B=%3d\r\n\n", mean_red, mean_green, mean_blue);

		//		if(mean_red > RED_VALUE){red = 1;}
		//		if(mean_green > GREEN_VALUE){green = 1;}
		//		if(mean_blue > BLUE_VALUE){blue = 1;}

		if((mean_red > 12) && (mean_green <10) && (mean_blue < 13)){red = 1;}
		if((mean_green > 12)&&(mean_red < 10)&&(mean_blue < 14)){green = 1;}
		if((mean_blue > 13)&&(mean_red < 5)&&(mean_green < 30)){blue = 1;}

		if(red && !green && !blue){// && !green && !blue
				set_led(LED1, ON);
		}else{
				set_led(LED1, OFF);
								}
		if(green && !red && !blue){// && !green && !blue
				set_led(LED3, ON);
		}else{
				set_led(LED3, OFF);
		}
		if(blue && !green && !red){// && !green && !blue
				set_led(LED5, ON);
		}else{
				set_led(LED5, OFF);
		}
    }
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
