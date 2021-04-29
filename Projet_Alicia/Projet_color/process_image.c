#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <leds.h>

#include <process_image.h>



static float distance_cm = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
//=========================================
//===== FONCTION POUR DETERMINER LA COULEUR
//=========================================
//Idée de comparer chaque bit à bit pour déterminer la composante dominante

/*
 * Alternative 2 pour trouver les couleurs
 * FONCTION QUI PREND DIRECTEMENT LES BON BUFFERS
 */
void find_color(uint8_t *buffer){
		uint16_t mean_red = 0, mean_blue=0, mean_green=0;
		uint8_t red = 0, green=0, blue=0;
		uint16_t red_image = 0, green_image=0, blue_image=0;

//		static uint32_t  mean_filter = 0;

		//			image_r[i/2] = ((uint8_t)img_buff_ptr[i]&0xF8)>>3;//rouge				POUR EXTRACT >>3
		//			image_g[i/2] = ((uint8_t)img_buff_ptr[i]&0x07)<<3 | ((uint8_t)img_buff_ptr[i+1]&0xE0)>>5 ;//vert				POUR EXTRACT
		//			image_b[i/2] = ((uint8_t)img_buff_ptr[i+1]&0x1F);//blue

			//performs an average
		for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i+=2){

			red_image = ((uint8_t)buffer[i]&0xF8)>>3;
			green_image = ((uint8_t)buffer[i]&0x07)<<3;
			blue_image = ((uint8_t)buffer[i+1]&0x1F);

//			chprintf((BaseSequentialStream *)&SDU1, "green=%d \n", green_image);
//			chprintf((BaseSequentialStream *)&SDU1, "R=%3d, G=%3d, B=%3d\r\n\n", red_image, green_image, blue_image);

			mean_red += red_image;
			mean_green += green_image;
			mean_blue += blue_image;
//				chprintf((BaseSequentialStream *)&SDU1, "valeur=%d \n", buffer[i]);
		}
		mean_red /= (IMAGE_BUFFER_SIZE/2);
		mean_green /= (IMAGE_BUFFER_SIZE/2);
		mean_blue /= (IMAGE_BUFFER_SIZE/2);
//		mean_filter = 0.5*mean+0.5*mean_filter;

		chprintf((BaseSequentialStream *)&SDU1, "R=%3d, G=%3d, B=%3d\r\n\n", mean_red, mean_green, mean_blue);
//		chprintf((BaseSequentialStream *)&SDU1, "mean=%d \n", mean);

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
//	red = (int)buffer[i]&0xF8;
//	green = (int)(buffer[i]&0x07)<<5 | (buffer[i+1]&0xE0)>>3;
//	blue = (int)(buffer[i+1]&0x1F)<<3;
/*
 * Alternative 3 pour trouver les couleurs :'(
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


uint16_t extract_line_width(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a :::BEGIN:::
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
		//if a begin was found, search for an :::END:::
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
		width = last_width;
		return 0;
	}else{
		last_width = width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.
		return 1;
	}

	//sets a maximum width or returns the measured width
//	if((PXTOCM/width) > MAX_DISTANCE){
//		return PXTOCM/MAX_DISTANCE;
//	}else{
//		return width;
//	}
}
//FIN DU CHECK

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

static THD_WORKING_AREA(waProcessImage, 4096); //Je suis montée de 1024 à 2048 car il n'y avait pas assez de place pour les 3 couleurs
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	uint16_t lineWidth = 0, red=0, green = 0, blue=0;

	bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		// pour rouge on part de i = 0 et on met l'hexadécmal de 0XF8
		//pour vert on a 2x la ligne image[i/2] et on change l'hexa
		//pour bleu on part de  et on met l'hexadécimal 0X1F
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){

//			r4 r3 r2 r1  r0 g5 g4 g3         g2 g1 g0 b4  b3 b2 b1 b0

//			image_r[i/2] = ((uint8_t)img_buff_ptr[i]&0xF8)>>3;//rouge				POUR EXTRACT >>3
//			image_g[i/2] = ((uint8_t)img_buff_ptr[i]&0x07)<<3 | ((uint8_t)img_buff_ptr[i+1]&0xE0)>>5 ;//vert				POUR EXTRACT
//			image_b[i/2] = ((uint8_t)img_buff_ptr[i+1]&0x1F);//blue				POUR EXTRACT

			image[i/2] = (uint8_t)img_buff_ptr[i];//rouge	et vert
			image[i/2+1] = (uint8_t)img_buff_ptr[i+1] ;//vert	et bleu
		}
		send_to_computer = false;

		//search for a line in the image and gets its width in pixels
		find_color(image);


		//converts the width into a distance between the robot and the camera
		if(lineWidth){
			distance_cm = PXTOCM/lineWidth;//CHECK
		}

		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;
    }
}

float get_distance_cm(void){//CHECKER
	return distance_cm;
}

uint16_t get_line_position(void){//CHECKER
	return line_position;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
