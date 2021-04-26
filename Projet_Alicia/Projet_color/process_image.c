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

void find_color_2(uint8_t *buffer){
	uint16_t i = 0, b=0, g=0, r=0;
	uint32_t blue_m = 0, green_m = 0, red_m = 0;
	uint32_t blue = 0, green = 0, red = 0;
//	uint16_t size_c = IMAGE_BUFFER_SIZE/2; // pour définir combien il faut de pixel pour qu'on considère
											//qu'il y a bien un panneau de tel couleur devant
											//Pour le moment on a pris la moitié des pixels (à voir si on dit moins?)
	//static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	red_m=0;
	green_m = 0;
	blue_m=0;
	//déterminer quelle couleur est dominante
	while(i < (IMAGE_BUFFER_SIZE))
			{
					red = (int)buffer[i]&0xF8;
					green = (int)(buffer[i]&0x07)<<5 | (buffer[i+1]&0xE0)>>3;
					blue = (int)(buffer[i+1]&0x1F)<<3;

//					if((blue > red) && (blue > green)){blue_m++;}
//					if((green > red) && (green > blue)){green_m++;}
//					if((red > blue) && (red > green)){red_m++;}
//
//					if((blue > red) && (blue > green)){blue_m++;}
//					if((green > red) && (green > blue)){green_m++;}
//					if((red > blue) && (red > green)){red_m++;}

							red_m += red;
							green_m += green;
							blue_m += blue;

					i+=2;
				}


	if((red_m > blue_m) && (red_m > green_m)){r=1;}
	if((green_m > red_m) && (green_m > blue_m)){g=1;}
	if((blue_m > red_m) && (blue_m > green_m)){b=1;}



//	if(blue_m > size_c){
//		b=1;
//	}
//	if(green_m > size_c){
//			g=1;
//		}
//	if(red_m > size_c){
//			r=1;
//		}
	//Action qu'implique les couleurs

	if(r==1){
		palTogglePad(GPIOD, GPIOD_LED1);//RED --> LED1
		palSetPad(GPIOD, GPIOD_LED3);
		palSetPad(GPIOD, GPIOD_LED5);
	}
	if(g==1){
		palTogglePad(GPIOD, GPIOD_LED3);//GREEN --> LED3
		palSetPad(GPIOD, GPIOD_LED1);
		palSetPad(GPIOD, GPIOD_LED5);
//		toggle_rgb_led(LED2, GREEN_LED, 100);
//		toggle_rgb_led(LED4, GREEN_LED, 50);
//		toggle_rgb_led(LED6, GREEN_LED, 100);
//		toggle_rgb_led(LED8, GREEN_LED, 50);
		set_rgb_led(LED2, 0, 255, 0);     // __> PROBLEME DE LIBRAIRIE ENCORE!!! :(((

	}
	if(b==1){
		palTogglePad(GPIOD, GPIOD_LED5);//BLUE --> LED5
		palSetPad(GPIOD, GPIOD_LED1);
		palSetPad(GPIOD, GPIOD_LED3);
//		toggle_rgb_led(LED2, BLUE_LED, 100);
//		toggle_rgb_led(LED4, BLUE_LED, 50);
//		toggle_rgb_led(LED6, BLUE_LED, 100);
//		toggle_rgb_led(LED8, BLUE_LED, 50);

		r=0;
		g=0;
		b=0;
	}
}


/*
 * Alternative 2 pour trouver les couleurs
 * FONCTION QUI PREND DIRECTEMENT LES BON BUFFERS
 */
uint32_t find_color(uint8_t *buffer, uint8_t value){
		uint32_t mean = 0;
//		static uint32_t  mean_filter = 0;


			//performs an average
		for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
				mean += buffer[i];
				chprintf((BaseSequentialStream *)&SDU1, "valeur=%d \n", buffer[i]);
		}
		mean /= IMAGE_BUFFER_SIZE;
//		mean_filter = 0.5*mean+0.5*mean_filter;

//		chprintf((BaseSequentialStream *)&SDU1, "mean=%d \n", mean);
		if(mean < value){//Pourquoi c'est mean < value et pas l'inverse?
			return 1;
		}else{
			return 0;
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

//CHECKER ?
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
	uint8_t image_r[IMAGE_BUFFER_SIZE] = {0};
	uint8_t image_g[IMAGE_BUFFER_SIZE] = {0};
	uint8_t image_b[IMAGE_BUFFER_SIZE] = {0};
	uint16_t lineWidth = 0, red=0, green = 0, blue=0;

	bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		// pour rouge on part de i = 0 et on met l'hexadécmal de 0XF8
		//pour vert on a 2x la ligne image[i/2] et on change l'hexa
		//pour bleu on part de  et on met l'hexadécimal 0X1F
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte


//			image[i/2+1] = (uint8_t)img_buff_ptr[i+1]&0xFF;//bleu + vert     POUR FIND COLOR 2
//			image[i/2] = (uint8_t)img_buff_ptr[i]&0xFF;//rouge + vert			POUR FIND COLOR 2
			image_r[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;//rouge				POUR EXTRACT
			image_g[i/2] = ((uint8_t)img_buff_ptr[i]&0x07)<<5 | ((uint8_t)img_buff_ptr[i+1]&0xE0)>>3 ;//vert				POUR EXTRACT
			image_b[i/2] = ((uint8_t)img_buff_ptr[i+1]&0x1F)<<3;//blue				POUR EXTRACT



//			red = (int)buffer[i]&0xF8;
			//	green = (int)(buffer[i]&0x07)<<5 | (buffer[i+1]&0xE0)>>3;
			//	blue = (int)(buffer[i+1]&0x1F)<<3;
		}
		send_to_computer = false;

		//Checker la couleur
//		find_color(image);  //version RGB
//		extract_color(image); //version flanc montant ou descendant

		//search for a line in the image and gets its width in pixels
//		red = extract_line_width(image);
//		red = extract_color(image_r);
//		green = extract_color(image_g);
//		blue = extract_color(image_b);

		red= find_color(image_r, RED_VALUE);
		green= find_color(image_g, RED_VALUE);
		blue= find_color(image_b, RED_VALUE);


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
