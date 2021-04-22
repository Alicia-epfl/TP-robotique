#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

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
void find_color(uint8_t *buffer){
	uint16_t i = 0, color = 0;
	uint16_t blue_m = 0, green_m = 0, red_m = 0;
	uint16_t size_c = IMAGE_BUFFER_SIZE/2; // pour définir combien il faut de pixel pour qu'on considère
										//qu'il y a bien un panneau de tel couleur devant
										//Pour le moment on a pris la moitié des pixels (à voir si on dit moins?)

	 palTogglePad(GPIOD, GPIOD_LED7);
	//EST CE QU'IL SERAIT PLUS INTERSSANT DE DETECTER UNE LIGNE POUR PAS AVOIR DE PROBLEME DANS UNE SALLE BLEU/VERTE/ROUGE?
	while(i < (IMAGE_BUFFER_SIZE))
			{
		//je considère [r4 r3 r2 r1 r0] [g5 g4 g3 g2 g1 g0] [b4 b3 b2 b1 b0]
		//test blue
		// j'ai choisi le mask [0 0 0 ? ?] [0 0 0 0 ? ?] [? 1 1 1 1]
			    if((buffer[i]|0001100001110000) ==  0001100001111111)
			    {
			       blue_m++;
			    }
			    //test green
			    	// j'ai choisi le mask [0 0 0 ? ?] [? ? 1 1 1 1] [0 0 0 ? ?]
			    	if((buffer[i]|0001111000000011) ==  0001111111100011)
			    	{
			    		green_m++;
			    	}
			    	//test red
			   	// j'ai choisi le mask [? 1 1 1 1] [0 0 0 0 ? ?] [0 0 0 ? ?]
			    	if((buffer[i]|1000000001100011) ==  1111100001100011)
			    	{
			    		red_m++;
			    	}
			    i++;
		}

	//si 2 sont 1 --> alors c'est un mélange des couleurs donc pas la couleur en question
	//par exemple bleu + rouge = violet ≠rouge ≠ bleu
	 if (red_m >size_c)
	{
		 color = color|0x04; //[0000][0100]
	}
	 if (green_m >size_c)
		 {
		 color = color|0x02; //[0000][010]
		 }
	 if (blue_m >size_c)
	 {
		 color = color|0x01; //[0000][0001]
	 }

	 //Qu'est ce que ça fait si telle couleur est détectée
	 switch(color)
	 {
	 case 0:
		 break;
	 case 1:
		 palTogglePad(GPIOD, GPIOD_LED1);
		 break;
	 case 2:
		 palTogglePad(GPIOD, GPIOD_LED5);
		 break;
	 case 3:
		 break;
	 case 4:
		 palTogglePad(GPIOD, GPIOD_LED3);
		 break;
	 default:
		 break;
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
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
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
		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
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
	}else{
		last_width = width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.
	}

	//sets a maximum width or returns the measured width
	if((PXTOCM/width) > MAX_DISTANCE){
		return PXTOCM/MAX_DISTANCE;
	}else{
		return width;
	}
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

static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	uint16_t lineWidth = 0;

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
			image[i/2+1] = (uint8_t)img_buff_ptr[i+1]&0xFF;//bleu + vert
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xFF;//rouge + vert
			//ICI ON PREND TOUT
		}
		send_to_computer = false;

		//Checker la couleur
		find_color(image);

		//search for a line in the image and gets its width in pixels
		lineWidth = extract_line_width(image);//CHECK

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
