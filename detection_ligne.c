/*
 * detection_ligne.c
 *
 *  Created on: 5 mai 2022
 *      Author: tbous
 */

#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <detection_ligne.h>

#define IMAGE_BUFFER_SIZE		640
#define CENTER_PIXEL			320
#define WIDTH_SLOPE				5 // initialement 5
#define MIN_LINE_WIDTH			30
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define NB_LINES   				2

static int16_t line_position=0;


//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 * extract_line_position : update the static variable line1_position with the position of line 1
 */
void extract_line_position(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint16_t mean = 0;

	int32_t position =0;


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
		}else{
			position = (begin + end)/2; //gives the line position.
		}

		line_position = position;

		}
/*
 * get_line_position : returns the value in the static variable line_position for use in another file
 */
int16_t get_line_position(void){

	return line_position;
}

/*
 * CaptureImage : allows you to capture an image
 */
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

/*
 * ProcessImage : extracting the red pixels and calling the position extraction function
 */
static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	//uint16_t lineWidth = 0;

	bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)((img_buff_ptr[i]&0xF8));
		}

		//regular call of the function to update the static position variable
		extract_line_position(image);

		/*
		// for test with the python scrypt
		if(send_to_computer){
			//sends to the computer the image
		SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;
		*/
    }
}

/*
 * process_image_start : starts the two threads
 */
void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}


