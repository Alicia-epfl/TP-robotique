#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define PI					3.1415926536f// f pour float
//CAMERA
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				10
#define WALL						15
#define RED_VALUE				10
#define GREEN_VALUE				19
#define BLUE_VALUE				10
#define MIN_LINE_WIDTH			40
//PI
#define ROTATION_THRESHOLD		10//no need
#define ROTATION_COEFF			2 //no need
#define GOAL_DISTANCE 			20//probablement à mettre en dessous de la détection des capteurs IR
#define MAX_DISTANCE 			25
#define ERROR_THRESHOLD			20	//[cm] because of the noise of the camera
#define KP						3
#define KI 						0.1	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
//PI rotation
#define ERROR_THRE_ROT			40// nb de steps pour π/16 radians

//LEDS
#define TOGGLE					2
#define ON						1
#define OFF						0
//RGB LEDS
#define	RED_CYAN					0
#define	GREEN_CYAN				57
#define	BLUE_CYAN				26
#define	RED_MAUVE				31
#define	GREEN_MAUVE				30
#define	BLUE_MAUVE				31
//PROXIMITY
#define	THRESHOLD				100//1000 s'actionne au touché

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

/*Ces fonctions permettent la communication avec le terminal via screen pour des uint8_t ou des floats*/
void SendUint8ToComputer(uint8_t* data, uint16_t size);
void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size);
//void readyAnimation(void);

#ifdef __cplusplus
}
#endif

#endif
