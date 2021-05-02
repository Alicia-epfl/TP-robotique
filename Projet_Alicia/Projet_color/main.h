#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
//CAMERA
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				10
#define WALL						15
#define RED_VALUE				10
#define GREEN_VALUE				19
#define BLUE_VALUE				10
#define MIN_LINE_WIDTH			40
//PID
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2 
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			10.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define KP						800.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
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
#define	THRESHOLD				1000

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

/*Ces fonctions permettent la communication avec le terminal via screen pour des uint8_t ou des floats*/
void SendUint8ToComputer(uint8_t* data, uint16_t size);
void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
