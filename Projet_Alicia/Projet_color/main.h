#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define PI					3.14f// f pour float
#define NO_SPEED					0
#define MAX_SPEED				600
//CAMERA
#define IMAGE_BUFFER_SIZE		640
//PI
#define GOAL_DISTANCE 			5//[mm] Distance d'un objet à laquelle la vitesse va approcher zéro
#define MAX_DISTANCE 			25
#define ERROR_THRESHOLD			5	//[mm]
#define KP						3
#define KI 						0.1	//ne doit pas être à zéro
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
//PI rotation
#define ERROR_THRE_ROT			40// nb de steps pour π/16 radians
//PI alignment
#define ERROR_THRE_AL			30//seuil de différence entre les capteurs IR de côt
#define ROT_COEF					2
//PI diagonales
#define DIAG_DETECT				1000//valeur à laquelle le robot réagit
#define DIAG_SPEED				400 //vitesse constante donnée au robot s'il rencontre un obstacle en diagonale
#define GOAL_DIAG				350//Valeur estimée grâce à ce que renvoie le capteur dans le terminal
#define ERROR_THRE_DIAG			50
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
#define	RED_ORANGE				31
#define	GREEN_ORANGE				51
#define	BLUE_ORANGE				0
#define GREEN					63
#define RED						31
#define NO_COL					0//pas de cette composante de couleur
//PROXIMITY
#define IR_THRESHOLD				200// Distance à laquelle les capteurs IR détectent un obstacle   500 avant
#define IR_TRES_SIDE				200//capteurs sur les côté pour voir un peu plus loin qu'ils vont être bloqués   30 avant
#define RECORD_THRES				200 //threshold des capteurs IR pour demander à la caméra d'arrêter de checker du bleu car trop proche
#define TOF_RECORD				150//distance du Time-of-Flight pour activer la détection des couleurs
//CAPTEURS IR --> numérotation
#define IR1						0//capteur IR1: avant droite
#define IR2						1//capteur IR2: avant diagonale droite
#define IR3						2//capteur IR3: droite
#define IR4						3//capteur IR4: arrière droite
#define IR5						4//capteur IR5: arrière gauche
#define IR6						5//capteur IR6: gauche
#define IR7						6//capteur IR7: avant diagonale gauche
#define IR8						7//capteur IR8: avant gauche


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

/*Ces fonctions permettent la communication avec le terminal via screen pour des uint8_t ou des floats*/
void SendUint8ToComputer(uint8_t* data, uint16_t size);
void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size);

/*fonction des variables renvoyées par la fsm*/
uint8_t get_stop_fsm(void);
uint8_t get_record_allowed_fsm(void);
uint8_t get_avoid_allowed_fsm(void);
uint8_t get_sound_allowed_fsm(void);


#ifdef __cplusplus
}
#endif

#endif
