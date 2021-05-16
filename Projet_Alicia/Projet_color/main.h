/**
 * Fichier: main.c
 * auteurs: Nicolas Nouel et Alicia Mauroux
 *
 * @brief
 * Ce fichier est le coeur de notre programme. C'est ici que se fait toutes les initialisations.
 * C'est également ici que se trouve la majorité des defines
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constantes pour les différents segments du projet
#define PI						3.14f// f pour float
#define NO_SPEED					0// minimale
#define MAX_SPEED				900//maximale (par rapport à ce que l'on souhaite pour le labyrinthe)
#define HIGH_SPEED				400 //haute
#define MID_SPEED				200//moyenne
//CAMERA
#define IMAGE_BUFFER_SIZE		640
#define FACT_R_G					1//facteur de couleur entre le rouge et le vert
#define FACT_R_B					1//facteur de couleur entre le rouge et le bleu			FACTEURS DE COULEURS
#define FACT_G_R					1.3//facteur de couleur entre le vert et le rouge			POUR DETERMINER LA COULEUR
#define FACT_G_B					1//facteur de couleur entre le vert et le bleu			QUE VOIT LA CAMERA EN FACE
#define FACT_B_G					1//facteur de couleur entre le bleu et le vert			D'UNE CARTE
#define FACT_B_R					1.2//facteur de couleur entre le bleu et le rouge
#define TRES_BLUE				0//seuil nécessaire pour que ça soit du bleu (car très proche du vert)
//PI
#define GOAL_DISTANCE 			1//[mm] Distance d'un objet à laquelle la vitesse va approcher zéro
#define MAX_DISTANCE 			150//[mm]
#define ERROR_THRESHOLD			5	//[mm]
#define KP						4
#define KI 						0.1	//ne doit pas être à zéro
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
//PI rotation
#define ERROR_THRE_ROT			40// nb de steps pour π/16 radians
//PI alignment
#define ERROR_THRE_AL			50//seuil de différence entre les capteurs IR de côté
#define ROT_COEF					0.5//coefficient de combien influence la vitesse d'alignement sur la vitesse courante
#define KP_AL					2
#define KI_AL					0.1
//PI diagonales
#define DIAG_DETECT				600//valeur à laquelle le robot réagit
#define GOAL_DIAG				400//Valeur estimée grâce à ce que renvoie le capteur dans le terminal
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
#define	GREEN_MAUVE				20
#define	BLUE_MAUVE				31
#define	RED_ORANGE				31
#define	GREEN_ORANGE				41
#define	BLUE_ORANGE				0
#define GREEN					63
#define RED						31
#define NO_COL					0//pas de cette composante de couleur
#define 	GREEN_TRES				63//valeur maximale du vert
#define LIGHT_BLUE_FACT			1.5//facteur pour éclaircir le bleu
//PROXIMITY
#define IR_THRESHOLD				700// Distance à laquelle les capteurs IR détectent un obstacle
#define IR_THRESHOLD_BACK		300// Distance à laquelle les capteurs IR arrières détectent un obstacle
#define IR_TRES_SIDE				100//capteurs sur les côté pour voir un peu plus loin qu'ils vont être bloqués
#define RECORD_THRES				400 //threshold des capteurs IR pour demander à la caméra d'arrêter de checker les couleurs car trop proche
#define TOF_RECORD				150//[mm]distance du Time-of-Flight pour activer la détection des couleurs
//CAPTEURS IR --> numérotation
#define IR1						0//capteur IR1: avant droite
#define IR2						1//capteur IR2: avant diagonale droite
#define IR3						2//capteur IR3: droite
#define IR4						3//capteur IR4: arrière droite
#define IR5						4//capteur IR5: arrière gauche
#define IR6						5//capteur IR6: gauche
#define IR7						6//capteur IR7: avant diagonale gauche
#define IR8						7//capteur IR8: avant gauche

/** Robot bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

/*Ces fonctions permettent la communication avec le terminal via screen pour des uint8_t ou des floats*/
void SendUint8ToComputer(uint8_t* data, uint16_t size);
void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size);

// @brief
/*fonction des variables renvoyées par la fsm*/
uint8_t get_stop_fsm(void);
uint8_t get_record_allowed_fsm(void);
uint8_t get_avoid_allowed_fsm(void);
uint8_t get_sound_allowed_fsm(void);

/*@brief
 * Fonction d'animation pour le restart
 * Placée ici pour garder l'organisation "thread" avant les "fonctions"
 * dans le main*/
void reReadyAnimation(void);

// @brief
/* fonction de la gestion de run
 *
 * si run est activé, le jeu est lancé
 * si run est à zéro, le robot est en "standbye"*/
void toggle_run(void);


#ifdef __cplusplus
}
#endif

#endif
