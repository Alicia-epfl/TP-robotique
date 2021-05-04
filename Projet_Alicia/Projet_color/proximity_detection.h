/*
 * proximity_detection.h
 *
 *  Created on: Apr 29, 2021
 *      Author: Nico
 */

#ifndef PROXIMITY_DETECTION_H_
#define PROXIMITY_DETECTION_H_

void proxi_start(void);

int get_prox_front(void);

int get_prox_right_half(void);

int get_prox_right(void);

int get_prox_left(void);

int get_prox_left_half(void);

int get_prox_back(void);


#endif /* PROXIMITY_DETECTION_H_ */
