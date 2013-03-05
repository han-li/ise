#ifndef IN_ROBOT_H
#define IN_ROBOT_H

/*********************/
/* Ungenerated datas */
/*********************/

#include "nxt_motors.h" 
#include "ecrobot_interface.h"

typedef enum Robot_state
{
	ACCELERATE = 0,
	DECELERATE = 1,
	UNIFORM = 2,
	STOP_URGENCY = 3
} Robot_state;

/*************************/
/* End Ungenerated datas */
/*************************/

/* treshold distances, in centimeters */
#define MIN_DISTANCE			25
#define MAX_DISTANCE			30

/* light sensor characteristics */
#define MIN_LUMINOSITY			575
#define MAX_LUMINOSITY			695
#define	MIN_LUMINOSITY_TRESHOLD		620
#define MAX_LUMINOSITY_TRESHOLD		650
#define MIN_FORWARD 			55
#define MIN_BACKWARD 			-55
#define URGENCY_DISTANCE		15
/* motor characteristics */
#define MAX_SPEED				80

/* wheels characteristics, in millimeters */
#define PERIMETER				200

void computePID(int in_light, int *factorLeft, int *factorRight);
void computeSpeed(int factorLeft, int factorRight, int *out_speedLeft, int * out_speedRight, Robot_state *state, int *speed);
void selectState(int in_distance, Robot_state *state);
int correction( int input, int max, int min, int side);
#endif

