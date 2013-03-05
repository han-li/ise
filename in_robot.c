/**
* @file in_robot.c
*/


#include "in_robot.h"
#include <math.h>

#include "tpl_os.h"
#include "nxt_motors.h"        // for nxt_motor_set_speed()
#include "ecrobot_interface.h"   // for NXT_PORT_A
#include "ecrobot_private.h"


#define OS_START_SEC_VAR_32BIT
#include "tpl_memmap.h"

#define OS_STOP_SEC_VAR_32BIT
#include "tpl_memmap.h"

/*****  startOS *****/
#define APP_Task_task_idle_START_SEC_CODE
//#include "tpl_memmap.h"
/*
FUNC(int, OS_APPL_CODE) main(void)
{
    StartOS(OSDEFAULTAPPMODE);
    return 0;
}
*/
void StartupHook(void)
{
    ecrobot_init_sonar_sensor(NXT_PORT_S4);
    ecrobot_set_light_sensor_active(NXT_PORT_S3);
}

#define APP_Task_task_idle_STOP_SEC_CODE
//#include "tpl_memmap.h"

void ShutdownHook(StatusType error)
{
    ecrobot_term_sonar_sensor(NXT_PORT_S4);
    ecrobot_set_light_sensor_inactive(NXT_PORT_S3);
}


/**************** declaration of task ***************/
DeclareTask(run);
DeclareTask(changeStatus);

/**************** task run ******************/
#define APP_Task_run_START_SEC_VAR_UNSPECIFIED
#include "tpl_memmap.h"
 
VAR(int, AUTOMATIC) speedLeft = 0;
VAR(int, AUTOMATIC) speedRight = 0;
VAR(int, AUTOMATIC) acc = 1;
VAR(Robot_state, AUTOMATIC) state = UNIFORM;
VAR(int, AUTOMATIC) speed = 0;
VAR(int, AUTOMATIC) out_speed = 0;
VAR(int, AUTOMATIC) distance = 0;
VAR(int, AUTOMATIC) factorLeft = 1;
VAR(int, AUTOMATIC) factorRight = 1;
#define APP_Task_run_STOP_SEC_VAR_UNSPECIFIED
#include "tpl_memmap.h"

#define APP_Task_run_START_SEC_CODE
#include "tpl_memmap.h"

/**
 * Calcule l'angle que le robot doit prendre pour suivre la route.
 */
void computePID(int in_light, int *factorLeft, int *factorRight)
{   /*
	if(in_light < MIN_LUMINOSITY_TRESHOLD)
		*out_angle = (MIN_LUMINOSITY + MAX_LUMINOSITY) / 2 - in_light;
	else if(in_light > MAX_LUMINOSITY_TRESHOLD)
		*out_angle = -(in_light - (MIN_LUMINOSITY + MAX_LUMINOSITY) / 2);
	else
		*out_angle = 0;
     */
    
    if ((in_light < MIN_LUMINOSITY_TRESHOLD) || (in_light > MAX_LUMINOSITY_TRESHOLD)) {
      //  *factorLeft = (100 * in_light) / ((MAX_LUMINOSITY + MIN_LUMINOSITY) / 2);
      //  *factorRight = (100 * ((MAX_LUMINOSITY + MIN_LUMINOSITY) / 2)) / in_light;
	
    	*factorLeft=correction(in_light,MAX_LUMINOSITY,MIN_LUMINOSITY,1);
    	*factorRight=correction(in_light,MAX_LUMINOSITY,MIN_LUMINOSITY,-1);
	}
	else {
		*factorLeft = 100;
        *factorRight = 100;
    }
}

int correction( int input, int max, int min, int side) {
  	int target, interval;
	target = (min + max )/2;
	interval= (max-min)/2;
	return (100-side *(100* (input - target))/interval);
}
/**
 * Calcule la vitesse a appliquer aux deux moteurs en fonction de la correction de trajectoire  appliquer.
 */
void computeSpeed(int factorLeft, int factorRight, int *out_speedLeft, int *out_speedRight, Robot_state *state, int *speed)
{

   switch(*state)
    {
        case UNIFORM:
            
            break;
            
        case ACCELERATE:

		    *speed += acc;
            if (*speed >= MAX_SPEED) {
                *speed = MAX_SPEED;
            } 
		    if (*speed< MIN_FORWARD && *speed>= MIN_BACKWARD) {
			    *speed = MIN_FORWARD;
            }
            
            *out_speedLeft = (*speed * factorLeft)/100;
		    *out_speedRight = (*speed * factorRight)/100;

            break;
            
        case DECELERATE:
            
		    *speed -= acc;
            if (*speed <= -MAX_SPEED) {
                *speed = -MAX_SPEED;
            } 
		    if (*speed <= MIN_FORWARD && *speed > MIN_BACKWARD){
			    *speed = MIN_BACKWARD;
	    	}
            
   		    *out_speedLeft = (*speed * factorLeft)/100;
		    *out_speedRight = (*speed * factorRight)/100;

            break;

        case STOP_URGENCY:
            
		    *speed = -30;
		    *out_speedLeft = *speed;
		    *out_speedRight = *speed;
		    break;
    }

}
/*
TASK(run)
{   // donnee disponible dans bt_frame sur bt_length bytes.
    int light = ecrobot_get_light_sensor(NXT_PORT_S3);
    //speedLeft = ecrobot_get_motor_rev(NXT_PORT_A);
    //speedRight = ecrobot_get_motor_rev(NXT_PORT_C);
    
    //int out_speedLeft;
    //int out_speedRight;
    
    //int angle=0;
    
    int factorLeft = 1;
    int factorRight = 1;
    
    computePID(light, &factorLeft, &factorRight);
    
    computeSpeed(factorLeft, factorRight, &speedLeft, &speedRight, &state, &speed);
	
    nxt_motor_set_speed(NXT_PORT_A, speedLeft, 0);
    nxt_motor_set_speed(NXT_PORT_C, speedRight, 0);
    
display_clear(0);
display_goto_xy(0,0);
display_string("run\nState:");
display_int(state,2);
display_string("\nspeed (l r):\n");
display_int(speedLeft,3);
display_string(" ");
display_int(speedRight,3);
display_string("\nLight:");
display_int(light,2);
display_update();
    
    TerminateTask();
}

#define APP_Task_run_STOP_SEC_CODE
#include "tpl_memmap.h"




#define APP_Task_changeStatus_START_SEC_CODE
#include "tpl_memmap.h"
*/
/* OK */
void selectState(int in_distance, Robot_state *state)
{
	if(in_distance < URGENCY_DISTANCE)
	*state = STOP_URGENCY;
	else if(in_distance < MIN_DISTANCE)
        *state = DECELERATE;
	else if(in_distance > MAX_DISTANCE)
        *state = ACCELERATE;
	else
        *state = UNIFORM;
}
/*
TASK(changeStatus)
{
    int distance = ecrobot_get_sonar_sensor(NXT_PORT_S4);
    
    selectState(distance, &state);
	
//display_clear(0);
display_goto_xy(0,5);
display_string("changeStatus\nDistance:");
display_int(distance,2);
display_update();
	
    TerminateTask();
}

#define APP_Task_changeStatus_STOP_SEC_CODE
#include "tpl_memmap.h"


*/

/*****************    ISR    *******************/
/**********  Button Stop  ****************/
/*
 #define APP_ISR_isr_button_stop_START_SEC_CODE
 #include "tpl_memmap.h"
 ISR(isr_button_stop)
 {
 	ShutdownOS(E_OK);
 }
 #define APP_ISR_isr_button_stop_STOP_SEC_CODE
 #include "tpl_memmap.h"
 */
/*
 */
 /************* Sensor Sonar  ***************/
/*
#define APP_ISR_isr_sensor_sonar_START_SEC_CODE
#include "tpl_memmap.h"
ISR(isr_sensor_sonar)
{
    ecrobot_get_sonar_sensor()
}
#define APP_ISR_isr_sensor_sonar_STOP_SEC_CODE
#include "tpl_memmap.h"
*/
