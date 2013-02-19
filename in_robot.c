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

FUNC(int, OS_APPL_CODE) main(void)
{
    StartOS(OSDEFAULTAPPMODE);
    return 0;
}

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
VAR(int, AUTOMATIC) acc = 20;
VAR(Robot_state, AUTOMATIC) state = UNIFORM;
VAR(int, AUTOMATIC) speed = 0;
VAR(int, AUTOMATIC) distance = 0;

#define APP_Task_run_STOP_SEC_VAR_UNSPECIFIED
#include "tpl_memmap.h"

#define APP_Task_run_START_SEC_CODE
#include "tpl_memmap.h"

/**
 * Calcule l'angle que le robot doit prendre pour suivre la route.
 */
void computePID(int in_light, int *out_angle)
{
	if(in_light < MIN_LUMINOSITY_TRESHOLD)
		*out_angle = (MIN_LUMINOSITY + MAX_LUMINOSITY) / 2 - in_light;
	else if(in_light > MAX_LUMINOSITY_TRESHOLD)
		*out_angle = -(in_light - (MIN_LUMINOSITY + MAX_LUMINOSITY) / 2);
	else
		*out_angle = 0;
}

/**
 * Calcule la vitesse a appliquer aux deux moteurs en fonction de la correction de trajectoire  appliquer.
 */
void computeSpeed(int in_angle, int *out_speedLeft, int *out_speedRight, Robot_state *state, int *speed)
{
    switch(*state)
    {
        case UNIFORM:
            
            break;
            
        case ACCELERATE:
            
            if (*out_speedLeft == MAX_SPEED) {
                *out_speedLeft = MAX_SPEED;
            } else {
                *out_speedLeft += acc;
            }
            
            if (*out_speedRight == MAX_SPEED) {
                *out_speedRight = MAX_SPEED;
            } else {
                *out_speedRight += acc;
            }
            
            *speed += acc;

            break;
            
        case DECELERATE:
            if (*out_speedLeft == -MAX_SPEED) {
                *out_speedLeft = -MAX_SPEED;
            } else {
                *out_speedLeft -= acc;
            }
            
            if (*out_speedRight == -MAX_SPEED) {
                *out_speedRight = -MAX_SPEED;
            } else {
                *out_speedRight -= acc;
            }
            
            *speed -= acc;

            break;
    }
    //*out_speedLeft = *speed;
    //*out_speedRight = *speed;

}

TASK(run)
{   // donnee disponible dans bt_frame sur bt_length bytes.
    int light = ecrobot_get_light_sensor(NXT_PORT_S3);
    //speedLeft = ecrobot_get_motor_rev(NXT_PORT_A);
    //speedRight = ecrobot_get_motor_rev(NXT_PORT_C);
    
    //int out_speedLeft;
    //int out_speedRight;
    
    int angle;
    
    computePID(light, &angle);
    
    computeSpeed(angle, &speedLeft, &speedRight, &state, &speed);
	
    nxt_motor_set_speed(NXT_PORT_A, speedLeft, 0);
    nxt_motor_set_speed(NXT_PORT_C, speedRight, 0);
    
    TerminateTask();
}
#define APP_Task_run_STOP_SEC_CODE
#include "tpl_memmap.h"




#define APP_Task_changeStatus_START_SEC_CODE
#include "tpl_memmap.h"

/* OK */
void selectState(int in_distance, Robot_state *state)
{
	if(in_distance < MIN_DISTANCE)
        *state = DECELERATE;
	else if(in_distance > MAX_DISTANCE)
        *state = ACCELERATE;
	else
        *state = UNIFORM;
}

TASK(changeStatus)
{
    int distance = ecrobot_get_sonar_sensor(NXT_PORT_S3);
    
    selectState(distance, &state);
	
    TerminateTask();
}

#define APP_Task_changeStatus_STOP_SEC_CODE
#include "tpl_memmap.h"




/*****************    ISR    *******************/
/**********  Button Stop  ****************/
 #define APP_ISR_isr_button_stop_START_SEC_CODE
 #include "tpl_memmap.h"
 ISR(isr_button_stop)
 {
 	ShutdownOS(E_OK);
 }
 #define APP_ISR_isr_button_stop_STOP_SEC_CODE
 #include "tpl_memmap.h"
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
