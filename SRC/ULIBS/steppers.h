/* 
 * File:   steppers.h
 * Author: pablo
 *
 * Created on 22 de mayo de 2023, 11:59 AM
 */

#ifndef STEPPERS_H
#define	STEPPERS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "pines.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "drv8814.h"
#include "xprintf.h"
    
typedef enum { STEPPER_REV = 0, STEPPER_FWD = 1 } t_stepper_dir;

void stepper_init_outofrtos(void);
void stepper_init_phase(void);
void stepper_next_phase( t_stepper_dir dir);
void stepper_set_phase(t_stepper_dir dir, uint16_t dtime);
void stepper_stop(void);
bool stepper_is_running(void);

void stepper_move( t_stepper_dir dir, uint16_t npulses, uint16_t dtime, uint16_t ptime );
bool stepper_test( char *s_cmd, char *s_dir, char *s_npulses, char *s_dtime, char *s_ptime);



#ifdef	__cplusplus
}
#endif

#endif	/* STEPPERS_H */

