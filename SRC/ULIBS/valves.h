/* 
 * File:   valves.h
 * Author: pablo
 *
 * Created on 24 de mayo de 2023, 03:45 PM
 */

#ifndef VALVES_H
#define	VALVES_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "pines.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "drv8814.h"
#include "xprintf.h"

void valve_A_open(void);
void valve_A_close(void);
void valve_B_open(void);
void valve_B_close(void);

bool valve_test( char *s_valveId, char *s_action);

#ifdef	__cplusplus
}
#endif

#endif	/* VALVES_H */

