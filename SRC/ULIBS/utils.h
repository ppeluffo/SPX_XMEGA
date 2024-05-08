/* 
 * File:   utils.h
 * Author: pablo
 *
 * Created on 4 de septiembre de 2023, 03:43 PM
 */

#ifndef UTILS_H
#define	UTILS_H

#ifdef	__cplusplus
extern "C" {
#endif


#include <avr/pgmspace.h>
#include "stdint.h"
    
uint32_t sysTicks;

uint8_t u_hash(uint8_t seed, char ch );
void vApplicationTickHook( void );

#ifdef	__cplusplus
}
#endif

#endif	/* UTILS_H */

