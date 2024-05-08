/* 
 * File:   led.h
 * Author: pablo
 *
 * Created on August 29, 2023, 3:17 PM
 */

#ifndef LED_H
#define	LED_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <avr/io.h>
#include "FreeRTOS.h"
#include "task.h"
    
#define LED_PORT	PORTF
#define LED_PIN_bm	PIN7_bm
#define LED_PIN_bp  PIN7_bp
    
#define PRENDER_LED() ( LED_PORT.OUT |= LED_PIN_bm )
#define APAGAR_LED() ( LED_PORT.OUT &= ~LED_PIN_bm )
#define TOGGLE_LED()    ( LED_PORT.OUT ^= 1UL << LED_PIN_bp);
    
void LED_init(void);
void led_flash(void);


#ifdef	__cplusplus
}
#endif

#endif	/* LED_H */

