/* 
 * File:   drv8814.h
 * Author: pablo
 *
 * Created on 22 de mayo de 2023, 10:39 AM
 */

#ifndef DRV8814_H
#define	DRV8814_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "pines.h"
#include "stdint.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdbool.h"
    
void DRV8814_init(void);
uint8_t DRV8814_FC1_read(void);
uint8_t DRV8814_FC2_read(void);
//
void DRV8814_awake(void);
void DRV8814_sleep(void);
void DRV8814_pulse_Amas_Amenos(uint16_t dtime );
void DRV8814_pulse_Amenos_Amas(uint16_t dtime );
void DRV8814_pulse_Bmas_Bmenos(uint16_t dtime );
void DRV8814_pulse_Bmenos_Bmas(uint16_t dtime );
//
bool DRV8814_test( char *pinName, char *action);

#define DRV8814_power_on()  ( SET_DRV8814_PWR )
#define DRV8814_power_off() ( CLEAR_DRV8814_PWR )

#ifdef	__cplusplus
}
#endif

#endif	/* DRV8814_H */

