/* 
 * File:   consignas.h
 * Author: pablo
 *
 * Created on 12 de septiembre de 2023, 10:26 AM
 */

#ifndef CONSIGNAS_H
#define	CONSIGNAS_H

#ifdef	__cplusplus
extern "C" {
#endif

    
#include "FreeRTOS.h"
#include "task.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
    
#include "xprintf.h"
#include "drv8814.h"
#include "valves.h"
#include "rtc79410.h"
    
typedef struct {
    bool enabled;
	uint16_t consigna_diurna;
	uint16_t consigna_nocturna;
} consigna_conf_t;

void consigna_config_defaults(void);
bool consigna_config( char *s_enable, char *s_cdiurna, char *s_cnocturna );
void consigna_print_configuration(void);
uint8_t consigna_hash(uint8_t f_hash(uint8_t seed, char ch )  );
void consigna_read_local_config( consigna_conf_t *consigna_system_conf );
void consigna_update_local_config( consigna_conf_t *consigna_system_conf);
void consigna_set_diurna(void);
void consigna_set_nocturna(void);
void consigna_service(void);

#ifdef	__cplusplus
}
#endif

#endif	/* CONSIGNAS_H */

