/* 
 * File:   ainputs.h
 * Author: pablo
 *
 * Created on 4 de septiembre de 2023, 03:39 PM
 */

#ifndef AINPUTS_H
#define	AINPUTS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "math.h"
    
#include "xprintf.h"
#include "ina3221.h"    
#include "pines.h"
#include "utils.h"
    
// Configuracion de canales analogicos
    
#define AIN_PARAMNAME_LENGTH	12
#define NRO_ANALOG_CHANNELS     3

#define PWRSENSORES_SETTLETIME_MS   5000

typedef struct {
    bool enabled;
	uint8_t imin;
	uint8_t imax;
	float mmin;
	float mmax;
	char name[AIN_PARAMNAME_LENGTH];
	float offset;
} ainputs_channel_conf_t;

typedef struct {
    ainputs_channel_conf_t channel[NRO_ANALOG_CHANNELS];
} ainputs_conf_t;

void ainputs_init_outofrtos( SemaphoreHandle_t semph);
void ainputs_update_local_config( ainputs_conf_t *ainputs_system_conf);
void ainputs_read_local_config( ainputs_conf_t *ainputs_system_conf);
void ainputs_init(uint8_t samples_count);
void ainputs_awake(void);
void ainputs_sleep(void);
bool ainputs_config_channel( uint8_t ch, char *s_enable, char *s_aname,char *s_imin,char *s_imax,char *s_mmin,char *s_mmax,char *s_offset );
void ainputs_config_defaults();
void ainputs_print_configuration(void);
uint16_t ainputs_read_channel_raw(uint8_t ch );
float ainputs_read_channel_mag(uint8_t ch, uint16_t an_raw_val);
void ainputs_read_channel ( uint8_t ch, float *mag, uint16_t *raw );
void ainputs_prender_sensores(void);
void ainputs_apagar_sensores(void);
bool ainputs_test_read_channel( uint8_t ch );
void ainputs_config_debug(bool debug );
bool ainputs_read_debug(void);
uint8_t ainputs_hash( void );

void AINPUTS_ENTER_CRITICAL(void);
void AINPUTS_EXIT_CRITICAL(void);


#ifdef	__cplusplus
}
#endif

#endif	/* AINPUTS_H */

