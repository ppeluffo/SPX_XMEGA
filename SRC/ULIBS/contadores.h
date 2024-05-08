/* 
 * File:   contadores.h
 * Author: pablo
 *
 * Created on July 19, 2023, 5:04 PM
 */

#ifndef CONTADORES_H
#define	CONTADORES_H

#ifdef	__cplusplus
extern "C" {
#endif


#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

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

#define CNT_PARAMNAME_LENGTH	12
#define NRO_COUNTER_CHANNELS     2

#define MAX_RB_CAUDAL_STORAGE_SIZE  5
    
typedef enum { CAUDAL = 0, PULSOS } t_counter_modo;

// Configuracion de canales de contadores
typedef struct {
    bool enabled;
	char name[CNT_PARAMNAME_LENGTH];
	float magpp;
    t_counter_modo modo_medida;
    uint8_t rb_size;
} counter_channel_conf_t;

typedef struct {
    counter_channel_conf_t channel[NRO_COUNTER_CHANNELS];
} counters_conf_t;

StaticTimer_t CNT0_xTimerBuffer, CNT1_xTimerBuffer;
TimerHandle_t CNT0_xTimer, CNT1_xTimer;

void counters_init_outofrtos( SemaphoreHandle_t semph);
void counters_update_local_config( counters_conf_t *counters_system_conf);
void counters_read_local_config( counters_conf_t *counters_system_conf);

void CNT0_init(void);
void CNT1_init(void);
void CNT0_clear(void);
void CNT1_clear(void);
uint8_t CNT0_pin_read(void);
uint8_t CNT1_pin_read(void);
float CNT0_read( void );
float CNT1_read( void );

void CNT0_create_timer(void);
void CNT1_create_timer(void);
void CNT0_restore_interrupt(void);
void CNT1_restore_interrupt(void);
void CNT0_TimerCallback( TimerHandle_t xTimer );
void CNT1_TimerCallback( TimerHandle_t xTimer );
void CNT0_start_timer(void);
void CNT1_start_timer(void);
void CNT0_stop_timer(void);
void CNT1_stop_timer(void);


void counters_config_defaults( void );
void counters_print_configuration( void );
bool counters_config_channel( uint8_t ch, char *s_enable, char *s_name, char *s_magpp, char *s_modo, char *s_rb_size );
void counters_config_debug(bool debug );
bool counters_read_debug(void);
void counter_FSM(uint8_t i );


void counters_convergencia(void);
void counters_read( float *l_counters );

uint8_t counters_hash( void );

void counters_test_rb(char *data);

#ifdef	__cplusplus
}
#endif

#endif	/* CONTADORES_H */

