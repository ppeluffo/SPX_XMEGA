/* 
 * File:   piloto.h
 * Author: pablo
 *
 * Created on 26 de mayo de 2023, 03:18 PM
 */

#ifndef PILOTO_H
#define	PILOTO_H

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
#include "steppers.h"
#include "rtc79410.h"
#include "ainputs.h"
    
typedef enum { ST_PRODUCTOR = 0, ST_CONSUMIDOR } t_fsm_pilotos;

#define MAX_PILOTO_PSLOTS	12

typedef struct {		// Elemento de piloto: presion, hora.
	uint16_t pTime;
	float presion;
} st_piloto_slot_t;

// PILOTO
typedef struct {
    bool enabled;
	uint16_t pulsesXrev;
	uint16_t pWidth;
    uint8_t ch_pA;
    uint8_t ch_pB;
	st_piloto_slot_t pltSlots[ MAX_PILOTO_PSLOTS ];
} piloto_conf_t;


void piloto_init_outofrtos( SemaphoreHandle_t semph);

void piloto_update_local_config( piloto_conf_t *piloto_system_conf);
void piloto_read_local_config( piloto_conf_t *piloto_system_conf);

void piloto_init(void);
bool piloto_configurado(void);
void piloto_productor(void);
void piloto_consumidor(void);
void piloto_productor_handler_slots(void);
void piloto_productor_handler_online( float presion );
void piloto_productor_handler_cmdline( float presion);

void piloto_config_defaults(void);
bool piloto_config_enable(char *s_enable );
bool piloto_config_pwidth(char *s_pwidth );
bool piloto_config_pulseXrev(char *s_pulseXrev );
bool piloto_config_slot( uint8_t slot, char *s_ptime, char *s_presion );
void piloto_print_configuration(void);
bool piloto_cmd_set_presion(char *s_presion);

uint8_t piloto_hash(uint8_t f_hash(uint8_t seed, char ch )  );
void piloto_config_debug(bool debug );


#ifdef	__cplusplus
}
#endif

#endif	/* PILOTO_H */

