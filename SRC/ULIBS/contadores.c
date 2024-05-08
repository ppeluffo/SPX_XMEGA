
#include "contadores.h"

typedef enum { P_OUTSIDE=0, P_CHECKING, P_INSIDE, P_FILTER } pulse_t;

// Estructura de control de los contadores.
typedef struct {
	float caudal;                    // Caudal instantaneo en c/pulso
	uint32_t ticks_count;            // total de ticks entre el pulso actual y el anterior
    uint8_t  debounceTicks_count;    // ticks desde el flanco del pulso a validarlos
	uint16_t pulse_count;            // contador de pulsos
    bool count_enabled;              // 
} counter_cbk_t;

static counter_cbk_t CNTCB[NRO_COUNTER_CHANNELS];
static bool f_debug_counters;

// Los caudales los almaceno en un RB y lo que doy es el promedio !!
typedef struct {
    float caudal;
} t_caudal_s;

t_caudal_s caudal_storage_0[MAX_RB_CAUDAL_STORAGE_SIZE];
t_caudal_s caudal_storage_1[MAX_RB_CAUDAL_STORAGE_SIZE];
rBstruct_s caudal_RB_0,caudal_RB_1;

// Configuracion local del sistema de contadores
counters_conf_t counters_conf;

void promediar_rb_caudal(void);

static SemaphoreHandle_t countersLocalSem;


// -----------------------------------------------------------------------------
void counters_init_outofrtos( SemaphoreHandle_t semph)
{
    //f_debug_counters = true;
    
    countersLocalSem = semph;
}
// -----------------------------------------------------------------------------
void counters_update_local_config( counters_conf_t *counters_system_conf)
{
    while ( xSemaphoreTake( countersLocalSem, ( TickType_t ) 5 ) != pdTRUE )
  		vTaskDelay( ( TickType_t)( 1 ) );
    memcpy( &counters_conf, counters_system_conf, sizeof(counters_conf_t));
    xSemaphoreGive( countersLocalSem );
}
// -----------------------------------------------------------------------------
void counters_read_local_config( counters_conf_t *counters_system_conf)
{
    while ( xSemaphoreTake( countersLocalSem, ( TickType_t ) 5 ) != pdTRUE )
  		vTaskDelay( ( TickType_t)( 1 ) );
    memcpy( counters_system_conf, &counters_conf, sizeof(counters_conf_t)); 
    xSemaphoreGive( countersLocalSem );
}
// -----------------------------------------------------------------------------
void counters_config_defaults( void )
{
    /*
     * Realiza la configuracion por defecto de los canales digitales.
     */

uint8_t i = 0;

	for ( i = 0; i < NRO_COUNTER_CHANNELS; i++ ) {
		//snprintf_P( counters_conf[i].name, CNT_PARAMNAME_LENGTH, PSTR("X%d\0"),i );
        snprintf_P( counters_conf.channel[i].name, CNT_PARAMNAME_LENGTH, PSTR("X") );
        counters_conf.channel[i].enabled = false;
		counters_conf.channel[i].magpp = 1;
        counters_conf.channel[i].modo_medida = CAUDAL;
        counters_conf.channel[i].rb_size = 1;
	}
}
//------------------------------------------------------------------------------
void counters_print_configuration( void )
{
    /*
     * Muestra la configuracion de todos los canales de contadores en la terminal
     * La usa el comando tkCmd::status.
     */
    
uint8_t i = 0;

    xprintf_P(PSTR("Counters:\r\n"));
    xprintf_P(PSTR(" debug: "));
    f_debug_counters ? xprintf_P(PSTR("true\r\n")) : xprintf_P(PSTR("false\r\n"));
    
	for ( i = 0; i < NRO_COUNTER_CHANNELS; i++) {
        
        if ( counters_conf.channel[i].enabled ) {
            xprintf_P( PSTR(" c%d: +"),i);
        } else {
            xprintf_P( PSTR(" c%d: -"),i);
        }
                
        xprintf_P( PSTR("[%s,magpp=%.03f,"),counters_conf.channel[i].name, counters_conf.channel[i].magpp );
        if ( counters_conf.channel[i].modo_medida == CAUDAL ) {
            xprintf_P(PSTR("CAUDAL,"));
        } else {
            xprintf_P(PSTR("PULSO,"));
        }
        
        xprintf_P( PSTR("rbsize=%d]\r\n"), counters_conf.channel[i].rb_size );
    }       
}
//------------------------------------------------------------------------------
bool counters_config_channel( uint8_t ch, char *s_enable, char *s_name, char *s_magpp, char *s_modo, char *s_rb_size )
{
	// Configuro un canal contador.
	// channel: id del canal
	// s_param0: string del nombre del canal
	// s_param1: string con el valor del factor magpp.
	//
	// {0..1} dname magPP

bool retS = false;

    //xprintf_P(PSTR("DEBUG COUNTERS: en=%s,name=%s,magpp=%s,modo=%s,rbsize=%s\r\n"), s_enable,s_name,s_magpp,s_modo,s_rb_size  );

	if ( s_name == NULL ) {
		return(retS);
	}

	if ( ( ch >=  0) && ( ch < NRO_COUNTER_CHANNELS ) ) {

        // Enable ?
        if (!strcmp_P( strupr(s_enable), PSTR("TRUE"))  ) {
            counters_conf.channel[ch].enabled = true;
        
        } else if (!strcmp_P( strupr(s_enable), PSTR("FALSE"))  ) {
            counters_conf.channel[ch].enabled = false;
        }
        
		// NOMBRE
		snprintf_P( counters_conf.channel[ch].name, CNT_PARAMNAME_LENGTH, PSTR("%s"), s_name );

		// MAGPP
		if ( s_magpp != NULL ) { counters_conf.channel[ch].magpp = atof(s_magpp); }

        // MODO ( PULSO/CAUDAL )
		if ( s_modo != NULL ) {
			if ( strcmp_P( strupr(s_modo), PSTR("PULSO")) == 0 ) {
				counters_conf.channel[ch].modo_medida = PULSOS;

			} else if ( strcmp_P( strupr(s_modo) , PSTR("CAUDAL")) == 0 ) {
				counters_conf.channel[ch].modo_medida = CAUDAL;

			} else {
				xprintf_P(PSTR("ERROR: counters modo: PULSO/CAUDAL only!!\r\n"));
                return (false);
			}
		}
        
        if ( s_rb_size != NULL ) {
            counters_conf.channel[ch].rb_size = atoi(s_rb_size);
        } else {
            counters_conf.channel[ch].rb_size = 1;
        }
        
        if ( counters_conf.channel[ch].rb_size > MAX_RB_CAUDAL_STORAGE_SIZE ) {
            counters_conf.channel[ch].rb_size = 1;
        }
        
		retS = true;
	}

	return(retS);

}
//------------------------------------------------------------------------------
void counters_config_debug(bool debug )
{
    if ( debug ) {
        f_debug_counters = true;
    } else {
        f_debug_counters = false;
    }
}
//------------------------------------------------------------------------------
bool counters_read_debug(void)
{
    return (f_debug_counters);
}
//------------------------------------------------------------------------------
uint8_t counters_hash( void )
{
    
uint8_t hash_buffer[32];
uint8_t i,j;
uint8_t hash = 0;
char *p;

    // Calculo el hash de la configuracion de los contadores
    for(i=0; i < NRO_COUNTER_CHANNELS; i++) {

        memset(hash_buffer, '\0', sizeof(hash_buffer));
        j = 0;
        if ( counters_conf.channel[i].enabled ) {
            j += sprintf_P( (char *)&hash_buffer[j], PSTR("[C%d:TRUE,"), i );
        } else {
            j += sprintf_P( (char *)&hash_buffer[j], PSTR("[C%d:FALSE,"), i );
        }
        j += sprintf_P( (char *)&hash_buffer[j], PSTR("%s,"), counters_conf.channel[i].name );
        j += sprintf_P( (char *)&hash_buffer[j], PSTR("%.03f,"), counters_conf.channel[i].magpp );
        
        if ( counters_conf.channel[i].modo_medida == 0 ) {
            j += sprintf_P( (char *)&hash_buffer[j], PSTR("CAUDAL,"));
        } else {
            j += sprintf_P( (char *)&hash_buffer[j], PSTR("PULSOS,"));
        }

        j += sprintf_P( (char *)&hash_buffer[j], PSTR("%d]"), counters_conf.channel[i].rb_size );

       
        p = (char *)hash_buffer;
        while (*p != '\0') {
            hash = u_hash(hash, *p++);
        }
        
        //xprintf_P(PSTR("HASH_CNT:<%s>, hash=%d\r\n"),hash_buffer, hash );
    }
 
    return(hash);
    
}
//------------------------------------------------------------------------------
// OPERATIVA DE LOS TIMERS
//------------------------------------------------------------------------------
void counters_convergencia(void)
{
    /*
     * Esta opcion es para asegurar la convergencia del valor
     * en la medida que el caudal es 0.
     * En este caso debemos insertar un registro en el ringbuffer en 0 si 
     * en el periodo no llegaron pulsos.
     * OJO: Esto va antes de poner los contadores en 0. !!!
     */
  
uint8_t cnt;
t_caudal_s rb_element;

    //xprintf_P(PSTR("DEBUG COUNTERS CLEAR\r\n"));
    for ( cnt=0; cnt < NRO_COUNTER_CHANNELS; cnt++) {
        
        // Aseguro la convergencia a 0 de los caudales
        if ( CNTCB[cnt].pulse_count == 0 ) {
            rb_element.caudal = 0.0;
            if (cnt==0) {
                rBstruct_insert_at_tail( &caudal_RB_0, &rb_element );
            } else {
                rBstruct_insert_at_tail( &caudal_RB_1, &rb_element );
            }
        }
    }

}
//------------------------------------------------------------------------------
float CNT0_read( void )
{    
    //promediar_rb_caudal();
    
    if ( counters_conf.channel[0].modo_medida == CAUDAL ) {
        return( CNTCB[0].caudal );
    } else {
        return( (float) CNTCB[0].pulse_count );
    }
}
//------------------------------------------------------------------------------
float CNT1_read( void )
{    
    //promediar_rb_caudal();
    
    if ( counters_conf.channel[1].modo_medida == CAUDAL ) {
        return ( CNTCB[1].caudal );
    } else {
        return ( (float) CNTCB[1].pulse_count );
    }
}
//------------------------------------------------------------------------------
void CNT0_init(void)
{
    CNT0_CONFIG();
    
    CNTCB[0].caudal = 0.0;          
	CNTCB[0].ticks_count = 0;
    CNTCB[0].debounceTicks_count = 0;
    CNTCB[0].pulse_count = 0;
	CNTCB[0].count_enabled = false;
      
    CNT0_clear();
    
    rBstruct_CreateStatic ( 
       &caudal_RB_0, 
       &caudal_storage_0, 
       //MAX_RB_CAUDAL_STORAGE_SIZE, 
       counters_conf.channel[0].rb_size,
       sizeof(t_caudal_s), 
       true  
    );
    
    CNT0_create_timer();
    CNT0_restore_interrupt();
    
}
// -----------------------------------------------------------------------------
void CNT1_init(void)
{
    CNT0_CONFIG();
    
    CNTCB[1].caudal = 0.0;          
	CNTCB[1].ticks_count = 0;
    CNTCB[1].debounceTicks_count = 0;
    CNTCB[1].pulse_count = 0;
	CNTCB[1].count_enabled = false;
      
    CNT1_clear();
    
    rBstruct_CreateStatic ( 
       &caudal_RB_1, 
       &caudal_storage_1, 
       //MAX_RB_CAUDAL_STORAGE_SIZE, 
       counters_conf.channel[1].rb_size,
       sizeof(t_caudal_s), 
       true  
    );
    
    CNT1_create_timer();
    CNT1_restore_interrupt();
    
}
//------------------------------------------------------------------------------
void CNT0_clear(void)
{
    CNTCB[0].pulse_count = 0;
    CNTCB[0].caudal = 0.0;
}
//------------------------------------------------------------------------------
void CNT1_clear(void)
{
    CNTCB[1].pulse_count = 0;
    CNTCB[1].caudal = 0.0;
}
//------------------------------------------------------------------------------
uint8_t CNT0_pin_read(void)
{
    return ( ( CNT0_PORT.IN & CNT0_PIN_bm ) >> CNT0_PIN) ;
}
// -----------------------------------------------------------------------------
uint8_t CNT1_pin_read(void)
{
    return ( ( CNT1_PORT.IN & CNT1_PIN_bm ) >> CNT1_PIN) ;
}
// -----------------------------------------------------------------------------
void promediar_rb_caudal(void)
{
uint8_t i;
t_caudal_s rb_element;
float q0,q1, Qavg0,Qavg1;

    // Promedio los ringBuffers
    Qavg0=0.0;
    for (i=0; i < counters_conf.channel[0].rb_size; i++) {
        
        rb_element = caudal_storage_0[i];
        q0 = rb_element.caudal;
        Qavg0 += q0;
        if ( f_debug_counters ) {
            xprintf_P(PSTR("DEBUG: i=%d [q0=%0.3f, avgQ0=%0.3f]\r\n"), i, q0, Qavg0 );
        }        
    }
    Qavg0 /= counters_conf.channel[0].rb_size;
    CNTCB[0].caudal = Qavg0;
    if ( f_debug_counters ) {
        xprintf_P(PSTR("DEBUG: Qavg0=%0.3f\r\n"), Qavg0 );
    }
    
    
    Qavg1=0.0;
    for (i=0; i < counters_conf.channel[1].rb_size; i++) {
        
        rb_element = caudal_storage_1[i];
        q1 = rb_element.caudal;
        Qavg1 += q1;
        if ( f_debug_counters ) {
            xprintf_P(PSTR("DEBUG: i=%d [q1=%0.3f, avgQ1=%0.3f]\r\n"), i, q1, Qavg1 );
        }        
    }
    Qavg1 /= counters_conf.channel[1].rb_size;
    CNTCB[1].caudal = Qavg1;
    if ( f_debug_counters ) {
        xprintf_P(PSTR("DEBUG: Qavg1=%0.3f\r\n"), Qavg1);
    }
    
}
//------------------------------------------------------------------------------
void CNT0_create_timer(void)
{
    
	CNT0_xTimer = xTimerCreateStatic ("CNT0",
			pdMS_TO_TICKS( 10 ),
			pdTRUE,
			( void * ) 0,
			CNT0_TimerCallback,
			&CNT0_xTimerBuffer
			);    
}
//------------------------------------------------------------------------------
void CNT1_create_timer(void)
{
    
	CNT1_xTimer = xTimerCreateStatic ("CNT1",
			pdMS_TO_TICKS( 10 ),
			pdTRUE,
			( void * ) 0,
			CNT1_TimerCallback,
			&CNT1_xTimerBuffer
			);    
}
// -----------------------------------------------------------------------------
void CNT0_start_timer(void)
{
    xTimerStart(CNT0_xTimer, 10);
    
}
// -----------------------------------------------------------------------------
void CNT1_start_timer(void)
{
    xTimerStart(CNT1_xTimer, 10);
    
}
// -----------------------------------------------------------------------------
void CNT0_stop_timer(void)
{
    xTimerStop(CNT0_xTimer, 10);
    
}
// -----------------------------------------------------------------------------
void CNT1_stop_timer(void)
{
    xTimerStop(CNT1_xTimer, 10);
    
}
// -----------------------------------------------------------------------------
void CNT0_TimerCallback( TimerHandle_t xTimer )
{
 
uint32_t duracion_pulso_ticks;
t_caudal_s rb_element;

    // Controlo el periodo de debounce. (2ticks = 20ms)
    if (  CNTCB[0].count_enabled ) {
        CNTCB[0].debounceTicks_count++;
                   
        // Debounce. Veo si el pulso permance valido
        if ( CNTCB[0].debounceTicks_count == 2 ) {  
        
            if ( CNT0_pin_read() == 1 ) {
                // El pulso se mantiene. Lo cuento.
                CNTCB[0].pulse_count++;
                
                // Calculo el caudal instantaneo
                if ( counters_conf.channel[0].modo_medida == CAUDAL ) {
                    // Tengo 1 pulso en N ticks.
                    // 1 pulso -------> ticks_counts mS
                    // magpp (mt3) ---> ticks_counts mS
                    // Elticks es 1ms en el Frtos.!!!
                    
                    if ( sysTicks > CNTCB[0].ticks_count ) {
                        duracion_pulso_ticks = sysTicks - CNTCB[0].ticks_count;
                    } else {
                        duracion_pulso_ticks = 0xFFFFFFFF - sysTicks + CNTCB[0].ticks_count;
                    }
                    CNTCB[0].ticks_count = sysTicks;
                    
                    if ( duracion_pulso_ticks > 0 ) {
                        CNTCB[0].caudal =  (( counters_conf.channel[0].magpp * 3600000) /  duracion_pulso_ticks  ); // En mt3/h  
                    } else {   
                        CNTCB[0].caudal = 0;
                    } 
                    // Guardo el caudal en el RB
                    rb_element.caudal = CNTCB[0].caudal;
                    rBstruct_Poke(&caudal_RB_0, &rb_element);  
                }
                
                if ( f_debug_counters ) {
                    if ( counters_conf.channel[0].modo_medida == CAUDAL ) {
                        xprintf_P( PSTR("COUNT_0: Q0=%0.3f, P0=%d\r\n"), CNTCB[0].caudal, CNTCB[0].pulse_count );
                    } else {
                        xprintf_P( PSTR("COUNT_P0=%d\r\n"), CNTCB[0].pulse_count );
                    }
                }
            }
            return;
        }
        
        // Despues de 5 ticks alto, no cuento mas. Espero que baje.
        if ( CNTCB[0].debounceTicks_count == 5 ) {
            // No cuento mas ticks. Salgo cuando el pulso suba
            CNTCB[0].count_enabled = false;
            return;
        }  
        
    } else {
        // Espero que termine el pulso volviendo el pin a 0.
        if ( CNT0_pin_read() == 0 ) {
            // Apago el timer.
            CNT0_stop_timer();
            // El pulso ya bajo: rearmo la interrupcion
            CNT0_restore_interrupt();          
        }
        return;
    } 
 
}
//------------------------------------------------------------------------------
void CNT1_TimerCallback( TimerHandle_t xTimer )
{
uint32_t duracion_pulso_ticks;
t_caudal_s rb_element;

   // Controlo el periodo de debounce. (2ticks = 20ms)
    if (  CNTCB[1].count_enabled ) {
        CNTCB[1].debounceTicks_count++;
                   
         // Debounce. Veo si el pulso permance valido
        if ( CNTCB[1].debounceTicks_count == 2 ) {  
        
            if ( CNT1_pin_read() == 1 ) {
                // El pulso se mantiene. Lo cuento.
                CNTCB[1].pulse_count++;
                
                // Calculo el caudal instantaneo
                if ( counters_conf.channel[1].modo_medida == CAUDAL ) {
                    // Tengo 1 pulso en N ticks.
                    // 1 pulso -------> ticks_counts mS
                    // magpp (mt3) ---> ticks_counts mS
                    // Elticks es 1ms en el Frtos.!!!
                    
                    if ( sysTicks > CNTCB[1].ticks_count ) {
                        duracion_pulso_ticks = sysTicks - CNTCB[1].ticks_count;
                    } else {
                        duracion_pulso_ticks = 0xFFFFFFFF - sysTicks + CNTCB[1].ticks_count;
                    }
                    CNTCB[1].ticks_count = sysTicks;
                    
                    if ( duracion_pulso_ticks > 0 ) {
                        CNTCB[1].caudal =  (( counters_conf.channel[1].magpp * 3600000) /  duracion_pulso_ticks  ); // En mt3/h  
                    } else {   
                        CNTCB[1].caudal = 0;
                    } 
                    // Guardo el caudal en el RB
                    rb_element.caudal = CNTCB[1].caudal;
                    rBstruct_Poke(&caudal_RB_1, &rb_element);  
                }
                
                if ( f_debug_counters ) {
                    if ( counters_conf.channel[1].modo_medida == CAUDAL ) {
                        xprintf_P( PSTR("COUNT_1: Q1=%0.3f, P1=%d\r\n"), CNTCB[1].caudal, CNTCB[1].pulse_count );
                    } else {
                        xprintf_P( PSTR("COUNT_P1=%d\r\n"), CNTCB[1].pulse_count );
                    }
                }
            }
            return;
        }
        
        if ( CNTCB[1].debounceTicks_count == 5 ) {
            // No cuento mas ticks. Salgo cuando el pulso suba
            CNTCB[1].count_enabled = false;
            return;
        }  
        
    } else {
    
        if ( CNT1_pin_read() == 0 ) {
            // Apago el timer.
            CNT1_stop_timer();
            // El pulso ya bajo: rearmo la interrupcion
            CNT1_restore_interrupt();          
        }
        return;
    } 
}
//------------------------------------------------------------------------------
void CNT0_restore_interrupt(void)
{

	PORTA.INT0MASK = PIN2_bm;
	PORTA.INTCTRL = PORT_INT0LVL0_bm;
	PORTA.INTFLAGS = PORT_INT0IF_bm;

}
//------------------------------------------------------------------------------
void CNT1_restore_interrupt(void)
{

	PORTB.INT0MASK = PIN2_bm;
	PORTB.INTCTRL = PORT_INT0LVL0_bm;
	PORTB.INTFLAGS = PORT_INT0IF_bm;
}
//------------------------------------------------------------------------------
ISR ( PORTA_INT0_vect )
{
    // Q0 es PA2
	// Esta ISR se activa cuando el contador D2 (PA2) genera un flaco se subida.
    // Arranca el timer (on-shot) para generar el debounce.
    // Se desactiva.
    // Al expirar el timer cuenta el pulso y hace las cuentas si es caudal o no.

	// Deshabilita la interrupcion por ahora ( enmascara )
	PORTA.INT0MASK = 0x00;
	PORTA.INTCTRL = 0x00;
    
    // Arranca el timer de debounce e invoca al callback c/10ms
    CNTCB[0].debounceTicks_count = 0;
    CNTCB[0].count_enabled = true;
    CNT0_start_timer();

}
//------------------------------------------------------------------------------
ISR( PORTB_INT0_vect )
{
    // Q1 es PB2
	// Esta ISR se activa cuando el contador D1 (PB2) genera un flaco se subida.

	PORTB.INT0MASK = 0x00;
	PORTB.INTCTRL = 0x00;
	//PORTF.OUTTGL = 0x80;	// Toggle A2
	//IO_set_LED_KA();

    // Arranca el timer de debounce
    CNTCB[1].debounceTicks_count = 0;
    CNTCB[1].count_enabled = true;
    CNT1_start_timer();
}
//------------------------------------------------------------------------------
