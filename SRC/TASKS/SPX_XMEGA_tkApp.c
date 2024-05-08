/*
 * File:   tkPiloto.c
 * Author: pablo
 *
 */


#include "SPX_XMEGA.h"
#include "piloto.h"

//------------------------------------------------------------------------------
void tkAPP(void * pvParameters)
{

	/*
     * Tarea que implementa el sistema de piloto para controlar una
     * valvula reguladora.
     * Impleentamos un modelo productor - consumidor.
     * 
     * Si el piloto esta configurado, ejecuta esta tarea
     * Solo si NO esta configurado y si la consigna, ejecuta la consigna.
     * EL PILOTO TIENE PRIORIDAD !!
     * 
     */
    
( void ) pvParameters;


	while (! run_tasks )
		vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );

	vTaskDelay( ( TickType_t)( 500 / portTICK_PERIOD_MS ) );
    xprintf_P(PSTR("Starting tkAPP..\r\n"));
        
	for( ;; )
	{
        /*
         * Corre cada 1 minuto porque el timeslot se mide como hhmm y no queremos
         * que se realmacene la orden de un mismo tslot
         * 
         */
        
        KICK_WDG(APP_WDG_bp);
        
		vTaskDelay( ( TickType_t)( 30000 / portTICK_PERIOD_MS ) );
        
        if ( systemConf.piloto_conf.enabled ) {
            piloto_productor();
            piloto_consumidor();
            
        } else if (systemConf.consigna_conf.enabled ) {
            consigna_service();
        }
        

	}
}
//------------------------------------------------------------------------------

