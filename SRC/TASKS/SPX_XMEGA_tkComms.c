#include "SPX_XMEGA.h"
#include "frtos_cmd.h"

/*
 * El RS485B esta conectado al UART0 que es el que atiende el modem
 * El buffer debe ser suficientemente grande para recibir los frames de
 * configuracion del servidor
 */

//------------------------------------------------------------------------------
void tkRS485A(void * pvParameters)
{

	// Esta es la primer tarea que arranca.

( void ) pvParameters;
uint8_t c = 0;

    while ( ! run_tasks )
        vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );
    
    xprintf_P(PSTR("Starting tkRS485A..\r\n" ));
        
	// loop
	for( ;; )
	{
         
        KICK_WDG(COMMS_WDG_bp);
        
		c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
		// el read se bloquea 50ms. lo que genera la espera.
        while ( xfgetc( fdRS485A, (char *)&c ) == 1 ) {
            WAN_put(c);
        }
        
        if ( WAN_sleeping() ) {
            vTaskDelay( ( TickType_t)( 30000 / portTICK_PERIOD_MS ) );
        } else {
            vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
        }
	}    
}
//------------------------------------------------------------------------------
