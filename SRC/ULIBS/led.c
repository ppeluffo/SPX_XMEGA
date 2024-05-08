
#include "led.h"

// ---------------------------------------------------------------
void LED_init(void)
{
	// Configura el pin del led como output
	LED_PORT.DIR |= LED_PIN_bm;	
	APAGAR_LED();
}
// ---------------------------------------------------------------
void led_flash(void)
{
	// Prende el led 50ms y lo apaga
	PRENDER_LED();
	vTaskDelay( (TickType_t)( 50 / portTICK_PERIOD_MS ) );
	APAGAR_LED();
}
// ---------------------------------------------------------------

