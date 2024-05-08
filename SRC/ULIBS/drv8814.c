
#include "drv8814.h"
#include "string.h"
#include <avr/pgmspace.h>

#include "xprintf.h"

// -----------------------------------------------------------------------------
void DRV8814_init(void)
{
    // Configura los pines del DRV para mover los steppers.
    CONFIG_DRV8814_RESET;
    CONFIG_DRV8814_SLEEP;
    CONFIG_DRV8814_AEN;
    CONFIG_DRV8814_BEN;
    CONFIG_DRV8814_APH;
    CONFIG_DRV8814_BPH;
    CONFIG_DRV8814_PWR;
    //
}
// -----------------------------------------------------------------------------
bool DRV8814_test( char *pinName, char *action)
{

    if (! strcmp_P( strupr(pinName), PSTR("PWR") ) ) {
        if (! strcmp_P( strupr(action), PSTR("ON"))) {
            DRV8814_power_on();
            return(true);
        }
        
        if (!strcmp_P( strupr(action), PSTR("OFF"))) {
            DRV8814_power_off();
            return(true);
        }
        return (false);
    }
        
    if (! strcmp_P( strupr(pinName), PSTR("RESET") ) ) {
        if (! strcmp_P( strupr(action), PSTR("SET"))) {
            SET_DRV8814_RESET;
            return(true);
        }
        
        if (!strcmp_P( strupr(action), PSTR("CLEAR"))) {
            CLEAR_DRV8814_RESET;
            return(true);
        }
        return (false);
    }
    
    if (!strcmp_P( strupr(pinName), PSTR("SLEEP"))) {
        if (!strcmp_P( strupr(action), PSTR("SET"))) {
            SET_DRV8814_SLEEP;
            return(true);
        }
        
        if (!strcmp_P( strupr(action), PSTR("CLEAR"))) {
            CLEAR_DRV8814_SLEEP;
            return(true);
        }
        return (false);
    }    
    
    if (!strcmp_P( strupr(pinName), PSTR("AENA"))) {
        if (!strcmp_P( strupr(action), PSTR("SET"))) {
            SET_DRV8814_AEN;
            return(true);
        }
        
        if (!strcmp_P( strupr(action), PSTR("CLEAR"))) {
            CLEAR_DRV8814_AEN;
            return(true);
        }
        return (false);
    }

    if (!strcmp_P( strupr(pinName), PSTR("BENA"))) {
        if (!strcmp_P( strupr(action), PSTR("SET"))) {
            SET_DRV8814_BEN;
            return(true);
        }
        
        if (!strcmp_P( strupr(action), PSTR("CLEAR"))) {
            CLEAR_DRV8814_BEN;
            return(true);
        }
        return (false);
    }
 
    if (!strcmp_P( strupr(pinName), PSTR("APH"))) {
        if (!strcmp_P( strupr(action), PSTR("SET"))) {
            SET_DRV8814_APH;
            return(true);
        }
        
        if (!strcmp_P( strupr(action), PSTR("CLEAR"))) {
            CLEAR_DRV8814_APH;
            return(true);
        }
        return (false);
    }
    
    if (!strcmp_P( strupr(pinName), PSTR("BPH"))) {
        if (!strcmp_P( strupr(action), PSTR("SET"))) {
            SET_DRV8814_BPH;
            return(true);
        }
        
        if (!strcmp_P( strupr(action), PSTR("CLEAR"))) {
            CLEAR_DRV8814_BPH;
            return(true);
        }
        return (false);
    }
    
    return (false);
}
// -----------------------------------------------------------------------------
void DRV8814_awake(void)
{
	// Saco al driver 8814 de reposo.
	SET_DRV8814_RESET;
	SET_DRV8814_SLEEP;
}
//------------------------------------------------------------------------------
void DRV8814_sleep(void)
{
	// Pongo en reposo
	CLEAR_DRV8814_RESET;
	CLEAR_DRV8814_SLEEP;
}
//------------------------------------------------------------------------------
void DRV8814_pulse_Amas_Amenos(uint16_t dtime )
{
	// Aout1 = H, Aout2 = L
	SET_DRV8814_APH;	// Direccion del pulso forward

	SET_DRV8814_AEN;	// Habilito el pulso
    if (dtime > 0) {
        vTaskDelay( ( TickType_t)( dtime / portTICK_PERIOD_MS ) );
        CLEAR_DRV8814_AEN;	// Deshabilito el pulso
    }

}
//------------------------------------------------------------------------------
void DRV8814_pulse_Amenos_Amas(uint16_t dtime )
{
	// Aout1 = L, Aout2 = H
	CLEAR_DRV8814_APH;	// Direccion del pulso reverse

	SET_DRV8814_AEN;	// Habilito el pulso
    if (dtime > 0) {
        vTaskDelay( ( TickType_t)( dtime / portTICK_PERIOD_MS ) );
        CLEAR_DRV8814_AEN;	// Deshabilito el pulso
    }
}
//------------------------------------------------------------------------------
void DRV8814_pulse_Bmas_Bmenos(uint16_t dtime )
{
	// Bout1 = H, Bout2 = L
   	SET_DRV8814_BPH;	// Direccion del pulso forward

	SET_DRV8814_BEN;	// Habilito el pulso
    if (dtime > 0) {
        vTaskDelay( ( TickType_t)( dtime / portTICK_PERIOD_MS ) );
        CLEAR_DRV8814_BEN;	// Deshabilito el pulso
    }
}
//------------------------------------------------------------------------------
void DRV8814_pulse_Bmenos_Bmas(uint16_t dtime )
{
	// Bout1 = L, Bout2 = H 
	CLEAR_DRV8814_BPH;	// Direccion del pulso reverse

	SET_DRV8814_BEN;	// Habilito el pulso
    if (dtime > 0) {
        vTaskDelay( ( TickType_t)( dtime / portTICK_PERIOD_MS ) );
        CLEAR_DRV8814_BEN;	// Deshabilito el pulso
    }
}
//------------------------------------------------------------------------------
