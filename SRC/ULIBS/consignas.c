
#include "consignas.h"

consigna_conf_t consigna_conf;

//------------------------------------------------------------------------------
void consigna_set_nocturna(void)
{
	// ( VA open / VB close ) -> ( VA close / VB open )
	// Open VB con lo que el punto común de las válvulas queda a pAtm y la VA puede operar correctamente.
	// Close VA.

    // Proporciono corriente.
	DRV8814_power_on();
	// Espero 15s que se carguen los condensasores
	vTaskDelay( ( TickType_t)( 25000 / portTICK_RATE_MS ) );
    
    DRV8814_awake();
    // Close VA,VB
    DRV8814_pulse_Amenos_Amas(10);
    DRV8814_pulse_Bmenos_Bmas(10);
    
    vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
    
    // Open VA
    DRV8814_pulse_Amas_Amenos(10);
    
    DRV8814_sleep();
    
    DRV8814_power_off();
}
//------------------------------------------------------------------------------
void consigna_set_diurna(void)
{

    // Proporciono corriente.
	DRV8814_power_on();
	// Espero 15s que se carguen los condensasores
	vTaskDelay( ( TickType_t)( 25000 / portTICK_RATE_MS ) );
    
    DRV8814_awake();
    // Close VA,VB
    DRV8814_pulse_Amenos_Amas(10);
    DRV8814_pulse_Bmenos_Bmas(10);
    
    vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
    
    // Open VB
    DRV8814_pulse_Bmas_Bmenos(10);
    
    DRV8814_sleep();
    
    DRV8814_power_off();
    
}
//------------------------------------------------------------------------------
void consigna_config_defaults(void)
{
    consigna_conf.enabled = false;
    consigna_conf.consigna_diurna = 700;
    consigna_conf.consigna_nocturna = 2300;
}
//------------------------------------------------------------------------------
bool consigna_config( char *s_enable, char *s_cdiurna, char *s_cnocturna )
{
    
    //xprintf_P(PSTR("CONSIGNA DEBUG: %s,%s,%s\r\n"),s_enable,s_cdiurna,s_cnocturna);
    
    if (!strcmp_P( strupr(s_enable), PSTR("TRUE"))  ) {
        consigna_conf.enabled = true;
    }
    
    if (!strcmp_P( strupr(s_enable), PSTR("FALSE"))  ) {
        consigna_conf.enabled = false;
    }
    
    consigna_conf.consigna_diurna = atoi(s_cdiurna);
    consigna_conf.consigna_nocturna = atoi(s_cnocturna);
    
    return (true);
    
}
//------------------------------------------------------------------------------
void consigna_print_configuration(void)
{
    xprintf_P( PSTR("Consigna:\r\n"));
    if (  ! consigna_conf.enabled ) {
        xprintf_P( PSTR(" status=disabled\r\n"));
        return;
    }
    
    xprintf_P( PSTR(" status=enabled\r\n"));
	xprintf_P( PSTR(" cDiurna=%04d, cNocturna=%04d\r\n"), consigna_conf.consigna_diurna, consigna_conf.consigna_nocturna  );

}
//------------------------------------------------------------------------------
uint8_t consigna_hash(uint8_t f_hash(uint8_t seed, char ch )  )
{
    
uint8_t hash = 0;
char *p;
uint8_t hash_buffer[64];

   // Calculo el hash de la configuracion modbus

    memset(hash_buffer, '\0', sizeof(hash_buffer) );
    if ( consigna_conf.enabled ) {
        sprintf_P( (char *)&hash_buffer, PSTR("[TRUE,%04d,%04d]"),consigna_conf.consigna_diurna, consigna_conf.consigna_nocturna);
    } else {
        sprintf_P( (char *)&hash_buffer, PSTR("[FALSE,%04d,%04d]"),consigna_conf.consigna_diurna, consigna_conf.consigna_nocturna);
    }
    p = (char *)hash_buffer;
    while (*p != '\0') {
        hash = f_hash(hash, *p++);
    }
    //xprintf_P(PSTR("HASH_PILOTO:%s, hash=%d\r\n"), hash_buffer, hash ); 
    return(hash);   
}
//------------------------------------------------------------------------------
void consigna_update_local_config( consigna_conf_t *consigna_system_conf)
{
        
    memcpy( &consigna_conf, consigna_system_conf, sizeof(consigna_conf_t));

}
// -----------------------------------------------------------------------------
void consigna_read_local_config( consigna_conf_t *consigna_system_conf)
{
    
    memcpy( consigna_system_conf, &consigna_conf, sizeof(consigna_conf_t)); 

}
//------------------------------------------------------------------------------
void consigna_service(void)
{
 
RtcTimeType_t rtcDateTime;
uint16_t now;

	// Chequeo y aplico.
	// Las consignas se chequean y/o setean en cualquier modo de trabajo, continuo o discreto
	memset( &rtcDateTime, '\0', sizeof(RtcTimeType_t));
	if ( ! RTC_read_dtime(&rtcDateTime) ) {
		xprintf_P(PSTR("CONSIGNA ERROR: I2C:RTC chequear_consignas\r\n\0"));
		return;
	}

    now = rtcDateTime.hour * 100 + rtcDateTime.min;
    
	// Consigna diurna ?
	if ( now == consigna_conf.consigna_diurna  ) {
		consigna_set_diurna();
		xprintf_P(PSTR("Set CONSIGNA diurna %04d\r\n"), now );
		return;
	}

	// Consigna nocturna ?
	if ( now == consigna_conf.consigna_nocturna  ) {
		consigna_set_nocturna();
		xprintf_P(PSTR("Set CONSIGNA nocturna %04d\r\n"),now);
		return;
	}
    
}
//------------------------------------------------------------------------------
