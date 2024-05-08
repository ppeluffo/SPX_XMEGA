#include "ainputs.h"

// Factor por el que hay que multitplicar el valor raw de los INA para tener
// una corriente con una resistencia de 7.32 ohms.
// Surge que 1LSB corresponde a 40uV y que la resistencia que ponemos es de 7.32 ohms
// 1000 / 7.32 / 40 = 183 ;
#define INA_FACTOR  183

static int8_t sensores_prendidos = 0;

static bool f_debug_ainputs;

// Los caudales los almaceno en un RB y lo que doy es el promedio !!
typedef struct {
    float ain;
} t_ain_s;

#define MAX_RB_AIN_STORAGE_SIZE  10
t_ain_s ain_storage_0[MAX_RB_AIN_STORAGE_SIZE];
t_ain_s ain_storage_1[MAX_RB_AIN_STORAGE_SIZE];
t_ain_s ain_storage_2[MAX_RB_AIN_STORAGE_SIZE];
rBstruct_s ain_RB_0,ain_RB_1,ain_RB_2;

static uint8_t max_rb_ain_storage_size;

// Configuracion local del sistema analogico
ainputs_conf_t ainputs_conf;

static SemaphoreHandle_t ainputsLocalSem;

// Semafor para acceder a medir los canales ( ainputs y pilotos )
SemaphoreHandle_t sem_AINPUTS;
StaticSemaphore_t AINPUTS_xMutexBuffer;

//------------------------------------------------------------------------------
void ainputs_init_outofrtos( SemaphoreHandle_t semph)
{
    /*
     * Asignamos el semaforo para acceder a la configuracion y
     * creo el semaforo local para acceder a medida de los canales.
     */
    
    ainputsLocalSem = semph;
    
    sem_AINPUTS = xSemaphoreCreateMutexStatic( &AINPUTS_xMutexBuffer );
}
//------------------------------------------------------------------------------
void ainputs_update_local_config( ainputs_conf_t *ainputs_system_conf)
{
    /*
     * Copia la configuracion del systemConf en la local
     */
    while ( xSemaphoreTake( ainputsLocalSem, ( TickType_t ) 5 ) != pdTRUE )
  		vTaskDelay( ( TickType_t)( 1 ) );
    memcpy( &ainputs_conf, ainputs_system_conf , sizeof(ainputs_conf_t));
    xSemaphoreGive( ainputsLocalSem );
}
//------------------------------------------------------------------------------
void ainputs_read_local_config( ainputs_conf_t *ainputs_system_conf)
{
    /*
     * Copia la configuracion local en el systemConf
     */
    while ( xSemaphoreTake( ainputsLocalSem, ( TickType_t ) 5 ) != pdTRUE )
  		vTaskDelay( ( TickType_t)( 1 ) );
    memcpy( ainputs_system_conf, &ainputs_conf, sizeof(ainputs_conf_t));
    xSemaphoreGive( ainputsLocalSem );
}
//------------------------------------------------------------------------------    
void ainputs_init(uint8_t samples_count)
{
    /*
     * Inicializa el INA.
     * Como usa el FRTOS, debe correrse luego que este este corriendo.!!!
     */
    
    INA_config(INA_A, CONF_INA_AVG128);
    INA_config(INA_B, CONF_INA_AVG128);
    INA_config(INA_A, CONF_INA_SLEEP);
    INA_config(INA_B, CONF_INA_SLEEP);
    
    max_rb_ain_storage_size = samples_count;
    
    if ( max_rb_ain_storage_size > MAX_RB_AIN_STORAGE_SIZE ) {
        max_rb_ain_storage_size = MAX_RB_AIN_STORAGE_SIZE;
    }
    
    rBstruct_CreateStatic ( 
        &ain_RB_0, 
        &ain_storage_0, 
        max_rb_ain_storage_size, 
        sizeof(t_ain_s), 
        true  
    );
    
    rBstruct_CreateStatic ( 
        &ain_RB_1, 
        &ain_storage_1, 
        max_rb_ain_storage_size, 
        sizeof(t_ain_s), 
        true  
    );
        
    rBstruct_CreateStatic ( 
        &ain_RB_2, 
        &ain_storage_2, 
        max_rb_ain_storage_size, 
        sizeof(t_ain_s), 
        true  
    );
}
//------------------------------------------------------------------------------
void ainputs_awake(void)
{
    /*
     * Saca al INA del estado de reposo LOW-POWER
     */
    INA_awake();
}
//------------------------------------------------------------------------------
void ainputs_sleep(void)
{
    /*
     * Pone al INA en estado LOW-POWER
     */
    INA_sleep();
}
//------------------------------------------------------------------------------
bool ainputs_config_channel( uint8_t ch, char *s_enable, char *s_aname,char *s_imin,char *s_imax,char *s_mmin,char *s_mmax,char *s_offset )
{

	/*
     * Configura los canales analogicos. Es usada tanto desde el modo comando como desde el modo online por gprs.
     */ 


bool retS = false;

	if ( s_aname == NULL ) {
		return(retS);
	}

	if ( ( ch >=  0) && ( ch < NRO_ANALOG_CHANNELS ) ) {

        // Enable ?
        if (!strcmp_P( strupr(s_enable), PSTR("TRUE"))  ) {
            ainputs_conf.channel[ch].enabled = true;
        } else if (!strcmp_P( strupr(s_enable), PSTR("FALSE"))  ) {
            ainputs_conf.channel[ch].enabled = false;
        }
        
        snprintf_P( ainputs_conf.channel[ch].name, AIN_PARAMNAME_LENGTH, PSTR("%s"), s_aname );

		if ( s_imin != NULL ) {
			ainputs_conf.channel[ch].imin = atoi(s_imin);
		}

		if ( s_imax != NULL ) {
			ainputs_conf.channel[ch].imax = atoi(s_imax);
		}

		if ( s_offset != NULL ) {
			ainputs_conf.channel[ch].offset = atof(s_offset);
		}

		if ( s_mmin != NULL ) {
			ainputs_conf.channel[ch].mmin = atof(s_mmin);
		}

		if ( s_mmax != NULL ) {
			ainputs_conf.channel[ch].mmax = atof(s_mmax);
		}

		retS = true;
	}

	return(retS);
}
//------------------------------------------------------------------------------
void ainputs_config_defaults(void)
{
    	/*
         * Realiza la configuracion por defecto de los canales digitales.
         */

uint8_t i = 0;

	for ( i = 0; i < NRO_ANALOG_CHANNELS; i++) {
        ainputs_conf.channel[i].enabled = false;
		ainputs_conf.channel[i].imin = 0;
		ainputs_conf.channel[i].imax = 20;
		ainputs_conf.channel[i].mmin = 0.0;
		ainputs_conf.channel[i].mmax = 10.0;
		ainputs_conf.channel[i].offset = 0.0;
		snprintf_P( ainputs_conf.channel[i].name, AIN_PARAMNAME_LENGTH, PSTR("X") );
	}

}
//------------------------------------------------------------------------------
void ainputs_print_configuration(void)
{
    /*
     * Muestra la configuracion de todos los canales analogicos en la terminal
     * La usa el comando tkCmd::status.
     */
    
uint8_t i = 0;

    xprintf_P(PSTR("Ainputs:\r\n"));
    xprintf_P(PSTR(" debug: "));
    f_debug_ainputs ? xprintf_P(PSTR("true\r\n")) : xprintf_P(PSTR("false\r\n"));

	for ( i = 0; i < NRO_ANALOG_CHANNELS; i++) {
        
        if ( ainputs_conf.channel[i].enabled ) {
            xprintf_P( PSTR(" a%d: +"),i);
        } else {
            xprintf_P( PSTR(" a%d: -"),i);
        }
        
        xprintf_P( PSTR("[%s, %d-%d mA/ %.02f,%.02f | %.03f]\r\n"),
            ainputs_conf.channel[i].name,
            ainputs_conf.channel[i].imin,
            ainputs_conf.channel[i].imax,
            ainputs_conf.channel[i].mmin,
            ainputs_conf.channel[i].mmax,
            ainputs_conf.channel[i].offset
            );
    }
}
//------------------------------------------------------------------------------
uint16_t ainputs_read_channel_raw(uint8_t ch )
{

        /*
         * Lee el valor del INA del canal analogico dado
         */
    
uint8_t dev_id = 0;
uint8_t ina_reg = 0;
uint16_t an_raw_val = 0;
uint8_t MSB = 0;
uint8_t LSB = 0;
char res[3] = { '\0','\0', '\0' };
int8_t xBytes = 0;
//float vshunt;

	switch ( ch ) {
	case 0:
        dev_id = INA_A;
		ina_reg = INA3221_CH1_SHV;
		break;
	case 1:
        dev_id = INA_A;
		ina_reg = INA3221_CH2_SHV;
		break;
	case 2:
        dev_id = INA_A;
		ina_reg = INA3221_CH3_SHV;
		break;
    case 99:
         // Battery ??
        dev_id = INA_B;
        ina_reg = INA3221_CH1_BUSV;
        break;
	default:
		return(-1);
		break;
	}

	// Leo el valor del INA.
	xBytes = INA_read( dev_id, ina_reg, res ,2 );

	if ( xBytes == -1 )
		xprintf_P(PSTR("ERROR I2C: ainputs_read_channel_raw.\r\n\0"));

	an_raw_val = 0;
	MSB = res[0];
	LSB = res[1];
	an_raw_val = ( MSB << 8 ) + LSB;
	an_raw_val = an_raw_val >> 3;

    if ( f_debug_ainputs ) {
        xprintf_P( PSTR("INA: ch=%d, reg=%d, MSB=0x%x, LSB=0x%x, ANV=(0x%x)%d\r\n") ,ch, ina_reg, MSB, LSB, an_raw_val, an_raw_val );
    }
    
//	vshunt = (float) an_raw_val * 40 / 1000;
//	xprintf_P( PSTR("out->ACH: ch=%d, ina=%d, reg=%d, MSB=0x%x, LSB=0x%x, ANV=(0x%x)%d, VSHUNT = %.02f(mV)\r\n\0") ,channel_id, ina_id, ina_reg, MSB, LSB, an_raw_val, an_raw_val, vshunt );

	return( an_raw_val );
}
//------------------------------------------------------------------------------
float ainputs_read_channel_mag(uint8_t ch, uint16_t an_raw_val)
{
    /*
     * Convierte el valor raw a la magnitud.
     */
    
float an_mag_val = 0.0;
float I = 0.0;
float M = 0.0;
float P = 0.0;
uint16_t D = 0;

    // Battery ??
    if ( ch == 99 ) {
        // Convierto el raw_value a la magnitud ( 8mV por count del A/D)
        an_mag_val =  0.008 * an_raw_val;
        if ( f_debug_ainputs ) {
            xprintf_P(PSTR("ANALOG: A%d (RAW=%d), BATT=%.03f\r\n\0"), ch, an_raw_val, an_mag_val );
        }
        goto quit;
    }
    
    
	// Convierto el raw_value a corriente
	I = (float) an_raw_val / INA_FACTOR;
    
	// Calculo la magnitud
	P = 0;
	D = ainputs_conf.channel[ch].imax - ainputs_conf.channel[ch].imin;
	an_mag_val = 0.0;
	if ( D != 0 ) {
		// Pendiente
		P = (float) ( ainputs_conf.channel[ch].mmax  -  ainputs_conf.channel[ch].mmin ) / D;
		// Magnitud
		M = (float) ( ainputs_conf.channel[ch].mmin + ( I - ainputs_conf.channel[ch].imin ) * P);

		// Al calcular la magnitud, al final le sumo el offset.
		an_mag_val = M + ainputs_conf.channel[ch].offset;
		// Corrijo el 0 porque sino al imprimirlo con 2 digitos puede dar negativo
		if ( fabs(an_mag_val) < 0.01 )
			an_mag_val = 0.0;

	} else {
		// Error: denominador = 0.
		an_mag_val = -999.0;
	}

    if ( f_debug_ainputs ) {
        xprintf_P(PSTR("ANALOG: A%d (RAW=%d), I=%.03f\r\n\0"), ch, an_raw_val, I );
        xprintf_P(PSTR("ANALOG: Imin=%d, Imax=%d\r\n\0"), ainputs_conf.channel[ch].imin, ainputs_conf.channel[ch].imax );
        xprintf_P(PSTR("ANALOG: mmin=%.03f, mmax=%.03f\r\n\0"), ainputs_conf.channel[ch].mmin, ainputs_conf.channel[ch].mmax );
        xprintf_P(PSTR("ANALOG: D=%d, P=%.03f, M=%.03f\r\n\0"), D, P, M );
        xprintf_P(PSTR("ANALOG: an_raw_val=%d, an_mag_val=%.03f\r\n\0"), an_raw_val, an_mag_val );
    }

quit:
        
	return(an_mag_val);
}
//------------------------------------------------------------------------------
void ainputs_read_channel ( uint8_t ch, float *mag, uint16_t *raw )
{
	/*
	Lee un canal analogico y devuelve el valor convertido a la magnitud configurada.
	Es publico porque se utiliza tanto desde el modo comando como desde el modulo de poleo de las entradas.
	Hay que corregir la correspondencia entre el canal leido del INA y el canal fisico del datalogger
	io_channel. Esto lo hacemos en AINPUTS_read_ina.

	la funcion read_channel_raw me devuelve el valor raw del conversor A/D.
	Este corresponde a 40uV por bit por lo tanto multiplico el valor raw por 40/1000 y obtengo el valor en mV.
	Como la resistencia es de 7.32, al dividirla en 7.32 tengo la corriente medida.
	Para pasar del valor raw a la corriente debo hacer:
	- Pasar de raw a voltaje: V = raw * 40 / 1000 ( en mV)
	- Pasar a corriente: I = V / 7.32 ( en mA)
	- En un solo paso haria: I = raw / 3660
	  3660 = 40 / 1000 / 7.32.
	  Este valor 3660 lo llamamos INASPAN y es el valor por el que debo multiplicar el valor raw para que con una
	  resistencia shunt de 7.32 tenga el valor de la corriente medida. !!!!
	*/


uint16_t an_raw_val = 0;
float an_mag_val = 0.0;
//t_ain_s rb_element;
//float avg;
//uint8_t i;


    AINPUTS_ENTER_CRITICAL();
    
	// Leo el valor del INA.(raw)
	an_raw_val = ainputs_read_channel_raw( ch );
    // Lo convierto a la magnitud
    an_mag_val = ainputs_read_channel_mag( ch, an_raw_val);
    
    /*
    // Lo almaceno en el RB correspondiente
    rb_element.ain = an_mag_val;
    switch(ch) {
        case 0:
            rBstruct_Poke(&ain_RB_0, &rb_element);
            break;
        case 1:
            rBstruct_Poke(&ain_RB_1, &rb_element);
            break;
        case 2:
            rBstruct_Poke(&ain_RB_2, &rb_element);
            break;
        case 99:
            rBstruct_Poke(&ain_RB_2, &rb_element);
            break;
            
    }
    // Promedio los resultados del RB y devuelvo este dato
    avg=0.0;
    for(i=0; i < max_rb_ain_storage_size; i++) {
        switch(ch) {
            case 0:
                rb_element = ain_storage_0[i];    
                break;
            case 1:
                rb_element = ain_storage_1[i];    
                break;
            case 2:
                rb_element = ain_storage_2[i];    
                break;     
        }
        avg += rb_element.ain;
        if ( f_debug_ainputs ) {
            xprintf_P(PSTR("DEBUG AIN%02d: %d,value=%0.3f,avg=%0.3f\r\n"), ch, i, rb_element.ain, avg );
        }
    }
    avg /= max_rb_ain_storage_size;
    
    */ 
    
	*raw = an_raw_val;
	*mag = an_mag_val;
    //*mag = avg;
    
    AINPUTS_EXIT_CRITICAL();

}
//------------------------------------------------------------------------------
void ainputs_prender_sensores(void)
{
    /* 
     * Para ahorrar energia los canales se prenden cuando se necesitan y luego
     * se apagan
     * El contador sensores_prendidos ( protegido con semaforo ) nos dice si
     * esta prendido o no.
     */

uint32_t sleep_time_ms;
int8_t sp;

	// Lo prendo virtualmente ( solo si estaba en 0 (apagado) y paso a 1 )
    AINPUTS_ENTER_CRITICAL();
    sensores_prendidos++;
    sp = sensores_prendidos;
    
    if ( f_debug_ainputs ) {
        xprintf_P( PSTR("AINPUTS Prender Sensores: count=%d\r\n") , sp );
    }
    
    if ( sp == 1 ) {
        // Prendo físicamente
        ainputs_awake();
		SET_VSENSORS420();
	}
    
    AINPUTS_EXIT_CRITICAL();
    
    // Normalmente espero 1s de settle time que esta bien para los sensores
	// pero cuando hay un caudalimetro de corriente, necesita casi 5s
	// vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
    sleep_time_ms = (uint32_t)PWRSENSORES_SETTLETIME_MS / portTICK_PERIOD_MS; 
    vTaskDelay( ( TickType_t)sleep_time_ms );
}
//------------------------------------------------------------------------------
void ainputs_apagar_sensores(void)
{

int8_t sp;

    AINPUTS_ENTER_CRITICAL();
    if ( sensores_prendidos > 0 ) {
        sensores_prendidos--;
    }
    
    sp = sensores_prendidos;
    
    if ( f_debug_ainputs ) {
        xprintf_P( PSTR("AINPUTS Apagar Sensores: count=%d\r\n") , sp );
    }
    
    // Solo cuando nadie lo esta usando, lo apago físicamente.
    if ( sp == 0 ) {
        CLEAR_VSENSORS420();
        ainputs_sleep();
    }
    AINPUTS_EXIT_CRITICAL();
}
//------------------------------------------------------------------------------
bool ainputs_test_read_channel( uint8_t ch )
{
  
float mag;
uint16_t raw;

    if ( ( ch == 0 ) || (ch == 1 ) || ( ch == 2) || ( ch == 99)) {
        
        ainputs_prender_sensores();
        ainputs_read_channel ( ch, &mag, &raw );
        xprintf_P(PSTR("AINPUT ch%d=%0.3f\r\n"), ch, mag);
        ainputs_apagar_sensores();
        return(true);
    } else {
        return(false);
    }

}
//------------------------------------------------------------------------------
void ainputs_config_debug(bool debug )
{
    if ( debug ) {
        f_debug_ainputs = true;
    } else {
        f_debug_ainputs = false;
    }
    
}
//------------------------------------------------------------------------------
bool ainputs_read_debug(void)
{
    return (f_debug_ainputs);
}
//------------------------------------------------------------------------------
uint8_t ainputs_hash( void )
{
    
uint8_t hash_buffer[64];
uint8_t i,j;
uint8_t hash = 0;
char *p;

    // Calculo el hash de la configuracion de las ainputs
    for(i=0; i<NRO_ANALOG_CHANNELS; i++) {
        memset(hash_buffer, '\0', sizeof(hash_buffer));
        j = 0;
        if ( ainputs_conf.channel[i].enabled ) {
            j += sprintf_P( (char *)&hash_buffer[j], PSTR("[A%d:TRUE,"), i );
        } else {
            j += sprintf_P( (char *)&hash_buffer[j], PSTR("[A%d:FALSE,"), i );
        }
        j += sprintf_P( (char *)&hash_buffer[j], PSTR("%s,"), ainputs_conf.channel[i].name );
        j += sprintf_P( (char *)&hash_buffer[j], PSTR("%d,%d,"), ainputs_conf.channel[i].imin, ainputs_conf.channel[i].imax );
        j += sprintf_P( (char *)&hash_buffer[j], PSTR("%.02f,%.02f,"), ainputs_conf.channel[i].mmin, ainputs_conf.channel[i].mmax );
        j += sprintf_P( (char *)&hash_buffer[j], PSTR("%.02f]"), ainputs_conf.channel[i].offset);    
        p = (char *)hash_buffer;
        while (*p != '\0') {
            hash = u_hash(hash, *p++);
        }
        // xprintf_P(PSTR("HASH_AIN:<%s>, hash=%d\r\n"), hash_buffer, hash );
    }
    return(hash);
    
}
//------------------------------------------------------------------------------
void AINPUTS_ENTER_CRITICAL(void)
{
    while ( xSemaphoreTake( sem_AINPUTS, ( TickType_t ) 5 ) != pdTRUE )
  		vTaskDelay( ( TickType_t)( 10 ) );   
}
//------------------------------------------------------------------------------
void AINPUTS_EXIT_CRITICAL(void)
{
    xSemaphoreGive( sem_AINPUTS );
}
//------------------------------------------------------------------------------
