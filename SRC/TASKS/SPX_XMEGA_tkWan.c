#include "SPX_XMEGA.h"
#include "frtos_cmd.h"
        
SemaphoreHandle_t sem_WAN;
StaticSemaphore_t WAN_xMutexBuffer;
#define MSTOTAKEWANSEMPH ((  TickType_t ) 10 )

#define WAN_TX_BUFFER_SIZE 255
uint8_t wan_tx_buffer[WAN_TX_BUFFER_SIZE];

#define HASH_BUFFER_SIZE 64
uint8_t hash_buffer[HASH_BUFFER_SIZE];

static bool f_debug_comms;

typedef enum { WAN_APAGADO=0, WAN_OFFLINE, WAN_ONLINE_CONFIG, WAN_ONLINE_DATA } wan_states_t;

typedef enum { DATA=0, BLOCK, BLOCKEND } tx_type_t;

typedef enum { CONTINUO, DISCRETO, MIXTO_ON, MIXTO_OFF } pwr_modo_now_t;

static uint8_t wan_state;

struct {
    dataRcd_s dr;
    bool dr_ready;
} drWanBuffer;

bool link_up4data;

static void wan_state_apagado(void);
static void wan_state_offline(void);
static void wan_state_online_config(void);
static void wan_state_online_data(void);

static bool wan_process_frame_linkup(void);
static bool wan_process_frame_recoverId(void);
static bool wan_process_rsp_recoverId(void);
static bool wan_process_frame_configBase(void);
static bool wan_process_rsp_configBase(void);
static bool wan_process_frame_configAinputs(void);
static bool wan_process_rsp_configAinputs(void);
static bool wan_process_frame_configCounters(void);
static bool wan_process_rsp_configCounters(void);
static bool wan_process_frame_configPiloto(void);
static bool wan_process_rsp_configPiloto(void);
static bool wan_process_frame_configConsigna(void);
static bool wan_process_rsp_configConsigna(void);

static bool wan_send_from_memory(void);
static bool wan_process_frame_data(dataRcd_s *dr);
static void wan_load_dr_in_txbuffer(dataRcd_s *dr, uint8_t *buff, uint16_t buffer_size );

static bool wan_process_rsp_data(void);
static pwr_modo_now_t wan_check_pwr_modo_now(void);

bool wan_process_from_dump(char *buff, bool ultimo );

void wan_PRENDER_MODEM(void);
void wan_APAGAR_MODEM(void);

#define DEBUG_WAN       true
#define NO_DEBUG_WAN    false

#define SLEEP_TIME_MIXTO_OFF    1800

bool f_inicio;

bool wan_sleeping;

static bool wan_check_response ( const char *s);
static void wan_xmit_out(bool debug_flag );
static void wan_print_RXbuffer(void);

fat_s l_fat1;

//------------------------------------------------------------------------------
void tkWAN(void * pvParameters)
{

	// Esta es la primer tarea que arranca.

( void ) pvParameters;

    while ( ! run_tasks )
        vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );

    sem_WAN = xSemaphoreCreateMutexStatic( &WAN_xMutexBuffer );
    lBchar_CreateStatic ( &wan_lbuffer, wan_buffer, WAN_RX_BUFFER_SIZE );
    
    xprintf_P(PSTR("Starting tkWAN..\r\n" ));
    vTaskDelay( ( TickType_t)( 500 / portTICK_PERIOD_MS ) );
    
    wan_state = WAN_APAGADO;
    f_inicio = true;
    link_up4data = false;
    wan_sleeping = false;
    
	// loop
	for( ;; )
	{
            
        KICK_WDG(WAN_WDG_bp);
        
        switch(wan_state) {
            case WAN_APAGADO:
                wan_state_apagado();
                break;
            case WAN_OFFLINE:
                wan_state_offline();
                break;
            case WAN_ONLINE_CONFIG:
                wan_state_online_config();
                break;
            case WAN_ONLINE_DATA:
                wan_state_online_data();
                break;
            default:
                xprintf_P(PSTR("ERROR: WAN STATE ??\r\n"));
                wan_state = WAN_APAGADO;
                break;
        }
        vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
	}    
}
//------------------------------------------------------------------------------
static void wan_state_apagado(void)
{
    /*
     * Implemento la FSM que espera con el modem apagado.
     * Si estoy en modo continuo o modo mixto ( en horario continuo), salgo.
     * En otro caso estoy en modo discreto ( o mixto en horario discreto). En 
     * este caso espero TimerDial.
     */
    
int16_t sleep_ticks;
uint32_t ulNotifiedValue;
BaseType_t xResult;
pwr_modo_now_t pwr_modo_now;


    xprintf_P(PSTR("WAN:: State APAGADO\r\n"));   
    link_up4data = false;  
    wan_APAGAR_MODEM();
    vTaskDelay( ( TickType_t)( 5000 / portTICK_PERIOD_MS ) );
    
    // Cuando inicio me conecto siempre sin esperar para configurarme y vaciar la memoria.
    if ( f_inicio ) {
        f_inicio = false;
        //xprintf_P(PSTR("DEBUG Finicio\r\n"));
        goto exit;
    }
    
    pwr_modo_now = wan_check_pwr_modo_now();
    switch(pwr_modo_now) {

        case CONTINUO:
            xprintf_P(PSTR("WAN:: DEBUG: pwrmodo continuo\r\n"));
            vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
            goto exit;
            break;
            
        case DISCRETO:
            xprintf_P(PSTR("WAN:: DEBUG: pwrmodo discreto\r\n"));
            sleep_ticks = systemConf.timerdial;
            wan_sleeping = true;
            xprintf_P(PSTR("tkWan going to sleep %d secs..\r\n" ), systemConf.timerdial);
            
            while (1) {
                // Espero de a 1 min para poder entrar en tickless.
                // Duermo monitoreando las señales.
                KICK_WDG(WAN_WDG_bp);
                xResult = xTaskNotifyWait( 0x00, ULONG_MAX, &ulNotifiedValue, ( TickType_t)( 60000 / portTICK_PERIOD_MS ) );
                if ( xResult == pdTRUE ) {
                    if ( ( ulNotifiedValue & DATA_FRAME_READY ) != 0 ) {
                        //xprintf_P(PSTR("DEBUG: WAN SIGNAL RCVD\r\n"));
                        goto exit;
                    }
                }
                sleep_ticks -= 60;
                if ( sleep_ticks <= 0) {
                    goto exit;
                }
                //
            }    
            break;
            
        case MIXTO_ON:
            xprintf_P(PSTR("WAN:: DEBUG pwrmodo mixto,ON.\r\n"));
            vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
            goto exit;
            break;
            
        case MIXTO_OFF: 
            // Debo dormir en intervalos de 30 mins.
            sleep_ticks = SLEEP_TIME_MIXTO_OFF;
            wan_sleeping = true;
            xprintf_P(PSTR("WAN:: DEBUG: pwrmodo mixto,OFF.\r\n"));
            xprintf_P(PSTR("tkWan going to sleep..%d secs\r\n" ), sleep_ticks );
            
            while (1) {
                // Espero de a 1 min para poder entrar en tickless.
                // Duermo monitoreando las señales.
                KICK_WDG(WAN_WDG_bp);
                xResult = xTaskNotifyWait( 0x00, ULONG_MAX, &ulNotifiedValue, ( TickType_t)( 60000 / portTICK_PERIOD_MS ) );
                if ( xResult == pdTRUE ) {
                    if ( ( ulNotifiedValue & DATA_FRAME_READY ) != 0 ) {
                        //xprintf_P(PSTR("DEBUG: WAN SIGNAL RCVD\r\n"));
                        goto exit;
                    }
                }
                sleep_ticks -= 60;
                if ( sleep_ticks <= 0) {
                    goto exit;
                }
                //
            }    

            break;
            
        default:
            xprintf_P(PSTR("WAN:: DEBUG: pwrmodo NO RECONOCIDO: ERROR !!!\r\n"));
            break;
    }
     
exit:
    
    KICK_WDG(WAN_WDG_bp);
    wan_sleeping = false;
    wan_PRENDER_MODEM();
    wan_state = WAN_OFFLINE;
    return;
  
}
//------------------------------------------------------------------------------
static void wan_state_offline(void)
{
    /*
     * Espera que exista link.
     * Para esto envia cada 10s un PING hacia el servidor.
     * Cuando recibe respuesta (PONG), pasa a la fase de enviar
     * los frames de re-configuracion.
     * Las respuestas del server las maneja el callback de respuestas.
     * Este prende flags para indicar al resto que tipo de respuesta llego.
     */
 
    xprintf_P(PSTR("WAN:: State OFFLINE\r\n"));
    
    if ( ! wan_process_frame_linkup() ) {
        wan_state = WAN_APAGADO;
        goto quit;
    }
    
    wan_state = WAN_ONLINE_CONFIG;
    
quit:
                
    return;
}
//------------------------------------------------------------------------------
static void wan_state_online_config(void)
{
    /*
     * Se encarga de la configuracion.
     * Solo lo hace una sola vez por lo que tiene una flag estatica que
     * indica si ya se configuro.
     */
  
//uint16_t i;
    
    xprintf_P(PSTR("WAN:: State ONLINE_CONFIG\r\n"));
 
    if ( ! wan_process_frame_recoverId() ) {
        // Espero 10 minutos y reintento.
        // Apago la tarea del system para no llenar la memoria al pedo
        /*
        vTaskSuspend( xHandle_tkSys );
        xprintf_P(PSTR("WAN:: Espero 10 mins.\r\n"));
        for (i=0; i<10; i++) {
             vTaskDelay( ( TickType_t)( 60000 / portTICK_PERIOD_MS ) );
             kick_wdt(SYS_WDG_bp);
        }
        
        xprintf("Reset..\r\n");
        reset();
    
        //wan_state = WAN_OFFLINE;
        //goto quit;
         */
    }
    
    if ( ! wan_process_frame_configBase() ) {
        // No puedo configurarse o porque el servidor no responde
        // o porque da errores. Espero 1H
        xprintf_P(PSTR("WAN:: Errores en configuracion. Espero 30mins..!!\r\n"));
        systemConf.timerdial = 1800;
        systemConf.timerpoll = 1800;
        systemConf.pwr_modo = PWR_DISCRETO;
        wan_state = WAN_APAGADO;
        return;
    }
    
    wan_process_frame_configAinputs();
    wan_process_frame_configCounters();
    wan_process_frame_configPiloto();
    wan_process_frame_configConsigna();
    
    // Con todos los modulos configurados, los recargo
    ainputs_read_local_config(&systemConf.ainputs_conf);
    counters_read_local_config(&systemConf.counters_conf);
    piloto_read_local_config(&systemConf.piloto_conf);
    consigna_read_local_config(&systemConf.consigna_conf);

    save_config_in_NVM();    
    wan_state = WAN_ONLINE_DATA;
    
//quit:
                
    return;

}
//------------------------------------------------------------------------------
static void wan_state_online_data(void)
{
    /*
     * Este estado es cuando estamos en condiciones de enviar frames de datos
     * y procesar respuestas.
     * Al entrar, si hay datos en memoria los transmito todos.
     * Luego, si estoy en modo continuo, no hago nada: solo espero.
     * Si estoy en modo discreto, salgo
     */

pwr_modo_now_t pwr_modo_now;
uint32_t sleep_time_ms;
bool res;

    xprintf_P(PSTR("WAN:: State ONLINE_DATA\r\n"));
    
    link_up4data = true;
   
// ENTRY:
    
    // Si hay datos en memoria los transmito todos y los borro en bloques de a 8
    xprintf_P(PSTR("WAN:: ONLINE: dump memory...\r\n"));  
    // Vacio la memoria.
    wan_send_from_memory();  
    //   
    pwr_modo_now = wan_check_pwr_modo_now();
 
    if ( (pwr_modo_now == DISCRETO) ||  (pwr_modo_now == MIXTO_OFF ) ) {
       wan_state = WAN_APAGADO;
       goto quit;
    }
    
    // En modo continuo me quedo esperando por datos para transmitir. 
    while( ( (pwr_modo_now == CONTINUO ) ||  (pwr_modo_now == MIXTO_ON ) ) ) {
         
        pwr_modo_now = wan_check_pwr_modo_now();
        //FAT_read(&l_fat1);
        //xprintf_P( PSTR("WAN D:: wrPtr=%d,rdPtr=%d,count=%d\r\n"), l_fat1.head, l_fat1.tail, l_fat1.count );
        
        // Hay datos para transmitir
        if ( drWanBuffer.dr_ready ) {
            res =  wan_process_frame_data( &drWanBuffer.dr);
            if (res) {
                drWanBuffer.dr_ready = false;
            } else {
                // Cayo el enlace ???
                wan_state = WAN_OFFLINE;
                goto quit;
            }
        }
        
        
        // Espero que hayan mas datos
         // Vuelvo a chequear el enlace cada 1 min( tickeless & wdg ).
        KICK_WDG(WAN_WDG_bp);
        sleep_time_ms = ( TickType_t)(  (60000) / portTICK_PERIOD_MS );
        ulTaskNotifyTake( pdTRUE, sleep_time_ms );;  
    }
    
    wan_state = WAN_APAGADO;
   
quit:
    
    link_up4data = false;
    return;        
}
//------------------------------------------------------------------------------
static pwr_modo_now_t wan_check_pwr_modo_now(void)
{
    
RtcTimeType_t rtc;
uint16_t now;
uint16_t pwr_on;
uint16_t pwr_off;


    switch(systemConf.pwr_modo) {
        
        case PWR_CONTINUO:
            return (CONTINUO);
            break;
            
        case PWR_DISCRETO:
            return (DISCRETO);
            break;
            
        case PWR_MIXTO:
            // Vemos si estoy en horario de continuo o de apagado
            RTC_read_dtime(&rtc);
            now = rtc.hour * 100 + rtc.min;
            pwr_on = systemConf.pwr_hhmm_on;
            pwr_off = systemConf.pwr_hhmm_off;
        
            // Caso 1:
            if ( pwr_on < pwr_off ) {
                if ( (pwr_on < now ) && ( now < pwr_off)) {
                    return (MIXTO_ON);
                } 
            }
        
            // Caso 2:
            if ( pwr_on > pwr_off ) {
                if ( ( now < pwr_off) || (now > pwr_on)) {
                    return(MIXTO_ON);
                } 
            }
            
            return(MIXTO_OFF);
            break;
    } 
}
//------------------------------------------------------------------------------
static bool wan_process_frame_linkup(void)
{
    /*
     * Envia los PING para determinar si el enlace está establecido
     * Intento durante 2 minutos mandando un ping cada 10s.
     */
    
uint16_t tryes;
uint16_t timeout;
bool retS = false;

    xprintf_P(PSTR("WAN:: LINK.\r\n"));
 
    // Armo el frame
    while ( xSemaphoreTake( sem_WAN, MSTOTAKEWANSEMPH ) != pdTRUE )
        vTaskDelay( ( TickType_t)( 1 ) );
    memset(wan_tx_buffer, '\0', WAN_TX_BUFFER_SIZE);
    sprintf_P( (char*)&wan_tx_buffer, PSTR("ID=%s&TYPE=%s&VER=%s&CLASS=PING"), systemConf.dlgid, FW_TYPE, FW_REV );
        
    // Proceso
    tryes = 6;
    while (tryes-- > 0) {
        wan_xmit_out(DEBUG_WAN);
    
        // Espero respuesta chequeando cada 1s durante 15s.
        timeout = 15;
        while ( timeout-- > 0) {
            vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
            if ( wan_check_response("CLASS=PONG")) {        
                wan_print_RXbuffer();
                xprintf_P(PSTR("WAN:: Link up.\r\n"));
                retS = true;
                goto exit_;
            }
        }
    }
    
    // Expiro el tiempo sin respuesta del server.
    xprintf_P(PSTR("WAN:: Link up timeout !!\r\n"));
    retS = false;
    
exit_:
               
    xSemaphoreGive( sem_WAN );
    return(retS);
 
}
//------------------------------------------------------------------------------
static bool wan_process_frame_recoverId(void)
{
    /*
     * Este proceso es para recuperar el ID cuando localmente está en DEFAULT.
     * Intento 2 veces mandar el frame.
     */
    
uint8_t tryes = 0;
uint8_t timeout = 0;
uint8_t fptr;
bool retS = false;

    xprintf_P(PSTR("WAN:: RECOVERID.\r\n"));

    // Si el nombre es diferente de DEFAULT no tengo que hacer nada
    if ( strcmp_P( strupr( systemConf.dlgid), PSTR("DEFAULT")) != 0 ) {
        return(true);
    }

    // Armo el buffer
    while ( xSemaphoreTake( sem_WAN, MSTOTAKEWANSEMPH ) != pdTRUE )
        vTaskDelay( ( TickType_t)( 1 ) );
    memset(wan_tx_buffer, '\0', WAN_TX_BUFFER_SIZE);
    fptr = 0;
    fptr = sprintf_P( (char*)&wan_tx_buffer[fptr], PSTR("ID=%s&TYPE=%s&VER=%s&CLASS=RECOVER&UID=%s"), systemConf.dlgid, FW_TYPE, FW_REV, NVMEE_read_serial());

    // Proceso. Envio hasta 2 veces el frame y espero hasta 10s la respuesta
    tryes = 2;
    while (tryes-- > 0) {
        
        wan_xmit_out(DEBUG_WAN);
    
        // Espero respuesta chequeando cada 1s durante 10s.
        timeout = 10;
        while ( timeout-- > 0) {
            vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
            if ( wan_check_response("CLASS=RECOVER")) {
                wan_print_RXbuffer();
                wan_process_rsp_recoverId();
                retS = true;
                goto exit_;
            } else if ( wan_check_response("CONFIG=ERROR")) {
                wan_print_RXbuffer();
                xprintf_P(PSTR("WAN:: RECOVERID ERROR: El servidor no reconoce al datalogger !!\r\n"));
                retS = false;
                goto exit_;
            }
        }
    }
 
    // Expiro el tiempo sin respuesta del server.
    xprintf_P(PSTR("WAN:: RECOVERID ERROR:Timeout en server rsp.!!\r\n"));
    retS = false;
    
exit_:
               
    xSemaphoreGive( sem_WAN );

    return(retS);
}
//------------------------------------------------------------------------------
static bool wan_process_rsp_recoverId(void)
{
    /*
     * Extraemos el DLGID del frame y lo reconfiguramos
     * RXFRAME: <html><body><h1>CLASS=RECOVER&ID=xxxx</h1></body></html>
     *          <html><body><h1>CLASS=RECOVER&ID=DEFAULT</h1></body></html>
     */
    
char localStr[32] = { 0 };
char *stringp = NULL;
char *token = NULL;
char *delim = "&,;:=><";
char *ts = NULL;
char *p;

    p = lBchar_get_buffer(&wan_lbuffer);
    
    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
	memset(localStr,'\0',sizeof(localStr));
	ts = strstr( p, "RECOVER");
	strncpy(localStr, ts, sizeof(localStr));
	stringp = localStr;
	token = strsep(&stringp,delim);	    // RECOVER
	token = strsep(&stringp,delim);	 	// ID
	token = strsep(&stringp,delim);	 	// TEST01
	// Copio el dlgid recibido al systemConf.
	memset(systemConf.dlgid,'\0', sizeof(systemConf.dlgid) );
	strncpy( systemConf.dlgid, token, DLGID_LENGTH);
	
    //save_config_in_NVM();
	xprintf_P( PSTR("WAN:: Reconfig DLGID to %s\r\n\0"), systemConf.dlgid );
	return(true);
}
//------------------------------------------------------------------------------
static bool wan_process_frame_configBase(void)
{
     /*
      * Envo un frame con el hash de la configuracion BASE.
      * El server me puede mandar OK o la nueva configuracion que debo tomar.
      * Lo reintento 2 veces
      */
    
uint8_t tryes = 0;
uint8_t timeout = 0;
bool retS = false;
uint8_t hash;

    xprintf_P(PSTR("WAN:: CONFIG_BASE.\r\n"));   
    
    // Armo el buffer
    while ( xSemaphoreTake( sem_WAN, MSTOTAKEWANSEMPH ) != pdTRUE )
        vTaskDelay( ( TickType_t)( 1 ) );
    memset(wan_tx_buffer, '\0', WAN_TX_BUFFER_SIZE);
    hash = confbase_hash();
    sprintf_P( (char*)&wan_tx_buffer, PSTR("ID=%s&TYPE=%s&VER=%s&CLASS=CONF_BASE&UID=%s&HASH=0x%02X"), systemConf.dlgid, FW_TYPE, FW_REV, NVMEE_read_serial(), hash );
    // Proceso. Envio hasta 2 veces el frame y espero hasta 10s la respuesta
    tryes = 2;
    while (tryes-- > 0) {
        
        wan_xmit_out(DEBUG_WAN);
    
        // Espero respuesta chequeando cada 1s durante 10s.
        timeout = 10;
        while ( timeout-- > 0) {
            vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
            if ( wan_check_response("CONFIG=ERROR")) {
                xprintf_P(PSTR("WAN:: CONF_BASE ERROR: El servidor no reconoce al datalogger !!\r\n"));
                retS = false;
                goto exit_;
            
            } else if ( wan_check_response("CONF_BASE&CONFIG=OK")) {
                wan_print_RXbuffer();
                retS = true;
                goto exit_;
            
            } else if ( wan_check_response("CLASS=CONF_BASE")) {
                wan_print_RXbuffer();
                wan_process_rsp_configBase();
                retS = true;
                goto exit_;
            } 
        }
    }
 
    // Expiro el tiempo sin respuesta del server.
    xprintf_P(PSTR("WAN:: CONFIG_BASE ERROR: Timeout en server rsp.!!\r\n"));
    retS = false;
    
exit_:
               
    xSemaphoreGive( sem_WAN );
    return(retS);   
}
//------------------------------------------------------------------------------
static bool wan_process_rsp_configBase(void)
{
    /*
     * Recibe la configuracion BASE.
     * RXFRAME: <html><body><h1>CLASS=CONF_BASE&TPOLL=30&TDIAL=900&PWRMODO=CONTINUO&PWRON=1800&PWROFF=1440&SAMPLES=1&ALMLEVEL=5</h1></body></html>                        
     *                          CLASS=CONF_BASE&CONFIG=OK
     *                          
     */
    
char localStr[32] = { 0 };
char *stringp = NULL;
char *token = NULL;
char *delim = "&,;:=><";
char *ts = NULL;
char *p;
bool retS = false;

    p = lBchar_get_buffer(&wan_lbuffer);
    
    if  ( strstr( p, "CONFIG=OK") != NULL ) {
        retS = true;
       goto exit_;
    }
     
    if  ( strstr( p, "CONFIG=ERROR") != NULL ) {
        xprintf_P(PSTR("WAN:: CONF ERROR: El servidor no reconoce al datalogger !!\r\n"));
        // Reconfiguro para espera 1h.!!!
        xprintf_P(PSTR("WAN:: Reconfigurado para reintentar en 1H !!\r\n"));
        systemConf.pwr_modo = PWR_DISCRETO;
        systemConf.timerdial = 3600;
        systemConf.timerpoll = 3600;
        retS = false;
        goto exit_;    
    }
         
    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
	memset(localStr,'\0',sizeof(localStr));
	ts = strstr( p, "TPOLL=");
    if  ( ts != NULL ) {
        strncpy(localStr, ts, sizeof(localStr));
        stringp = localStr;
        token = strsep(&stringp,delim);	 	// TPOLL
        token = strsep(&stringp,delim);	 	// timerpoll
        config_timerpoll(token);
        xprintf_P( PSTR("WAN:: Reconfig TIMERPOLL to %d\r\n\0"), systemConf.timerpoll );
    }
    //
    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
    memset(localStr,'\0',sizeof(localStr));
	ts = strstr( p, "TDIAL=");
    if  ( ts != NULL ) {
        strncpy(localStr, ts, sizeof(localStr));
        stringp = localStr;
        token = strsep(&stringp,delim);	 	// TDIAL
        token = strsep(&stringp,delim);	 	// timerdial
        config_timerdial(token);
        xprintf_P( PSTR("WAN:: Reconfig TIMERDIAL to %d\r\n\0"), systemConf.timerdial );
    }
    //
    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
    memset(localStr,'\0',sizeof(localStr));
	ts = strstr( p, "PWRMODO=");
	if  ( ts != NULL ) {
        strncpy(localStr, ts, sizeof(localStr));
        stringp = localStr;
        token = strsep(&stringp,delim);	 	// PWRMODO
        token = strsep(&stringp,delim);	 	// pwrmodo_string
        config_pwrmodo(token);
        xprintf_P( PSTR("WAN:: Reconfig PWRMODO to %s\r\n\0"), token );
    }
    //
    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
    memset(localStr,'\0',sizeof(localStr));
	ts = strstr( p, "PWRON=");
	if  ( ts != NULL ) {
        strncpy(localStr, ts, sizeof(localStr));
        stringp = localStr;
        token = strsep(&stringp,delim);	 	// PWRON
        token = strsep(&stringp,delim);	 	// pwron
        config_pwron(token);
        xprintf_P( PSTR("WAN:: Reconfig PWRON to %d\r\n\0"), systemConf.pwr_hhmm_on );
    }
    //
    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
    memset(localStr,'\0',sizeof(localStr));
	ts = strstr( p, "PWROFF=");
	if  ( ts != NULL ) {
        strncpy(localStr, ts, sizeof(localStr));
        stringp = localStr;
        token = strsep(&stringp,delim);	 	// PWROFF
        token = strsep(&stringp,delim);	 	// pwroff
        config_pwroff(token);
        xprintf_P( PSTR("WAN:: Reconfig PWROFF to %d\r\n\0"), systemConf.pwr_hhmm_off );
    }
    //
    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
    memset(localStr,'\0',sizeof(localStr));
	ts = strstr( p, "SAMPLES=");
	if  ( ts != NULL ) {
        strncpy(localStr, ts, sizeof(localStr));
        stringp = localStr;
        token = strsep(&stringp,delim);	 	// SAMPLES
        token = strsep(&stringp,delim);	 	// samples
        config_samples(token);
        xprintf_P( PSTR("WAN:: Reconfig SAMPLES to %d\r\n\0"), systemConf.samples_count );
    }
    // 
    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
    memset(localStr,'\0',sizeof(localStr));
	ts = strstr( p, "ALMLEVEL=");
	if  ( ts != NULL ) {
        strncpy(localStr, ts, sizeof(localStr));
        stringp = localStr;
        token = strsep(&stringp,delim);	 	// ALARM
        token = strsep(&stringp,delim);	 	// alarm
        config_almlevel(token);
        xprintf_P( PSTR("WAN:: Reconfig ALARM to %d\r\n\0"), systemConf.alarm_level );
    }
    // 
    retS = true;
    
exit_:
                
    return(retS);      
}
//------------------------------------------------------------------------------
static bool wan_process_frame_configAinputs(void)
{
     /*
      * Envo un frame con el hash de la configuracion de entradas analogicas.
      * El server me puede mandar OK o la nueva configuracion que debo tomar.
      * Lo reintento 2 veces
      */
    
uint8_t tryes = 0;
uint8_t timeout = 0;
bool retS = false;
uint8_t hash = 0;


    xprintf_P(PSTR("WAN:: CONFIG_AINPUTS.\r\n"));
 
    // Armo el buffer
    while ( xSemaphoreTake( sem_WAN, MSTOTAKEWANSEMPH ) != pdTRUE )
        vTaskDelay( ( TickType_t)( 1 ) );
    
    memset(wan_tx_buffer, '\0', WAN_TX_BUFFER_SIZE);
    hash = ainputs_hash();
    sprintf_P( (char*)&wan_tx_buffer, PSTR("ID=%s&TYPE=%s&VER=%s&CLASS=CONF_AINPUTS&HASH=0x%02X"), systemConf.dlgid, FW_TYPE, FW_REV, hash );

    // Proceso. Envio hasta 2 veces el frame y espero hasta 10s la respuesta
    tryes = 2;
    while (tryes-- > 0) {
        
        wan_xmit_out(DEBUG_WAN);
    
        // Espero respuesta chequeando cada 1s durante 10s.
        timeout = 10;
        while ( timeout-- > 0) {
            vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
            if ( wan_check_response("CONFIG=ERROR")) {
                xprintf_P(PSTR("WAN:: CONF_AIPUTS ERROR: El servidor no reconoce al datalogger !!\r\n"));
                retS = false;
                goto exit_;
            
            } else if ( wan_check_response("CONF_AINPUTS&CONFIG=OK")) {
                wan_print_RXbuffer();
                retS = true;
                goto exit_;
                
            } else if ( wan_check_response("CLASS=CONF_AINPUTS")) {
                wan_print_RXbuffer();
                wan_process_rsp_configAinputs();
                retS = true;
                goto exit_;
            } 
        }
    }
 
    // Expiro el tiempo sin respuesta del server.
    xprintf_P(PSTR("WAN:: CONFIG_AINPUTS ERROR: Timeout en server rsp.!!\r\n"));
    retS = false;
    
exit_:
               
    xSemaphoreGive( sem_WAN );
    return(retS);      
}
//------------------------------------------------------------------------------
static bool wan_process_rsp_configAinputs(void)
{
   /*
     * Procesa la configuracion de los canales analogicos
     * RXFRAME: <html><body><h1>CLASS=CONF_AINPUTS&A0=true,pA,4,20,0.0,10.0,0.0&A1=true,pB,4,20,0.0,10.0,0.0&A2=false,X,4,20,0.0,10.0,0.0</h1></body></html>
     *                          CLASS=CONF_AINPUTS&CONFIG=OK
     */
    
char *ts = NULL;
char localStr[32] = { 0 };
char *stringp = NULL;
char *tk_name= NULL;
char *tk_enable= NULL;
char *tk_iMin= NULL;
char *tk_iMax = NULL;
char *tk_mMin = NULL;
char *tk_mMax = NULL;
char *tk_offset = NULL;
char *delim = "&,;:=><";
uint8_t ch;
char str_base[8];
char *p;
bool retS = false;

    p = lBchar_get_buffer(&wan_lbuffer);
    
    if  ( strstr( p, "CONFIG=OK") != NULL ) {
        retS = true;
       goto exit_;
    }

	// A?
	for (ch=0; ch < NRO_ANALOG_CHANNELS; ch++ ) {
        
        vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
        
		memset( &str_base, '\0', sizeof(str_base) );
		snprintf_P( str_base, sizeof(str_base), PSTR("A%d\0"), ch );

		if ( strstr( p, str_base) != NULL ) {
			memset(localStr,'\0',sizeof(localStr));
            ts = strstr( p, str_base);
			strncpy( localStr, ts, sizeof(localStr));
			stringp = localStr;
			tk_name = strsep(&stringp,delim);		//A0
            tk_enable = strsep(&stringp,delim);     // enable
			tk_name = strsep(&stringp,delim);		//name
			tk_iMin = strsep(&stringp,delim);		//iMin
			tk_iMax = strsep(&stringp,delim);		//iMax
			tk_mMin = strsep(&stringp,delim);		//mMin
			tk_mMax = strsep(&stringp,delim);		//mMax
			tk_offset = strsep(&stringp,delim);		//offset

            //xprintf_P(PSTR("DEBUG_AINPUT: %d [%s][%s][%s][%s][%s]\r\n"), ch, tk_enable, tk_name , tk_iMin, tk_iMax, tk_mMin );
            
			ainputs_config_channel( ch, tk_enable, tk_name , tk_iMin, tk_iMax, tk_mMin, tk_mMax, tk_offset );
			xprintf_P( PSTR("WAN:: Reconfig A%d\r\n"), ch);
		}
	}
    retS = true;
   
exit_:
                
    return(retS);
}
//------------------------------------------------------------------------------
static bool wan_process_frame_configCounters(void)
{
     /*
      * Envo un frame con el hash de la configuracion de contadores.
      * El server me puede mandar OK o la nueva configuracion que debo tomar.
      * Lo reintento 2 veces
      */
    
uint8_t tryes = 0;
uint8_t timeout = 0;
bool retS = false;
uint8_t hash = 0;

    xprintf_P(PSTR("WAN:: CONFIG_COUNTERS.\r\n"));
    
    // Armo el buffer
    while ( xSemaphoreTake( sem_WAN, MSTOTAKEWANSEMPH ) != pdTRUE )
        vTaskDelay( ( TickType_t)( 1 ) );
    
    memset(wan_tx_buffer, '\0', WAN_TX_BUFFER_SIZE);
    hash = counters_hash();
    sprintf_P( (char*)&wan_tx_buffer, PSTR("ID=%s&TYPE=%s&VER=%s&CLASS=CONF_COUNTERS&HASH=0x%02X"), systemConf.dlgid, FW_TYPE, FW_REV, hash );

    // Proceso. Envio hasta 2 veces el frame y espero hasta 10s la respuesta
    tryes = 2;
    while (tryes-- > 0) {
        
        wan_xmit_out(DEBUG_WAN);
    
        // Espero respuesta chequeando cada 1s durante 10s.
        timeout = 10;
        while ( timeout-- > 0) {
            vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
            if ( wan_check_response("CONFIG=ERROR")) {
                xprintf_P(PSTR("WAN:: CONF_COUNTERS ERROR: El servidor no reconoce al datalogger !!\r\n"));
                retS = false;
                goto exit_;
            
            } else if ( wan_check_response("CONF_COUNTERS&CONFIG=OK")) {
                wan_print_RXbuffer();
                retS = true;
                goto exit_;
                
            } else if ( wan_check_response( "CLASS=CONF_COUNTERS")) {
                wan_print_RXbuffer();
                wan_process_rsp_configCounters();
                retS = true;
                goto exit_;
            } 
           
        }
    }
 
    // Expiro el tiempo sin respuesta del server.
    xprintf_P(PSTR("WAN:: CONFIG_COUNTERS ERROR: Timeout en server rsp.!!\r\n"));
    retS = false;
    
exit_:
               
    xSemaphoreGive( sem_WAN );
    return(retS);   
    
 
}
//------------------------------------------------------------------------------
static bool wan_process_rsp_configCounters(void)
{
    /*
     * Procesa la configuracion de los canales contadores
     * RXFRAME: <html><body><h1>CLASS=CONF_COUNTERS&C0=TRUE,q0,0.01,CAUDAL,3&C1=FALSE,X,0.0,CAUDAL,4</h1></body></html>
     * 
     */

char *ts = NULL;
char localStr[32] = { 0 };
char *stringp = NULL;
char *tk_name = NULL;
char *tk_enable= NULL;
char *tk_magpp = NULL;
char *tk_modo = NULL;
char *tk_rbsize = NULL;
char *delim = "&,;:=><";
uint8_t ch;
char str_base[8];
char *p;
bool retS = false;

    p = lBchar_get_buffer(&wan_lbuffer);
    
    if  ( strstr( p, "CONFIG=OK") != NULL ) {
       retS = true;
       goto exit_;
    }

	// C?
	for (ch=0; ch < NRO_COUNTER_CHANNELS; ch++ ) {
        
        vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
        
		memset( &str_base, '\0', sizeof(str_base) );
		snprintf_P( str_base, sizeof(str_base), PSTR("C%d"), ch );

		if ( strstr( p, str_base) != NULL ) {
			memset(localStr,'\0',sizeof(localStr));
			ts = strstr( p, str_base);
			strncpy(localStr, ts, sizeof(localStr));
			stringp = localStr;
			tk_enable = strsep(&stringp,delim);		//C0
            tk_enable = strsep(&stringp,delim);     //enable
			tk_name = strsep(&stringp,delim);		//name
			tk_magpp = strsep(&stringp,delim);		//magpp
			tk_modo = strsep(&stringp,delim);       //modo
            tk_rbsize = strsep(&stringp,delim);     //rbsize

            //xprintf_P(PSTR("DEBUG: ch=%d,enable=%s,name=%s magpp=%s,modo=%s,rbsize=%s\r\n"), ch, tk_enable, tk_name, tk_magpp, tk_modo, tk_rbsize);
			
            counters_config_channel( ch , tk_enable, tk_name , tk_magpp, tk_modo, tk_rbsize );         
			xprintf_P( PSTR("WAN:: Reconfig C%d\r\n"), ch);
		}
	}
    
    retS = true;
    
exit_:
               
	return(retS);

}
//------------------------------------------------------------------------------
static bool wan_process_frame_configPiloto(void)
{
    /*
      * Envo un frame con el hash de la configuracion del pilotos.
      * El server me puede mandar OK o la nueva configuracion que debo tomar.
      * Lo reintento 2 veces
      */
    
uint8_t tryes = 0;
uint8_t timeout = 0;
bool retS = false;
uint8_t hash = 0;

    xprintf_P(PSTR("WAN:: CONFIG_PILOTO.\r\n"));
 
    // Armo el buffer
    while ( xSemaphoreTake( sem_WAN, MSTOTAKEWANSEMPH ) != pdTRUE )
        vTaskDelay( ( TickType_t)( 1 ) );
    memset(wan_tx_buffer, '\0', WAN_TX_BUFFER_SIZE);
    hash = piloto_hash( u_hash );
    sprintf_P( (char*)&wan_tx_buffer, PSTR("ID=%s&TYPE=%s&VER=%s&CLASS=CONF_PILOTO&HASH=0x%02X"), systemConf.dlgid, FW_TYPE, FW_REV, hash );

    // Proceso. Envio hasta 2 veces el frame y espero hasta 10s la respuesta
    tryes = 2;
    while (tryes-- > 0) {
        
        wan_xmit_out(DEBUG_WAN);
    
        // Espero respuesta chequeando cada 1s durante 10s.
        timeout = 10;
        while ( timeout-- > 0) {
            vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
            if ( wan_check_response("CONFIG=ERROR")) {
                xprintf_P(PSTR("WAN:: CONF_PILOTO ERROR: El servidor no reconoce al datalogger !!\r\n"));
                retS = false;
                goto exit_;
            
            } else if ( wan_check_response("CONF_PILOTO&CONFIG=OK")) {
                wan_print_RXbuffer();
                retS = true;
                goto exit_;
                
            } else if ( wan_check_response( "CLASS=CONF_PILOTO" )) {
                wan_print_RXbuffer();
                wan_process_rsp_configPiloto();
                retS = true;
                goto exit_;
            } 
        }
    }
 
    // Expiro el tiempo sin respuesta del server.
    xprintf_P(PSTR("WAN:: CONFIG_PILOTO ERROR: Timeout en server rsp.!!\r\n"));
    retS = false;
    
exit_:
               
    xSemaphoreGive( sem_WAN );
    return(retS);       
}
//------------------------------------------------------------------------------
static bool wan_process_rsp_configPiloto(void)
{
   /*
     * Procesa la configuracion de los canales Modbus
     * RXFRAME: <html><body><h1>CLASS=CONF_PILOTO&ENABLE=TRUE&PULSEXREV=100&PWIDTH=10&
     *                                             S0=230,1.34&
     *                                             S1=650,2.4&
     *                                             .....
     *                                             S11=2330,3.2</h1></body></html>
     *                          CLASS:CONF_PILOTO;CONFIG:OK
     * 
     */

    
char *ts = NULL;
char localStr[48] = { 0 };
char *stringp = NULL;
char *tk_enable= NULL;
char *tk_pulsexrev= NULL;
char *tk_pwidth= NULL;
char *tk_stime = NULL;
char *tk_spres = NULL;
char *delim = "&,;:=><";
uint8_t slot;
char str_base[8];
char *p;
bool retS = false;

    p = lBchar_get_buffer(&wan_lbuffer);
    
    if  ( strstr( p, "CONFIG=OK") != NULL ) {
        retS = true;
       goto exit_;
    }

    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
    memset(localStr,'\0',sizeof(localStr));
	ts = strstr( p, "ENABLE=");
    if  ( ts != NULL ) {
        strncpy(localStr, ts, sizeof(localStr));
        stringp = localStr;
        tk_enable = strsep(&stringp,delim);	 	// ENABLE
        tk_enable = strsep(&stringp,delim);	 	// TRUE/FALSE
        piloto_config_enable(tk_enable);
        
        //xprintf_P(PSTR("DEBUG_PILOTO enable=[%s]\r\n"),tk_enable);
        xprintf_P( PSTR("WAN:: Reconfig PILOTO ENABLE to %s\r\n"), tk_enable );
    }

    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
    memset(localStr,'\0',sizeof(localStr));
	ts = strstr( p, "PULSEXREV=");
    if  ( ts != NULL ) {
        strncpy(localStr, ts, sizeof(localStr));
        stringp = localStr;
        tk_pulsexrev = strsep(&stringp,delim);	 	// PULSEXREV
        tk_pulsexrev = strsep(&stringp,delim);	 	// 1500
        piloto_config_pulseXrev(tk_pulsexrev);
        //xprintf_P(PSTR("DEBUG_PILOTO tk_pulsexreve=[%s]\r\n"),tk_pulsexrev);
        xprintf_P( PSTR("WAN:: Reconfig PILOTO PULSEXREV to %s\r\n"), tk_pulsexrev );
    }
    
    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
    memset(localStr,'\0',sizeof(localStr));
	ts = strstr( p, "PWIDTH=");
    if  ( ts != NULL ) {
        strncpy(localStr, ts, sizeof(localStr));
        stringp = localStr;
        tk_pwidth = strsep(&stringp,delim);	 	// PWIDTH
        tk_pwidth = strsep(&stringp,delim);	 	// 10
        piloto_config_pwidth(tk_pwidth);
        xprintf_P( PSTR("WAN:: Reconfig PILOTO PWIDTH to %s\r\n"), tk_pwidth );
    }
    
    //
	// SLOTS: Sx:time,pres
	for (slot=0; slot < MAX_PILOTO_PSLOTS; slot++ ) {
        
        vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
		memset( &str_base, '\0', sizeof(str_base) );
		snprintf_P( str_base, sizeof(str_base), PSTR("S%d"), slot );

		if ( strstr( p, str_base) != NULL ) {
			memset(localStr,'\0',sizeof(localStr));
            ts = strstr( p, str_base);
			strncpy( localStr, ts, sizeof(localStr));
			stringp = localStr;
			tk_stime = strsep(&stringp,delim);	   //Sx
            tk_stime = strsep(&stringp,delim);     //time
			tk_spres = strsep(&stringp,delim);	   //pres
                    
			piloto_config_slot( slot,tk_stime, tk_spres );
			xprintf_P( PSTR("WAN:: Reconfig PILOTO SLOT %d\r\n"), slot);
		}
	}
    retS = true;
   
exit_:
                
    return(retS);
}
//------------------------------------------------------------------------------
static bool wan_process_frame_configConsigna(void)
{
    /*
      * Envo un frame con el hash de la configuracion del consignas.
      * El server me puede mandar OK o la nueva configuracion que debo tomar.
      * Lo reintento 2 veces
      */
    
uint8_t tryes = 0;
uint8_t timeout = 0;
bool retS = false;
uint8_t hash = 0;

    xprintf_P(PSTR("WAN:: CONFIG_CONSIGNA.\r\n"));
 
    // Armo el buffer
    while ( xSemaphoreTake( sem_WAN, MSTOTAKEWANSEMPH ) != pdTRUE )
        vTaskDelay( ( TickType_t)( 1 ) );
    memset(wan_tx_buffer, '\0', WAN_TX_BUFFER_SIZE);
    hash = consigna_hash( u_hash );
    sprintf_P( (char*)&wan_tx_buffer, PSTR("ID=%s&TYPE=%s&VER=%s&CLASS=CONF_CONSIGNA&HASH=0x%02X"), systemConf.dlgid, FW_TYPE, FW_REV, hash );

    // Proceso. Envio hasta 2 veces el frame y espero hasta 10s la respuesta
    tryes = 2;
    while (tryes-- > 0) {
        
        wan_xmit_out(DEBUG_WAN);
    
        // Espero respuesta chequeando cada 1s durante 10s.
        timeout = 10;
        while ( timeout-- > 0) {
            vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
            if ( wan_check_response("CONFIG=ERROR")) {
                xprintf_P(PSTR("WAN:: CONF_CONSIGNA ERROR: El servidor no reconoce al datalogger !!\r\n"));
                retS = false;
                goto exit_;
            
            } else if ( wan_check_response("CONF_CONSIGNA&CONFIG=OK")) {
                wan_print_RXbuffer();
                retS = true;
                goto exit_;
                
            } else if ( wan_check_response( "CLASS=CONF_CONSIGNA" )) {
                wan_print_RXbuffer();
                wan_process_rsp_configConsigna();
                retS = true;
                goto exit_;
            } 
        }
    }
 
    // Expiro el tiempo sin respuesta del server.
    xprintf_P(PSTR("WAN:: CONFIG_CONSIGNA ERROR: Timeout en server rsp.!!\r\n"));
    retS = false;
    
exit_:
               
    xSemaphoreGive( sem_WAN );
    return(retS);       
}
//------------------------------------------------------------------------------
static bool wan_process_rsp_configConsigna(void)
{
   /*
     * Procesa la configuracion de los canales Modbus
     * RXFRAME: <html><body><h1>CLASS=CONF_CONSIGNA&ENABLE=TRUE&DIURNA=700&NOCTURNA=2300</h1></body></html>
     *                          CLASS:CONF_CONSIGNA;CONFIG:OK
     * 
     */
  
char localStr[64] = { 0 };
char *token = NULL;
char *tk_enable = NULL;
char *tk_diurna = NULL;
char *tk_nocturna = NULL;
char *delim = "<>/&=";
char *ts = NULL;
char *p;
bool retS = false;

    p = lBchar_get_buffer(&wan_lbuffer);
    
    if  ( strstr( p, "CONFIG=OK") != NULL ) {
        retS = true;
       goto exit_;
    }
     
    if  ( strstr( p, "CONFIG=ERROR") != NULL ) {
        xprintf_P(PSTR("WAN:: CONF ERROR !!\r\n"));
        retS = false;
        goto exit_;    
    }
         
    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
	memset(localStr,'\0',sizeof(localStr));
	ts = strstr( p, "ENABLE=");
    if  ( ts != NULL ) {
        strncpy(localStr, ts, sizeof(localStr));
        token = strtok (localStr, delim);       
        while ( token != NULL ) {
            //xprintf_P(PSTR("TOK=%s\r\n"), token ); //printing each token
            if (strstr(token,"ENABLE") != NULL ) {
                token = strtok(NULL, delim); 
                tk_enable = token;
                continue;
            } else if (strstr(token,"DIURNA") != NULL ) {
                token = strtok(NULL, delim); 
                tk_diurna = token;
                continue;
            } else if (strstr(token,"NOCTURNA") != NULL ) {
                token = strtok(NULL, delim); 
                tk_nocturna = token;
                continue;
            }
            token = strtok(NULL, delim);
        }
	 	// 
    }
    //xprintf_P( PSTR("WAN:: Reconfig CONSIGNA to: %s,%s,%s\r\n"), tk_enable, tk_diurna, tk_nocturna);
    xprintf_P( PSTR("WAN:: Reconfig CONSIGNA\r\n"));
    consigna_config( tk_enable, tk_diurna, tk_nocturna );
    retS = true;
   
exit_:
                
    return(retS);
}
//------------------------------------------------------------------------------
static bool wan_send_from_memory(void)
{
    /*
     * Lee el FS y transmite los registros acumulados.
     * Al recibir un OK los borra.
     * Envia de a 1.
     */
    
dataRcd_s dr;
bool retS = false;

    xprintf_P(PSTR("WAN:: Dump memory...\r\n"));
    /*
    while ( FS_readRcd( &dr, sizeof(dataRcd_s) )) {
        
        retS = wan_process_frame_data(&dr);
        if ( ! retS) {
            goto quit;
        }
        
        FAT_read(&l_fat);
        xprintf_P( PSTR("Mem: wrPtr=%d,rdPtr=%d,count=%d\r\n"),l_fat.head, l_fat.tail, l_fat.count );

    }
     */
    
    FAT_read(&l_fat1);
    xprintf_P( PSTR("WAN:: wrPtr=%d,rdPtr=%d,count=%d\r\n"), l_fat1.head, l_fat1.tail, l_fat1.count );
    
    while ( l_fat1.count > 0 ) {
        KICK_WDG(WAN_WDG_bp);
        xprintf_P( PSTR("WAN: wrPtr=%d,rdPtr=%d,count=%d\r\n"),l_fat1.head, l_fat1.tail, l_fat1.count );
        
        if ( FS_readRcd( &dr, sizeof(dataRcd_s) ) ) {
            retS = wan_process_frame_data(&dr);
            if ( ! retS) {
                goto quit;
            }
        } else {
            goto quit;
        }
        
        FAT_read(&l_fat1);
    }
    
quit:
                
    xprintf_P(PSTR("WAN:: Memory Empty\r\n"));
    //FAT_flush();
    
    return (retS);


}
//------------------------------------------------------------------------------
static bool wan_process_frame_data(dataRcd_s *dr)
{
   /*
    * Armo el wan_tx_buffer.
    * Transmite los datos pasados en la estructura dataRcd por el
    * puerto definido como WAN
    */ 

uint8_t tryes = 0;
uint8_t timeout = 0;
bool retS = false;

    if (f_debug_comms) {
        xprintf_P(PSTR("WAN:: DATA.\r\n"));
    }

    while ( xSemaphoreTake( sem_WAN, MSTOTAKEWANSEMPH ) != pdTRUE )
        vTaskDelay( ( TickType_t)( 1 ) );
    
    //goto exit_;
    
    // Guardo el DR trasmitido en memoria para compararlo y ver si se disparan alarmas.
    // En dataRcd_previo tengo el ultimo dato transmitido !!!.
    //memcpy(&dataRcd_previo, dr, sizeof(dataRcd_previo));
   
    // Formateo(escribo) el dr en el wan_tx_buffer
    wan_load_dr_in_txbuffer(dr, (uint8_t *)&wan_tx_buffer,WAN_TX_BUFFER_SIZE );
    
    // Proceso. Envio hasta 2 veces el frame y espero hasta 10s la respuesta
    tryes = 2;
    while (tryes-- > 0) {
        
        wan_xmit_out(DEBUG_WAN);
        // Espero respuesta chequeando cada 1s durante 10s.
        timeout = 10;
        while ( timeout-- > 0) {
            vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
            if ( wan_check_response("CLASS=DATA")) {
                wan_print_RXbuffer();
                wan_process_rsp_data();
                retS = true;
                goto exit_;
            }
        }
        
        xprintf_P(PSTR("WAN:: DATA RETRY\r\n"));
        
    }
 
    // Expiro el tiempo sin respuesta del server.
    xprintf_P(PSTR("WAN:: DATA ERROR: Timeout en server rsp.!!\r\n"));
    retS = false;
    
exit_:

    xSemaphoreGive( sem_WAN );
    return(retS);
    
}
//------------------------------------------------------------------------------
static void wan_load_dr_in_txbuffer(dataRcd_s *dr, uint8_t *buff, uint16_t buffer_size )
{
    /*
     * Toma un puntero a un dr y el wan_tx_buffer y escribe el dr en este
     * con el formato para transmitir.
     * En semaforo lo debo tomar en la funcion que invoca !!!.
     */

uint8_t i;
int16_t fptr;

   // Armo el buffer
    memset(buff, '\0', buffer_size);
    fptr = 0;
    fptr = sprintf_P( (char *)&buff[fptr], PSTR("ID=%s&TYPE=%s&VER=%s&CLASS=DATA"), systemConf.dlgid, FW_TYPE, FW_REV);   
         
    // Clock
    fptr += sprintf_P( (char *)&buff[fptr], PSTR("&DATE=%02d%02d%02d"), dr->rtc.year,dr->rtc.month, dr->rtc.day );
    fptr += sprintf_P( (char *)&buff[fptr], PSTR("&TIME=%02d%02d%02d"), dr->rtc.hour,dr->rtc.min, dr->rtc.sec);
    
    // Analog Channels:
    for ( i=0; i < NRO_ANALOG_CHANNELS; i++) {
        if ( strcmp ( systemConf.ainputs_conf.channel[i].name, "X" ) != 0 ) {
            fptr += sprintf_P( (char*)&buff[fptr], PSTR("&%s=%0.2f"), systemConf.ainputs_conf.channel[i].name, dr->l_ainputs[i]);
        }
    }
        
    // Counter Channels:
    for ( i=0; i < NRO_COUNTER_CHANNELS; i++) {
        if ( strcmp ( systemConf.counters_conf.channel[i].name, "X" ) != 0 ) {
            fptr += sprintf_P( (char*)&buff[fptr], PSTR("&%s=%0.3f"), systemConf.counters_conf.channel[i].name, dr->l_counters[i]);
        }
    }
       
    // Battery
    fptr += sprintf_P( (char*)&buff[fptr], PSTR("&bt=%0.3f"), dr->battery);
    
    //fptr += sprintf_P( (char*)&buff[fptr], PSTR("\r\n"));
    
}
//------------------------------------------------------------------------------
static bool wan_process_rsp_data(void)
{
   /*
     * Procesa las respuestas a los frames DATA
     * Podemos recibir CLOCK, RESET, PILOTO
     */
    
char localStr[32] = { 0 };
char *ts = NULL;
char *stringp = NULL;
char *token = NULL;
char *delim = "&,;:=><";
char *p;

    p = lBchar_get_buffer(&wan_lbuffer);

    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
    //xprintf_P(PSTR("WAN:: response\r\n") );
    
    if ( strstr( p, "CLOCK") != NULL ) {
        memset(localStr,'\0',sizeof(localStr));
        ts = strstr( p, "CLOCK=");
        strncpy(localStr, ts, sizeof(localStr));
        stringp = localStr;
        token = strsep(&stringp,delim);			// CLOCK
        token = strsep(&stringp,delim);			// 1910120345

        // Error en el string recibido
        if ( strlen(token) < 10 ) {
            // Hay un error en el string que tiene la fecha.
            // No lo reconfiguro
            xprintf_P(PSTR("WAN:: data_resync_clock ERROR:[%s]\r\n"), token );
            return(false);
        } else {
            data_resync_clock( token, false );
        }
    }
    
    if ( strstr( p, "PILOTO") != NULL ) {
        memset(localStr,'\0',sizeof(localStr));
        ts = strstr( p, "PILOTO=");
        strncpy(localStr, ts, sizeof(localStr));
        stringp = localStr;
        token = strsep(&stringp,delim);			// PILOTO
        token = strsep(&stringp,delim);			// 3.45
        piloto_productor_handler_online(atof(token));
    }

    if ( strstr( p, "RESMEM") != NULL ) {
        xprintf_P(PSTR("WAN:: RESET MEMORY order from Server !!\r\n"));
        vTaskDelay( ( TickType_t)( 2000 / portTICK_PERIOD_MS ) );
        reset_memory_remote();
    }
    
    if ( strstr( p, "RESET") != NULL ) {
        xprintf_P(PSTR("WAN:: RESET order from Server !!\r\n"));
        vTaskDelay( ( TickType_t)( 2000 / portTICK_PERIOD_MS ) );
        reset();
    }
    
    return(true);  
}
//------------------------------------------------------------------------------
static void wan_xmit_out(bool debug_flag )
{
    /* 
     * Transmite el buffer de tx por el puerto comms configurado para wan.
     */
    
    // Antes de trasmitir siempre borramos el Rxbuffer
    lBchar_Flush(&wan_lbuffer);
    xfprintf_P( fdRS485A, PSTR("%s"), &wan_tx_buffer[0]);

    if (f_debug_comms || debug_flag) {
        xprintf_P( PSTR("Xmit-> %s\r\n"), &wan_tx_buffer[0]);
    }
    
}
//------------------------------------------------------------------------------
static bool wan_check_response ( const char *s)
{
    /*
     * Retorna true/false si encuentra el patron *s en el buffer de rx wan.
     */
    
char *p;
    
    p = lBchar_get_buffer(&wan_lbuffer);
        
    if  ( strstr( p, "</html>") != NULL ) {
        
        if (f_debug_comms)
            xprintf_P( PSTR("rxbuff-> %s\r\n"), p);

		// Recibi un frame completo  
        if ( strstr( p, s) != NULL ) {
            return(true);
        }
    }
    return (false);
}
//------------------------------------------------------------------------------
static void wan_print_RXbuffer(void)
{

char *p;
            
    p = lBchar_get_buffer(&wan_lbuffer);
    //if (f_debug_comms) {
        xprintf_P(PSTR("Rcvd-> %s\r\n"), p );
    //}
}
//------------------------------------------------------------------------------
//
// FUNCIONES PUBLICAS
//
//------------------------------------------------------------------------------
bool WAN_process_data_rcd( dataRcd_s *dataRcd)
{
    /*
     * El procesamiento implica transmitir si estoy con el link up, o almacenarlo
     * en memoria para luego transmitirlo.
     * Copio el registro a uno local y prendo la flag que indica que hay datos 
     * para transmitir.
     * Si hay un dato aun en el buffer, lo sobreescribo !!!
     * 
     */
   
bool retS = false;
fat_s l_fat;   

    // link up
    if ( link_up4data  ) {
        /*
         * Hay enlace. 
         * Guardo el dato en el buffer.
         * Indico que hay un dato listo para enviar
         * Aviso (despierto) para que se transmita.
         */
        //xprintf_P(PSTR("WAN:: New dataframe.\r\n"));
        
        memcpy( &drWanBuffer.dr, dataRcd, sizeof(dataRcd_s));
        drWanBuffer.dr_ready = true;    
        // Aviso al estado online que hay un frame listo
        while ( xTaskNotify(xHandle_tkWAN, SGN_FRAME_READY , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );
		}
        retS = true;
        
    } else {
        // Guardo en memoria
        xprintf_P(PSTR("WAN:: Save frame in EE.\r\n"));
        retS = FS_writeRcd( dataRcd, sizeof(dataRcd_s) );
        if ( ! retS  ) {
            // Error de escritura o memoria llena ??
            xprintf_P(PSTR("WAN:: WR ERROR\r\n"));
        }
        // Stats de memoria
        FAT_read(&l_fat);
        xprintf_P( PSTR("wrPtr=%d,rdPtr=%d,count=%d\r\n"),l_fat.head,l_fat.tail, l_fat.count );
        
    }
    return(retS);
}
//------------------------------------------------------------------------------
void WAN_put(uint8_t c)
{
    /*
     * Funcion usada por rutinas que manejan los RX de los diferentes comm ports
     * para poner los datos recividos en la cola WAN cuando estos puertos se
     * configuran para WAN
     */
    
    lBchar_Put( &wan_lbuffer, c);
   
}
//------------------------------------------------------------------------------
void WAN_print_configuration(void)
{
    
    xprintf_P(PSTR("Comms:\r\n"));
    xprintf_P(PSTR(" debug: "));
    f_debug_comms ? xprintf_P(PSTR("true\r\n")) : xprintf_P(PSTR("false\r\n"));
    xprintf_P(PSTR(" port: RS485A\r\n"));
    
}
//------------------------------------------------------------------------------
void WAN_config_debug(bool debug )
{
    if ( debug ) {
        f_debug_comms = true;
    } else {
        f_debug_comms = false;
    }
}
//------------------------------------------------------------------------------
bool WAN_read_debug(void)
{
    return (f_debug_comms);
}
//------------------------------------------------------------------------------
void wan_PRENDER_MODEM(void)
{
    MODEM_PRENDER();
    xprintf_P(PSTR("WAN:: PRENDER MODEM\r\n"));
}
//------------------------------------------------------------------------------
void wan_APAGAR_MODEM(void)
{
    MODEM_APAGAR();
    xprintf_P(PSTR("WAN:: APAGAR MODEM\r\n"));
}
//------------------------------------------------------------------------------
void WAN_kill_task(void)
{
    // Mata la tarea pero deja todo ordenado para que
    // los datos se guarden en memoria.
    if ( xHandle_tkWAN != NULL ) {
        vTaskSuspend( xHandle_tkWAN );
        xHandle_tkWAN = NULL;
        link_up4data = false;
        wan_APAGAR_MODEM();
    }
}
//------------------------------------------------------------------------------        
bool WAN_sleeping(void)
{
    return (wan_sleeping);
}
//------------------------------------------------------------------------------  
