
#include "SPX_XMEGA.h"
#include "frtos_cmd.h"

static void cmdClsFunction(void);
static void cmdHelpFunction(void);
static void cmdResetFunction(void);
static void cmdStatusFunction(void);
static void cmdWriteFunction(void);
static void cmdReadFunction(void);
static void cmdTestFunction(void);
static void cmdConfigFunction(void);

static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void );

#define T_SLEEPING  60000
#define T_AWAKE     90000

bool sleeping;

//------------------------------------------------------------------------------
void tkCmd(void * pvParameters)
{

	// Esta es la primer tarea que arranca.

( void ) pvParameters;

    while ( ! run_tasks )
        vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );

uint8_t c = 0;
int32_t awake_counter;

    FRTOS_CMD_init();

    FRTOS_CMD_register( "cls", cmdClsFunction );
	FRTOS_CMD_register( "help", cmdHelpFunction );
    FRTOS_CMD_register( "reset", cmdResetFunction );
    FRTOS_CMD_register( "status", cmdStatusFunction );
    FRTOS_CMD_register( "write", cmdWriteFunction );
    FRTOS_CMD_register( "read", cmdReadFunction );
    FRTOS_CMD_register( "test", cmdTestFunction );
    FRTOS_CMD_register( "config", cmdConfigFunction );
    
    xprintf_P(PSTR("Starting tkCMD..\r\n" ));
    xprintf_P(PSTR("Spymovil %s %s %s %s \r\n") , HW_MODELO, FRTOS_VERSION, FW_REV, FW_DATE);
       
    awake_counter = T_AWAKE;   
    sleeping = false;
    
	// loop
	for( ;; )
	{
         
        KICK_WDG(CMD_WDG_bp);
        
		c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
		// el read se bloquea 10ms. lo que genera la espera.
		//while ( frtos_read( fdTERM, (char *)&c, 1 ) == 1 ) {
        while ( xgetc( (char *)&c ) == 1 ) {
            FRTOS_CMD_process(c);
            awake_counter = T_AWAKE;    // Reinicio c/tecla apretada
        }
        
        if ( awake_counter > 0 ) {
            awake_counter--;
            sleeping = false;
            if ( awake_counter == 1 ) {
                xprintf_P(PSTR("tkCmd going to sleep..\r\n" ));
            }
        } else {
            sleeping = true;
        }
        
        if ( sleeping ) {
            // Duermo 60 secs
            vTaskDelay( ( TickType_t)( T_SLEEPING / portTICK_PERIOD_MS ) );  
        } else {
            // Espero 10ms si no hay caracteres en el buffer
            vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );  
        }
               
	}    
}
//------------------------------------------------------------------------------
static void cmdTestFunction(void)
{

    
char localStr[48] = { '\0' };
char *stringp = NULL;
char *tk_enable = NULL;
char *tk_diurna = NULL;
char *tk_nocturna = NULL;
char *delim = "&,;:=><";

    FRTOS_CMD_makeArgv();

        // STACKS SIZE
    if (!strcmp_P( strupr(argv[1]), PSTR("STACKS"))  ) {
        u_check_stacks_usage();
        pv_snprintfP_OK();
        return;
    }
    
    if (!strcmp_P( strupr(argv[1]), PSTR("CONSIGNA"))  ) { 
        strncpy(localStr, "ENABLE=TRUE&DIURNA=730&NOCTURNA=2345</html>", sizeof(localStr));
        xprintf_P(PSTR("CONSIGNA DEBUG: localStr=[%s]\r\n"), localStr);
        stringp = &localStr[0];
        xprintf_P(PSTR("CONSIGNA DEBUG: stringp=[%s]\r\n"), stringp);
        tk_enable = strsep(&stringp,delim);	 	// ENABLE
        tk_enable = strsep(&stringp,delim);	 	// TRUE/FALSE      
        xprintf_P(PSTR("CONSIGNA DEBUG: tkenable1=[%s]\r\n"),tk_enable);
        
        tk_diurna = strsep(&stringp,delim);	 	
        tk_diurna = strsep(&stringp,delim);	 	  
        xprintf_P(PSTR("CONSIGNA DEBUG: diurna=[%s]\r\n"),tk_diurna);
        
        tk_nocturna = strsep(&stringp,delim);	 	
        tk_nocturna = strsep(&stringp,delim);	 	  
        xprintf_P(PSTR("CONSIGNA DEBUG: tk_nocturna=[%s]\r\n"),tk_nocturna);
        
        consigna_config( tk_enable, tk_diurna, tk_nocturna);
        
    }
    
    if (!strcmp_P( strupr(argv[1]), PSTR("KILL"))  ) {
                
        if (!strcmp_P( strupr(argv[2]), PSTR("WAN"))  ) { 
            WAN_kill_task();
            return;
        }
        
        if (!strcmp_P( strupr(argv[2]), PSTR("SYS"))  ) {
            if ( xHandle_tkSys != NULL ) {
                vTaskSuspend( xHandle_tkSys );
                xHandle_tkSys = NULL;
            }
            return;
        }        
        xprintf_P(PSTR("test kill {sys,wan}\r\n"));
        return;
    }

 
    if (!strcmp_P( strupr(argv[1]), PSTR("MODEM"))  ) {
        if (!strcmp_P( strupr(argv[2]), PSTR("PRENDER"))  ) { 
            MODEM_PRENDER();
            return;
        }
        
        if (!strcmp_P( strupr(argv[2]), PSTR("APAGAR"))  ) {
            MODEM_APAGAR();
            return;
        }        
        xprintf_P(PSTR("test kill {sys,wan}\r\n"));
        return;
    }
    
    
    xprintf_P( PSTR("Test: poll, readrcd {pos},readdrcd {pos}, fsdebugon, fsdebugoff, write, kill {wan,sys}\r\n"));
    return;
       
}
//------------------------------------------------------------------------------
static void cmdHelpFunction(void)
{

    FRTOS_CMD_makeArgv();
        
    if ( !strcmp_P( strupr(argv[1]), PSTR("WRITE"))) {
		xprintf_P( PSTR("-write:\r\n"));
        xprintf_P( PSTR("  (ee,nvmee,rtcram) {pos string} {debug}\r\n"));
        xprintf_P( PSTR("  rtc YYMMDDhhmm\r\n"));
        xprintf_P( PSTR("  ina {a|b} {confValue}\r\n"));
        xprintf_P( PSTR("  vsensors420 {on/off}\r\n"));
        xprintf_P( PSTR("  k1,k2 {open/close}\r\n"));
        xprintf_P( PSTR("  rs485a {string}\r\n"));
        xprintf_P( PSTR("  piloto {pres}\r\n"));        
        xprintf_P( PSTR("  drv8814 {SLEEP,RESET,AENA,BENA,APH,BPH},{SET,CLEAR}\r\n"));
        xprintf_P( PSTR("  drv8814 {FC1,FC2}, GET\r\n"));
        xprintf_P( PSTR("  drv8814 pwr {on|off}\r\n"));
        xprintf_P( PSTR("  stepper move {FW,REV},npulses,dtime,ptime\r\n"));
        xprintf_P( PSTR("          awake,sleep,pha01,pha10,phb01,phb10\r\n"));
        xprintf_P( PSTR("  valve {A,B} {open,close}\r\n"));
        xprintf_P( PSTR("  consigna {diurna|nocturna}\r\n"));
        xprintf_P( PSTR("  sleep {true|false}\r\n"));
       
        
    }  else if ( !strcmp_P( strupr(argv[1]), PSTR("READ"))) {
		xprintf_P( PSTR("-read:\r\n"));
        xprintf_P( PSTR("  (ee,nvmee,rtcram) {pos} {lenght} {debug}\r\n"));
        xprintf_P( PSTR("  rtc {short | long }\r\n"));
        xprintf_P( PSTR("  ina {a|b} {conf|chXshv|chXbusv|mfid|dieid}\r\n"));
        xprintf_P( PSTR("  cnt {0,1}, fc1,fc_alta,fc2,fc_baja\r\n"));
        xprintf_P( PSTR("  serial,devid\r\n"));
        xprintf_P( PSTR("  ainput {n}\r\n"));
        xprintf_P( PSTR("  cnt {0,1}\r\n"));
        xprintf_P( PSTR("  rs485a\r\n"));
        
    }  else if ( !strcmp_P( strupr(argv[1]), PSTR("CONFIG"))) {
		xprintf_P( PSTR("-config:\r\n"));
        xprintf_P( PSTR("  default\r\n"));
        xprintf_P( PSTR("  dlgid\r\n"));
        xprintf_P( PSTR("  save,load\r\n"));
        xprintf_P( PSTR("  timerpoll, timerdial, samples {1..10}\r\n"));
        xprintf_P( PSTR("  pwrmodo {continuo,discreto,mixto}, pwron {hhmm}, pwroff {hhmm}\r\n"));
        xprintf_P( PSTR("  debug {analog,counters,comms,modbus,piloto,none} {true/false}\r\n"));
        xprintf_P( PSTR("  ainput {0..%d} enable{true/false} aname imin imax mmin mmax offset\r\n"),( NRO_ANALOG_CHANNELS - 1 ) );
        xprintf_P( PSTR("  counter {0..%d} enable{true/false} cname magPP modo(PULSO/CAUDAL),rbsize\r\n"), ( NRO_COUNTER_CHANNELS - 1 ) );
        xprintf_P( PSTR("  piloto enable{true/false},ppr {nn},pwidth {nn}\r\n"));
        xprintf_P( PSTR("         slot {idx} {hhmm} {pout}\r\n"));
        xprintf_P( PSTR("  consigna enable {true|false| hhmm_diurna hhmm_nocturna\r\n"));
        
	} else if (!strcmp_P( strupr(argv[1]), PSTR("RESET"))) {
		xprintf_P( PSTR("-reset\r\n"));
        xprintf_P( PSTR("  memory {soft|hard}\r\n"));
        
    } else if (!strcmp_P( strupr(argv[1]), PSTR("TEST"))) {
		xprintf_P( PSTR("-test\r\n"));  
        
    }  else {
        // HELP GENERAL
        xprintf("Available commands are:\r\n");
        xprintf("-cls\r\n");
        xprintf("-help\r\n");
        xprintf("-status\r\n");
        xprintf("-reset\r\n");
        xprintf("-config...\r\n");
    }
   
	xprintf("Exit help \r\n");

}
//------------------------------------------------------------------------------
static void cmdClsFunction(void)
{
	// ESC [ 2 J
	xprintf("\x1B[2J\0");
}
//------------------------------------------------------------------------------
static void cmdResetFunction(void)
{
    
    FRTOS_CMD_makeArgv();
    
    // Reset memory ??
    if (!strcmp_P( strupr(argv[1]), PSTR("MEMORY"))) {
        
        /*
         * No puedo estar usando la memoria !!!
         */     
        
        if ( xHandle_tkCtl != NULL )
            vTaskSuspend( xHandle_tkSys );
        
        if ( xHandle_tkRS485A != NULL )
            vTaskSuspend( xHandle_tkRS485A );
        
        if ( xHandle_tkWAN != NULL )
            vTaskSuspend( xHandle_tkWAN );
        
        if ( !strcmp_P( strupr(argv[2]), PSTR("SOFT"))) {
			FS_format(false );
		} else if ( !strcmp_P( strupr(argv[2]), PSTR("HARD"))) {
			FS_format(true);
		} else {
			xprintf_P( PSTR("ERROR\r\nUSO: reset memory {hard|soft}\r\n"));
			return;
		}
    }
    
    xprintf("Reset..\r\n");
    reset();
    pv_snprintfP_OK();
}
//------------------------------------------------------------------------------
static void cmdStatusFunction(void)
{

    // https://stackoverflow.com/questions/12844117/printing-defined-constants

fat_s l_fat;

    xprintf("Spymovil %s %s TYPE=%s, VER=%s %s \r\n" , HW_MODELO, FRTOS_VERSION, FW_TYPE, FW_REV, FW_DATE);
 
    // Memoria
    FAT_read(&l_fat);
	xprintf_P( PSTR("FileSystem: blockSize=%d,rcdSize=%d,blocks=%d,wrPtr=%d,rdPtr=%d,count=%d\r\n"),FS_PAGE_SIZE, sizeof(dataRcd_s), FF_MAX_RCDS, l_fat.head,l_fat.tail, l_fat.count );

    xprintf_P(PSTR("Config:\r\n"));
    xprintf_P(PSTR(" date: %s\r\n"), RTC_logprint(FORMAT_LONG));
    xprintf_P(PSTR(" dlgid: %s\r\n"), systemConf.dlgid );
    xprintf_P(PSTR(" signature: %s\r\n"), NVMEE_read_serial() );
    xprintf_P(PSTR(" timerdial=%d\r\n"), systemConf.timerdial);
    xprintf_P(PSTR(" timerpoll=%d\r\n"), systemConf.timerpoll);
    xprintf_P(PSTR(" samples=%d\r\n"), systemConf.samples_count);

    print_pwr_configuration();
    WAN_print_configuration();
	ainputs_print_configuration();
    counters_print_configuration();
    piloto_print_configuration();
    consigna_print_configuration();
    
    xprintf_P(PSTR("Values:\r\n"));
    xprintf_P(PSTR(" Frame: "));
    xprint_dr( get_system_dr());
}
//------------------------------------------------------------------------------
static void cmdReadFunction(void)
{
    
    FRTOS_CMD_makeArgv();
  
    if (! strcmp_P( strupr(argv[1]), PSTR("FC1") ) ) {
        xprintf_P(PSTR("FC1=%d\r\n"), FC1_read() );
        return;
    }

    if (! strcmp_P( strupr(argv[1]), PSTR("FC_ALTA") ) ) {
        xprintf_P(PSTR("FC_alta(1)=%d (1:open,0:close)\r\n"), FC1_read() );
        return;
    }
    
    if (! strcmp_P( strupr(argv[1]), PSTR("FC2") ) ) {
        xprintf_P(PSTR("FC2=%d\r\n"), FC2_read() );
        return;
    }
    
    if (! strcmp_P( strupr(argv[1]), PSTR("FC_BAJA") ) ) {
        xprintf_P(PSTR("FC_baja(2)=%d (1:open,0:close)\r\n"), FC2_read() );
        return;
    }

    // CNT{0,1}
	// read cnt
	if (!strcmp_P( strupr(argv[1]), PSTR("CNT")) ) {
        if ( atoi(argv[2]) == 0 ) {
            xprintf_P(PSTR("CNT0=%d\r\n"), CNT0_pin_read());
            pv_snprintfP_OK();
            return;
        }
        if ( atoi(argv[2]) == 1 ) {
            xprintf_P(PSTR("CNT1=%d\r\n"), CNT1_pin_read());
            pv_snprintfP_OK();
            return;
        }
        pv_snprintfP_ERR();
        return;
	}
    
    // AINPUT
    // read ainput {n}
	if (!strcmp_P( strupr(argv[1]), PSTR("AINPUT"))  ) {
        ainputs_test_read_channel( atoi(argv[2]) ) ? pv_snprintfP_OK(): pv_snprintfP_ERR();
		return;
	}
    
    // EE
	// read ee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("EE")) ) {
        EE_test_read( argv[2], argv[3], argv[4] );
		return;
	}
    
    // RTC
	// read rtc { long | short }
    if (!strcmp_P( strupr(argv[1]), PSTR("RTC")) ) {
        if (!strcmp_P( strupr(argv[2]), PSTR("LONG")) ) {
            RTC_read_time(FORMAT_LONG);
            pv_snprintfP_OK();
            return;
        }
        if (!strcmp_P( strupr(argv[2]), PSTR("SHORT")) ) {
            RTC_read_time(FORMAT_SHORT);
            pv_snprintfP_OK();
            return;
        }
        
        RTC_read_time(FORMAT_LONG);
        pv_snprintfP_OK();
        return;
    }
        
	// RTC SRAM
	// read rtcram address length
	if (!strcmp_P( strupr(argv[1]), PSTR("RTCRAM"))) {
		RTCSRAM_test_read ( argv[2], argv[3] );
		return;
	}    
    
    // INA
	// read ina regName
	if (!strcmp_P( strupr(argv[1]), PSTR("INA"))  ) {
		INA_awake();
		INA_test_read ( argv[2], argv[3] );
		INA_sleep();
		return;
	}
    
    // NVMEE
	// read nvmee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("NVMEE")) ) {
		NVMEE_test_read ( argv[2], argv[3] );
		return;
	}  
    
    // SERIAL
    if (!strcmp_P( strupr(argv[1]), PSTR("SERIAL")) ) {
		xprintf_P( PSTR("uSERIAL=%s\r\n"), NVMEE_read_serial() );
		return;
	}  

    // DEVID
    if (!strcmp_P( strupr(argv[1]), PSTR("DEVID")) ) {
        if ( strstr( NVMEE_read_device_ID(), "1e9843") ) {
            xprintf_P(PSTR("(AVR XMEGA256 A3BU)\r\n"));
        } else 	if ( strstr( NVMEE_read_device_ID(), "1e9842") ) {
            xprintf_P(PSTR("(AVR XMEGA256 A3U)\r\n"));
        } else {
            xprintf_P(PSTR("(AVR)\r\n"));
        }
		return;
	}
    
    // CMD NOT FOUND
	xprintf("ERROR\r\nCMD NOT DEFINED\r\n");
	return;
 
}
//------------------------------------------------------------------------------
static void cmdWriteFunction(void)
{

    FRTOS_CMD_makeArgv();

    // COMMAND
    // write command {true|false}
    if (!strcmp_P( strupr(argv[1]), PSTR("SLEEP")) ) {
        if (!strcmp_P( strupr(argv[2]), PSTR("TRUE")) ) {
            sleeping = true;
            pv_snprintfP_OK();
            return;
        }
        if (!strcmp_P( strupr(argv[2]), PSTR("FALSE")) ) {
            sleeping = false;
            pv_snprintfP_OK();
            return;
        }
        pv_snprintfP_ERR();
        return; 
       
	}
    
    // CONSIGNA
    // write consigna {diurna|nocturna}
    if (!strcmp_P( strupr(argv[1]), PSTR("CONSIGNA")) ) {
        if (!strcmp_P( strupr(argv[2]), PSTR("DIURNA")) ) {
            consigna_set_diurna();
            pv_snprintfP_OK();
            return;
        }
        if (!strcmp_P( strupr(argv[2]), PSTR("NOCTURNA")) ) {
            consigna_set_nocturna();
            pv_snprintfP_OK();
            return;
        }
        pv_snprintfP_ERR();
        return; 
       
	}
    
    // PILOTO
    if ( strcmp_P( strupr(argv[1]), PSTR("PILOTO")) == 0 ) {
		piloto_cmd_set_presion(argv[2]) ? pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}  
    
    // STEPPER
    if (!strcmp_P( strupr(argv[1]), PSTR("STEPPER"))  ) {
        stepper_test( argv[2],argv[3],argv[4],argv[5], argv[6])? pv_snprintfP_OK() : pv_snprintfP_ERR();
        return;
    }
    
    // DRV8814
    if (!strcmp_P( strupr(argv[1]), PSTR("DRV8814"))  ) {
        DRV8814_test(argv[2],argv[3])? pv_snprintfP_OK() : pv_snprintfP_ERR();
        return;
    }
    
    // K1
    if (!strcmp_P( strupr(argv[1]), PSTR("K1")) ) {
        if (!strcmp_P( strupr(argv[2]), PSTR("OPEN")) ) {
            RELE_K1_OPEN();
            pv_snprintfP_OK();
            return;
        }
        if (!strcmp_P( strupr(argv[2]), PSTR("CLOSE")) ) {
            RELE_K1_CLOSE();
            pv_snprintfP_OK();
            return;
        }
        pv_snprintfP_ERR();
        return; 
       
	}

    // RS485A
    // write rs485a {string}
    if ((strcmp_P( strupr(argv[1]), PSTR("RS485A")) == 0) ) {
        xfprintf_P( fdRS485A, PSTR("%s\r\n"), argv[2]);
        pv_snprintfP_OK();
        return;
    }
    
    // write VSENSORS420 on/off
	if (!strcmp_P( strupr(argv[1]), PSTR("VSENSORS420")) ) {
        if (!strcmp_P( strupr(argv[2]), PSTR("ON")) ) {
            SET_VSENSORS420();
            pv_snprintfP_OK();
            return;
        }
        if (!strcmp_P( strupr(argv[2]), PSTR("OFF")) ) {
            CLEAR_VSENSORS420();
            pv_snprintfP_OK();
            return;
        }
        pv_snprintfP_ERR();
        return;
	}

   	// EE
	// write ee pos string
	if ((strcmp_P( strupr(argv[1]), PSTR("EE")) == 0) ) {
		( EE_test_write ( argv[2], argv[3], argv[4] ) > 0)?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

    // RTC
	// write rtc YYMMDDhhmm
	if ( strcmp_P( strupr(argv[1]), PSTR("RTC")) == 0 ) {
		( RTC_write_time( argv[2]) > 0)?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}
    
	// RTC SRAM
	// write rtcram pos string
	if ( (strcmp_P( strupr(argv[1]), PSTR("RTCRAM")) == 0)  ) {
		( RTCSRAM_test_write ( argv[2], argv[3] ) > 0)?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}
    
    // INA
	// write ina rconfValue
	// Solo escribimos el registro 0 de configuracion.
	if ((strcmp_P( strupr(argv[1]), PSTR("INA")) == 0) ) {
        INA_awake();
		( INA_test_write ( argv[2], argv[3] ) > 0)?  pv_snprintfP_OK() : pv_snprintfP_ERR();
        INA_sleep();
		return;
	}
    
    // NVMEE
	// write nvmee pos string
	if ( (strcmp_P( strupr(argv[1]), PSTR("NVMEE")) == 0)) {
		NVMEE_test_write ( argv[2], argv[3] );
		pv_snprintfP_OK();
		return;
	}
    
    // CMD NOT FOUND
	xprintf("ERROR\r\nCMD NOT DEFINED\r\n");
	return;
 
}
//------------------------------------------------------------------------------
static void cmdConfigFunction(void)
{
    
    FRTOS_CMD_makeArgv();
    
    // CONSIGNA
    // config consigna {true|false| hhmm_diurna hhmm_nocturna
    if (!strcmp_P( strupr(argv[1]), PSTR("CONSIGNA"))) {
        consigna_config( argv[2], argv[3], argv[4]) ? pv_snprintfP_OK() : pv_snprintfP_ERR();
        return;
    }
    
    // DLGID
	if (!strcmp_P( strupr(argv[1]), PSTR("DLGID"))) {
		if ( argv[2] != NULL ) {
            memset(systemConf.dlgid,'\0', sizeof(systemConf.dlgid) );
			memcpy(systemConf.dlgid, argv[2], sizeof(systemConf.dlgid));
			systemConf.dlgid[DLGID_LENGTH - 1] = '\0';
			pv_snprintfP_OK();
               return;
		}
		pv_snprintfP_ERR();
		return;
	}

    // DEFAULT
	// config default
	if (!strcmp_P( strupr(argv[1]), PSTR("DEFAULT"))) {
		config_default();
		pv_snprintfP_OK();
		return;
	}

	// SAVE
	// config save
	if (!strcmp_P( strupr(argv[1]), PSTR("SAVE"))) {       
		( save_config_in_NVM() == true )?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}
    
    // LOAD
	// config load
	if (!strcmp_P( strupr(argv[1]), PSTR("LOAD"))) {
		load_config_from_NVM();
		pv_snprintfP_OK();
		return;
	}

    // TIMERPOLL
    // config timerpoll val
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERPOLL")) ) {
        config_timerpoll(argv[2]) ? pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}
    
    // TIMERDIAL
    // config timerdial val
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERDIAL")) ) {
		config_timerdial(argv[2]) ? pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}
    
    // AINPUT
	// ainput {0..%d} enable aname imin imax mmin mmax offset
	if (!strcmp_P( strupr(argv[1]), PSTR("AINPUT")) ) {
		ainputs_config_channel ( atoi(argv[2]), argv[3], argv[4], argv[5], argv[6], argv[7], argv[8], argv[9]);
        pv_snprintfP_OK();
		return;
	}
        
    // COUNTER
    // counter {0..%d} enable cname magPP modo(PULSO/CAUDAL),rbsize
	if (!strcmp_P( strupr(argv[1]), PSTR("COUNTER")) ) {
        counters_config_channel( atoi(argv[2]), argv[3], argv[4], argv[5], argv[6], argv[7]  );
        pv_snprintfP_OK();
		return;
	}
    
    // DEBUG
    // config debug (ainput, counter, comms) (true,false)
    if (!strcmp_P( strupr(argv[1]), PSTR("DEBUG")) ) {
        config_debug( argv[2], argv[3]) ? pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
    }
    
    // CMD NOT FOUND
	xprintf("ERROR\r\nCMD NOT DEFINED\r\n\0");
	return;
 
}
//------------------------------------------------------------------------------
static void pv_snprintfP_ERR(void)
{
	xprintf("error\r\n\0");
}
//------------------------------------------------------------------------------
static void pv_snprintfP_OK(void )
{
	xprintf("ok\r\n\0");
}
//------------------------------------------------------------------------------
