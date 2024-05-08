/*
 * i2c.c
 *
 *  Created on: 18/10/2015
 *      Author: pablo
 *
 *  Modificadas 20/05/2017 para adecuarlas al SP6K basado en xmega256
 *  El bus se trabaja en modo poleado.
 *  La velocidad se fija en 100Khz.
 *  Se utiliza el bus implementado en el puerto E.
 *
 *  Cuando el sistema arranca, el estado del bus es UNKNOWN. Se puede forzar a IDLE escribiendo
 *  los bits correspondientes del STATUS.
 *  Para pasarlo de nuevo a UNKNOWN solo un reset o deshabilitando el TWI.
 *  Para sacarlo de BUSY hay que generar un STOP
 *
 *  Al escribir en ADDR,DATA o STATUS se borran las flags. Esto hay que hacerlo de ultimo
 *  ya que luego de c/operacion, mientras las flags esten prendidas el master mantiene el bus.
 *
 *  CUANDO EL BUS QUEDA MAL NO TENGO FORMA DE SALIR QUE NO SEA APAGANDO Y PRENDIENDO !!!
 */


#include "drv_i2c_spx.h"
#include "xprintf.h"

bool pv_i2c_send_Address_packet( uint8_t slaveAddress, uint8_t directionBit);
bool pv_i2c_send_Data_packet( uint8_t *dataBuffer, size_t length );
bool pv_i2c_set_bus_idle( void );
bool pv_i2c_rcvd_Data_packet( uint8_t *dataBuffer, size_t length );
bool pv_i2c_read_byte( uint8_t response_flag, char *rxByte );
bool pv_i2c_waitForComplete(void);

bool drv_i2c_debug_flag;

//------------------------------------------------------------------------------
void drv_I2C_init(void)
{
uint16_t bitrate_div = 0;
uint16_t bitrateKHz = 100;

	// El pin PE1(SCK) lo pongo como output.( master genera el reloj )
	// No es necesario pero si se tranca el bus, debo clockearlo
	CONFIG_SCL();
    
	// calculate bitrate division
	bitrate_div = ((F_CPU / (2 * ( bitrateKHz * 1000) )) - 5);
	TWIE.MASTER.BAUD = (uint8_t) bitrate_div;
	TWIE.MASTER.CTRLA = 0x00;
	TWIE.MASTER.CTRLA |= ( 1<<TWI_MASTER_ENABLE_bp);	// Enable TWI
	//TWIE.MASTER.CTRLA = TWI_MASTER_INTLVL_LO_gc | TWI_MASTER_RIEN_bm | TWI_MASTER_WIEN_bm | TWI_MASTER_ENABLE_bm;
	TWIE.MASTER.CTRLB  = 0;
	TWIE.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;
	//
}
//------------------------------------------------------------------------------
void drv_I2C_config_debug( bool debug_flag)
{
    drv_i2c_debug_flag = debug_flag;  
}
//------------------------------------------------------------------------------
void drv_I2C_reset(void)
{

uint8_t i = 0;

	// https://stackoverflow.com/questions/5497488/failed-twi-transaction-after-sleep-on-xmega
	// There is a common problem on I2C/TWI where the internal state machine gets stuck in an
	// intermediate state if a transaction is not completed fully. The slave then does not respond
	// correctly when addressed on the next transaction. This commonly happens when the master
	// is reset or stops outputting the SCK signal part way through the read or write.
	// A solution is to toggle the SCK line manually 8 or 9 times before starting any data
	// transactions so the that the internal state machines in the slaves are all reset to the
	///start of transfer point and they are all then looking for their address byte

	// La forma de resetear el bus es deshabilitando el TWI
	TWIE.MASTER.CTRLA &= ~( 1<<TWI_MASTER_ENABLE_bp);	// Disable TWI
	vTaskDelay( 5 );

	// Clockeo el SCK varias veces para destrabar a los slaves
	CONFIG_SCL();
	for (i=0; i<10;i++) {
		SET_SCL();
		vTaskDelay( 1 );
		CLEAR_SCL();
		vTaskDelay( 1 );
	}
	// Lo dejo en reposo alto
	SET_SCL();

	TWIE.MASTER.CTRLA |= ( 1<<TWI_MASTER_ENABLE_bp);	// Enable TWI

//	TWIE.MASTER.CTRLC =  TWI_MASTER_CMD_REPSTART_gc;	// Send START

	// El status esta indicando errores. Debo limpiarlos antes de usar la interface.
	if ( TWIE.MASTER.STATUS & TWI_MASTER_ARBLOST_bm ) {
		TWIE.MASTER.STATUS |= TWI_MASTER_ARBLOST_bm;
	}
	if ( TWIE.MASTER.STATUS & TWI_MASTER_BUSERR_bm ) {
		TWIE.MASTER.STATUS |= TWI_MASTER_BUSERR_bm;
	}
	if ( TWIE.MASTER.STATUS & TWI_MASTER_WIF_bm ) {
		TWIE.MASTER.STATUS |= TWI_MASTER_WIF_bm;
	}
	if ( TWIE.MASTER.STATUS & TWI_MASTER_RIF_bm ) {
		TWIE.MASTER.STATUS |= TWI_MASTER_RIF_bm;
	}

	TWIE.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;	// Pongo el status en 01 ( idle )
	vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

}
//------------------------------------------------------------------------------
int16_t drv_I2C_master_write ( const uint8_t devAddress, 
        const uint16_t dataAddress, 
        const uint8_t dataAddress_length, 
        char *pvBuffer, 
        size_t xBytes )
{

bool retV = false;
int xReturn = -1;
uint8_t packet[3];

    if ( drv_i2c_debug_flag ) {
		xprintf( PSTR("drv_I2C_master_write: START\r\n"));
        xprintf( PSTR("  devAddr=%02X\r\n"),devAddress);
        xprintf( PSTR("  dataAddr=%04X\r\n"),dataAddress );
        xprintf( PSTR("  dataAddrLength=%d\r\n"),dataAddress_length );
        xprintf( PSTR("  bytes2write=%d\r\n"), xBytes );
	}

	// Paso 1: PONER EL BUS EN CONDICIONES
    // Fuerzo al bus al estado idle.
	if ( ! pv_i2c_set_bus_idle() ) {
		drv_I2C_reset();
        goto i2c_quit;
	}

	// Pass2: Mando un START y el SLAVE_ADDRESS + WR (SLA_W).
	if ( ! pv_i2c_send_Address_packet( devAddress, I2C_DIRECTION_BIT_WRITE) ) {
		goto i2c_quit;
	}

	// Pass3: DATA_ADDRESS Mando la direccion interna del slave donde voy a escribir.
    if ( dataAddress_length == 2 ) {
        packet[0] = (dataAddress >> 8);         // HIGH_BYTE
        packet[1] = (dataAddress & 0x00FF );    // LOW_BYTE
    }  else {
        packet[0] = (dataAddress & 0x00FF );    // LOW_BYTE
    }
	if ( ! pv_i2c_send_Data_packet ( packet, dataAddress_length ))  goto i2c_quit;

    // Paso 4: DATA
	// Mando el buffer de datos. Debo recibir 0x28 (DATA_ACK) en c/u
	if ( ! pv_i2c_send_Data_packet ((uint8_t *)pvBuffer, xBytes ))  goto i2c_quit;
    
	xReturn = xBytes;
	retV = true;
    
i2c_quit:

	// Pass4) STOP
	TWIE.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;

    if ( drv_i2c_debug_flag ) {
        xprintf(PSTR("STOP\r\n"));
    }

	// En caso de error libero la interface forzando el bus al estado IDLE
	if ( !retV )
		drv_I2C_reset();

	return(xReturn);
}
//------------------------------------------------------------------------------
int drv_I2C_master_read ( const uint8_t devAddress,
					const uint16_t dataAddress,
					const uint8_t dataAddress_length,
					char *pvBuffer,
					size_t xBytes )
{
	// En el caso del ADC, el read no lleva la parte de mandar la SLA+W. !!!!!

bool retV = false;
int xReturn = -1;
uint8_t packet[3];

    if ( drv_i2c_debug_flag ) {
		xprintf( "drv_I2C_master_write: START\r\n");
        xprintf( "  devAddr=%02X\r\n",devAddress);
        xprintf( "  dataAddr=%04X\r\n",dataAddress );
        xprintf( "  dataAddrLength=%d\r\n",dataAddress_length );
        xprintf( "  bytes2read=%d\r\n", xBytes );
	}

	// Paso 1: PONER EL BUS EN CONDICIONES
    // Fuerzo al bus al estado idle.
	if ( ! pv_i2c_set_bus_idle() ) {
		drv_I2C_reset();
		goto i2c_quit;
	}

	// Pass1: Mando un START y el SLAVE_ADDRESS (SLA_W).
	if ( ! pv_i2c_send_Address_packet( devAddress, I2C_DIRECTION_BIT_WRITE) ) goto i2c_quit;

	// Pass2: DATA_ADDRESS Mando la direccion interna del slave donde voy a escribir.
    if ( dataAddress_length == 2 ) {
        packet[0] = (dataAddress >> 8);         // HIGH_BYTE
        packet[1] = (dataAddress & 0x00FF );    // LOW_BYTE
    }  else {
        packet[0] = (dataAddress & 0x00FF );    // LOW_BYTE
    }  
	if ( ! pv_i2c_send_Data_packet ( packet, dataAddress_length ))  goto i2c_quit;


	// Pass3: Mando un START y el SLAVE_ADDRESS (SLA_R).
    //        REPEATED START + DEVICE_ADDRESS + RD
	if ( ! pv_i2c_send_Address_packet( devAddress, I2C_DIRECTION_BIT_READ) ) goto i2c_quit;

	// Pass4: Leo todos los bytes requeridos y respondo a c/u con ACK.
	if ( ! pv_i2c_rcvd_Data_packet( (uint8_t *)pvBuffer, xBytes )) goto i2c_quit;

    // Pass5: STOP
    // No lleva porque ya mande un NACK. Si lo pongo el proximo read me va a dar error !!!
	//TWIE.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
    
    if ( drv_i2c_debug_flag ) {
        xprintf("STOP\r\n");
    }
    
	xReturn = xBytes;
	retV = true;

i2c_quit:

	// En caso de error libero la interface forzando el bus al estado IDLE
	if ( !retV )
		drv_I2C_reset();

	return(xReturn);

}
//------------------------------------------------------------------------------
bool pv_i2c_send_Address_packet( uint8_t slaveAddress, uint8_t directionBit)
{

	// Pass1: Mando un START y el SLAVE_ADDRESS (SLA_W/R)
	// El start se genera automaticamente al escribir en el reg MASTER.ADDR.
	// Esto tambien resetea todas las flags.
	// La salida correcta es con STATUS = 0x62.
	// El escribir el ADDR borra todas las flags.

char txbyte;	// (SLA_W/R) Send slave address
bool ret_code = false;

	if ( directionBit == I2C_DIRECTION_BIT_WRITE ) {
		txbyte = slaveAddress & ~0x01;
	} else {
		txbyte = slaveAddress | 0x01;
	}

    // Esto genera un START y se envia el devAddress
    // Ademas borra cualquier BUSERR que hubiese.
	TWIE.MASTER.ADDR = txbyte;
	if ( ! pv_i2c_waitForComplete() ) goto i2c_exit;

	// Primero evaluo no tener errores.
	if ( ( TWIE.MASTER.STATUS & TWI_MASTER_ARBLOST_bm) != 0 ) goto i2c_exit;
	if ( ( TWIE.MASTER.STATUS & TWI_MASTER_BUSERR_bm) != 0 ) goto i2c_exit;

	// ACK o NACK ?
	if ( ( TWIE.MASTER.STATUS & TWI_MASTER_RXACK_bm) != 0 ) {
		// NACK
        xprintf("pvI2C_write_slave_address: NACK status=0x%02x\r\n",  TWIE.MASTER.STATUS );
		goto i2c_exit;
	} else {
		// ACK
		ret_code = true;
		goto i2c_exit;
	}

i2c_exit:

    if ( ret_code == false) {
        xprintf("pv_i2c_send_Address_packet ERROR: ST=0x%02x\r\n", TWIE.MASTER.STATUS );
    }

    // DEBUG
    if ( drv_i2c_debug_flag ) {
        xprintf("ADDR=0x%02x, ST=0x%02x\r\n", txbyte, TWIE.MASTER.STATUS );
    }

	return(ret_code);

}
//------------------------------------------------------------------------------
bool pv_i2c_send_Data_packet( uint8_t *dataBuffer, size_t length )
{

uint8_t bytesWritten;
uint8_t txbyte;
bool retS = false;
    
    // DEBUG
    if ( drv_i2c_debug_flag ) {
        xprintf_P(PSTR("SEND DATAPKT:"));
    }

	// No hay datos para enviar: dummy write.
	if ( length == 0 )
		return(true);

	// Mando el buffer de datos. Debo recibir 0x28 (DATA_ACK) en c/u
	for ( bytesWritten=0; bytesWritten < length; bytesWritten++ ) {
		txbyte = *dataBuffer++;
		TWIE.MASTER.DATA = txbyte;		// send byte
        
        // DEBUG
        if ( drv_i2c_debug_flag ) {
            if ( (bytesWritten % 8) == 0 ) {
                xprintf_P(PSTR("\r\n%02d: "), bytesWritten);
            }
            
            xprintf("[0x%02x,ST=0x%02x] ", txbyte, TWIE.MASTER.STATUS );
        }

		if ( ! pv_i2c_waitForComplete() ) {
            xprintf("pv_I2C_send_data: ERROR (status=%d)\r\n", TWIE.MASTER.STATUS );
            goto i2c_exit;
        }

		// NACK ?
		if ( (TWIE.MASTER.STATUS & TWI_MASTER_RXACK_bm) != 0 )  {
            xprintf("pv_I2C_send_data: NACK (status=%d)\r\n", TWIE.MASTER.STATUS );
            goto i2c_exit;
        }
        
	}

	// Envie todo el buffer
    retS = true;

i2c_exit:

    // DEBUG
    if ( drv_i2c_debug_flag ) {
         xprintf_P(PSTR("\r\n"));   
    }
	return(retS);

}
//------------------------------------------------------------------------------
bool pv_i2c_set_bus_idle( void )
{
    // Intenta hasta 3 veces poner el bus en estado idle
	// Para comenzar una operacion el bus debe estar en IDLE o OWENED.
	// Intento pasarlo a IDLE hasta 3 veces antes de abortar, esperando 100ms
	// entre c/intento.

uint8_t	reintentos = I2C_MAXTRIES;

	while ( reintentos-- > 0 ) {

		// Los bits CLKHOLD y RXACK son solo de read por eso la mascara !!!
        if (  ( ( TWIE.MASTER.STATUS & TWI_MASTER_BUSSTATE_gm ) == TWI_MASTER_BUSSTATE_IDLE_gc ) ||
				( ( TWIE.MASTER.STATUS & TWI_MASTER_BUSSTATE_gm ) == TWI_MASTER_BUSSTATE_OWNER_gc ) ) {
			return(true);
            
		} else {
			// El status esta indicando errores. Debo limpiarlos antes de usar la interface.
			if ( (TWIE.MASTER.STATUS & TWI_MASTER_ARBLOST_bm) != 0 ) {
				TWIE.MASTER.STATUS |= TWI_MASTER_ARBLOST_bm;
			}
			if ( (TWIE.MASTER.STATUS & TWI_MASTER_BUSERR_bm) != 0 ) {
				TWIE.MASTER.STATUS |= TWI_MASTER_BUSERR_bm;
			}
			if ( (TWIE.MASTER.STATUS & TWI_MASTER_WIF_bm) != 0 ) {
				TWIE.MASTER.STATUS |= TWI_MASTER_WIF_bm;
			}
			if ( (TWIE.MASTER.STATUS & TWI_MASTER_RIF_bm) != 0 ) {
				TWIE.MASTER.STATUS |= TWI_MASTER_RIF_bm;
			}

			TWIE.MASTER.STATUS |= TWI_MASTER_BUSSTATE_IDLE_gc;	// Pongo el status en 01 ( idle )
			vTaskDelay( ( TickType_t)( 10 /  portTICK_PERIOD_MS ) );
		}
	}

	// No pude pasarlo a IDLE: Error !!!
    xprintf("pv_i2c_set_bus_idle ERROR!!: status=0x%02x\r\n\0", TWIE.MASTER.STATUS );

	return(false);
}
//------------------------------------------------------------------------------
bool pv_i2c_rcvd_Data_packet( uint8_t *dataBuffer, size_t length )
{

char rxByte;
bool retS = false;
uint8_t i;

    // DEBUG
    if ( drv_i2c_debug_flag ) {
        xprintf_P(PSTR("RCVD DATAPKT:"));
    }

    i = 0;
	while(length > 1) {
		if ( ! pv_i2c_read_byte( ACK, &rxByte) ) goto i2c_quit;
		*dataBuffer++ = rxByte;
		// decrement length
		length--;
        
        // DEBUG
        if ( drv_i2c_debug_flag ) {
            if ( (i % 8) == 0 ) {
                xprintf_P(PSTR("\r\n%02d: "), i);
            }
            xprintf("[0x%02x,ST=0x%02x] ", rxByte, TWIE.MASTER.STATUS );
        }
        i++;
	}

	// accept receive data and nack it (last-byte signal)
	// Ultimo byte.
	if ( ! pv_i2c_read_byte( NACK, &rxByte) ) goto i2c_quit;
	*dataBuffer++ = rxByte;
    // DEBUG
    if ( drv_i2c_debug_flag ) {
        xprintf("[0x%02x,ST=0x%02x]", rxByte, TWIE.MASTER.STATUS );
    }

	retS = true;

i2c_quit:

    if ( retS == false) {
        xprintf("pv_i2c_rcvd_Data_packet ERROR: ST=0x%02x\r\n", TWIE.MASTER.STATUS );
    }

    // DEBUG
    if ( drv_i2c_debug_flag ) {
         xprintf_P(PSTR("\r\n"));
    }

	return(retS);
}
//------------------------------------------------------------------------------
bool pv_i2c_read_byte( uint8_t response_flag, char *rxByte )
{

bool ret_code = false;
uint8_t currentStatus = 0;
	
	// Espero 1 byte enviado por el slave
	if ( ! pv_i2c_waitForComplete() ) {
        xprintf(" pvI2C_read_byte TO: st=0x%02x\r\n", TWIE.MASTER.STATUS );
        goto i2c_exit;
    }
	currentStatus = TWIE.MASTER.STATUS;

	*rxByte = TWIE.MASTER.DATA;
    
	if (response_flag == ACK) {
        TWIE.MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;                   // Send ACK y Mas bytes     
	} else { 
        TWIE.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc; // NACK Ultimo byte + STOP
	}

	// Primero evaluo no tener errores.
	if ( (currentStatus & TWI_MASTER_ARBLOST_bm) != 0 ) {
        xprintf("pvI2C_read_byte ARBLOST: 0x%02x, st=0x%02x\r\n",*rxByte,currentStatus );
        goto i2c_exit;
    }
	if ( (currentStatus & TWI_MASTER_BUSERR_bm) != 0 ) {
        xprintf("pvI2C_read_byte BUSERR: 0x%02x, st=0x%02x\r\n",*rxByte,currentStatus );
        goto i2c_exit;
    }

	ret_code = true;
   

i2c_exit:

	return(ret_code);
}
//------------------------------------------------------------------------------
bool pv_i2c_waitForComplete(void)
{

uint8_t ticks_to_wait = 30;		// 3 ticks ( 30ms es el maximo tiempo que espero )
bool retS = false;

	// wait for i2c interface to complete write operation ( MASTER WRITE INTERRUPT )
	while ( ticks_to_wait-- > 0 ) {
		if ( ( ( TWIE.MASTER.STATUS & TWI_MASTER_WIF_bm) != 0 ) || ( ( TWIE.MASTER.STATUS & TWI_MASTER_RIF_bm) != 0 ) ) {
			retS = true;
			goto quit;
		}
		vTaskDelay( ( TickType_t)( 5 ) );
	}

	retS = false;
    xprintf("pv_i2c_waitForComplete ERROR: ST=0x%02x\r\n", TWIE.MASTER.STATUS );


quit:

	return(retS);

}
//------------------------------------------------------------------------------
