/*
 * l_i2c.c
 *
 *  Created on: 26 de mar. de 2018
 *      Author: pablo
 */

#include "i2c.h"

char buffer[10] = { 0 };

bool f_debug_TWI0;

//------------------------------------------------------------------------------
void I2C_init(void)
{
    f_debug_TWI0 = false;

}
//------------------------------------------------------------------------------
int16_t I2C_write ( int fdTWI, uint8_t devAddress, 
        uint16_t dataAddress, 
        uint8_t dataAddress_length, 
        char *data, 
        uint8_t data_length )
{
	

int16_t xReturn = 0U;
uint8_t i2c_error_code = 0;
   

    if ( f_debug_TWI0  ) 
        xprintf_P(PSTR("I2C_write: devAddress=0x%02X, dataAddress=0x%02X, dataAddressLength=%d, dataLength=%d\r\n"), devAddress,dataAddress,dataAddress_length,data_length);

    frtos_ioctl( fdTWI, ioctl_OBTAIN_BUS_SEMPH, NULL);
    
    // Prendo el debug
    if ( f_debug_TWI0  ) {
        frtos_ioctl( fdTWI, ioctl_I2C_SET_DEBUG, NULL);
        //xprintf_P(PSTR("I2C_write: f_debug_TWI0=True\r\n"));
    }
    
	// 1) Indicamos el periferico i2c en el cual queremos escribir ( variable de 8 bits !!! )
	frtos_ioctl( fdTWI, ioctl_I2C_SET_DEVADDRESS, &devAddress );
    if ( f_debug_TWI0  ) 
        xprintf_P(PSTR("I2C_write: SET_DEVADDRESS=0x%02X\r\n"),devAddress);

 	// 2) Indicamos al direccion interna del chip donde comenzar a escribir
	frtos_ioctl( fdTWI, ioctl_I2C_SET_DATAADDRESS, &dataAddress );
    if ( f_debug_TWI0  ) 
        xprintf_P(PSTR("I2C_write: SET_DATAADDRESS=0x%02X\r\n"),dataAddress);
    
    frtos_ioctl( fdTWI, ioctl_I2C_SET_DATAADDRESSLENGTH, &dataAddress_length );
    if ( f_debug_TWI0  ) 
        xprintf_P(PSTR("I2C_write: SET_DATAADDRESSLENGTH=0x%02X\r\n"),dataAddress_length);
    
	// 3) Por ultimo escribimos. No controlo fronteras.
	xReturn = frtos_write( fdTWI, data, data_length);

 	// 4) Controlo errores
	i2c_error_code = frtos_ioctl( fdTWI, ioctl_I2C_GET_LAST_ERROR, NULL );
	if (i2c_error_code != I2C_OK ) {
        xprintf_P(PSTR("I2C_write: ERROR [%d]\r\n"), i2c_error_code);
		xReturn = -1;
	}
    
    // Apago el debug
    if ( f_debug_TWI0 ) 
        frtos_ioctl( fdTWI, ioctl_I2C_CLEAR_DEBUG, NULL);
    
    frtos_ioctl( fdTWI, ioctl_RELEASE_BUS_SEMPH, NULL);
    
	return(xReturn);
}
//------------------------------------------------------------------------------
int16_t I2C_read  ( int fdTWI, uint8_t devAddress, 
        uint16_t dataAddress, 
        uint8_t dataAddress_length, 
        char *data, 
        uint8_t data_length )
{

	// Implementa solo la parte de lectura del ciclo.
	// La primera parte que es solo escribir la direccion de donde leer la hacemos
	// con I2C_write_R1. ( Dummy Write )

int16_t xReturn = 0U;
uint8_t i2c_error_code = 0;

    frtos_ioctl( fdTWI, ioctl_OBTAIN_BUS_SEMPH, NULL);

        // Prendo el debug
    if ( f_debug_TWI0 && ( fdTWI == fdI2C0) ) 
        frtos_ioctl( fdTWI, ioctl_I2C_SET_DEBUG, NULL);
    
	// 1) Indicamos el periferico i2c en el cual queremos escribir ( variable de 8 bits !!! )
	frtos_ioctl( fdTWI, ioctl_I2C_SET_DEVADDRESS, &devAddress );

	// 2) Indicamos al direccion interna del chip donde comenzar a escribir
	frtos_ioctl( fdTWI, ioctl_I2C_SET_DATAADDRESS, &dataAddress );
	frtos_ioctl( fdTWI, ioctl_I2C_SET_DATAADDRESSLENGTH, &dataAddress_length );

	// 3) Leemos. No controlo fronteras.
	xReturn = frtos_read( fdTWI, data, data_length);

	// 4) Controlo errores.
	i2c_error_code = frtos_ioctl( fdTWI, ioctl_I2C_GET_LAST_ERROR, NULL );
	if (i2c_error_code != I2C_OK ) {
        xprintf_P(PSTR("I2C_write: ERROR [%d]\r\n"), i2c_error_code);
		xReturn = -1;
	}

	if (xReturn != data_length ) {
        xprintf_P(PSTR("I2C_write: ERROR dlength[%d,%d]\r\n"), xReturn, data_length);
		xReturn = -1;
	}

        // Apago el debug
    if ( f_debug_TWI0 && ( fdTWI == fdI2C0) ) 
        frtos_ioctl( fdTWI, ioctl_I2C_CLEAR_DEBUG, NULL);
    
    frtos_ioctl( fdTWI, ioctl_RELEASE_BUS_SEMPH, NULL);
    
	return(xReturn);

}
//------------------------------------------------------------------------------
void I2C_set_debug( uint8_t busId )
{
    if ( busId == 0)
        f_debug_TWI0 = true;
}
//------------------------------------------------------------------------------
void I2C_clear_debug( uint8_t busId )
{
    if ( busId == 0)
        f_debug_TWI0 = false;
}
//------------------------------------------------------------------------------
