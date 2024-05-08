/*
 * l_nvm.c
 *
 *  Created on: 18 feb. 2019
 *      Author: pablo
 */

#include "nvmee.h"

bool device_id_ok = false;

char nvmid_str[32] = { 0 };
char nvm_device_id_str[24] = { 0 };

char nvm_str_buffer[38];

//------------------------------------------------------------------------------
char *NVMEE_read_serial( void )
{
	// El signature lo leo una sola vez.
	// Luego, como lo tengo en la memoria, no lo leo mas.
    
static bool signature_ok = false;
struct nvm_device_serial xmega_id;
uint16_t op_code = NVM_READ_SERIAL;

	if ( ! signature_ok ) {
        frtos_ioctl( fdNVM, ioctl_OBTAIN_BUS_SEMPH, NULL);
        frtos_ioctl( fdNVM, ioctl_NVM_SET_OPERATION, &op_code);
        frtos_read( fdNVM, (void *)&xmega_id, 0);
        frtos_ioctl( fdNVM, ioctl_RELEASE_BUS_SEMPH, NULL);
        //
		signature_ok = true;
		snprintf( nvmid_str, 32 ,"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",xmega_id.lotnum0,xmega_id.lotnum1,xmega_id.lotnum2,xmega_id.lotnum3,xmega_id.lotnum4,xmega_id.lotnum5,xmega_id.wafnum,xmega_id.coordx0,xmega_id.coordx1,xmega_id.coordy0,xmega_id.coordy1  );
	}

	return( nvmid_str );
}
//------------------------------------------------------------------------------
char *NVMEE_read_device_ID( void )
{
	// El signature lo leo una sola vez.
	// Luego, como lo tengo en la memoria, no lo leo mas.

struct nvm_device_id xmega_device_id;
uint16_t op_code = NVM_READ_ID;

	if ( ! device_id_ok ) {
        frtos_ioctl( fdNVM, ioctl_OBTAIN_BUS_SEMPH, NULL);
        frtos_ioctl( fdNVM, ioctl_NVM_SET_OPERATION, &op_code);
        frtos_read( fdNVM, (void *)&xmega_device_id, 0);
        frtos_ioctl( fdNVM, ioctl_RELEASE_BUS_SEMPH, NULL);
        //
		device_id_ok = true;
		snprintf( nvm_device_id_str, 24 ,"%02x%02x%02x",xmega_device_id.devid0,xmega_device_id.devid1,xmega_device_id.devid2);
	}
	return( nvm_device_id_str );
}
//------------------------------------------------------------------------------
int16_t NVMEE_read ( uint16_t dataAddress, char *data, uint8_t data_length )
{
        
uint16_t addr = dataAddress;
int16_t xReturn;
uint16_t op_code = NVM_READ_BUFFER;

    frtos_ioctl( fdNVM, ioctl_OBTAIN_BUS_SEMPH, NULL);
    frtos_ioctl( fdNVM, ioctl_NVM_SET_EEADDRESS, &addr);
    frtos_ioctl( fdNVM, ioctl_NVM_SET_OPERATION, &op_code);
    xReturn = frtos_read( fdNVM, data, data_length);
    frtos_ioctl( fdNVM, ioctl_RELEASE_BUS_SEMPH, NULL); 
    return (xReturn);
}
//------------------------------------------------------------------------------
int16_t NVMEE_write ( uint16_t dataAddress, char *data, uint8_t data_length )
{
    
uint16_t addr = dataAddress;
int16_t xReturn;

    frtos_ioctl( fdNVM, ioctl_OBTAIN_BUS_SEMPH, NULL);
    frtos_ioctl( fdNVM, ioctl_NVM_SET_EEADDRESS, &addr);
    frtos_ioctl( fdNVM, NVM_WRITE_BUFFER, NULL);
    xReturn = frtos_write( fdNVM, data, data_length);
    frtos_ioctl( fdNVM, ioctl_RELEASE_BUS_SEMPH, NULL); 
    return (xReturn);   
}
//------------------------------------------------------------------------------
void NVMEE_test_read( char *addr, char *size )
{
	// Funcion de testing de la EEPROM interna del micro xmega
	// Lee de una direccion de la memoria una cantiad de bytes y los imprime
	// parametros: *addr > puntero char a la posicion de inicio de lectura
	//             *size >  puntero char al largo de bytes a leer
	// retorna: -1 error
	//			nro.de bytes escritos

char buffer[32] = { 0 };
int length = (uint8_t)(atoi( size));

    NVMEE_read  ( (uint16_t)(atoi(addr)), buffer, length );
	buffer[length] = '\0';
	xprintf_P( PSTR( "NVMEE RD: %s\r\n"),buffer);

}
//------------------------------------------------------------------------------
void NVMEE_test_write( char *addr, char *str )
{
	// Funcion de testing de la EEPROM interna del micro xmega
	// Escribe en una direccion de memoria un string
	// parametros: *addr > puntero char a la posicion de inicio de escritura
	//             *str >  puntero char al texto a escribir
	// retorna: -1 error
	//			nro.de bytes escritos

	// Calculamos el largo del texto a escribir en la eeprom.

uint8_t length = 0;
char *p = NULL;

	p = str;
	while (*p != 0) {
		p++;
		length++;
	}

    NVMEE_write ( (uint16_t)(atoi(addr)), str, length );
	xprintf_P( PSTR( "NVMEE WR %d bytes\r\n\0"),length);
	return;


}
//------------------------------------------------------------------------------
