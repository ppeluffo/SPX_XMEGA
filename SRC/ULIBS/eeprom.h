/*
 * ee_sp5K.h
 *
 *  Created on: 26/10/2015
 *      Author: pablo
 */

#ifndef SRC_SPXR7_LIBS_L_EEPROM_H_
#define SRC_SPXR7_LIBS_L_EEPROM_H_

#include "i2c.h"
#include "xprintf.h"

#define EE_DEBUG_ON     true
#define EE_DEBUG_OFF    false
//------------------------------------------------------------------------------------
// La memoria EE M24M02 es de 1024 paginas de 256 bytes o sea de 256Kbytes.
// Se accede con 16 bits ( 2 bytes de direcciones ) por lo que con los bit A16 y A17
// se indican en la direccion de la memoria en el bus I2C
// EEADDRESS = 0xA | 0 A17 A16 r/w
// Esto lo controlamos en I2C.C

//--------------------------------------------------------------------------------
// API START

int16_t EE_test_write( char *addr, char *str, char *debug );
int16_t EE_test_read( char *addr, char *size, char *debug );

int16_t EE_read( uint16_t rdAddress, char *data, uint8_t length, bool debug );
int16_t EE_write( uint16_t wrAddress, char *data, uint8_t length, bool debug );

// API END
//--------------------------------------------------------------------------------

#endif /* SRC_SPXR7_LIBS_L_EEPROM_H_ */
