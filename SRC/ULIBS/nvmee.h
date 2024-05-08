/*
 * l_nvm.h
 *
 *  Created on: 18 feb. 2019
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_NVM_L_NVM_H_
#define SRC_SPX_LIBS_L_NVM_L_NVM_H_


#include "frtos-io.h"
#include "stdio.h"
#include "xprintf.h"

typedef struct {
	union {
		struct {
			uint8_t devid0;
			uint8_t devid1;
			uint8_t devid2;
			uint8_t sernum0;
            uint8_t sernum1;
            uint8_t sernum2;
            uint8_t sernum3;
            uint8_t sernum4;
            uint8_t sernum5;
            uint8_t sernum6;
            uint8_t sernum7;
            uint8_t sernum8;
            uint8_t sernum9;
            uint8_t sernum10;
            uint8_t sernum11;
            uint8_t sernum12;
            uint8_t sernum13;
            uint8_t sernum14;
            uint8_t sernum15;

		};
		uint8_t byte[19];
	};
} nvm_device_serial_id_t;

char *NVMEE_read_serial( void );
char *NVMEE_read_device_ID( void );

int16_t NVMEE_read  ( uint16_t dataAddress, char *data, uint8_t data_length );
int16_t NVMEE_write ( uint16_t dataAddress, char *data, uint8_t data_length );

void NVMEE_test_read( char *addr, char *size );
void NVMEE_test_write( char *addr, char *str );


#endif /* SRC_SPX_LIBS_L_NVM_L_NVM_H_ */
