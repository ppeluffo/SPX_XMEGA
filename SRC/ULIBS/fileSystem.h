/* 
 * File:   fileSystem.h
 * Author: pablo
 *
 * Created on 18 de diciembre de 2022, 06:46 AM
 */

#ifndef FILESYSTEM_H
#define	FILESYSTEM_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "frtos-io.h"

#include "stdint.h"
#include "stdlib.h"
#include "string.h"
    
    
#include "xprintf.h"
#include "eeprom.h"
#include "i2c.h"
#include "rtc79410.h"
    
#define FF_SIZE_IN_KB	128		// Tamanio en KB de la eeprom externa.
#define FF_RECD_SIZE	64		// Tamanio del registro
#define FS_WRBUFF_SIZE  64
#define FS_RDBUFF_SIZE  64
#define FS_PAGE_SIZE    64      //256
    
#define FF_ADDR_START	0		// Posicion inicial
//#define FF_MAX_RCDS		64	// Cantidad de registros ( max 4096 en M24CM02 ).
#define FF_MAX_RCDS		1024    // ( FF_SIZE_IN_KB * 1024 / FF_RECD_SIZE )
//#define FF_MAX_RCDS		512
    
#define FF_WRTAG	0xC5	// 1100 0101

/*
 * *****************************************************************************
 * Tabla de movimientos de la FAT
 *
 *       | rcdsXwr | rcdsXrd | rcdsXdel
 * -------------------------------------
 * WRITE |   -1    |   +1    |    x
 * READ  |    x    |   -1    |    +1
 * DEL   |   +1    |    x    |    -1
 * -------------------------------------
 *
 * Memoria vacia: rcds4wr = MAX, rcds4del = 0;
 * Memoria llena: rcds4wr = 0, rcds4del = MAX;
 * Memoria toda leida: rcds4rd = 0;
 *
 */
/*
 * Defino el FS como un buffer circular de la EEPROM.
 * Solo necesito el control ya que el buffer es la EEPROM
 */
typedef struct {
	uint16_t head;
	uint16_t tail;
	uint16_t count;
	uint16_t length;
} fat_s;

fat_s FAT;


char fs_wr_buffer[FS_WRBUFF_SIZE];
char fs_rd_buffer[FS_RDBUFF_SIZE];

bool fs_debug;

//-----------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------
bool FAT_flush(void);
bool FS_open(void);
void FS_init(void);
bool FS_writeRcd( void *dr, uint8_t xSize );
bool FS_readRcd( void *dr, uint8_t xSize );
bool FS_readRcdByPos( uint16_t pos, void *dr, uint8_t xSize, bool detail );
void FS_format(bool fullformat);
uint8_t fs_chksum8(const char *buff, size_t len);
void FAT_read( fat_s *dstfat);
int16_t FS_dump( bool (*funct)(char *buff, bool ultimo), int16_t blocksize);
void FS_delete( int16_t ndrcds);

void FS_set_debug(void);
void FS_clear_debug(void);

//------------------------------------------------------------------------------



#ifdef	__cplusplus
}
#endif

#endif	/* FILESYSTEM_H */

