/* 
 * File:   linearBuffer.h
 * Author: pablo
 *
 * Created on 16 de febrero de 2022, 04:09 PM
 */

#ifndef LINEARBUFFER_H
#define	LINEARBUFFER_H

#ifdef	__cplusplus
extern "C" {
#endif


//#include "xc.h"
#include <avr/io.h>
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdbool.h"

#include "FreeRTOS.h"
#include "task.h"
#include "xprintf.h"
    
    
//--------------------------------------------------------------------------------------------
//! cBuffer structure
typedef struct struct_lBuffer
{
	char *buff;		///< the physical memory address where the buffer is stored
	uint16_t size;		///< the allocated size of the buffer
	uint16_t ptr;         ///< the index into the buffer where the data starts
} lBuffer_s;

void lBchar_CreateStatic ( lBuffer_s *lB, char *storage_area, uint16_t size  );
bool lBchar_Put( lBuffer_s *lB, char cChar );
bool lBchar_Get( lBuffer_s *lB, char *cChar );
void lBchar_Flush( lBuffer_s *lB );
uint16_t lBchar_GetCount( lBuffer_s *lB );
uint16_t lBchar_GetFreeCount( lBuffer_s *lB );
bool lBchar_isFull( lBuffer_s *lB );
bool lBchar_isEmpty( lBuffer_s *lB );
char *lBchar_get_buffer(lBuffer_s *lB);
void lBchar_print(lBuffer_s *lB );


#ifdef	__cplusplus
}
#endif

#endif	/* LINEARBUFFER_H */

