/*
 * l_ringBuffer.h
 *
 *  Created on: 8 dic. 2018
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_RINGBUFFER_H_
#define SRC_SPX_LIBS_L_RINGBUFFER_H_


#include "stdlib.h"
#include "string.h"
#include "stdbool.h"

#include "FreeRTOS.h"
#include "task.h"

//--------------------------------------------------------------------------------------------
// Ring Buffers de Chars
typedef struct {
	uint8_t *buff;
	int16_t head;
	int16_t tail;
	int16_t count;
	int16_t length;
} rBchar_s;

void rBchar_CreateStatic ( rBchar_s *rB, uint8_t *storage_area, uint16_t size  );
bool rBchar_Poke( rBchar_s *rB, char cChar );
bool rBchar_PokeFromISR( rBchar_s *rB, char cChar );
bool rBchar_Pop( rBchar_s *rB, char *cChar );
bool rBchar_PopFromISR( rBchar_s *rB, char *cChar );
void rBchar_Flush( rBchar_s *rB );
uint16_t rBchar_GetCount( rBchar_s *rB );
uint16_t rBchar_GetFreeCount( rBchar_s *rB );
bool rBchar_ReachLowWaterMark( rBchar_s *rB);
bool rBchar_ReachHighWaterMark( rBchar_s *rB );
bool rBchar_isFull( rBchar_s *rB );
void rBchar_fill(rBchar_s *rB );
//--------------------------------------------------------------------------------------------
// Ring Buffers de Estructuras
typedef struct {
	void *buff;
	uint16_t head;
	uint16_t tail;
	uint16_t count;
	uint16_t length;
	uint16_t elementsize;
    bool overwrite;
} rBstruct_s;

void rBstruct_CreateStatic ( rBstruct_s *rB, void *storage_area, uint16_t buffersize, uint16_t elementsize, bool f_overwrite  );
bool rBstruct_Poke( rBstruct_s *rB, void *element );
bool rBstruct_Pop( rBstruct_s *rB, void *element );
bool rBstruct_insert_at_tail( rBstruct_s *rB, void *element );
bool rBstruct_PopRead( rBstruct_s *rB, void *element );
void rBstruct_Flush( rBstruct_s *rB );
uint16_t rBstruct_GetCount( rBstruct_s *rB );
uint16_t rBstruct_GetFreeCount( rBstruct_s *rB );
uint16_t rBstruct_GetHead( rBstruct_s *rB );
uint16_t rBstruct_GetTail( rBstruct_s *rB );


#endif /* SRC_SPX_LIBS_L_RINGBUFFER_H_ */
