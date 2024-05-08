/*
 * l_ringBuffer.c
 *
 *  Created on: 11 jul. 2018
 *      Author: pablo
 */

#include "ringBuffer.h"

//------------------------------------------------------------------------------
/*
 *  RING BUFFERS DE ESTRUCTURAS
 */
void rBstruct_CreateStatic ( rBstruct_s *rB, void *storage_area, uint16_t buffersize, uint16_t elementsize, bool f_overwrite  )
{
	rB->buff = storage_area;
	rB->head = 0;	// start
	rB->tail = 0;	// end
	rB->count = 0;
	rB->length = buffersize;
	rB->elementsize = elementsize;
    rB->overwrite = f_overwrite;

}
//------------------------------------------------------------------------------
bool rBstruct_Poke( rBstruct_s *rB, void *element )
{

	/*
     *  Coloco una estructura presionTask ( put_on_top ) en la FIFO
     *  Dependiendo del overwrite esperamos tener espacio o no.
     *
     */

bool ret = false;
void *p;

	taskENTER_CRITICAL();

	// Si el buffer esta vacio ajusto los punteros
	if( rB->count == 0) {
		rB->head = rB->tail = 0;
	}

    // Si esta lleno y no tengo overwrite salgo
    if ( rB->length == rB->count) {
        if ( !rB->overwrite) {
            goto exit;
        }
    }
    
    // Guardo
	p =  (rB->buff);
	p += sizeof(uint8_t)*( rB->head * rB->elementsize);
	memcpy( p, element, rB->elementsize );
    
    // Avanzo.
	// 1 - Avanzo el head en modo circular.
	rB->head = ( rB->head  + 1 ) % ( rB->length );
    // 2 - Si no esta lleno avanzo el contador
    if ( rB->length > rB->count)  {
        ++rB->count;
    } else {
        // Avanzo el tail.
        rB->tail = ( rB->tail  + 1 ) % ( rB->length );
    }
    
	ret = true;

exit:
    
	taskEXIT_CRITICAL();
	return(ret);

}
//------------------------------------------------------------------------------
bool rBstruct_Pop( rBstruct_s *rB, void *element )
{

	// Saco un valor ( get_from_tail ) de la FIFO.

bool ret = false;
void *p;

	taskENTER_CRITICAL();
	//  Si el buffer esta vacio retorno.
	if( rB->count == 0) {
		rB->head = rB->tail = 0;
		taskEXIT_CRITICAL();
		return(ret);
	}

	p =  (rB->buff);
	p += sizeof(uint8_t)*( rB->tail * rB->elementsize);

	memcpy( element, p, rB->elementsize );
	
    --rB->count;
	// Avanzo en modo circular
	rB->tail = ( rB->tail  + 1 ) % ( rB->length );
	ret = true;

	taskEXIT_CRITICAL();
	return(ret);
}
//------------------------------------------------------------------------------
bool rBstruct_insert_at_tail( rBstruct_s *rB, void *element )
{

	// Pone el ultimo elemento (tail) en 0 y lo saca del RB.
    // Esto es porque al calcular los promedios, no usamos la cola sino que
    // debemos ponerlo a 0 antes.

bool ret = false;
void *p;

	taskENTER_CRITICAL();
	//  Si el buffer esta vacio retorno.
	if( rB->count == 0) {
		rB->head = rB->tail = 0;
		taskEXIT_CRITICAL();
		return(ret);
	}

	p =  (rB->buff);
	p += sizeof(uint8_t)*( rB->tail * rB->elementsize);

    memcpy( p, element, rB->elementsize );
    
	--rB->count;
	// Avanzo en modo circular
	rB->tail = ( rB->tail  + 1 ) % ( rB->length );
	ret = true;

	taskEXIT_CRITICAL();
	return(ret);
}
//------------------------------------------------------------------------------
bool rBstruct_PopRead( rBstruct_s *rB, void *element )
{

	// Leo un valor ( red_from_tail ) de la FIFO.

bool ret = false;
void *p;

	taskENTER_CRITICAL();
	//  Si el buffer esta vacio retorno.
	if( rB->count == 0) {
		rB->head = rB->tail = 0;
		taskEXIT_CRITICAL();
		return(ret);
	}

	p =  (rB->buff);
	p += sizeof(uint8_t)*( rB->tail * rB->elementsize);

	memcpy( element, p, rB->elementsize );
	// No muevo el puntero

	ret = true;

	taskEXIT_CRITICAL();
	return(ret);
}
//------------------------------------------------------------------------------
void rBstruct_Flush( rBstruct_s *rB )
{

	rB->head = 0;
	rB->tail = 0;
	rB->count = 0;
	memset( (rB->buff), '\0', (rB->length * rB->elementsize) );
}
//------------------------------------------------------------------------------
uint16_t rBstruct_GetCount( rBstruct_s *rB )
{

	return(rB->count);

}
//------------------------------------------------------------------------------
uint16_t rBstruct_GetFreeCount( rBstruct_s *rB )
{

	return(rB->length - rB->count);

}
//------------------------------------------------------------------------------
uint16_t rBstruct_GetHead( rBstruct_s *rB )
{

	return(rB->head);

}
//------------------------------------------------------------------------------
uint16_t rBstruct_GetTail( rBstruct_s *rB )
{

	return(rB->tail);

}
//------------------------------------------------------------------------------
/*
 *  RING BUFFERS DE CHAR
 */
bool rBchar_Poke( rBchar_s *rB, char cChar )
{

int next;

	taskENTER_CRITICAL();

    // Buffer lleno
    if ( rB->count == rB->length) {
        taskEXIT_CRITICAL();
        return(false);        
    }
    
    // Guardo
    rB->buff[rB->head] = cChar;
    (rB->count)++;

    // Avanzo en modo circular
    next = rB->head + 1;
    if ( next >= rB->length) {
        next = 0;
    }
    rB->head = next;

	taskEXIT_CRITICAL();
	return(true);

}
//------------------------------------------------------------------------------------
bool rBchar_PokeFromISR( rBchar_s *rB, char cChar )
{
int next;

   // Buffer lleno
    if ( rB->count == rB->length) {
        return(false);        
    }
    
    // Guardo
    rB->buff[rB->head] = cChar;
    (rB->count)++;

    // Avanzo en modo circular
    next = rB->head + 1;
    if ( next >= rB->length) {
        next = 0;
    }
    rB->head = next;

	return(true);

}
//------------------------------------------------------------------------------------
bool rBchar_Pop( rBchar_s *rB, char *cChar )
{

int next;

	// Voy a leer un dato

	taskENTER_CRITICAL();

    if ( rB->count == 0 ) {    // Esta vacio
        taskEXIT_CRITICAL();
		return(false);
    }
    
    // Avanzo tail en modo circular    
    *cChar = rB->buff[rB->tail];
	(rB->count)--;
    next = rB->tail + 1;
    if ( next >= rB->length) {
        next = 0;
    }
    rB->tail = next;
    
    taskEXIT_CRITICAL();
    return (true);
    

}
//------------------------------------------------------------------------------------
bool rBchar_PopFromISR( rBchar_s *rB, char *cChar )
{
    
int next;

	// Voy a leer un dato

    if ( rB->count == 0 ) {    // Esta vacio
		return(false);
    }
    
    // Avanzo tail en modo circular    
    *cChar = rB->buff[rB->tail];
	(rB->count)--;
    next = rB->tail + 1;
    if ( next >= rB->length) {
        next = 0;
    }
    rB->tail = next;
    
    return (true);
}
//------------------------------------------------------------------------------------
void rBchar_Flush( rBchar_s *rB )
{

	rB->head = 0;
	rB->tail = 0;
	rB->count = 0;
	memset(rB->buff,'\0', rB->length );
}
//------------------------------------------------------------------------------
void rBchar_CreateStatic ( rBchar_s *rB, uint8_t *storage_area, uint16_t size  )
{
	rB->buff = storage_area;
	rB->head = 0;	// start
	rB->tail = 0;	// end
	rB->count = 0;
	rB->length = size;
}
//------------------------------------------------------------------------------
uint16_t rBchar_GetCount( rBchar_s *rB )
{

uint16_t count;

    taskENTER_CRITICAL();
    count = rB->count;
    taskEXIT_CRITICAL();
    
	return(count);

}
//------------------------------------------------------------------------------
uint16_t rBchar_GetFreeCount( rBchar_s *rB )
{

uint16_t freecount;

    taskENTER_CRITICAL();
    freecount = rB->length - rB->count;
    taskEXIT_CRITICAL();
    
	return(freecount);


}
//------------------------------------------------------------------------------
bool rBchar_ReachLowWaterMark( rBchar_s *rB)
{
bool retS = false;

    taskENTER_CRITICAL();
	if ( rB->count  <= 3 )
		retS = true;
    
    taskEXIT_CRITICAL();
	return(retS);

}
//------------------------------------------------------------------------------
bool rBchar_ReachHighWaterMark( rBchar_s *rB )
{
bool retS = false;

    taskENTER_CRITICAL();
	if ( rB->count  >= ( 5 ) )
		retS = true;
    
    taskEXIT_CRITICAL();

	return(retS);

}
//------------------------------------------------------------------------------
bool rBchar_isFull( rBchar_s *rB )
{
bool retS = false;

    taskENTER_CRITICAL();
    
	if ( rB->count  == rB->length )
		retS = true;
    taskEXIT_CRITICAL();
	return(retS);

}
//------------------------------------------------------------------------------
