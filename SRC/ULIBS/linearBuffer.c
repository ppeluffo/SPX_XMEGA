
#include "linearBuffer.h"

//------------------------------------------------------------------------------
void lBchar_CreateStatic ( lBuffer_s *lB, char *storage_area, uint16_t size )
{
   	lB->buff = storage_area;
	lB->ptr = 0;	
	lB->size = size;	
}
//------------------------------------------------------------------------------
bool lBchar_Put( lBuffer_s *lB, char cChar )
{
    // Guarda un dato en el buffer si hay lugar.
    
    if( ! lBchar_isFull(lB) ) {
        lB->buff[lB->ptr] = cChar;
		++lB->ptr;
        return(true);
    }
    return(false);
}
//------------------------------------------------------------------------------
bool lBchar_Get( lBuffer_s *lB, char *cChar )
{
    // Lee un dato del buffer a donde apunta el puntero. 
    
    if ( !lBchar_isEmpty(lB)) {
        --lB->ptr;
        *cChar = lB->buff[lB->ptr];
        return(true);
    }
    return(false);
}
//------------------------------------------------------------------------------
void lBchar_Flush( lBuffer_s *lB )
{
//uint8_t i;
    
	lB->ptr = 0;
	memset( (void *)lB->buff, '\0', (size_t)lB->size );
}
//------------------------------------------------------------------------------
uint16_t lBchar_GetCount( lBuffer_s *lB )
{
    return(lB->ptr);
}
//------------------------------------------------------------------------------
uint16_t lBchar_GetFreeCount( lBuffer_s *lB )
{
    return(lB->size - lB->ptr);
}
//------------------------------------------------------------------------------
bool lBchar_isFull( lBuffer_s *lB )
{
    return(lB->ptr == lB->size);
}
//------------------------------------------------------------------------------
bool lBchar_isEmpty( lBuffer_s *lB )
{
    return( !lBchar_isFull(lB));
}
//------------------------------------------------------------------------------
char *lBchar_get_buffer( lBuffer_s *lB )
{
    return(lB->buff);
}
//------------------------------------------------------------------------------
void lBchar_print(lBuffer_s *lB )
{
     xprintf( "BUFF [%s]\r\n", lB->buff );
}
//------------------------------------------------------------------------------
