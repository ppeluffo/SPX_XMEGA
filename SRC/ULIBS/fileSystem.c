
/*
 * l_file.c
 *
 *  Created on: 31/10/2015
 *      Author: pablo
 */


#include "fileSystem.h"

SemaphoreHandle_t sem_FAT;
StaticSemaphore_t FAT_xMutexBuffer;
#define MSTOTAKEFATSEMPH ((  TickType_t ) 10 )

bool fs_hard_write( char *buff, uint8_t buff_size, uint16_t ptr, uint8_t *cks );
bool fs_hard_read( char *buff, uint8_t buff_size, uint16_t ptr, uint8_t *calc_cks, uint8_t *rd_cks);

//#define DEBUG_FS
void pv_fat_flush(void);
//------------------------------------------------------------------------------
void pv_fat_flush(void)
{
    
int16_t xBytes;
        
	FAT.head = 0;	// start
	FAT.tail = 0;	// end
	FAT.count = 0;
	FAT.length = FF_MAX_RCDS;
    
    xBytes = RTC_write( FAT_ADDRESS, (char *)&FAT, sizeof(fat_s) );
	if ( xBytes == -1 ) {
		xprintf_P(PSTR("ERROR: I2C:RTC:pv_fat_flush\r\n"));
    }

}
//------------------------------------------------------------------------------
bool FAT_flush(void)
{
    
int16_t xBytes;
bool retS = false;

    while ( xSemaphoreTake(sem_FAT, ( TickType_t ) 5 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 1 ) );
        
	FAT.head = 0;	// start
	FAT.tail = 0;	// end
	FAT.count = 0;
	FAT.length = FF_MAX_RCDS;
    
    xBytes = RTC_write( FAT_ADDRESS, (char *)&FAT, sizeof(fat_s) );
	if ( xBytes == -1 ) {
		xprintf_P(PSTR("ERROR: I2C:RTC:FAT_flush\r\n"));
        goto quit;
    }
    retS = true;

quit:

    xSemaphoreGive( sem_FAT);
    return(retS);

}
//------------------------------------------------------------------------------
void FAT_read( fat_s *dstfat)
{
    while ( xSemaphoreTake(sem_FAT, ( TickType_t ) 5 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 1 ) );
     
    // 20230808: Si algun parametro es negativo, la reseteo.
    // Cambio la fat de int_16 a uint_16

    if ( ( FAT.count < 0) || ( FAT.head < 0 ) || ( FAT.length < 0) || (FAT.tail < 0) ) {
        xprintf_P(PSTR("FAT ERROR: flush\r\n"));
        pv_fat_flush();
    }
    
    if ( FAT.length > FF_MAX_RCDS ) {
        xprintf_P(PSTR("FAT ERROR: flush\r\n"));
        pv_fat_flush();
    }
    
    if ( ( FAT.count > FF_MAX_RCDS) || ( FAT.head > FF_MAX_RCDS) || ( FAT.tail > FF_MAX_RCDS )) {
        xprintf_P(PSTR("FAT ERROR: flush\r\n"));
        pv_fat_flush();
    }
    
    memcpy( dstfat, &FAT, sizeof(fat_s));
    
    xSemaphoreGive( sem_FAT);
}
//------------------------------------------------------------------------------
bool FS_open(void)
{
	// Inicializa el sistema del file system leyendo de la SRAM del RTC la FAT
	// y cargandola en memoria.
	// Debo chequear la consistencia y si esta mal indicarlo y reiniciarla

bool retS = false;
int16_t xBytes;

	while ( xSemaphoreTake(sem_FAT, ( TickType_t ) 5 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 1 ) );

    xBytes = RTC_read( FAT_ADDRESS, (char *)&FAT, sizeof(fat_s) );
	if ( xBytes == -1 ) {
		xprintf_P(PSTR("ERROR: I2C:RTC:FAT_open\r\n"));
        goto quit;
    }
    
    FAT.length = FF_MAX_RCDS;
    xprintf_P( PSTR("FAT: head=%d, tail=%d,count=%d, length=%d\r\n"), FAT.head,FAT.tail, FAT.count, FAT.length );

    retS = true;
    
quit:
    xSemaphoreGive( sem_FAT);
	return(retS);

}
//------------------------------------------------------------------------------
void FS_init(void)
{
    fs_debug = false;
	sem_FAT = xSemaphoreCreateMutexStatic( &FAT_xMutexBuffer );
}
//------------------------------------------------------------------------------
bool FS_writeRcd( void *dr, uint8_t xSize )
{
    /*
     * La eeprom actua como un ringbuffer.
     * Escribo en la posicion apuntada por head y lo avanzo en modo circular.
     * El proceso de lectura hw lo hace en forma externa
     */
    
int16_t xBytes;
uint16_t next;
bool retS = false;
uint8_t cks;

    if (xSize > FS_WRBUFF_SIZE ) {
        xSize = FS_WRBUFF_SIZE;
    }
    
    while ( xSemaphoreTake(sem_FAT, ( TickType_t ) 5 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 1 ) );

   // Buffer lleno
    if ( FAT.count == FAT.length) {
        xprintf_P(PSTR("ERROR: FS full.!! \r\n"));
        retS = false;
        goto quit;
    }
    
    // copio los datos recibidos del dr al buffer ( 0..(xSize-1)) y los escribo
    memset( fs_wr_buffer, 0xFF, FS_WRBUFF_SIZE );
	memcpy ( fs_wr_buffer, dr, xSize );
    retS = fs_hard_write( fs_wr_buffer, FS_WRBUFF_SIZE, FAT.head, &cks);
    if (fs_debug)
        xprintf_P(PSTR("FSwrite: head=%d,tail=%d,count=%d,cks=0x%02x\r\n"), FAT.head, FAT.tail, FAT.count, cks);
    
    // Avanzo el puntero en forma circular
    FAT.count++;
    next = FAT.head + 1;
    if ( next >= FAT.length) {
        next = 0;
    }
    FAT.head = next;
    // Actualizo la fat por si se resetea el equipo
    xBytes = RTC_write( FAT_ADDRESS, (char *)&FAT, sizeof(fat_s) );
	if ( xBytes == -1 ) {
		xprintf_P(PSTR("ERROR: I2C:RTC:FAT_save\r\n"));
        retS = false;
        goto quit;
    }
    
quit:

    xSemaphoreGive( sem_FAT);
    return(retS);
    
}
//------------------------------------------------------------------------------
bool FS_readRcd( void *dr, uint8_t xSize )
{
    /*
     * Leo de la eeprom de la posicion apuntada por tail.
     * Verifico que no este vacia.
     * El proceso de lectura hw lo hace en forma externa
     * 
     */

uint16_t next;
int16_t xBytes;
bool retS = false;
uint8_t calc_cks, rd_cks;

	// Lo primero es obtener el semaforo
	while ( xSemaphoreTake(sem_FAT, ( TickType_t ) 5 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 1 ) );

    // Verifico que no este vacia.
    if ( FAT.count == 0 ) {   
        retS = false;
        goto quit;
    }
    
    retS = fs_hard_read( fs_rd_buffer, FS_RDBUFF_SIZE, FAT.tail, &calc_cks, &rd_cks);
    
    if (fs_debug)
        xprintf_P(PSTR("FSread: head=%d,tail=%d,count=%d,calc_cks=0x%02x,rd_cks=0x%02x\r\n"), FAT.head, FAT.tail, FAT.count, calc_cks, rd_cks);
    
    if ( retS ) {
        memcpy( dr, &fs_rd_buffer, xSize );
    }

    // Está todo bien. Ajusto los punteros.
    if ( FAT.count > 0 ) {
        FAT.count--;
    } else {
        // ERROR
        xprintf_P(PSTR("ERROR: FAT count < 0 !!!. Flush.\r\n"));
        pv_fat_flush();
        goto quit;
    }
    
    next = FAT.tail + 1;
    if ( next >= FAT.length) {
        next = 0;
    }
    FAT.tail = next;
    // Actualizo la fat por si se resetea el equipo
    xBytes = RTC_write( FAT_ADDRESS, (char *)&FAT, sizeof(fat_s) );
	if ( xBytes == -1 ) {
		xprintf_P(PSTR("ERROR: I2C:RTC:FAT_save\r\n"));
        retS = false;
        goto quit;
    }
    
    retS = true;
    
quit:

    xSemaphoreGive( sem_FAT);
    return(retS);
}
//------------------------------------------------------------------------------
bool FS_readRcdByPos( uint16_t pos, void *dr, uint8_t xSize, bool detail )
{
    /*
     * Leo de la eeprom de la posicion apuntada por tail.
     * Verifico que no este vacia.
     * 
     */

uint8_t calc_cks, rd_cks;
bool retS = false;
uint8_t i;

	// Lo primero es obtener el semaforo
	while ( xSemaphoreTake(sem_FAT, ( TickType_t ) 5 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 1 ) );

    retS = fs_hard_read( fs_rd_buffer, FS_RDBUFF_SIZE, pos, &calc_cks, &rd_cks);
    
    if (fs_debug)
        xprintf_P(PSTR("FSreadByPos: head=%d,tail=%d,count=%d,calc_cks=0x%02x,rd_cks=0x%02x\r\n"), FAT.head, FAT.tail, FAT.count, calc_cks, rd_cks);
 
    if (detail) {    
        for (i=0; i<FS_RDBUFF_SIZE; i++) {
            if ( (i%8) == 0 ) {
                xprintf_P(PSTR("\r\n%02d: "),i);
            }
            xprintf_P(PSTR("[0x%02x] "), fs_rd_buffer[i]);
        }
        xprintf_P(PSTR("\r\n"));       
    }
    
    if ( retS ) {
        memcpy( dr, fs_rd_buffer, xSize );
    }
 
    xSemaphoreGive( sem_FAT);
    return(retS);
}
//------------------------------------------------------------------------------
void FS_format(bool fullformat)
{
	// Inicializa la memoria reseteando solo la FAT.
	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// Para que no salga por watchdog, apago las tareas previamente !!!!
	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


uint16_t i = 0;
bool retS = false;
uint8_t cks;

    FAT_flush();
    
	if ( fullformat ) {
        
        while ( xSemaphoreTake(sem_FAT, ( TickType_t ) 5 ) != pdTRUE )
            vTaskDelay( ( TickType_t)( 1 ) );

        memset( fs_wr_buffer, 0xFF, FS_WRBUFF_SIZE );
	
		// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// Para que no salga por watchdog, apago las tareas previamente !!!!
		// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //xprintf_P( PSTR("FF_format: hard clear\r\n"));
		// Borro fisicamente los registros
		for ( i = 0; i < FF_MAX_RCDS; i++ ) {
            retS = fs_hard_write( fs_wr_buffer, FS_WRBUFF_SIZE, i, &cks);
			if ( !retS )
				xprintf_P(PSTR("ERROR: I2C:EE:FS_format\r\n"));

			vTaskDelay( ( TickType_t)( 10 ) );

			if ( ( i > 0 ) && (i % 32) == 0 ) {
				xprintf_P( PSTR(" %04d"),i);
				if ( ( i > 0 ) && (i % 256) == 0 ) {
					xprintf_P( PSTR("\r\n"));
				}
			}
		}

        xSemaphoreGive( sem_FAT);
	}
   
}
//------------------------------------------------------------------------------
int16_t FS_dump( bool (*funct)(char *buff, bool ultimo ), int16_t blocksize)
{
    /*
     * Leo todos los registros y los proceso de a uno con la funcion
     * pasada como parámetro.
     * Esta puede imprimirlos en pantalla o copiarlos y enviarlos.
     * blocksize es la cantidad de registros a procesar (dump)
     * Si es -1, los proceso todos.
     */
    
int16_t ptr;
int16_t i;
bool retS = false;
uint8_t calc_cks, rd_cks;
int16_t retV = -1;

	while ( xSemaphoreTake(sem_FAT, ( TickType_t ) 5 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 1 ) );

    if (  FAT.count == 0 ) {
        xprintf_P(PSTR("FS Dump: MEM EMPTY\r\n"));
        retV = 0;
        goto quit;
    }

    ptr = FAT.tail;
    for (i=0; i<FAT.count && i<blocksize; i++) {
        //xprintf_P(PSTR("PTR=%d, I=%d\r\n"),ptr,i);
        retS = fs_hard_read( fs_rd_buffer, FS_RDBUFF_SIZE, ptr, &calc_cks, &rd_cks);
        
        if (fs_debug)
            xprintf_P(PSTR("FSdump: ptr=%d, head=%d,tail=%d,count=%d,calc_cks=0x%02x,rd_cks=0x%02x\r\n"), ptr, FAT.head, FAT.tail, FAT.count, calc_cks, rd_cks);
        
        ptr++;
        // Avanzo el puntero en forma circular
        if ( ptr >= FAT.length) {
            ptr = 0;
        }
        
        if ( i == ( blocksize - 1)) { 
            // Ultimo frame del bloque
            retS = funct(&fs_rd_buffer[0], true );
        } else if ( i == ( FAT.count - 1)) {
            // Ultimo frame de la fat
            retS = funct(&fs_rd_buffer[0], true );        
        } else {    
            // Ejecuto la funcion que procesa el buffer.
            retS = funct(&fs_rd_buffer[0], false);
        }
        
        // Si algun registro me da error, salgo.
        if ( !retS) {
            retV = -1;
            goto quit;
        }
        
    }
    retV = i;
    
    xprintf_P(PSTR("FS Dump %d\r\n"), i);
    
    if ( FAT.count == 0)
        xprintf_P( PSTR("MEM bd EMPTY\r\n"));  
    
quit:
    
    xSemaphoreGive( sem_FAT);   
    // Retorno la cantidad de registros procesados.

    //if ( FAT.count == 0) {
    //    FAT_flush()
    //}

    return(retV);

}
//------------------------------------------------------------------------------
void FS_delete( int16_t ndrcds )
{
    /*
     * Borra del FS ndrcds registros.
     * Comienza en el head y va avanzando.
     * Si ndrcds = -1, borra toda la memoria.
     */

uint16_t i;
uint16_t max_rcds;
int next;
uint8_t cks;
bool retS = false;
int16_t xBytes;

    while ( xSemaphoreTake(sem_FAT, ( TickType_t ) 5 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 1 ) );

    if (  FAT.count == 0 ) {
        xprintf_P(PSTR("FS Delete: MEM EMPTY\r\n"));
        goto quit;
    }

    // Calculo los limites
    if (ndrcds == -1) {
        max_rcds = FAT.count;
    } else {
        max_rcds = ndrcds;
    }

     /*
     * Borro y avanzo el puntero.
     */
	memset( fs_wr_buffer, 0xFF, FS_WRBUFF_SIZE );
    
    for (i=0; i<max_rcds; i++) {
        
        vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
        
        // Verifico que no este vacia.
        if ( FAT.count == 0 ) {   
            goto quit;
        }
 
        retS = fs_hard_write( fs_wr_buffer, FS_WRBUFF_SIZE, FAT.tail, &cks);
        if ( !retS) {    
            xprintf_P(PSTR("ERROR: I2C:EE:FS_delete(%d)\r\n"),FAT.tail);
            //goto quit;
        }
    
        // Está todo bien. Ajusto los punteros.
        FAT.count--;
        next =FAT.tail + 1;
        if ( next >= FAT.length) {
            next = 0;
        }
        FAT.tail = next;
    
        // Actualizo la fat por si se resetea el equipo
        xBytes = RTC_write( FAT_ADDRESS, (char *)&FAT, sizeof(fat_s) );
        if ( xBytes == -1 ) {
            xprintf_P(PSTR("ERROR: I2C:RTC:FAT_save\r\n"));
            goto quit;
        }
    }
    
    xprintf_P(PSTR("FS Delete %d\r\n"), ndrcds);
    
quit:

    xSemaphoreGive( sem_FAT);
}
//------------------------------------------------------------------------------
void FS_set_debug(void)
{
    fs_debug = true;
}
//------------------------------------------------------------------------------
void FS_clear_debug(void) 
{
    fs_debug = false;
}
//------------------------------------------------------------------------------
bool fs_hard_write( char *buff, uint8_t buff_size, uint16_t ptr, uint8_t *cks )
{
    /*
     * Toma un buffer lineal.
     * Calcula el checksum, termina el frame y lo escribe en la EEprom
     
     */
uint16_t wrAddress;
int16_t bytes_written = -1;

	// Calculo y grabo el checksum a continuacion del frame (en la pos.xSize)
	// El checksum es solo del dataFrame por eso paso dicho size.
    *cks = fs_chksum8( buff, (buff_size - 2) );
	buff[buff_size - 2] = *cks;
	// Grabo el tag para indicar que el registro esta escrito.
	buff[buff_size - 1] = FF_WRTAG;
	// 
	wrAddress = FF_ADDR_START + ptr * FS_PAGE_SIZE;
    //wrAddress = FF_ADDR_START + ptr * FF_RECD_SIZE;
    if ( ( wrAddress % FS_PAGE_SIZE) != 0 ) {
    //if ( ( wrAddress % 32) != 0 ) {
        xprintf_P(PSTR("ERROR:fs_hard_write wrAddress: ptr=%d, wrAddress=%d\r\n"), ptr, wrAddress);
        return(false);
    }
    
    if ( fs_debug )
        xprintf_P(PSTR("WR_ADDRESS=0x%04x, ptr=%d\r\n"), wrAddress, ptr);
    
	bytes_written = EE_write( wrAddress, buff, FF_RECD_SIZE, fs_debug );
    // Necesito al memos tw=5ms entre escrituras.
    vTaskDelay( ( TickType_t)( 20 / portTICK_PERIOD_MS ) );
    // Control de errores:
	if ( bytes_written != FF_RECD_SIZE ) {
		xprintf_P(PSTR("ERROR: I2C:EE:FS_writeRcd\r\n"));
        return(false);
    }
    
#ifdef DEBUG_FS
    uint8_t i;
    
    for (i=0; i<FF_RECD_SIZE; i++) {
        if ( (i%8) == 0 ) {
            xprintf_P(PSTR("\r\n%02d: "),i);
        }
        xprintf_P(PSTR("[0x%02x] "), fs_wr_buffer[i]);
    }
    xprintf_P(PSTR("\r\n"));
#endif
    
    return(true);
}
//------------------------------------------------------------------------------
bool fs_hard_read( char *buff, uint8_t buff_size, uint16_t ptr, uint8_t *calc_cks, uint8_t *rd_cks)
{
    /*
     * Leo HW un registro apuntado por la direccion ptr.
     */
    
uint8_t rdCheckSum = 0;
uint8_t calcCheckSum = 0;
uint16_t rdAddress = 0;
int16_t bytes_read = 0U;
bool retS = false;

	memset( buff, 0x00, buff_size );
	//rdAddress = FF_ADDR_START + ptr * FF_RECD_SIZE;
    rdAddress = FF_ADDR_START + ptr * FS_PAGE_SIZE;
    if ( ( rdAddress % FS_PAGE_SIZE) != 0 ) {
    //if ( ( rdAddress % 32) != 0 ) {
        xprintf_P(PSTR("ERROR:fs_hard_read rdAddress: ptr=%d, rdAddress=%d\r\n"), ptr, rdAddress);
        return(false);
    }
    
    if ( fs_debug )
        xprintf_P(PSTR("RD_ADDRESS=0x%04x, ptr=%d\r\n"), rdAddress, ptr);
    
	bytes_read = EE_read( rdAddress, buff, buff_size, fs_debug);
	if (bytes_read == -1 ) {
		xprintf_P(PSTR("ERROR: I2C:EE:FS_readRcd\r\n"));
        retS = false;
        goto quit;
    }
    
    // Control de checksums
    *calc_cks = fs_chksum8( buff, (buff_size - 2) );
    *rd_cks = buff[(buff_size - 2)];
	if ( rdCheckSum != calcCheckSum ) {
        xprintf_P(PSTR("ERROR: I2C:EE:FS_readRcd checksum: calc=0x%02x, mem=0x%02x\r\n"),*calc_cks, *rd_cks );
        retS = false;
		goto quit;
	}

	// Vemos si la ultima posicion tiene el tag de ocupado.
	if ( ( buff[buff_size - 1] )  != FF_WRTAG ) {
        xprintf_P(PSTR("ERROR: I2C:EE:FS_readRcd TAG\r\n"));
        retS = false;
		goto quit;
	}
    
    retS = true;
  
quit:
    return(retS);
    
}
//------------------------------------------------------------------------------
uint8_t fs_chksum8(const char *buff, size_t len)
{
uint8_t checksum = 0;

	for ( checksum = 0 ; len != 0 ; len-- )
		checksum += *(buff++);   // parenthesis not required!

	checksum = ~checksum;
	return (checksum);
}
//------------------------------------------------------------------------------
