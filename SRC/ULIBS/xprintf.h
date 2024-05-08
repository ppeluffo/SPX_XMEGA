/*
 * l_printf.h
 *
 *  Created on: 8 dic. 2018
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_PRINTF_H_
#define SRC_SPX_LIBS_L_PRINTF_H_

#include <avr/pgmspace.h>
    
void XPRINTF_init(void);
int xprintf( const char *fmt, ...);
int xfprintf( int fd, const char *fmt, ...);
int xprintf_P( PGM_P fmt, ...);
int xfprintf_P( int fd, PGM_P fmt, ...);
int xputs( const char *str );
int xfputs( int fd, const char *str );

int xnprintf( int fd, const char *pvBuffer, const uint16_t xBytes );

void putch(char c);
void xputChar(unsigned char c);
void xfputChar(int fd, unsigned char c);

void xputCharNS(unsigned char c);

#endif /* SRC_SPX_LIBS_L_PRINTF_H_ */
