/*
 * drv_uart_spx.c
 *
 *  Created on: 11 jul. 2018
 *      Author: pablo
 */

#include "drv_uart_spx.h"

void drv_set_baudrate(uint32_t baudRate, uint8_t *baudA, uint8_t *baudB, uint8_t *ctl );

//------------------------------------------------------------------------------
void drv_set_baudrate(uint32_t baudRate, uint8_t *baudA, uint8_t *baudB, uint8_t *ctl )
{
#if F_CPU == 32000000

	/* Set Baudrate to 115200 bps:
	 * Use the default I/O clock frequency that is 32 MHz.
	 * Los valores los extraigo de la planilla provista por Atmel
	 * 32Mhz
	 * BSEL = 2094
	 * BSCALE = -7
	 * CLK2X = 0
	 * %error = 0,01%
	 */
	switch(baudRate) {
	case 115200:
		*baudA = (uint8_t) 2094;
		*baudB = ( -7 << USART_BSCALE0_bp)|(2094 >> 8);
		break;
	case 9600:
		// 9600
		*baudA = (uint8_t) 3317;
		*baudB = ( -4 << USART_BSCALE0_bp)|(3317 >> 8);
		break;
	}

#endif

#if F_CPU == 8000000
		/* Set Baudrate to 115200 bps:
		 * Use the default I/O clock frequency that is 32 MHz.
		 * Los valores los extraigo de la planilla provista por Atmel
		 * 8Mhz
		 * BSEL = 983
		 * BSCALE = -7
		 * CLK2X = 1
		 * %error = 0,01%
		 */
	*baudA = (uint8_t) 983;
	*baudB = ( -7 << USART_BSCALE0_bp)|(983 >> 8);
		// Habilito CLK2X
	*ctl |= USART_CLK2X_bm;
#endif

#if F_CPU == 2000000
		/* Set Baudrate to 115200 bps:
		 * Use the default I/O clock frequency that is 2 MHz.
		 * Los valores los extraigo de la planilla provista por Atmel
		 * 2Mhz
		 * BSEL = 11
		 * BSCALE = -7
		 * CLK2X = 0
		 * %error = 0,08%
		 */
		*baudA = (uint8_t) 11;
		*baudB = ( -7 << USART_BSCALE0_bp)|(11 >> 8);
#endif
}
//------------------------------------------------------------------------------
// USART1: RS485A
//------------------------------------------------------------------------------
void drv_uart1_init(uint32_t baudrate )
{
    
uint8_t baudA, baudB, ctl;

    PORTE.DIRSET   = PIN3_bm;	// PE3 (TXD0) as output.
	PORTE.DIRCLR   = PIN2_bm;	// PE2 (RXD0) as input.
	// USARTE0, 8 Data bits, No Parity, 1 Stop bit.
	USARTE0.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;
    
    ctl = USARTE0.CTRLB;
	drv_set_baudrate( baudrate, &baudA, &baudB, &ctl);
	USARTE0.BAUDCTRLA = baudA;
	USARTE0.BAUDCTRLB = baudB;
	USARTE0.CTRLB = ctl;
    
	// Habilito la TX y RX
	USARTE0.CTRLB |= USART_RXEN_bm;
	USARTE0.CTRLB |= USART_TXEN_bm;
    
	// Habilito la interrupcion de Recepcion ( low level )
	// low level, RXint enabled
	USARTE0.CTRLA = ( USARTE0.CTRLA & ~USART_RXCINTLVL_gm ) | USART_RXCINTLVL_LO_gc;
    // Las transmisiones son por poleo no INT.
    
    rBchar_CreateStatic ( &TXRB_uart1, &uart1_txBuffer[0], UART1_TXSIZE  );
    rBchar_CreateStatic ( &RXRB_uart1, &uart1_rxBuffer[0], UART1_RXSIZE  );

}
//------------------------------------------------------------------------------
ISR(USARTE0_RXC_vect)
{

char cChar;

	cChar = USARTE0.DATA;
	rBchar_PokeFromISR( &RXRB_uart1, cChar );
}
//------------------------------------------------------------------------------
// USART2: TERM
//------------------------------------------------------------------------------
void drv_uart2_init(uint32_t baudrate )
{
    
uint8_t baudA, baudB, ctl;

    PORTF.DIRSET   = PIN3_bm;	// PD3 (TXD0) as output.
	PORTF.DIRCLR   = PIN2_bm;	// PD2 (RXD0) as input.
	// USARTF0, 8 Data bits, No Parity, 1 Stop bit.
	USARTF0.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;
    
    ctl = USARTF0.CTRLB;
	drv_set_baudrate( baudrate, &baudA, &baudB, &ctl);
	USARTF0.BAUDCTRLA = baudA;
	USARTF0.BAUDCTRLB = baudB;
	USARTF0.CTRLB = ctl;
    
	// Habilito la TX y RX
	USARTF0.CTRLB |= USART_RXEN_bm;
	USARTF0.CTRLB |= USART_TXEN_bm;
    
	// Habilito la interrupcion de Recepcion ( low level )
	// low level, RXint enabled
	USARTF0.CTRLA = ( USARTF0.CTRLA & ~USART_RXCINTLVL_gm ) | USART_RXCINTLVL_LO_gc;
    // Las transmisiones son por poleo no INT.
    
    rBchar_CreateStatic ( &TXRB_uart2, &uart2_txBuffer[0], UART2_TXSIZE  );
    rBchar_CreateStatic ( &RXRB_uart2, &uart2_rxBuffer[0], UART2_RXSIZE  );

}
//------------------------------------------------------------------------------
ISR(USARTF0_RXC_vect)
{

char cChar;

	cChar = USARTF0.DATA;
	rBchar_PokeFromISR( &RXRB_uart2, cChar );
}
//----------------------------------------------------------------------------------------
