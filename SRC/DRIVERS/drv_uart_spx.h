/*
 * drv_uart_spx.h
 *
 *  Created on: 8 dic. 2018
 *      Author: pablo
 */

#ifndef SRC_SPX_DRIVERS_DRV_UART_SPX_H_
#define SRC_SPX_DRIVERS_DRV_UART_SPX_H_

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "FreeRTOS.h"
#include "ringBuffer.h"


//------------------------------------------------------------------------------
#define UART1_TXSIZE	8	// trasmito por poleo. Si uso interrupcion lo subo a 128
uint8_t uart1_txBuffer[UART1_TXSIZE];
#define UART1_RXSIZE	64	// 
uint8_t uart1_rxBuffer[UART1_RXSIZE];
rBchar_s TXRB_uart1, RXRB_uart1;

void drv_uart1_init(uint32_t baudrate );

//------------------------------------------------------------------------------
#define UART2_TXSIZE	8	// trasmito por poleo. Si uso interrupcion lo subo a 128
uint8_t uart2_txBuffer[UART2_TXSIZE];
#define UART2_RXSIZE	64	// 
uint8_t uart2_rxBuffer[UART2_RXSIZE];
rBchar_s TXRB_uart2, RXRB_uart2;

void drv_uart2_init(uint32_t baudrate );

//------------------------------------------------------------------------------



#endif /* SRC_SPX_DRIVERS_DRV_UART_SPX_H_ */
