#pragma once
// $Id: uart.h,v 1.3 2009/07/23 22:53:39 gian Exp $

#include "common.h"

//#define UART_BAUD 9600
#define BAUD_9600	1
#define BAUD_2400	2
#define BAUD_38400	3

//Perform UART startup initialization.
void	USART_init(uint8_t b);

// Send one character to the UART.
unsigned char	USART_rx(void);

// Receive one character from the UART.  The actual reception is
// line-buffered, and one character is returned from the buffer at
// each invokation.
void	USART_tx(unsigned char);
void 	USART_tx_S(const char *);

void 	USART_tx_P(const char *);

