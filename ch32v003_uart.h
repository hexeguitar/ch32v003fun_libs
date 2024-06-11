#ifndef _CH32V003_UART_H
#define _CH32V003_UART_H
/**
 * @file ch32v003_uart.h
 * @author Piotr Zapart
 * @brief CH32V003 generic UART library, 8N1, default 115200baud, No parity, no flow control
 * @version 0.1
 * @date 2023-07-22
 * 
 *  Caution! 
 *  ============================================================
 *  Library will not work if FUNCONF_USE_UARTPRINTF is defined!
 *  ============================================================
 * 
 *	USART1 mapping configuration high (used in conjunction with AFIO PCFR1 register bit2
 *	USART1RM [21,2]).
 * 	00: default mapping (CK/PD4, TX/PD5, RX/PD6, CTS/PD3, RTS/PC2, SW_RX/PD5).
 *	01: Remapping (CK/PD7, TX/PD0, RX/PD1, CTS/PC3, RTS/PC2, SW_RX/PD0).
 *	10: Remapping (CK/PD7, TX/PD6, RX/PD5, CTS/PC6, RTS/PC7, SW_RX/PD6).
 *	11: Remapping (CK/PC5, TX/PC0, RX/PC1, CTS/PC6, RTS/PC7, SW_RX/PC0).
 *
 * Usage:
 * 1. In funconfig.h define the baudrate:
 *    #define FUNCONF_UART_PRINTF_BAUD 115200 //used for uart library
 * 
 * 2. In main source file: optionally define the TXRX pin configuration. If not defined
 *    the standard TX=PD5, RX=PD6 is used, alternative settings:
 *    #define UART_PINS_TXPD0_RXPD1 or
 *    #define UART_PINS_TXPC0_RXPC1 or
 *    #define UART_PINS_TXPD6_RXPD5 - this is the TXRX swap setting
 * 
 * 3. TX or RX can be disabled separately, for TX only UART add the following define
 *    #define UART_MODE_TX
 *    for RX only: #define UART_MODE_RX
 *    Default setting is both, TX and RX enabled
 * 	
 * 4. In main source file add
 *    #define CH32V003_UART_IMPLEMENTATION
 * 5. Then include the header file
 *    #include "ch32v003_uart.h"
 * 6. Initialize the UART:
 * 		UART_init();
 * 	To send a byte:
 * 		UART_putc(dataByte);
 * 	To print something:
 * 		UART_printf(...)
 *  To receive a byte:
 * 		uint16 in = UART_getc();
 * 	The higher byte contains error flags, function returns UART_NO_DATA if all has been 
 *  read from the receive buffer.
 *  	if( in != UART_NO_DATA )
 * 				receivedByte = in & 0xFF;
 * 
 *  TX and RX buffer sizes are set to 32 bytes. Both values have to be power of 2! 
 */
#include <stdarg.h>
#include <string.h>
#include "ch32v003fun.h"
// use uart only if debug printf is enables
#if defined( FUNCONF_USE_DEBUGPRINTF ) && FUNCONF_USE_DEBUGPRINTF

#define UART_TX_BF_SIZE	32	// tx buffer size, has to be power of 2!
#define UART_RX_BF_SIZE	32	// rx buffer size, has to be power of 2!

#define UART_TX_BF_MASK	(UART_TX_BF_SIZE-1)
#define UART_RX_BF_MASK	(UART_RX_BF_SIZE-1)

#if ( UART_TX_BF_SIZE & UART_TX_BF_MASK )
#error TX buffer size is not a power of 2
#endif
#if ( UART_RX_BF_MASK & UART_RX_BF_SIZE )
#error RX buffer size is not a power of 2
#endif

#define UART_IT_ERROR_MASK			(0x0E)		// FE + NE + ORE
#define UART_ERR_CHKSUM				(0x0100)	// Checksum error flag	| USART1_STATR << 8
#define UART_ERR_FRAME     			(0x0200)	// Frame error flag		|
#define UART_ERR_NOISE				(0x0400)	// Noise error flag		| 
#define UART_ERR_OVERRUN 		    (0x0800)	// Overload error flag	|
#define UART_ERR_RXBUF_OVERFLOW 	(0x0100)	// RX buffer overflow
#define UART_NO_DATA          		(0x8000) 	// No data received

#define UART_AFIO_PCFR1_RESET_MASK	(0xFFDFFFFB)	// bit 2 & bit 21 

#if defined(UART_PINS_TXPD0_RXPD1)
	#define UART_PORT_RCC	RCC_APB2Periph_GPIOD
	#define UART_PORT		GPIOD 
	#define UART_PIN_TX 	0
	#define UART_PIN_RX 	1
	#define UART_AFIO_PCFR1	(1<<2)							//remap 01
#elif defined (UART_PINS_TXPC0_RXPC1)
	#define UART_PORT_RCC	RCC_APB2Periph_GPIOC
	#define UART_PORT		GPIOC
	#define UART_PIN_TX 	0
	#define UART_PIN_RX 	1
	#define UART_AFIO_PCFR1	((1<<21)|(1<<2))				//remap 11
#elif defined (UART_PINS_TXPD6_RXPD5)
	#define UART_PORT_RCC	RCC_APB2Periph_GPIOD
	#define UART_PORT		GPIOD 
	#define UART_PIN_TX 	6
	#define UART_PIN_RX 	5
	#define UART_AFIO_PCFR1	(1<<21)							// remap 10
#else 
	#define UART_PINS_TXPD5_RXPD6
	#define UART_PORT_RCC	RCC_APB2Periph_GPIOD
	#define UART_PORT		GPIOD 
	#define UART_PIN_TX 	5
	#define UART_PIN_RX 	6
	#define UART_AFIO_PCFR1	(UART_AFIO_PCFR1_RESET_MASK)	//default 00
#endif
// if no config is provided, use tx+rx as default
#if !defined(UART_MODE_TX) && !defined(UART_MODE_RX) && !defined(UART_MODE_TXRX)
	#define UART_MODE_TXRX
#endif	

#if defined (UART_MODE_TXRX)
	#define UART_MODE_TX
	#define UART_MODE_RX
#endif

void UART_init();			// baudrate set in funconf.h using FUNCONF_UART_PRINTF_BAUD

#if defined(CH32V003_UART_IMPLEMENTATION)

#if defined(UART_MODE_TX)
	#define UART_IT_TXE_ENABLE		(0x0080)
	#define UART_IT_TXE_DISABLE		(0xFF7F)
	uint8_t uart_tx_bf[UART_TX_BF_SIZE]; // transmit buffer
	static volatile uint8_t uart_txHead, uart_txTail;
	static volatile uint16_t lastTXerrorFlag = 0;
	void UART_putc(uint8_t data);
	int UART_write(const char *buf, int size);
	static int UART_puts(char *s, int len, void *buf);
	int UART_printf(const char* format, ...);		
	int mini_vpprintf(int (*puts)(char* s, int len, void* buf), void* buf, const char *fmt, va_list va);
#endif

#if defined (UART_MODE_RX)
	#define UART_IT_RXC_ENABLE		(0x0020)
	#define UART_IT_RXC_DISABLE		(0xFFDF)
	uint8_t uart_rx_bf[UART_RX_BF_SIZE]; // receive buffer	
	static volatile uint8_t uart_rxHead, uart_rxTail;
	static volatile uint16_t lastRXerrorFlag = 0;
	uint16_t UART_getc(void);
#endif
// --------------------------------------------------------
/**
 * @brief Initialize USART1 with predefined settings
 */
void UART_init()
{
	uint16_t tmp16 = USART_WordLength_8b | USART_Parity_No;
	RCC->APB2PCENR |= UART_PORT_RCC | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO;

	AFIO->PCFR1 &= UART_AFIO_PCFR1_RESET_MASK;
	AFIO->PCFR1 |= UART_AFIO_PCFR1;

#if defined(UART_MODE_TX)
	UART_PORT->CFGLR &= ~(0xf<<(4*UART_PIN_TX));
	UART_PORT->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*UART_PIN_TX);
	tmp16 |= USART_Mode_Tx;
	uart_txHead = 0;
	uart_txTail = 0;
#endif
#if defined (UART_MODE_RX)
	UART_PORT->CFGLR &= ~(0xf<<(4*UART_PIN_RX));
	UART_PORT->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_IN_PUPD)<<(4*UART_PIN_RX);
	tmp16 |= USART_Mode_Rx | UART_IT_RXC_ENABLE;	// enable TX and rx complete interrupt
	uart_rxHead = 0;
	uart_rxTail = 0;	
#endif
	NVIC_EnableIRQ(USART1_IRQn);
	USART1->CTLR1 =  tmp16;
	USART1->CTLR2 = USART_StopBits_1;
	USART1->CTLR3 = USART_HardwareFlowControl_None;
	USART1->BRR = UART_BRR;
	USART1->CTLR1 |= CTLR1_UE_Set;	
}
// --------------------------------------------------------
#if defined (UART_MODE_RX)
/**
 * @brief  return byte from ringbuffer  
 * 
 * @return uint16_t l	lower byte:  received byte from ringbuffer
          				higher byte: last receive error
 */
uint16_t UART_getc(void)
{    
    uint8_t tmp8, data;

    if ( uart_rxHead == uart_rxTail ) return UART_NO_DATA;

    tmp8 = (uart_rxTail + 1) & UART_RX_BF_MASK;
    uart_rxTail = tmp8; 
    
    data = uart_rx_bf[tmp8];
    
    return (lastRXerrorFlag | data);
}
#endif
// --------------------------------------------------------
#if defined (UART_MODE_TX)
/**
 * @brief write function required by printf
 * 
 * @param fd file descriptor
 * @param buf pointer to the source buffer
 * @param size how many bytes to send
 * @return int number of bytes sent
 */
int _write(int fd, const char *buf, int size)
{
	return UART_write(buf, size);
}
// --------------------------------------------------------
/**
 * @brief put one byte into the transmit buffer, start TX
 * 
 * @param data byte to send
 */
void UART_putc(uint8_t data)
{
    uint8_t tmp8;    
    tmp8  = (uart_txHead+ 1) & UART_TX_BF_MASK;
    while (tmp8 == uart_txTail);
    uart_tx_bf[tmp8] = data;
	uart_txHead = tmp8;
	USART1->CTLR1 |= UART_IT_TXE_ENABLE;
}
// --------------------------------------------------------
/**
 * @brief write bulk data
 * 
 * @param buf pointer to the source buffer
 * @param size how many bytes to send
 * @return int number 
 */
int UART_write(const char *buf, int size)
{
	int rem = size;
	for(int i = 0; i < size; i++)
	{
		UART_putc(*buf++);
	    rem--;
	}
	return rem;
}
// --------------------------------------------------------
/**
 * @brief Put string, fn required by mini_vpprintf
 */
static int UART_puts(char *s, int len, void *buf)
{
	return UART_write(s, len);
}
// --------------------------------------------------------
/**
 * @brief printf using UART
 */
int UART_printf(const char* format, ...)
{
	va_list args;
	va_start( args, format );
	int ret_status = mini_vpprintf(UART_puts, 0, format, args);
	va_end( args );
	return ret_status;
}
#endif
// --------------------------------------------------------
/**
 * @brief USART 1 interrupt handler
 */
void USART1_IRQHandler(void) __attribute__((interrupt));
void USART1_IRQHandler(void)
{
	uint16_t status = USART1->STATR;
	uint16_t lastErr = 0;
	uint8_t tmp8, data;
	
	// check for errors and store the flags
	if (status & UART_IT_ERROR_MASK)
	{
		lastErr = (status & UART_IT_ERROR_MASK)<<8;
	}
#if defined (UART_MODE_RX)
	if (status & UART_IT_RXC_ENABLE)  // byte received, the same bit as in CTRL1
	{
		data = USART1->DATAR;
		tmp8 = (uart_rxHead + 1) & UART_RX_BF_MASK;
		if (tmp8 == uart_rxTail)	lastErr |= UART_ERR_RXBUF_OVERFLOW;
		uart_rxHead = tmp8;
		uart_rx_bf[tmp8] = data;
	}
	lastRXerrorFlag = lastErr & 0xFF00;
#endif
#if defined(UART_MODE_TX)
	if (status & UART_IT_TXE_ENABLE)  // tx empty, the same bit as in CTRL1
	{
		if (uart_txHead != uart_txTail)
		{
			tmp8 = (uart_txTail + 1) & UART_TX_BF_MASK;
			uart_txTail = tmp8;
			USART1->DATAR = uart_tx_bf[tmp8];
		}
		else 		// all sent, disable interrupt
		{
			USART1->CTLR1 &= UART_IT_TXE_DISABLE;
		}
		lastTXerrorFlag = lastErr & 0xFF00;
	}

#endif
}
#endif // CH32V003_UART_IMPLEMENTATION
#endif // defined( FUNCONF_USE_DEBUGPRINTF ) && FUNCONF_USE_DEBUGPRINTF

#endif // _CH32V003_UART_H
