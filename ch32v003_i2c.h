#ifndef _CH32V003_I2C_H
#define _CH32V003_I2C_H
/**
 * @file ch32v003_i2c.h
 * @author Piotr Zapart (www.hexefx.com)
 * @brief i2c driver for the CH32V003 MCU
 * @version 0.1
 * @date 2023-07-01
 * 
 *  07-2023 - for now only the master and 7 bit address mode is implemented
 *  ------ WORK IN PROGRESS !!! -------
 * 
 * I2C pin mapping:
 * SCL=PC2, SDA=PC1, AFIO_PCFR1.I2C1REMAP1= 0, I2C1RM = 0 
 * SCL=PD1, SDA=PD0, AFIO_PCFR1.I2C1REMAP1= 0, I2C1RM = 1   <-- PD1 = SWIO !!!
 * SCL=PC5, SDA=PC6, AFIO_PCFR1.I2C1REMAP1= 1, I2C1RM = X
 * 
 * Usage:
 * 
 * 1. Optionally choose one of the altenate configurations, if not, 
 * 		the default setting (PC2+PC1) will be used 
 * #define I2C_PINS_SCLPD1_SDAPD0
 * #define I2C_PINS_SCLPC5_SDAPC6
 *
 * #define I2C_MODE_IRQ					// if using the interrrupt based driver
 * #define CH32V003_I2C_IMPLEMENTATION
 * #include "ch32v003_i2c.h"
 * 
 * in the main function:
 * 
 * call i2c_init(); 
 * 
 * To write data to a device with 1 byte register address range:
 * uint16_t err = i2c_write(	devAddr, 
 * 								regAddr, 
 * 								I2C_REGADDR_1B,
 * 								pointerToSrcBuffer, howManyBytes);
 * 
 *  * To write data to a device with 2 bytes register address range:
 *  uint16_t err = i2c_write(	devAddr, 
 * 								regAddr, 
 * 								I2C_REGADDR_2B,
 * 								pointerToSrcBuffer, howManyBytes);
 * To read data; 
 * uint16_t err = i2c_read(	devAddr, 
 * 							regAddr, 
 * 							I2C_REGADDR_1B,
 * 							pointerToDstBuffer, howManyBytes);
 * 
 * 2 byte address range, ie.: 24LC32 eeprom
 * uint16_t err = i2c_read(	devAddr, 
 * 							regAddr, 
 * 							I2C_REGADDR_2B,
 * 							pointerToDstBuffer, howManyBytes);
 * 
 * To scan the address space for devices:
 * void i2c_scan();
 * 
 * To ping a device:
 * uint8_t detected = i2c_ping(devAddr);
 * 
 * TODO: make the interrupt mode default?
 * TODO: error reporting
 */
#include <string.h>
#include "ch32v003fun.h"


#define I2C_AFIO_PCFR1_RESET_MASK	(0xFFBFFFFD)	// AFIO bit 1 & bit 22 

#if defined (I2C_PINS_SCLPD1_SDAPD0)
	#define I2C_PORT_RCC	RCC_APB2Periph_GPIOD
	#define I2C_PORT		GPIOD 
	#define I2C_PIN_SCL 	1
	#define I2C_PIN_SDA 	0
	#define I2C_AFIO_PCFR1	(1<<1)			//remap 01
#elif defined(I2C_PINS_SCLPC5_SDAPC6)
	#define I2C_PORT_RCC	RCC_APB2Periph_GPIOC
	#define I2C_PORT		GPIOC 
	#define I2C_PIN_SCL 	5
	#define I2C_PIN_SDA 	6
	#define I2C_AFIO_PCFR1	(1<<22)			//remap 01
#else
	#define I2C_PINS_SCLPC2_SDAPC1
	#define I2C_PORT_RCC	RCC_APB2Periph_GPIOC
	#define I2C_PORT		GPIOC 
	#define I2C_PIN_SCL 	2
	#define I2C_PIN_SDA 	1
	#define I2C_AFIO_PCFR1	0x00		//default 00
#endif

// use default 100kHz if nothing else is declared
#if !defined (I2C_CLKRATE)
	#define I2C_CLKRATE 	100000
#endif	
// I2C Logic clock rate - must be higher than Bus clock rate
#define I2C_PRERATE 		2000000

// uncomment this for high-speed 36% duty cycle, otherwise 33%
#define I2C_DUTY

// I2C Timeout count
#define I2C_TIMEOUT_MAX 	2000
// STAR1 mas for all i2c errors
#define I2C_STAR1_ERR_MASK	(	I2C_STAR1_PECERR 	| \
								I2C_STAR1_OVR 	 	| \
								I2C_STAR1_AF		| \
								I2C_STAR1_ARLO		| \
								I2C_STAR1_BERR		 )

#define I2C_INT_EN_MASK (I2C_CTLR2_ITBUFEN | I2C_CTLR2_ITEVTEN | I2C_CTLR2_ITERREN)

typedef enum
{
    I2C_RESULT_OK = 0,
    I2C_TIMEOUT_NOT_BUSY,
    I2C_TIMEOUT_NO_ACK
}i2c_result_e;

typedef enum
{
	I2C_REGADDR_1B = 0,			// register address range is 1 byte
	I2C_REGADDR_2B				// 2 bytes
}i2c_regAddr_bytes_e;

void i2c_init();
void i2c_setup();
static inline uint32_t i2c_chk_evt(uint32_t event_mask);
i2c_result_e i2c_write( uint16_t devAddr,
						uint16_t regAddr,
						i2c_regAddr_bytes_e regAddrBytes,
						uint8_t *data, uint8_t sz);
i2c_result_e i2c_read(	uint16_t devAddr,
						uint16_t regAddr,
						i2c_regAddr_bytes_e regAddrBytes,
						uint8_t *data, uint8_t sz);
i2c_result_e i2c_scan();
i2c_result_e i2c_ping(uint8_t addr, uint8_t *found) ;
/*----------------------------------------------------------*/
#ifdef CH32V003_I2C_IMPLEMENTATION

#ifdef I2C_MODE_IRQ
	volatile uint8_t *i2c_data_ptr; 				// pointer to the data buffer
	volatile uint8_t i2c_xfer_sz; 
	volatile uint8_t i2c_irq_state;
#endif
volatile uint16_t i2c_err_flags;
static inline uint16_t i2c_get_last_err(void);
static inline uint16_t i2c_get_last_err(void)
{
	uint16_t result = i2c_err_flags;
	i2c_err_flags = 0;
	return result;	
}
/*----------------------------------------------------------*/
void i2c_init() 
{
	RCC->APB2PCENR |= I2C_PORT_RCC | RCC_APB2Periph_AFIO;
	RCC->APB1PCENR |= RCC_APB1Periph_I2C1; 						// enable I2C
	// Setup SDA
	I2C_PORT->CFGLR &= ~(0x0F<<(4*I2C_PIN_SDA));
	I2C_PORT->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF)<<(4*I2C_PIN_SDA);	
	// Setup SCL
	I2C_PORT->CFGLR &= ~(0x0F<<(4*I2C_PIN_SCL));
	I2C_PORT->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF)<<(4*I2C_PIN_SCL);
	// alternate pin mapping
	AFIO->PCFR1 &= I2C_AFIO_PCFR1_RESET_MASK;
	AFIO->PCFR1 |= I2C_AFIO_PCFR1;

	i2c_setup();
}
/*----------------------------------------------------------*/
void i2c_setup() 
{
	uint16_t tmpu16;
	i2c_err_flags = 0;
	RCC->APB1PRSTR |= RCC_APB1Periph_I2C1;	// Reset I2C1 to init all regs
	RCC->APB1PRSTR &= ~RCC_APB1Periph_I2C1;

	tmpu16 = I2C1->CTLR2;
	tmpu16 &= ~I2C_CTLR2_FREQ;
	tmpu16 |= (FUNCONF_SYSTEM_CORE_CLOCK/I2C_PRERATE)&I2C_CTLR2_FREQ;
	I2C1->CTLR2 = tmpu16;

#if (I2C_CLKRATE <= 100000)		// standard mode good to 100kHz
	tmpu16 = ( FUNCONF_SYSTEM_CORE_CLOCK / (2*I2C_CLKRATE) ) & I2C_CKCFGR_CCR;
#else										// fast mode over 100kHz	
	#ifndef I2C_DUTY
		// 33% duty cycle
		tmpu16 = ( FUNCONF_SYSTEM_CORE_CLOCK / ( 3*I2C_CLKRATE ) ) & I2C_CKCFGR_CCR;
	#else
		// 36% duty cycle
		tmpu16 = ( FUNCONF_SYSTEM_CORE_CLOCK / ( 25*I2C_CLKRATE ) ) & I2C_CKCFGR_CCR;
		tmpu16 |= I2C_CKCFGR_DUTY;
	#endif
	tmpu16 |= I2C_CKCFGR_FS;
#endif
	I2C1->CKCFGR = tmpu16;
#ifdef I2C_MODE_IRQ
	NVIC_EnableIRQ(I2C1_EV_IRQn);			// enable IRQ driven operation	
	NVIC_EnableIRQ(I2C1_ER_IRQn); // Error interrupt
	i2c_irq_state = 0;						// initialize the state
	I2C1->CTLR1 |= I2C_NACKPosition_Next;
#endif
	I2C1->CTLR1 |= I2C_CTLR1_PE;			// Enable I2C	
}
/*----------------------------------------------------------*/
/**
 * @brief Check event status against a mask
 * 
 * @param event_mask mask to apply on the status word
 * @return uint32_t masked event status
 */
static inline uint32_t i2c_chk_evt(uint32_t event_mask)
{
	/* read order matters here! STAR1 before STAR2!! */
	uint32_t status = I2C1->STAR1 | ( I2C1->STAR2<<16 );
	return ( status & event_mask ) == event_mask;
}
/*----------------------------------------------------------*/
/**
 * @brief ping device address to check if it's responding
 * 
 * @param addr i2c device address
 * @param found pointer to return value, 1=found, 0=not found
 * @return i2c_result_e operation result
 */
i2c_result_e i2c_ping(uint8_t addr, uint8_t *found)
{
    i2c_result_e result = I2C_RESULT_OK;
	int32_t timeout = I2C_TIMEOUT_MAX;
	uint8_t reply = 1;
	while((I2C1->STAR2 & I2C_STAR2_BUSY) && (timeout--));
	if(timeout==-1)  	return I2C_TIMEOUT_NOT_BUSY;
	I2C1->CTLR1 |= I2C_CTLR1_START;
	while(!i2c_chk_evt(I2C_EVENT_MASTER_MODE_SELECT));
	I2C1->DATAR = (addr<<1) & 0xFE;							// send 7-bit address + write flag
	timeout = I2C_TIMEOUT_MAX;						// wait for transmit condition
	while((!i2c_chk_evt(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && (timeout--));
	if(timeout==-1)		reply = 0;							// if time out - no ack from the  device
	I2C1->CTLR1 |= I2C_CTLR1_STOP;	// set STOP condition
	if (found) *found = reply;
    return result;
}
/*----------------------------------------------------------*/
/**
 * @brief Scan the full range 0x00-0x7F of i2c addresses
 * 			and report back detected devices
 */
i2c_result_e i2c_scan()
{
	uint8_t i, found;
    i2c_result_e result = I2C_RESULT_OK;
	printf("   ");
	for (i = 0; i < 16; i++) { printf("%3x", i); }
	for (i = 0; i <= 127; i++)
	{
		if (i % 16 == 0)
		{
			printf("\n%02x:", i & 0xF0);
		}
        result = i2c_ping(i, &found);
		if (result !=I2C_RESULT_OK) {printf("I2C Error!"); break;}
		if (found) 	        { printf(" %02x", i);}
		else 				{ printf(" XX"); }		   
	}
	printf("\n\n");
    return result;
}
/*----------------------------------------------------------*/
/**
 * @brief sequential write 
 * 
 * @param devAddr 			i2c device address
 * @param regAddr 			register address (u16)
 * @param regAddrBytes 		reg address size in bytes: I2C_REGADDR_1B | I2C_REGADDR_2B
 * @param data 				pointer to a uint8_t source buffer
 * @param sz 				how many bytes to write
 * @return i2c_result_e 	
 */
i2c_result_e i2c_write( uint16_t devAddr,
						uint16_t regAddr,
						i2c_regAddr_bytes_e regAddrBytes,
						uint8_t *data, uint8_t sz)
{
	int32_t timeout;
	i2c_err_flags = 0x00;
#ifdef I2C_MODE_IRQ	
	while(i2c_irq_state);		// wait for previous packet to finish	
	i2c_xfer_sz = sz;			// init buffer for sending
	i2c_data_ptr = data;
#endif	
	// wait for not busy
	timeout = I2C_TIMEOUT_MAX;
	while((I2C1->STAR2 & I2C_STAR2_BUSY) && (timeout--));
	if(timeout==-1)		return I2C_TIMEOUT_NOT_BUSY;
	// Set START condition
	I2C1->CTLR1 |= I2C_CTLR1_START;
	// wait for master mode select
	while(!i2c_chk_evt(I2C_EVENT_MASTER_MODE_SELECT));
	I2C1->DATAR = (devAddr<<1) & 0xFE;		// send 7-bit address + write flag
	while((!i2c_chk_evt(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && (timeout--));
	if(timeout==-1)		return I2C_TIMEOUT_NO_ACK;
	if (regAddrBytes)
	{
		I2C1->DATAR = (regAddr>>8) & 0xFF;	// send hi address
		while(!(I2C1->STAR1 & I2C_STAR1_TXE));
	}	
	I2C1->DATAR = regAddr & 0xFF;			// send low address
	while(!(I2C1->STAR1 & I2C_STAR1_TXE));	

#ifdef I2C_MODE_IRQ
	// Enable interrupts
	I2C1->CTLR2 |= I2C_INT_EN_MASK;
	i2c_irq_state = 1;
#else	
	while(sz--)
	{	
		while(!(I2C1->STAR1 & I2C_STAR1_TXE));
		I2C1->DATAR = *data++; 							// send command
	}
	while(!i2c_chk_evt(I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C1->CTLR1 |= I2C_CTLR1_STOP;	// set STOP condition
#endif
	return I2C_RESULT_OK;
}
/*----------------------------------------------------------*/
/**
 * @brief sequential read 
 * 
 * @param devAddr 			i2c device address
 * @param regAddr 			register address (u16)
 * @param regAddrBytes 		reg address size in bytes: I2C_REGADDR_1B | I2C_REGADDR_2B
 * @param data 				pointer to a uint8_t receiver buffer
 * @param sz 				how many bytes to read
 * @return i2c_result_e 
 */
i2c_result_e i2c_read(	uint16_t devAddr,
						uint16_t regAddr,
						i2c_regAddr_bytes_e regAddrBytes,
						uint8_t *data, uint8_t sz)
{
	int32_t timeout;
#ifdef I2C_MODE_IRQ		
	while(i2c_irq_state);		// wait for previous packet to finish	
	i2c_xfer_sz = sz;			// init buffer for sending
	i2c_data_ptr = data;
#endif
	// wait for not busy
	timeout = I2C_TIMEOUT_MAX;
	while((I2C1->STAR2 & I2C_STAR2_BUSY) && (timeout--));
	if(timeout == -1)		return I2C_TIMEOUT_NOT_BUSY;
	// Set START condition
	I2C1->CTLR1 |= I2C_CTLR1_START;
	// wait for master mode select
	while(!i2c_chk_evt(I2C_EVENT_MASTER_MODE_SELECT));
	I2C1->DATAR = (devAddr<<1) & 0xFE;		// send 7-bit address + write flag
	timeout = I2C_TIMEOUT_MAX;		// wait for transmit condition
	while((!i2c_chk_evt(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && (timeout--));
	if(timeout == -1)		return I2C_TIMEOUT_NO_ACK;	
	if (regAddrBytes)
	{
		I2C1->DATAR = (regAddr>>8) & 0xFF;	// send hi address
		while(!(I2C1->STAR1 & I2C_STAR1_TXE));
	}	
	I2C1->DATAR = regAddr & 0xFF;			// send low address
	while(!(I2C1->STAR1 & I2C_STAR1_TXE));	
	
	if (sz > 1) I2C1->CTLR1 |= I2C_CTLR1_ACK;
	// Set repeat START condition
	I2C1->CTLR1 |= I2C_CTLR1_START;
	// wait for master mode select
	while(!i2c_chk_evt(I2C_EVENT_MASTER_MODE_SELECT));	
	I2C1->DATAR = (devAddr<<1) | 0x01;			// send 7-bit address + read flag
	timeout = I2C_TIMEOUT_MAX;		// wait for transmit condition
	while((!i2c_chk_evt(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) && (timeout--));
	if(timeout == -1)		return I2C_TIMEOUT_NO_ACK;
#ifdef I2C_MODE_IRQ
	// Enable interrupts
	I2C1->CTLR2 |= I2C_INT_EN_MASK;
	i2c_irq_state = 1;
	while (i2c_irq_state);
#else 
    while(sz--) 
	{
		if (!sz) I2C1->CTLR1 &= ~I2C_CTLR1_ACK; //signal it's the last byte
		while(!(I2C1->STAR1 & I2C_STAR1_RXNE));
		*data++ = I2C1->DATAR;
    }
	I2C1->CTLR1 |= I2C_CTLR1_STOP;	// set STOP condition	
#endif
	return I2C_RESULT_OK;	
}
/*----------------------------------------------------------*/
/**
 * @brief IRQ handler for I2C events
 * 
 * Side note about receiving the bytes: in order to avoid reading dummy bytes
 * caused by too late reset of the ACK bit in CTLR1 the ACK is enabled only for 
 * transfer sizes > 1 and while receving the one before last byte combined with 
 * I2C_NACKPosition_Next setting. 
 * Otherwise even if the read size is set to n, the i2c bus will try to read more
 * because the Master does not send NACK in time. This unnecessarily blocks the i2c
 * bus.
 */
#ifdef I2C_MODE_IRQ
void I2C1_EV_IRQHandler(void) __attribute__((interrupt));
void I2C1_EV_IRQHandler(void)
{
	uint16_t star1, err __attribute__((unused));
	uint8_t sz = i2c_xfer_sz;
	// read status, clear any events
	star1 = I2C1->STAR1;
                     // reset error flags;
	if(star1 & I2C_STAR1_TXE) // byte teansmitted
	{
		if(sz--)	
			I2C1->DATAR = *i2c_data_ptr++;			
		if(!sz)
		{
			I2C1->CTLR2 &= ~I2C_INT_EN_MASK;
			i2c_irq_state = 0;
			while(!i2c_chk_evt(I2C_EVENT_MASTER_BYTE_TRANSMITTED));		
			I2C1->CTLR1 |= I2C_CTLR1_STOP;
		}
	}
	if (star1 & I2C_STAR1_RXNE)			// data received
	{		
		if(sz--)	
		{
			if (sz == 1) I2C1->CTLR1 &= ~I2C_CTLR1_ACK; //signal it's the last byte
			*i2c_data_ptr++ = I2C1->DATAR;
		}
		if (!sz)
		{
			I2C1->CTLR2 &= ~I2C_INT_EN_MASK;
			i2c_irq_state = 0;
			I2C1->CTLR1 |= I2C_CTLR1_STOP;		// set STOP condition
		}
	}
	i2c_xfer_sz = sz;
}

void I2C1_ER_IRQHandler(void) __attribute__((interrupt));
void I2C1_ER_IRQHandler(void)
{
	uint16_t star1, err __attribute__((unused));
	star1 = I2C1->STAR1;
	err = star1 & I2C_STAR1_ERR_MASK;
	if (err)
	{
		i2c_err_flags = err; 						// store the error flags
		i2c_setup();								// reset the i2c bus
		return;
	}
    else i2c_err_flags = 0;  	
}


#endif // CH32V003_I2C_MODE_IRQ
#endif // CH32V003_I2C_IMPLEMENTATION

#endif // _CH32V003_I2C_H
