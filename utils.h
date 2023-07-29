#ifndef _UTILS_H
#define _UTILS_H

/**
 * various utility functions for ch32v003fun
 */

#include <stdint.h>

typedef int (*puts_ft)(char* s, int len, void* buf);

int mini_snprintf(char* buffer, unsigned int buffer_len, const char *fmt, ...);

int _write(int fd, const char *buf, int size);

static inline int32_t abs_i32(int32_t x)  __attribute__((always_inline,unused));
static inline int32_t abs_i32(int32_t x)
{
	return (x < 0)? -x : x;
}

/**
 * @brief Fast multiply for ch32v003, 
 * 			author: CNLohr
 * 			source: https://github.com/cnlohr/ch32v003fun/tree/master/examples/ws2812bdemo
 */
static inline uint32_t FastMultiply( uint32_t big_num, uint32_t small_num ) __attribute__((section(".data")));
static inline uint32_t FastMultiply( uint32_t big_num, uint32_t small_num )
{
	uint32_t ret = 0;
	uint32_t multiplicand = small_num;
	uint32_t mutliplicant = big_num;
	do
	{
		if( multiplicand & 1 )
			ret += mutliplicant;
		mutliplicant<<=1;
		multiplicand>>=1;
	} while( multiplicand );
	return ret;
}

#if defined (UTILS_IMPLEMETATION)

/**
 * @brief print a binary representation of max 32bit value
 * 
 * @param v 			input value
 * @param num_places 	how many places, ie. 8 for an 8bit input
 * @param puts_cb 		callback for bulk write function, if NULL
 * 						internal _write (debug printf of uart printf)
 * 						is used. 
 * 						Example:	printBin(0x5A5A, 16, NULL);
 */
void printBin(uint32_t v, uint8_t num_places, puts_ft puts_cb)
{
    uint32_t mask=0, n;

    for (n=1; n<=num_places; n++)
    {
        mask = (mask << 1) | 0x0001UL;
    }
    v = v & mask;  // truncate v to specified number of places
    while(num_places)
    {
        if (v & (0x0001 << (num_places-1))) 
        {
			if (puts_cb) 	puts_cb("1", 2, NULL);
			else		_write(0,"1", 2);
        }
        else                                
        {
			if (puts_cb) 	puts_cb("0", 2, NULL);
			else		_write(0,"0", 2);
        }
        --num_places;
        if(((num_places%4) == 0) && (num_places != 0))
        {
			if (puts_cb) 	puts_cb("_", 2, NULL);
			else		_write(0,"_", 2);
        }
    }
	if (puts_cb) 	puts_cb("\r\n", 5, NULL);
			else		_write(0,"\r\n", 5);
}

const uint32_t i32_pow10[10] = {1000000000, 100000000, 10000000, 1000000, 100000, 10000, 1000, 100, 10, 1};
/**
 * @brief simple itoa not using division or modulo
 * 
 * @param inVal input value
 * @param dstBf pointer to destination buffer
 * @return int  number of written bytes
 */
int i32toa(int32_t inVal, uint8_t *dstBf)
{
	int8_t digit[10] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
	int8_t idx = 0;
	uint8_t valStarted = 0;
	uint8_t neg = inVal < 0;
	uint8_t *p = dstBf;
	inVal = abs_i32(inVal);
	while( idx < (sizeof(i32_pow10)/sizeof(int32_t)) )
	{
		while (inVal >= 0)
		{
			inVal -= i32_pow10[idx];
			digit[idx]++;		
		}	
		inVal += i32_pow10[idx];
		if ((digit[idx] != 0 && !valStarted) || idx == 9) 
		{
			valStarted = 1;
			if (neg) *p++ = '-';
		}
		if (valStarted)		*p++ = '0' + digit[idx];
		idx++;
	}
	*p++ = '\0';
	return (p - dstBf);
}

#endif // UTILS_IMPLEMETATION

#endif // _UTILS_H
