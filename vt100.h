#ifndef _VT100_H
#define _VT100_H
/**
 * @file vt100.h
 * @author 	Piotr Zapart (www.hexefx.com)
 * @brief 	Little ANSI escape code library for the ch32v003fun project
 * @version 0.1
 * @date 2023-07-29
 * 
 * REQUIRES utils.h file! Otherwise add
 * 
 * int _write(int fd, const char *buf, int size);
 * at the top of your code to gain access to the internal _write function
 * 
 */

#include <stdint.h>
#include "ch32v003fun.h"
#include "utils.h"

typedef enum
{
	TR_ATTR_RESET = 0,
	TR_ATTR_BOLD,
	TR_ATTR_DIM,
	TR_ATTR_UNDERLINE,
	TR_ATTR_BLINK,
	TR_ATTR_REVERSE,
	TR_ATTR_HIDDEN,
	TR_ATTR_NOCHANGE = 0xFF
}tr_attr_t;

typedef enum
{
	TR_COLOR_BLACK = 0,
	TR_COLOR_RED,
	TR_COLOR_GREEN,
	TR_COLOR_YELLOW,
	TR_COLOR_BLUE,
	TR_COLOR_MAGENTA,
	TR_COLOR_CYAN,
	TR_COLOR_WHITE,
	TR_COLOR_NOCHANGE = 0xFF
}tr_color_t;

typedef enum
{
	TR_CURSOR_OFF	= 0,
	TR_CURSOR_ON	= 1
}tr_cursor_t;

typedef int (*tr_write_ft)(int fd, const char *buf, int size);
    
void tr_init(tr_write_ft write_callback);
void tr_reset(void);
void tr_cls(tr_cursor_t cur_onoff);						// clear screen, cursor on/off
void tr_setCursor(tr_cursor_t c);					// show cursor on/off
void tr_locate(uint16_t x, uint16_t y);				// set coordinates
void tr_textColor(tr_color_t cl);					// font color
void tr_bgColor(tr_color_t cl);			        	// background color
void tr_attr(	tr_attr_t atr, 						// set attributes: char attr., font color, background color
				tr_color_t fg, 						// use TR_ATTR_NOCHANGE or TR_COLOR_NOCHANGE
				tr_color_t bg);  					// to change chosen params only
void tr_attrReset(void);


void tr_fillLine(char ascii, uint8_t cnt);			// display a line cnt long using ascii char
void tr_clrLine_cur2end(void);
void tr_clrLine_beg2cur(void);
void tr_clrLine(void);

#if defined (VT100_IMPLEMENTATION)

#include "utils.h" 		// for _write

const char TR_RST[] = { "\x1b""c" };				// reset terminal
const char TR_UCLS[]  = { "\x1b""[2J" };			// clear screen
const char TR_UHOME[]  = { "\x1b""[;H" };			// cursor pos home
const char TR_UCUR_HIDE[]  = { "\x1b""[?25l" };		// cursor off
const char TR_UCUR_SHOW[]  = { "\x1b""[?25h" };		// cursor on
const char TR_U_ATTR_OFF[]  = { "\x1b""[m" };		// all attributes off
const char TR_CLR_LINE_CUR2END[] = {"\x1b""[0K"};	// clear line cursor->end
const char TR_CLR_LINE_BEG2CUR[] = {"\x1b""[1K"};	// clear line start->cursor
const char TR_CLR_LINE_ALL[] = {"\x1b""[2K"};		// 

tr_write_ft tr_write_cb;

/**
 * @brief Init the terminal, optionally provice a write callback,
 * 			compatible with ch32v003fun internal _write function.
 * 			NULL will make it use the internal _wrire
 * 
 * @param write_callback 
 */
void tr_init(tr_write_ft write_callback)
{
	if (write_callback) tr_write_cb = write_callback;
	else tr_write_cb = _write;
}

/**
 * @brief show or hide cursor
 * 
 * @param c TR_CURSOR_OFF or TR_CURSOR_ON
 */
void tr_setCursor(tr_cursor_t c)
{
	if (tr_write_cb)
    {
        if(c)    	tr_write_cb(0, TR_UCUR_SHOW, sizeof(TR_UCUR_SHOW));
	    else        tr_write_cb(0, TR_UCUR_HIDE, sizeof(TR_UCUR_HIDE));
    }
}
/**
 * @brief Clear screen
 * 
 * @param c cursor on/off
 */
void tr_cls(tr_cursor_t c) 
{
    if (tr_write_cb)
    {
        tr_write_cb(0, TR_U_ATTR_OFF, sizeof(TR_U_ATTR_OFF));
        tr_setCursor(c);
        tr_write_cb(0, TR_UCLS, sizeof(TR_UCLS));
        tr_write_cb(0, TR_UHOME, sizeof(TR_UHOME));
    }
}

/**
 * @brief print a separator line
 * 
 * @param ascii ascii symbol, ie. '-' (not "-")
 * @param cnt line length
 */
void tr_fillLine(char ascii, uint8_t cnt) 
{
    uint8_t i;
	char symbol[] = {ascii};
	char end[] = {0};
    if (tr_write_cb)
    {
        for(i=0; i<cnt; i++) tr_write_cb(0, (const char *)symbol, 1);
    }
	tr_write_cb(0, (const char *)end, 1);
}

/**
 * @brief set attibutes
 * 
 * @param atr 	text attibute, use TR_ATTR_NOCHANGE if not used
 * @param fg 	foreground color, use TR_COLOR_NOCHANGE if not used
 * @param bg 	background color, use TR_COLOR_NOCHANGE if not used
 */
void tr_attr(tr_attr_t atr, tr_color_t fg, tr_color_t bg) 
{
    if (tr_write_cb)
    { 
		uint8_t bf[10];
		uint8_t *p = bf;
		*p++  = 0x1b;
		*p++ = '[';
		if (atr != TR_ATTR_NOCHANGE)
		{
			*p++ = (uint8_t)atr + '0';
		}
		if (fg != TR_COLOR_NOCHANGE)
		{
			*p++ = ';';
			*p++ = '3';
			*p++ = (uint8_t)fg + '0';
		}
		if (bg != TR_COLOR_NOCHANGE)
		{
			*p++ = ';';
			*p++ = '4';
			*p++ = (uint8_t)bg + '0';			
		}
		*p++ = 'm';
		tr_write_cb(0, (const char *)bf, p - bf);
    }
}
/**
 * @brief Reset all attributes
 */
void tr_attrReset(void)
{
	if (tr_write_cb)
	{
		tr_write_cb(0, TR_U_ATTR_OFF, sizeof(TR_U_ATTR_OFF));
	}
}
/**
 * @brief Reset terminal
 */
void tr_reset(void)
{
	if (tr_write_cb)
	{
		tr_write_cb(0, TR_RST, sizeof(TR_RST));
	}
}

void tr_textColor(tr_color_t cl) 
{
    if (tr_write_cb)
    {
		uint8_t bf[5];
		bf[0] = 0x1b;
		bf[1] = '[';
		bf[2] = '3';
		bf[3] = (uint8_t)cl + '0';
		bf[4] = 'm';	
		tr_write_cb(0, (const char *)bf, 5);
    }
}
/**
 * @brief set background color
 * 
 * @param cl color value
 */
void tr_bgColor(tr_color_t cl) 
{
    if (tr_write_cb)
    {
		uint8_t bf[5];
		bf[0] = 0x1b;
		bf[1] = '[';
		bf[2] = '4';
		bf[3] = (uint8_t)cl + '0';
		bf[4] = 'm';	
		tr_write_cb(0, (const char *)bf, 5);
    }
}

/**
 * @brief Set cursor coordinates
 * 
 * @param x x position
 * @param y y position
 */
void tr_locate( uint16_t x, uint16_t y ) 
{
    if (tr_write_cb)
    {
		uint8_t lx, ly;
		// max bf length: 2*(uint16+'\0') + 4 = 2*(5+1)+4 = 16
		uint8_t bf[16];	

		bf[0] = 0x1b;
		bf[1] = '[';
		lx = i32toa(x, &bf[2]);
		bf[2+lx] = ';';
		ly = i32toa(y, &bf[3+lx]);
		bf[3+lx+ly] = 'H';
		tr_write_cb(0, (const char *)bf, 4+lx+ly);
    }
}

/**
 * @brief Clear line from cursor to end
 */
void tr_clrLine_cur2end(void)
{
    if (tr_write_cb)
    {
        tr_write_cb(0, TR_CLR_LINE_CUR2END, sizeof(TR_CLR_LINE_CUR2END));
    }
}

/**
 * @brief Clear line from the beginning to the cursos position
 */
void tr_clrLine_beg2cur(void)
{
    if (tr_write_cb)
    {
        tr_write_cb(0, TR_CLR_LINE_BEG2CUR, sizeof(TR_CLR_LINE_BEG2CUR));
    }
}

/**
 * @brief Clear line
 */
void tr_clrLine(void)
{
    if (tr_write_cb)
    {
        tr_write_cb(0, TR_CLR_LINE_ALL, sizeof(TR_CLR_LINE_ALL));
    }
}

#endif // VT100_IMPLEMENTATION

#endif // _VT100_H
