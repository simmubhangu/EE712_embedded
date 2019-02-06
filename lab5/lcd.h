/*
 * switch_handler.c
 *
 *  Created on: 25-Jan-2019
 *      Author: simmu
 */


#ifndef LCD_H_
#define LCD_H_

void lcd_init(void);
void lcd_command(char);
void lcd_char(char);
void lcd_string(char *);
void lcd_print(char,char,uint32_t, int);
void lcd_cursor(char,char);


#endif /* LCD_H_ */
