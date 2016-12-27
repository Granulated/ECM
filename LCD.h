// by Benjamin Hahn and Edward Brewin

#define _XTAL_FREQ 8000000

#include <xc.h>

// functions to send strings to LCD screen
void E_TOG(void);
void LCDout(unsigned char number);
void SendLCD(unsigned char Byte, char type);
void LCD_Init(void);
void SetLine(char line);
void LCD_String(char *string);
