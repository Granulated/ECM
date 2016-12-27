// functions to send strings to LCD screen
// by Benjamin Hahn and Edward Brewin

#include "LCD.h"
#include <xc.h>

//function to toggle enable bit on then off
void E_TOG(void) {
    //don?t forget to put a delay between the on and off
    //commands! 5us will be plenty.
    PORTCbits.RC0 = 1;
    __delay_us(5); // 5us delay ? remember to define _XTAL_FREQ
    PORTCbits.RC0 = 0;
    __delay_us(5);
}

//function to send four bits to the LCD
void LCDout(unsigned char number) {
    //set data pins using the four bits from number
    //LATC = LATC | (number & 0b
    PORTCbits.RC1 = number & 0b00000001;
    PORTCbits.RC2 = (number >> 1) & 0b00000001;
    PORTDbits.RD0 = (number >> 2) & 0b00000001;
    PORTDbits.RD1 = (number >> 3) & 0b00000001;

    //toggle the enable bit to send data
    //LATCbits.LATC0 = 1;
    E_TOG();
    __delay_us(5); // 5us delay
}

//function to send data/commands over a 4bit interface
void SendLCD(unsigned char Byte, char type) {
    // set RS pin whether it is a Command (0) or Data/Char (1)
    // using type as the argument
    PORTAbits.RA6 = type;

    // send high bits of Byte using LCDout function
    LCDout((Byte >> 4) & 0b00001111);
    __delay_us(10); // 10us delay

    // send low bits of Byte using LCDout function
    LCDout(Byte & 0b00001111);
    __delay_us(10);
}

void LCD_Init(void) {
    // set initial LAT output values (they start up in a random state)
    LATA = 0;
    LATC = 0;
    LATD = 0;

    // set LCD pins as output (TRIS registers)
    TRISA = 0;
    TRISC = 0;
    TRISD = 0;

    // Initialisation sequence code - see the data sheet
    //delay 15mS
    __delay_ms(16);
    //send 0b0011 using LCDout
    LCDout(0b0011);
    //delay 5ms
    __delay_ms(5);
    //send 0b0011 using LCDout
    LCDout(0b0011);
    //delay 200us
    __delay_us(200);
    //send 0b0011 using LCDout
    LCDout(0b0011);
    //delay 50us
    __delay_us(50);
    //send 0b0010 using LCDout set to four bit mode
    LCDout(0b0010);
    __delay_us(50);

    // now use SendLCD to send whole bytes - send function set, clear
    // screen, set entry mode, display on etc to finish initialisation

    //    SendLCD(0b00001011,0); // display off
    //    __delay_us(50);
    SendLCD(0b00101000, 0); //send function set (display line, character font)
    __delay_us(50);
    SendLCD(0b00001010, 0); // display off
    __delay_us(50);
    SendLCD(0b00000001, 0); // clear screen
    __delay_ms(2);
    SendLCD(0b00000110, 0); // set entry mode
    __delay_us(50);
    SendLCD(0b00001110, 0); // display on
    __delay_us(50);
}

//function to put cursor to start of line
void SetLine(char line) {

    if (line == 1) //Send 0x80 to set line to 1 (0x00 ddram address)
    {
        //SendLCD(0b10000000,0);
        //__delay_us(50);
        SendLCD(0x80, 0);
    } else if (line == 2) //Send 0xC0 to set line to 2 (0x40 ddram address)    
    {
        //SendLCD(0b11000000,0);
        //__delay_us(50);
        SendLCD(0xC0, 0);
    }

    __delay_us(50); // 50us delay
}

void LCD_String(char *string) {
    //While the data pointed to isn?t a 0x00 do below
    while (*string != 0) {
        //Send out the current byte pointed to
        // and increment the pointer
        SendLCD(*string++, 1);
        __delay_us(50); //so we can see each character
        //being printed in turn (remove delay if you want
        //your message to appear almost instantly)
    }
}

