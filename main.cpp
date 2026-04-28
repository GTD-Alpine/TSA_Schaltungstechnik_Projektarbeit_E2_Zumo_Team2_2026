//---------------------------------------------------------------------------------------------------
//
// Übung LCD 01.12.2025
//
//---------------------------------------------------------------------------------------------------

#define F_CPU 3686400UL
#define BaudRate 9600

#include <avr/io.h>
#include <avr/delay.h>
#include <stdlib.h>
//#include <avr/eeprom.h>
//#include <avr/wdt.h>
#include <avr/interrupt.h>
//#include "lcd.h"
//#include "lcd.c"
//#include <stdbool.h>

#define BYTE uint8_t

volatile BYTE count = 0;

ISR(INT0_vect){cli(); count++; sei();}
ISR(INT1_vect){cli(); count--; sei();}

int main (void)
{
    DDRD = 0x00;
    PORTD = 0xff;
    DDRB = 0xff;
    DDRC = 0xff;
    MCUCR = (1<<ISC01) | (1<<ISC11);
    GICR |= (1<<INT0);
    GICR |= (1<<INT1);
    sei();

    while(1){PORTC=count;PORTB=0xff;_delay_us(20);PORTB=0x00;_delay_us(20);}
}