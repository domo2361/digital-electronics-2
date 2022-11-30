/***********************************************************************
 * 
 * Blink LEDs in Arduino-style and use function from the delay library.
 * ATmega328P (Arduino Uno), 16 MHz, PlatformIO
 *
 * Copyright (c) 2022 Tomas Fryza
 * Dept. of Radio Electronics, Brno University of Technology, Czechia
 * This work is licensed under the terms of the MIT license.
 * 
 **********************************************************************/


/* Defines -----------------------------------------------------------*/
#define SW PD0                     
#define A PD1 
#define B PD2
#define SHORT_DELAY 250 // Delay in milliseconds
#ifndef F_CPU
# define F_CPU 16000000 // CPU frequency in Hz required for delay funcs
#endif



/* Includes ----------------------------------------------------------*/
#include <avr/io.h>     // AVR device-specific IO definitions
#include <util/delay.h> // Functions for busy-wait delay loops
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <gpio.h>           // GPIO library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <lcd.h>            // Peter Fleury's LCD library
#include <stdlib.h>         // C library. Needed for number conversions

/*                */
uint8_t R = 0;



/*                 */

// -----
// This part is needed to use Arduino functions but also physical pin
// names. We are using Arduino-style just to simplify the first lab.
//#include "Arduino.h"
//#define PB5 13          // In Arduino world, PB5 is called "13"
//#define PB0 8
// -----


/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Toggle LEDs and use delay library.
 * Returns:  none
 **********************************************************************/
int main(void)
{
    lcd_init(LCD_DISP_ON);
    char string[4];

    // Infinite loop
    while (1)
    {
      if (GPIO_read(&DDRD, A) > GPIO_read(&DDRD, B))
      {
        R++;
        if (R > 9)
        {
          R = 0;
        }
      }
      else if (GPIO_read(&DDRD, B) > GPIO_read(&DDRD, A))
      {
        R--;
        if (R==255)
        {
          R = 9;
        }
      }
      lcd_gotoxy(1, 0); lcd_puts("x:");
      itoa(R, string, 10);
      lcd_gotoxy(8, 0); lcd_puts("    ");
      lcd_gotoxy(8, 0); lcd_puts(string);
    }    
    return 0;
}
