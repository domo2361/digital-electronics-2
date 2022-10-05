/***********************************************************************
 * 
 * Blink a LED in Arduino-style and use function from the delay library.
 * ATmega328P (Arduino Uno), 16 MHz, PlatformIO
 *
 * Copyright (c) 2022 Tomas Fryza
 * Dept. of Radio Electronics, Brno University of Technology, Czechia
 * This work is licensed under the terms of the MIT license.
 * 
 **********************************************************************/


/* Defines -----------------------------------------------------------*/
//#define LED_RED PB0   // PB5 is AVR pin where green on-board LED 
#define LED_GREEN PB5   // PB5 is AVR pin where red on-board LED 
#define BUTTON PB0
                        // is connected
#define SHORT_DELAY 250 // Delay in milliseconds
#define LONG_DELAY 500 // Delay in milliseconds
#ifndef F_CPU
# define F_CPU 16000000 // CPU frequency in Hz required for delay funcs
#endif

/* Includes ----------------------------------------------------------*/
#include <avr/io.h>     // AVR device-specific IO definitions
#include <util/delay.h> // Functions for busy-wait delay loops
#include <gpio.h> 
#include <gpio.c>

// -----
// This part is needed to use Arduino functions but also physical pin
// names. We are using Arduino-style just to simplify the first lab.
//#include "Arduino.h"
#define PB5 13          // In Arduino world, PB5 is called "13"
#define PB0 8          // In Arduino world, PB0 is called "8"

// -----


/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Toggle one LED and use delay library.
 * Returns:  none
 **********************************************************************/
int main(void)
{
    //uint8_t led_value = LOW;  // Local variable to keep LED status

    // Set pin where on-board LED is connected as output
    //pinMode(LED_GREEN, OUTPUT);
    //pinMode(LED_RED, OUTPUT);
    GPIO_mode_output(&DDRB, LED_GREEN);
    GPIO_mode_input_pullup(&DDRB, BUTTON);

    // Infinite loop
    while (1)
    {
      if (GPIO_read(&PINB, BUTTON))
      {
        GPIO_write_high(&PORTB, LED_GREEN);
        _delay_ms(250);

        GPIO_write_low(&PORTB, LED_GREEN);
        _delay_ms(250);
      }
      else
        GPIO_write_low(&PORTB, LED_GREEN);
    
    }
    // Will never reach this
    return 0;
}