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
#define CLK PB3   //main output from rotary encoder                   
#define DT PB4    //reference output from rotary encoder
#define SW_ENC PB5  //button output from rotary encoder
#define SW_JS PD3   //button output from joystick

#define DIGIT_H 8   //height of one digit in pixels
#define DIGIT_W 5   //width of one digit in pixels
#define DISP_H 2    //height of display in digits
#define DISP_W 16   //width of display in digits

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
#include <uart.h>           // Peter Fleury's UART library

/*                */
uint16_t x_axis = 512;
uint16_t y_axis = 512;
uint8_t axis_select = 0;

uint8_t grid[DISP_W*DISP_H*DIGIT_H] = {0};
uint8_t cursor_pos_x = 9;
uint8_t cursor_pos_y = 13;

/*                 */



/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Toggle LEDs and use delay library.
 * Returns:  none
 **********************************************************************/
int main(void)
{   
  
    GPIO_mode_input_nopull(&DDRB, CLK);
    GPIO_mode_input_nopull(&DDRB, DT);
    GPIO_mode_input_pullup(&DDRB, SW_ENC);
    GPIO_mode_input_pullup(&DDRD, SW_JS);
    uint8_t encoder_ref = GPIO_read(&PINB, CLK);
    int8_t counter = 0;


    void toggle_pixel(void)
    { 
      //position of pixel in terms of 16x2 display digits
      uint8_t digit_pos_x = cursor_pos_x / DIGIT_W;
      uint8_t digit_pos_y = cursor_pos_y / DIGIT_H;
      //first byte of this digit inside the grid array
      uint8_t digit_in_grid = digit_pos_y * DISP_W * (DISP_H-1) * DIGIT_H + digit_pos_x * DIGIT_H;
      //location of byte containing the pixel inside the grid array
      uint8_t byte = digit_in_grid + cursor_pos_y % DIGIT_H;
      //location of the pixel in its byte, 0-5  MSB-LSB
      uint8_t bit = cursor_pos_x % DIGIT_W; 

      grid[byte] ^= (0b10000>>bit); 
      /*lcd_command(1<<LCD_CGRAM);
      for (uint8_t i = 0; i < DIGIT_H; i++)
      {
        lcd_data(grid[digit_in_grid + i]);
      }
      lcd_command(1<<LCD_DDRAM);
      lcd_gotoxy(digit_pos_x, digit_pos_y);
      lcd_putc(0x00);*/

      /*lcd_command(1<<LCD_CGRAM);
      for (uint8_t i = 0; i < 8*DIGIT_H; i++)
      {
        lcd_data(grid[i]);
      }
      lcd_command(1<<LCD_DDRAM);
      lcd_gotoxy(0,0);
      for (uint8_t i = 0; i < 8; i++)
        lcd_putc(i);*/

      lcd_command(1<<LCD_CGRAM);
      for (uint8_t i = 0; i < 2; i++)
      {
        for (uint8_t j = 0; j < DIGIT_H*4; j++)
        {
          lcd_data(grid[i*DIGIT_H*16 + j]);
        }
      }
      lcd_command(1<<LCD_DDRAM);
      for (uint8_t i = 0; i < 2; i++)
      {
        for (uint8_t j = 0; j < 4; j++)
        {
          lcd_gotoxy(i*16+j, i);
          lcd_putc(4*i+j);
        }
      } 
    }

    int8_t get_encoder(void)
    {
      int8_t step = 0;
      uint8_t clk_val = GPIO_read(&PINB, CLK);

      if(clk_val != encoder_ref){ 
        if(GPIO_read(&PINB, DT) != clk_val)
          //step = -1;
          counter--;
        else
          //step = 1;
          counter++;
      }else
        step = 0;
      
      encoder_ref = clk_val;
      return step;
    }

    uint8_t get_joystick(void)
    { 
      uint8_t position = 0;
      for(uint8_t i = 0; i < 2; i++)
      {
        if(i == 0)
        {
        // set ADC output variable to x axis
          axis_select = 0;
        // Select input channel ADC0 for x axis
          ADMUX &= ~((1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<MUX3));
        // Start ADC conversion
          ADCSRA |= (1<<ADSC);
          _delay_us(125);
        }
        else
        {
        // set ADC output variable to y axis
          axis_select = 1;
        // Select input channel ADC1 for y axis
          ADMUX |= (1<<MUX0);  ADMUX &= ~((1<<MUX1) | (1<<MUX2) | (1<<MUX3));
        // Start ADC conversion
          ADCSRA |= (1<<ADSC);
        }
      }
      if(x_axis < 400)
        //position = 4;
        cursor_pos_x--;
      else if(y_axis > 625)
        //position = 1;
        cursor_pos_y--;
      else if(y_axis < 400)
        //position = 3;
        cursor_pos_y++;
      else if(x_axis > 625)
        //position = 2;
        cursor_pos_x++;
      else
        position = 0;

      return position;
    }

    //
    lcd_init(LCD_DISP_ON);
  

    // Configure Analog-to-Digital Convertion unit
    // Select ADC voltage reference to "AVcc with external capacitor at AREF pin"
    ADMUX |= (1<<REFS0);  //setting REFS0 to 1
    ADMUX &= ~(1<<REFS1); //setting REFS1 to 0
    // Enable ADC module
    ADCSRA |= (1<<ADEN);
    // Enable conversion complete interrupt
    ADCSRA |= (1<<ADIE);
    // Set clock prescaler to 128
    ADCSRA |= ((1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2));

    // Enables interrupts by setting the global interrupt mask
    sei();
     
    
    // Infinite loop
    while (1)
    {
        char string[4];
        get_joystick();
        get_encoder();
        itoa(counter, string, 10);
        lcd_gotoxy(5,1);
        lcd_puts("    ");
        lcd_gotoxy(5,1);
        lcd_puts(string);
        if(GPIO_read(&PINB, SW_ENC) == 0)
        {
          toggle_pixel();
        }
        _delay_ms(15);
    }    
    return 0;

    

}

/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
 * Function: Timer/Counter1 overflow interrupt
 * Purpose:  Use single conversion mode and start conversion every 100 ms.
 **********************************************************************/



/**********************************************************************
 * Function: ADC complete interrupt
 * Purpose:  Display converted value on LCD screen.
 **********************************************************************/
ISR(ADC_vect)
{   
  if(axis_select == 0)
    x_axis = ADC;
  else 
    y_axis = ADC;
}

/*ISR(TIM1_OVF_vect)
{ 
    static uint8_t digit_block = 0;
    lcd_command(1<<LCD_CGRAM);
    for (uint8_t i = 0; i < DIGIT_H*8; i++)
    {
      lcd_data(grid[digit_block*DIGIT_H*8 + i]);
    }
    lcd_command(1<<LCD_DDRAM);
    lcd_gotoxy(digit_block*8, digit_block/2);
    for (uint8_t i = 0; i < 8; i++)
    {
      lcd_putc(i);
    }
    digit_block++;
    if(digit_block==4)
      digit_block = 0;
}*/
