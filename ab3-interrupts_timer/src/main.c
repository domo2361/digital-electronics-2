/* Defines */

#define LED_GREEN PB5
#define LED_RED PB0       // external LED-low


/* Includes */
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <gpio.h>           // GPIO library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC

int main(void)
{
    
    GPIO_mode_output(&DDRB, LED_GREEN);
    GPIO_mode_output(&DDRB, LED_RED);

    // Enable overflow interrupt
    TIM1_overflow_262ms();
    TIM1_overflow_interrupt_enable();

    //led blinking
    TIM0_overflow_16ms();
    TIM0_overflow_interrupt_enable();
    
    // Enables interrupts by setting the global interrupt mask
    sei();


    while(1)
    {


    }
    
    return 0;
}

// Interrupt service routines
ISR(TIMER1_OVF_vect)
{
    PORTB = PORTB ^ (1<<LED_GREEN); 
}


/* timer/counter0
external LED */

ISR(TIMER0_OVF_vect)
{
  uint8_t no_of_overflows = 0;

  no_of_overflows = no_of_overflows +1;

    PORTB = PORTB ^ (1<<LED_RED); 
}
