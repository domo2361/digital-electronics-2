# digital-electronics-2

# digital-electronics-2

## H1
*hello* 
**hello**
## H2
## H3

 if ( sla<120 ){
      ack = twi_start(sla, TWI_WRITE);
      twi_stop();

      itoa(sla, string,10);
      uart_puts(string);

      if (ack == 0){
        uart_puts("\t");
        uart_puts("OK");
      }
      sla++;
      uart_puts("\r\n");
    } 
