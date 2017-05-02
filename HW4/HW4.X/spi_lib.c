#include "spi_lib.h"
#include<math.h>
#include<sys/attribs.h>
#include<xc.h>  

#define CS LATBbits.LATB7 

unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}
void init() {
  // set up the chip select pin as an output
  // the chip select pin is used by the sram to indicate
  // when a command is beginning (clear CS to low) and when it
  // is ending (set CS high)
  TRISBbits.TRISB7 = 0;
  RPB8R=0b0011;
  SDI1R=0b0000;
  CS = 1;

  // Master - SPI4, pins are: SDI4(F4), SDO4(F5), SCK4(F13).  
  // we manually control SS4 as a digital output (F12)
  // since the pic is just starting, we know that spi is off. We rely on defaults here
 
  // setup spi4
  SPI1CON = 0;              // turn off the spi module and reset it
  SPI1BUF;                  // clear the rx buffer by reading from it
  SPI1BRG = 0x3;            // baud rate to 10 MHz [SPI4BRG = (80000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.ON = 1;       // turn on spi 4
  SPI1CONbits.MODE16=0;
  SPI1CONbits.MODE32=0;
  SPI1CON2bits.AUDEN=0;
                 
}
void setVoltage(unsigned char channel, unsigned char voltage){
    CS=0;
    if(channel==1){
      spi_io(0b10110000+(voltage >> 4)); //channel B
      spi_io((voltage&0b00001111) << 4);
      
   } else if(channel==0){ //channel A
      spi_io(0b00110000+(voltage >> 4));
      spi_io((voltage&0b00001111) << 4);
   } 
    CS=1;
}
unsigned char tWave(int a, int b, int c){
    double i; char o;
    i=255*(b*((double)a/c));
    o=(char) i;
    return o;
}

unsigned char sWave(int a, int b, int c){
    double i; char o;
    i=127*sin(2*3.14*(b*((double)a/c)))+127;
    o=(char) i;
    return o;
}