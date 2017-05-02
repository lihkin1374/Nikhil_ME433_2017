#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "spi_lib.h"

// DEVCFG0
#pragma config DEBUG = 0b11 // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV =  DIV_1// divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // use slowest wdt
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 2 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

#define CS LATBbits.LATB7
#define samps 1000
#define freqA 10
#define freqB 5

static volatile int tickA;
static volatile int tickB;

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Wave(void){
    char sVoltage; char tVoltage;
    sVoltage=sWave(freqA,tickA,samps);
    tVoltage=tWave(freqB,tickB,samps);
    
    setVoltage(0,sVoltage);
    setVoltage(1,tVoltage);
    
    tickA++;
    tickB++;
    
    if(tickA==((int)((double)samps/freqA))){
        tickA=0;        
    }
    
    if(tickB==((int)((double)samps/freqB))){
        tickB=0;        
    }
    IFS0bits.T2IF=0;
}

int main() {
    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
    
    init(); //initialize DAC chip
    
    T2CONbits.TCKPS = 1;  // set Timer2 prescaler
    PR2 = 23999;         // 48MHz/(1000Hz*2) - 1
    TMR2 = 0;             // set Timer2 to 0
    IPC2bits.T2IP = 5;   // set Timer2 interrupt priority to 5
    IPC2bits.T2IS = 0;   // set Timer2 sub priority
    IFS0bits.T2IF = 0;   // clear Timer2 interrupt flag
    IEC0bits.T2IE = 1;  // enable Timer2 interrupt
    T2CONbits.ON = 1;  // Turn on  Timer2
   
    
    // do your TRIS and LAT commands here
    TRISAbits.TRISA4=0;
    TRISBbits.TRISB4=1;
    LATAbits.LATA4=1;
    
    
    __builtin_enable_interrupts();
    
    while(1) {
        _CP0_SET_COUNT(0);
        while(_CP0_GET_COUNT()<25000){;} //0.001/(1/25000000)
        LATAINV=0b10000;
        _CP0_SET_COUNT(0);
        while(!PORTBbits.RB4){;}
	    // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
		  // remember the core timer runs at half the sysclk
    }
    
    return 0;
}
