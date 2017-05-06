#include <stdio.h>
#include <xc.h>           // processor SFR definitions
#include "ILI9163C.h"

// DEVCFG0 (data sheet page 241-242)
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1 (data sheet page 243-244)
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz (data sheet page 245-246)
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz (starts at 8, divide by 2)
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV (multiply by 24 to get 96)
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz (divide by 2 to get 48)
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB (divide by 2?)
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3 (data sheet page 247)
#pragma config USERID = 00000000 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

#define COLOR2 0x0000 // Background color for "off" pixels

void display_character(char c, char x, char y, unsigned short color1);
void draw_bar_x(char x_bar, char y_bar, unsigned short color_bar, char len, char w, char maxlen);

void display_character(char c, char x, char y, unsigned short color1) {
    // c = character, x,y = location, color1 = text, color2 = background 
    char char_row;
    char_row = c - 0x20; // c is what row in the array ascii to use
    int i, j; // loop through 5 times to display each column
    for (i = 0; i < 5; i++) {
        if ((x + i) < (128 - 5)) {
            for (j = 0; j < 8; j++) {
                if ((y + j) < (128 - 7)) {
                    if ((ASCII[char_row][i] >> j & 1) == 1) {
                        LCD_drawPixel(x + i, y + j, color1);
                    } else {
                        LCD_drawPixel(x + i, y + j, COLOR2);
                    }
                }
            }
        }
    }
}

void draw_string(char msg[100], char x, char y, unsigned short color1) {
    int k = 0;
    char c;
    while (msg[k] != 0) { // stop printing after the last letter
        c = msg[k];
        x = x + 5;
        display_character(c, x, y, color1);
        k++;
    }
}

void draw_bar_x(char x, char y, unsigned short color1, char len, char w, char maxlen) {
    
    int i, j;

    if (len > 0) { // positive
        // Draw 0 to length in color1
        for (i = 0; i < (len + 1); i++) {
            if ((x + i) < 128) {
                for (j = 0; j < (w + 1); j++) {
                    if ((y + j) < 128) {
                        LCD_drawPixel(x + i, y + j, color1);
                    }
                }
            }
        }
        // Draw length to max length in background color
        for (i = len + 1; i < (maxlen + 1); i++) {
            if ((x + i) < 128) {
                for (j = 0; j < (w + 1); j++) {
                    if ((y + j) < 128) {
                        LCD_drawPixel(x + i, y + j, COLOR2);
                    }
                }
            }
        }
    }
    else{ // negative
        // Draw 0 to length in color1
        for (i = 0; i > (len - 1); i--) {
            if ((x + i) < 128) {
                for (j = 0; j < (w + 1); j++) {
                    if ((y + j) < 128) {
                        LCD_drawPixel(x + i, y + j, color1);
                    }
                }
            }
        }
        // Draw length to max length in background color
        for (i = len - 1; i > (-maxlen - 1); i--) {
            if ((x + i) < 128) {
                for (j = 0; j < (w + 1); j++) {
                    if ((y + j) < 128) {
                        LCD_drawPixel(x + i, y + j, COLOR2);
                    }
                }
            }
        }
        
    }
}

int main() {
    char c, x, y, msg[100], x_bar, y_bar, len, w, maxlen;
    unsigned short color1, color_bar;
    x = 42;
    y = 32;
    color1 = WHITE;

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISBbits.TRISB4 = 1; // make push button an input
    TRISAbits.TRISA4 = 0; // make green LED an output
    LATAbits.LATA4 = 1; // set green LED initially high

    __builtin_enable_interrupts();
    SPI1_init();
    LCD_init();
    LCD_clearScreen(COLOR2);

    sprintf(msg, "HELLO");
    draw_string(msg, x, y, color1);

    while (1) {
        int n = 50, k = 0;
        float fps; 
        
        for (n = 50; n>-51; n--) {
             _CP0_SET_COUNT(0);
            x = 72;
            sprintf(msg, "%d", n);
            draw_string(msg, x, y, color1); //draw number

            x_bar = 64;
            y_bar = 55;
            color_bar = CYAN;
            len = n;
            w = 1;
            maxlen = 50;
            draw_bar_x(x_bar, y_bar, color_bar, len, w, maxlen);

            fps = 24000000.0/_CP0_GET_COUNT();
            sprintf(msg, "%3.2f", fps);
            draw_string(msg,40,75,BLUE);            
            
            _CP0_SET_COUNT(0);
            while (_CP0_GET_COUNT() < 48000000/2/5) {
            }; // delay 5 Hz = 200 ms
        }
        sprintf(msg, "%d", n);
        draw_string(msg, x, y, COLOR2); // clear the space between 50 and -50
        draw_bar_x(x_bar, y_bar, COLOR2, len, w, maxlen); // clear bar      

        _CP0_SET_COUNT(0);
        while (_CP0_GET_COUNT() < 48000000 / 10) {
        }; // delay 200 ms
    }
}

