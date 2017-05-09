#include <stdio.h>
#include <stdlib.h>
#include <xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "i2c_master.h"
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

#define SLAVE_ADDR 0x6B // slave address
#define COLOR2 0x0000 // Background color for "off" pixels

void init_IMU(void);
void i2c_read_multiple(unsigned char add, unsigned char register, unsigned char * data, int length);
unsigned char i2c_read_single(unsigned char add, unsigned char reg);
void display_character(char c, char x, char y, unsigned short color1);
void draw_string(char msg[100], char x, char y, unsigned short color1);
void draw_bar_x(char x_bar, char y_bar, unsigned short color_bar, char len, char w, char maxlen);
void draw_bar_y(char x_bar, char y_bar, unsigned short color_bar, char len, char w, char maxlen);
void process_data(unsigned char * data, signed short * new_data, int len);

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
    } else { // negative
        // Draw 0 to +max length in background color
         for (i = 0; i < (maxlen + 1); i++) {
            if ((x + i) < 128) {
                for (j = 0; j < (w + 1); j++) {
                    if ((y + j) < 128) {
                        LCD_drawPixel(x + i, y + j, COLOR2);
                    }
                }
            }
        }
        // Draw 0 to -length in color1
        for (i = 0; i > (len - 1); i--) {
            if ((x + i) < 128) {
                for (j = 0; j < (w + 1); j++) {
                    if ((y + j) < 128) {
                        LCD_drawPixel(x + i, y + j, color1);
                    }
                }
            }
        }
        // Draw -length to -max length in background color
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

void draw_bar_y(char x, char y, unsigned short color1, char len, char w, char maxlen) {

    int i, j;

    if (len > 0) { // positive
        // Draw 0 to length in color1
        for (i = 0; i < (len + 1); i++) {
            if ((y + i) < 128) {
                for (j = 0; j < (w + 1); j++) {
                    if ((x + j) < 128) {
                        LCD_drawPixel(x + j, y + i, color1);
                    }
                }
            }
        }
        // Draw length to max length in background color
        for (i = len + 1; i < (maxlen + 1); i++) {
            if ((y + i) < 128) {
                for (j = 0; j < (w + 1); j++) {
                    if ((x + j) < 128) {
                        LCD_drawPixel(x + j, y + i, COLOR2);
                    }
                }
            }
        }
    } else { // negative
        // Draw 0 to +max length in background color
        for (i = 0; i < (maxlen + 1); i++) {
            if ((y + i) < 128) {
                for (j = 0; j < (w + 1); j++) {
                    if ((x + j) < 128) {
                        LCD_drawPixel(x + j, y + i, COLOR2);
                    }
                }
            }
        }
        // Draw 0 to -length in color1
        for (i = 0; i > (len - 1); i--) {
            if ((y + i) < 128) {
                for (j = 0; j < (w + 1); j++) {
                    if ((x + j) < 128) {
                        LCD_drawPixel(x + j, y + i, color1);
                    }
                }
            }
        }
        // Draw -length to -max length in background color
        for (i = len - 1; i > (-maxlen - 1); i--) {
            if ((y + i) < 128) {
                for (j = 0; j < (w + 1); j++) {
                    if ((x + j) < 128) {
                        LCD_drawPixel(x + j, y + i, COLOR2);
                    }
                }
            }
        }

    }
}

void init_IMU() {
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1 | 0);
    i2c_master_send(0x10); // CTRL1_XL register
    i2c_master_send(0b10000010);
    i2c_master_stop();

    i2c_master_start();
    i2c_master_send((SLAVE_ADDR << 1) | 0);
    i2c_master_send(0x11); // CTRL2_G register
    i2c_master_send(0b10001000);
    i2c_master_stop();

    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1 | 0);
    i2c_master_send(0x12); // CTRL3_C register
    i2c_master_send(0b00000100);
    i2c_master_stop();
}

void i2c_read_multiple(unsigned char add, unsigned char reg, unsigned char * data, int len) {
    int i = 0;
    i2c_master_start();
    i2c_master_send(add << 1 | 0);
    i2c_master_send(reg);
    i2c_master_restart();
    i2c_master_send(add << 1 | 1);

    for (i = 0; i < (len - 1); i++) {
        data[i] = i2c_master_recv();
        i2c_master_ack(0);
    }

    data[len - 1] = i2c_master_recv();
    //data = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
}

unsigned char i2c_read_single(unsigned char add, unsigned char reg) {
    i2c_master_start();
    i2c_master_send(add << 1 | 0);
    i2c_master_send(reg);
    i2c_master_restart();
    i2c_master_send(add << 1 | 1);
    unsigned char data = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    return data;
}

void process_data(unsigned char * data, signed short * new_data, int len) {
    char i = 0, j = 0;
    while (i < len) {
        new_data[j] = data[i + 1] << 8 | data[i];
        i = i + 2;
        j++;
    }
}

unsigned char data;

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

    // do your TRIS and LAT commands here
    TRISBbits.TRISB4 = 1; // make push button an input
    TRISAbits.TRISA4 = 0; // make green LED an output
    LATAbits.LATA4 = 1; // set green LED initially high

    ANSELBbits.ANSB2 = 0; // set as digital
    ANSELBbits.ANSB3 = 0; // set as digital
    I2C2BRG = 233;
    I2C2CONbits.ON = 1;

    __builtin_enable_interrupts();

    SPI1_init();
    LCD_init();
    LCD_clearScreen(BLACK);
    init_IMU();

    while (1) {
        int k, m, x, y, w, maxlen, len = 14; // if wanted only accelerometer data, length = 6, reg = 0x28 
        signed short new_data[len / 2], x_acc, y_acc;
        char msg[100];
        unsigned char data[len];

        i2c_read_multiple(SLAVE_ADDR, 0x20, data, len);
        //data = i2c_read_single(SLAVE_ADDR, 0x0F);
        process_data(data, new_data, len);
        x = 64;
        x_acc = new_data[4]*(64./15000.);
        y = 64;
        y_acc = new_data[5]*(64./15000.);
        w = 2;
        maxlen = 64;

        sprintf(msg, "x acc: %3.0d", x_acc); // print x acceleration data
        draw_string(msg, 2, 20, WHITE);
        sprintf(msg, "y acc: %3.0d", y_acc); // print y acceleration data
        draw_string(msg, 2, 30, WHITE);

        draw_bar_x(x, y, BLUE, x_acc, w, maxlen); // draw x bar
        draw_bar_y(x, y, GREEN, y_acc, w, maxlen); // draw y bar
        
        _CP0_SET_COUNT(0);
        while (_CP0_GET_COUNT() < 24000000 / 10) {
        }; // 5 Hz update 
    }
}

