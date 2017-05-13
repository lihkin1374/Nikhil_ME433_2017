#include <xc.h>           // processor SFR definitions
#include "ILI9163C.h"
#define SLAVE_ADDR 0x6B // slave address
#define COLOR2 0x0000 // Background color for "off" pixels

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