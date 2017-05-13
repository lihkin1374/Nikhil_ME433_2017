#ifndef OTHER_FUNCTIONS_H__
#define OTHER_FUNCTIONS_H__
// Header file for with all functions written in homeworks 6 and 7

void init_IMU(void);
void i2c_read_multiple(unsigned char add, unsigned char register, unsigned char * data, int length);
unsigned char i2c_read_single(unsigned char add, unsigned char reg);
void display_character(char c, char x, char y, unsigned short color1);
void draw_string(char msg[100], char x, char y, unsigned short color1);
void draw_bar_x(char x_bar, char y_bar, unsigned short color_bar, char len, char w, char maxlen);
void draw_bar_y(char x_bar, char y_bar, unsigned short color_bar, char len, char w, char maxlen);
void process_data(unsigned char * data, signed short * new_data, int len);
#endif