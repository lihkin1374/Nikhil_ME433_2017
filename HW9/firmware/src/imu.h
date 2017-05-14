#ifndef IMU_H
#define IMU_H

#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<stdio.h>
#include "i2c.h"

void IMU_init(void);
void IMU_read(unsigned char address, unsigned char regis, unsigned char *data, int length);

#endif