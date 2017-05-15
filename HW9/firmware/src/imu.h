#ifndef IMU__H__
#define IMU__H__

#include <xc.h>           // processor SFR definitions
#include "i2c_master_noint.h"

#define OPCODE_WRITE 0b11010110     //A0 tied to 3.3V,A1,A2 tied to ground, lsb 0 is write operation
#define OPCODE_READ 0b11010111      //A0 tied to 3.3V,A1,A2 tied to ground, lsb 1 is read operation

void initIMU();
void I2C_read_multiple(unsigned char address, unsigned char register, unsigned char * data, int length);

#endif