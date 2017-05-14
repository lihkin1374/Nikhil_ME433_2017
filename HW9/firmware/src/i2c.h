#ifndef I2C_H
#define I2C_H

#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro

void i2c_master_setup(void);
void i2c_master_start(void);
void i2c_master_restart(void);
void i2c_master_send(unsigned char byte);
unsigned char i2c_master_recv(void);
void i2c_master_ack(int val);
void i2c_master_stop(void);

#endif