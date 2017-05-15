#include "imu.h"

void initIMU() {
    ANSELBbits.ANSB2 = 0;   // turn off Analog on PIC32 for B2 and B3
    ANSELBbits.ANSB3 = 0;
    
    i2c_master_setup();     // setup I2C on PIC32
    
    // initialize accel
    i2c_master_start();
    i2c_master_send(OPCODE_WRITE);  // device write opcode
    i2c_master_send(0x10);          // accelerometer setup
    i2c_master_send(0b10000010);    // turn on accelerometer, +-2g
    i2c_master_stop();
    // init gyro
    i2c_master_start();
    i2c_master_send(OPCODE_WRITE);  // device write opcode
    i2c_master_send(0x11);          // gyro setup
    i2c_master_send(0b10001000);    // 1000 dps sensitivity
    i2c_master_stop();
    // init read multiple registers in a row
    i2c_master_start();
    i2c_master_send(OPCODE_WRITE);  // device write opcode
    i2c_master_send(0x12);
    i2c_master_send(0b00000100);    // register address auto increment
    i2c_master_stop();
}

void I2C_read_multiple(unsigned char address, unsigned char reg, unsigned char * data, int length) {
    i2c_master_start();
    i2c_master_send(address<<1);
    i2c_master_send(reg);
    i2c_master_restart();
    i2c_master_send((address<<1)|1);
    int i;
    for (i = 0; i<length; i++) {
        data[i] = i2c_master_recv();
        if (i<(length-1))
            i2c_master_ack(0);
        else
            i2c_master_ack(1);
    }
    i2c_master_stop();
}