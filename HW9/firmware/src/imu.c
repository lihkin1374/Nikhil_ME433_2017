#include "imu.h"

void IMU_init(void){
	ANSELBbits.ANSB2 = 0;    // turn off analog
	ANSELBbits.ANSB3 = 0;
	i2c_master_setup();
/*	i2c_master_start();
	i2c_master_send(0xd4);  // (SAD+) write mode
	i2c_master_send(0x0f);  // want to read WHOAMI address
	i2c_master_restart();
	i2c_master_send(0xd5);  // (SAD+) read mode
	char who_am_i = i2c_master_recv(); // receive
	i2c_master_ack(1);
	i2c_master_stop();
		
	char message[STRLENGTH];
	sprintf(message,"0x%0x",who_am_i);
	print_string(message,48,32,BLACK);   // should be 0x69 or 0b01101001	
*/
	
	i2c_master_start();     // turn on the accelerometer
	i2c_master_send(0xd4);  // write mode
	i2c_master_send(0x10);  // CTRL1_XL register
	i2c_master_send(0x82);  // sample rate to 1.66 kHz, with 2g sensitivity, and 100 Hz filter
	i2c_master_stop();
	
	i2c_master_start();     // turn on the gyroscope
	i2c_master_send(0xd4);  // write mode
	i2c_master_send(0x11);  // CTRL2_G register
	i2c_master_send(0x88);  // sample rate to 1.66 kHz, with 1000 dps sensitivity
	i2c_master_stop();
	
	i2c_master_start();     // CTRL3_C register, which contains the IF_INC bit.
	i2c_master_send(0xd4);  // write mode
	i2c_master_send(0x04);  // only IF_INC is 1
	i2c_master_stop();
}


void IMU_read(unsigned char address, unsigned char regis, unsigned char *data, int length){
	i2c_master_start();
	unsigned char write = address<<1 | 0;        // address is 0xd4
	unsigned char read = address<<1 | 1;

	i2c_master_send(write);
	i2c_master_send(regis);         //for OUT_TEMP_L, register is 0x20, length = 14
									//for OUTX_L_G, register is 0x22, length = 12
	i2c_master_restart();
	i2c_master_send(read);
	
	int i = 0;
	for(i=0; i<length; i++){
		*data++ = i2c_master_recv();
		if(i==(length-1)){
			i2c_master_ack(1);  // make the ack so the slave knows we got it
			break;
		}
		i2c_master_ack(0);      // continue to read
	}         
	i2c_master_stop();
}