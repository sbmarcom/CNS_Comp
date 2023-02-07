
#ifndef REMORA_H
#define REMORA_H


#define JOINTS				3  			// Number of joints - set this the same as Remora firmware code!!!. Max 8 joints
#define VARIABLES          	6 			// Number of command values - set this the same Remora firmware code!!!
#define DIGITAL_OUTPUTS		16
#define DIGITAL_INPUTS		16

#define SPIBUFSIZE			14 			//(4+4*JOINTS+4*COMMANDS+1) //(MAX_MSG*4) //20  SPI buffer size ......FIFO buffer size is 64 bytes?

#define PRU_DATA			0xA0 	// "data" SPI payload
#define PRU_READ          	0xB0  // "read" SPI payload
#define PRU_WRITE         	0xC0  // "writ" SPI payload
#define PRU_ESTOP           0xD0  // "estp" SPI payload
#define PRU_GENERIC         0xE0 //generic spi payload for testing

#endif
