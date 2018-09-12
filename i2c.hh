#ifndef _I2C_HH
#define _I2C_HH

struct i2c_status_info {
	unsigned char* rx_fifo;
	unsigned int rx_bytes;
	unsigned char* tx_fifo;
	unsigned int tx_bytes;
	bool error;
};

#endif  // _I2C_HH
