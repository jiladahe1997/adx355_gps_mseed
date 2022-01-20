#ifndef __ADX355_DRIVER_H__
#define __ADX355_DRIVER_H__

int adx355_driver_init(void);
int write_reg(uint8_t address, uint8_t num, uint8_t* tx_buffer);
int read_reg(uint8_t address, uint8_t num, uint8_t* rx_buffer);


#endif