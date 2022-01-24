#ifndef __ADX355_DRIVER_H__
#define __ADX355_DRIVER_H__

int adx355_driver_init(void);
int write_reg_sync(uint8_t address, uint8_t num, uint8_t* tx_buffer);
int read_reg_sync(uint8_t address, uint8_t num, uint8_t* rx_buffer);

typedef void (*SPI_DMA_READ_CALLBACK)(uint8_t *buffer, uint8_t size, void *user_data);
typedef void (*SPI_DMA_WRITE_CALLBACK)(void);
int read_reg_async(uint8_t address, uint8_t num, SPI_DMA_READ_CALLBACK cb, void* cb_user_data);

#endif