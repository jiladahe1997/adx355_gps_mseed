/**
 * @author renmingrui
 * @date 2022-01-20
 * @brief adx355芯片驱动，使用HAL SPI库
 */
#include "rtthread.h"
#include "main.h"

#ifndef STATIC
#define STATIC static 
#endif

#define _HAL_SPI_EVENT_OK     1<<0
#define _HAL_SPI_EVENT_ERROR  1<<1
STATIC rt_event_t _hal_spi_event;
STATIC uint8_t *dma_rx_buffer;
STATIC uint8_t *dma_tx_buffer;
extern SPI_HandleTypeDef hspi2;

/* HAL SPI DMA 回调 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  rt_event_send(_hal_spi_event, _HAL_SPI_EVENT_OK);
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  rt_event_send(_hal_spi_event, _HAL_SPI_EVENT_OK);
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  rt_event_send(_hal_spi_event, _HAL_SPI_EVENT_ERROR);
}


/**
 * @brief adx355寄存器地址转换成spi消息
 *        比如寄存器地址 0x01，写入则将寄存器地址左移一位，且最右一位补0，变为0x02
 *                           读取则将寄存器地址左移一位，且最右一位补1,变为0x03
 */
enum ADX355_ADDRESS_TO_SPI_DIR {
    ADX355_ADDRESS_TO_SPI_WRITE,
    ADX355_ADDRESS_TO_SPI_READ
};
STATIC uint8_t adx355_address_to_spi(uint8_t address,enum ADX355_ADDRESS_TO_SPI_DIR dir){
    if(dir == ADX355_ADDRESS_TO_SPI_READ){
        return (address << 1) | 0x01;
    }else if(dir == ADX355_ADDRESS_TO_SPI_WRITE){
        return (address << 1) & 0xfe;
    }
    rt_kprintf("adx355_address_to_spi param error!");
    return 0;
}

/**
 * @brief 从adx355读取寄存器数据
 *        使用DMA读取, 阻塞式, 默认超时500ms
 * @param address 寄存器起始地址
 * @param num     寄存器数量
 * @param rx_buffer 读取到的数据存放缓冲区
 */

int read_reg(uint8_t address, uint8_t num, uint8_t* rx_buffer) {
    HAL_StatusTypeDef hal_ret;
    rt_err_t rt_ret;
    int ret;

    rt_uint32_t e;

    uint8_t spi_address = adx355_address_to_spi(address, ADX355_ADDRESS_TO_SPI_READ);


    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET);
    rt_memcpy(dma_tx_buffer, &spi_address, 1);
    hal_ret = HAL_SPI_Transmit_DMA(&hspi2, dma_tx_buffer, 1);
    if(hal_ret != HAL_OK){
        rt_kprintf("read_reg HAL_SPI_Transmit_DMA failed, error code is:[%d]\n", hal_ret);
        ret = -1;
        goto failed;
    }

    rt_ret = rt_event_recv(_hal_spi_event, _HAL_SPI_EVENT_OK|_HAL_SPI_EVENT_ERROR, 
                RT_EVENT_FLAG_OR| RT_EVENT_FLAG_CLEAR, 500, &e);
    if(rt_ret!=RT_EOK){
        rt_kprintf("read_reg HAL_SPI_Transmit_DMA rt_event_recv failed, error code is:[%d]\n",rt_ret);
        ret = -1;
        goto failed;
    }

    hal_ret = HAL_SPI_Receive_DMA(&hspi2, dma_rx_buffer, num);
    if(hal_ret != HAL_OK){
        rt_kprintf("read_reg HAL_SPI_Receive_DMA failed, error code is:[%d]\n", hal_ret);
        ret = -1;
        goto failed;
    }
    rt_ret = rt_event_recv(_hal_spi_event, _HAL_SPI_EVENT_OK|_HAL_SPI_EVENT_ERROR, 
                RT_EVENT_FLAG_OR| RT_EVENT_FLAG_CLEAR, 500, &e);
    if(rt_ret!=RT_EOK){
        rt_kprintf("read_reg HAL_SPI_Receive_DMA rt_event_recv failed, error code is:[%d]\n",rt_ret);
        ret = -1;
        goto failed;
    }
    rt_memcpy(rx_buffer, dma_rx_buffer, num);
    ret = 0;

failed:
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET);
    return ret;
}

/**
 * @brief 向adx355写寄存器数据
 *        使用DMA读取, 阻塞式
 * @param address 寄存器起始地址
 * @param num     寄存器数量
 * @param tx_buffer 要写入的数据
 */
int write_reg(uint8_t address, uint8_t num, uint8_t* tx_buffer) {
    HAL_StatusTypeDef hal_ret;
    rt_err_t rt_ret;
    int ret;
    rt_uint32_t e;
    
    uint8_t spi_address = adx355_address_to_spi(address, ADX355_ADDRESS_TO_SPI_WRITE);

    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET);
    hal_ret = HAL_SPI_Transmit_DMA(&hspi2, &spi_address, 1);
    if(hal_ret != HAL_OK){
        rt_kprintf("write_reg HAL_SPI_Transmit_DMA failed, error code is:[%d]\n", hal_ret);
        ret = -1;
        goto failed;
    }

    rt_ret = rt_event_recv(_hal_spi_event, _HAL_SPI_EVENT_OK|_HAL_SPI_EVENT_ERROR, 
                RT_EVENT_FLAG_OR| RT_EVENT_FLAG_CLEAR, 500, &e);
    if(rt_ret!=RT_EOK){
        rt_kprintf("write_reg HAL_SPI_Transmit_DMA rt_event_recv failed, error code is:[%d]\n",rt_ret);
        ret = -1;
        goto failed;
    }

    hal_ret = HAL_SPI_Transmit_DMA(&hspi2, tx_buffer, num);
    if(hal_ret != HAL_OK){
        rt_kprintf("write_reg HAL_SPI_Transmit_DMA failed, error code is:[%d]\n", hal_ret);
        ret = -1;
        goto failed;
    }
    rt_ret = rt_event_recv(_hal_spi_event, _HAL_SPI_EVENT_OK|_HAL_SPI_EVENT_ERROR, 
                RT_EVENT_FLAG_OR| RT_EVENT_FLAG_CLEAR, 500, &e);
    if(rt_ret!=RT_EOK){
        rt_kprintf("write_reg HAL_SPI_Transmit_DMA rt_event_recv failed, error code is:[%d]\n",rt_ret);
        ret = -1;
        goto failed;
    }
    ret = 0;

failed:
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET);
    return ret;
}


int adx355_driver_init(void){
    _hal_spi_event = rt_event_create("hal_spi_event", RT_IPC_FLAG_PRIO);
    if(_hal_spi_event == NULL) {
        rt_kprintf("hal_spi_event create failed!");
        return -1;
    }

    dma_rx_buffer = rt_malloc(128);
    dma_tx_buffer = rt_malloc(128);
    if(dma_rx_buffer == NULL || dma_tx_buffer == NULL){
        rt_kprintf("dma_rx_buffer dma_tx_buffer malloc failed!");
        return -1;
    }

    return 0;
}


