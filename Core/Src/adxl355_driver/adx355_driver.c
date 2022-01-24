/**
 * @author renmingrui
 * @date 2022-01-20
 * @brief adx355芯片驱动，使用HAL SPI库
 */
#include "rtthread.h"
#include "rthw.h"  // rt_hw_interrupt_disable
#include "main.h"
#include "adx355_driver.h"

#ifndef STATIC
#define STATIC static 
#endif

#define _HAL_SPI_EVENT_TxCpltCallback     1<<0
#define _HAL_SPI_EVENT_RxCpltCallback     1<<1
#define _HAL_SPI_EVENT_ERROR              1<<2
STATIC rt_event_t _hal_spi_event;

#define DMA_RX_TX_BUFFER_SIZE 128
STATIC uint8_t *dma_rx_buffer;
STATIC uint8_t *dma_tx_buffer;
extern SPI_HandleTypeDef hspi2;

#define _ADX355_DATA_ENOUGH               1<<0
STATIC rt_event_t adx355_data_enough;


STATIC void * __read_reg_cb_user_data;
STATIC SPI_DMA_READ_CALLBACK read_reg_cb=NULL;
STATIC SPI_DMA_WRITE_CALLBACK write_reg_cb=NULL;

/**
 * @note  一次DMA读取的数据量为X轴+Y轴+Z轴： 3*3字节=9字节
 *        
 *        1000SPS采样率下，每1ms读取一次，1s数据量为9000字节 约等于 9K
 *                        每1分钟的数据量为 540k
 *        200SPS采样率下，每5ms读取一次，1s数据量为1800字节 约等于 1.8K
 *                        每1分钟的数据量为 10800字节，约等于 108k
 */
#define ADX355_DATA_SIZE_EACH   9
#define ADX355_DATA_MQ_BUFFER   2*60*200*9
STATIC uint8_t* adx355_data_buffer;//传感器数据缓冲区
STATIC uint32_t adx355_data_buffer_counter=0;

/* HAL SPI DMA 回调 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if(write_reg_cb != NULL)
    write_reg_cb();

  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_SET);
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if(read_reg_cb != NULL)
    read_reg_cb(dma_rx_buffer+1, hspi->RxXferSize-1, __read_reg_cb_user_data);

  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_SET);
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if(read_reg_cb != NULL)
    read_reg_cb(dma_rx_buffer+1, hspi->RxXferSize-1, __read_reg_cb_user_data);

  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_SET);
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
struct __rx_buffer_info{
    uint8_t *rx_buffer;
    uint8_t num;
};
STATIC void __defaule_read_reg_sync_cb(uint8_t *buffer, uint8_t size, void *user_data){
    struct __rx_buffer_info* info = user_data;
    rt_memcpy(info->rx_buffer, buffer, info->num);
    rt_event_send(_hal_spi_event, _HAL_SPI_EVENT_RxCpltCallback);
}
int read_reg_sync(uint8_t address, uint8_t num, uint8_t* rx_buffer) {
    HAL_StatusTypeDef hal_ret;
    rt_err_t rt_ret;
    int ret;

    rt_uint32_t e;
    if(num >= DMA_RX_TX_BUFFER_SIZE){
        rt_kprintf("read_reg num can't not over DMA_RX_TX_BUFFER_SIZE:[%d], num:[%d]\n", DMA_RX_TX_BUFFER_SIZE, num);
        return -1;
    }

    uint8_t spi_address = adx355_address_to_spi(address, ADX355_ADDRESS_TO_SPI_READ);
    rt_memset(dma_tx_buffer, 0, DMA_RX_TX_BUFFER_SIZE);
    rt_memcpy(dma_tx_buffer, &spi_address, 1);

    read_reg_cb = __defaule_read_reg_sync_cb;
    struct __rx_buffer_info info = {
        rx_buffer = rx_buffer,
        num = num
    };
    __read_reg_cb_user_data = &info;

    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET);
    hal_ret = HAL_SPI_TransmitReceive_DMA(&hspi2, dma_tx_buffer, dma_rx_buffer, 1+num);
    if(hal_ret != HAL_OK){
        rt_kprintf("read_reg HAL_SPI_TransmitReceive_DMA failed, error code is:[%d]\n", hal_ret);
        ret = -1;
        goto failed;
    }

    rt_ret = rt_event_recv(_hal_spi_event, _HAL_SPI_EVENT_RxCpltCallback|_HAL_SPI_EVENT_ERROR, 
                RT_EVENT_FLAG_OR| RT_EVENT_FLAG_CLEAR, 500, &e);
    if(rt_ret!=RT_EOK){
        rt_kprintf("read_reg HAL_SPI_TransmitReceive_DMA rt_event_recv failed, error code is:[%d]\n",rt_ret);
        ret = -1;
        goto failed;
    }
    if((e & _HAL_SPI_EVENT_ERROR) != 0){
        rt_kprintf("read_reg HAL_SPI_TransmitReceive_DMA SPI hardware error, please check SPI registers\n");
        ret = -1;
        goto failed;
    }


    ret = 0;

failed:
    read_reg_cb = NULL;
    __read_reg_cb_user_data = NULL;
    return ret;
}

/**
 * @brief 向adx355写寄存器数据
 *        使用DMA读取, 阻塞式
 * @param address 寄存器起始地址
 * @param num     寄存器数量
 * @param tx_buffer 要写入的数据
 */
STATIC void __defaule_write_reg_sync_cb(void){
    rt_event_send(_hal_spi_event, _HAL_SPI_EVENT_TxCpltCallback);
}
int write_reg_sync(uint8_t address, uint8_t num, uint8_t* tx_buffer) {
    HAL_StatusTypeDef hal_ret;
    rt_err_t rt_ret;
    int ret;
    rt_uint32_t e;

    if(num >= DMA_RX_TX_BUFFER_SIZE){
        rt_kprintf("write_reg num can't not over DMA_RX_TX_BUFFER_SIZE:[%d], num:[%d]\n", DMA_RX_TX_BUFFER_SIZE, num);
        return -1;
    }

    uint8_t spi_address = adx355_address_to_spi(address, ADX355_ADDRESS_TO_SPI_WRITE);
    rt_memset(dma_tx_buffer, 0, DMA_RX_TX_BUFFER_SIZE);
    rt_memcpy(dma_tx_buffer, &spi_address, 1);
    rt_memcpy(dma_tx_buffer+1, tx_buffer, num);

    write_reg_cb = __defaule_write_reg_sync_cb;

    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET);
    hal_ret = HAL_SPI_Transmit_DMA(&hspi2, dma_tx_buffer, 1+num);
    if(hal_ret != HAL_OK){
        rt_kprintf("write_reg HAL_SPI_Transmit_DMA failed, error code is:[%d]\n", hal_ret);
        ret = -1;
        goto failed;
    }

    rt_ret = rt_event_recv(_hal_spi_event, _HAL_SPI_EVENT_TxCpltCallback|_HAL_SPI_EVENT_ERROR, 
                RT_EVENT_FLAG_OR| RT_EVENT_FLAG_CLEAR, 500, &e);
    if(rt_ret!=RT_EOK){
        rt_kprintf("write_reg HAL_SPI_Transmit_DMA rt_event_recv failed, error code is:[%d]\n",rt_ret);
        ret = -1;
        goto failed;
    }
    if((e & _HAL_SPI_EVENT_ERROR) != 0){
        rt_kprintf("write_reg HAL_SPI_TransmitReceive_DMA SPI hardware error, please check SPI registers\n");
        ret = -1;
        goto failed;
    }


    ret = 0;

failed:
    write_reg_cb=NULL;
    return ret;
}

/**
 * @brief 从adx355读取寄存器数据，异步
 * @param address 寄存器起始地址
 * @param num 寄存器数量
 * @param rx_buffer 读取到的数据缓冲区
 * @param cb   读取完成后的回调函数
 * @param cb_user_data 回调参数
 */
int read_reg_async(uint8_t address, uint8_t num, SPI_DMA_READ_CALLBACK cb, void* cb_user_data){
    HAL_StatusTypeDef hal_ret;
    int ret;

    if(num >= DMA_RX_TX_BUFFER_SIZE){
        rt_kprintf("read_reg num can't not over DMA_RX_TX_BUFFER_SIZE:[%d], num:[%d]\n", DMA_RX_TX_BUFFER_SIZE, num);
        return -1;
    }

    uint8_t spi_address = adx355_address_to_spi(address, ADX355_ADDRESS_TO_SPI_READ);
    rt_memset(dma_tx_buffer, 0, DMA_RX_TX_BUFFER_SIZE);
    rt_memcpy(dma_tx_buffer, &spi_address, 1);

    read_reg_cb = cb;
    __read_reg_cb_user_data = cb_user_data;
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET);
    hal_ret = HAL_SPI_TransmitReceive_DMA(&hspi2, dma_tx_buffer, dma_rx_buffer, 1+num);
    if(hal_ret != HAL_OK){
        rt_kprintf("read_reg HAL_SPI_TransmitReceive_DMA failed, error code is:[%d]\n", hal_ret);
        ret = -1;
        goto failed;
    }
    ret=0;

failed:
    return ret;
}

/* DRDY引脚中断处理 */
/**
 * @brief 此中断中启动一次DMA SPI传输
 *
 */
static void read_adx355_data_cb(uint8_t *buffer, uint8_t size, void *user_data){
    // 这里使用关中断的方式实现 锁机制
    register rt_base_t temp;
    temp = rt_hw_interrupt_disable();

    rt_memcpy(&adx355_data_buffer[adx355_data_buffer_counter++], buffer, size);

    rt_hw_interrupt_enable(temp); 
    if(adx355_data_buffer_counter % 100 == 0)
        rt_event_send(adx355_data_enough, _ADX355_DATA_ENOUGH);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 硬件连接 ADX355: DRDY --- PG12
    if(GPIO_Pin == GPIO_PIN_12){
        __NOP();
        //启动SPI DMA传输
        read_reg_async(0x08, 9, read_adx355_data_cb, NULL);
    }
}


/**
 * @brief 线程，数据量够1分钟后，启动存SD卡
 */
void adx355_driver_data_record_thread_entry(void *parameter){
    while(1){
        rt_uint32_t e;
        rt_err_t rt_ret;
        rt_ret = rt_event_recv(adx355_data_enough, _ADX355_DATA_ENOUGH , RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &e);
        if(rt_ret != RT_EOK){
            rt_kprintf("adx355_driver_data_record_thread_entry rt_event_recv failed! error code is:[%d]\n", rt_ret);
        }
        UNUSED(e);
        
        // //取出数据
        register rt_base_t temp;
        temp = rt_hw_interrupt_disable();

        rt_uint32_t x = adx355_data_buffer[0] << 12;
        x |= adx355_data_buffer[1] << 4;
        x |= adx355_data_buffer[2] >> 4;
        rt_uint32_t y = adx355_data_buffer[3] << 12;
        y |= adx355_data_buffer[4] << 4;
        y |= adx355_data_buffer[5] >> 4;
        rt_uint32_t z = adx355_data_buffer[6] << 12;
        z |= adx355_data_buffer[7] << 4;
        z |= adx355_data_buffer[8] >> 4;

        adx355_data_buffer_counter = 0;
        rt_hw_interrupt_enable(temp); 


        //debug打印第一条数据

        rt_kprintf("x:[%d] y:[%d] z:[%d]\n", x, y,z);
    }
}



int adx355_driver_init(){
    _hal_spi_event = rt_event_create("hal_spi_event", RT_IPC_FLAG_PRIO);
    if(_hal_spi_event == NULL) {
        rt_kprintf("hal_spi_event create failed!\n");
        return -1;
    }

    adx355_data_enough = rt_event_create("adx355_data_enough", RT_IPC_FLAG_PRIO);
    if(adx355_data_enough == NULL) {
        rt_kprintf("adx355_data_enough create failed!\n");
        return -1;
    }

    dma_rx_buffer = rt_malloc(DMA_RX_TX_BUFFER_SIZE);
    dma_tx_buffer = rt_malloc(DMA_RX_TX_BUFFER_SIZE);
    if(dma_rx_buffer == NULL || dma_tx_buffer == NULL){
        rt_kprintf("dma_rx_buffer dma_tx_buffer malloc failed!");
        return -1;
    }

    adx355_data_buffer = rt_malloc(ADX355_DATA_MQ_BUFFER);
    if(adx355_data_buffer == NULL) {
        rt_kprintf("adx355_data_buffer create failed!\n");
        return -1;
    }

    return 0;
}

STATIC int adx355_driver_init_later(void){
    rt_err_t ret;
    rt_thread_t adx355_driver_data_record_thread_t = rt_thread_create("adx355_data_rec",
                                    adx355_driver_data_record_thread_entry, NULL, 
                                    2048, 2, 20);
    if(adx355_driver_data_record_thread_t == NULL) {
        rt_kprintf("adx355_driver_data_record_thread_t create failed!\n");
        return -1;
    }
    ret = rt_thread_startup(adx355_driver_data_record_thread_t);
    if(ret != RT_EOK){
        rt_kprintf("rt_thread_startup(adx355_driver_data_record_thread_t) filed, error code is:[%d]\n", ret);
        return -1;
    }

    return 0;

}
INIT_APP_EXPORT(adx355_driver_init_later);