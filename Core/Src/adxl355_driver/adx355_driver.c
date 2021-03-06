/**
 * @author renmingrui
 * @date 2022-01-20
 * @brief adx355芯片驱动，使用HAL SPI库
 */
#include "rtthread.h"
#include "rthw.h"  // rt_hw_interrupt_disable
#include "main.h"
#include "adx355_driver.h"
#include "time.h" //struct time

#ifndef STATIC
#define STATIC static 
#endif

#define _HAL_SPI_EVENT_TxCpltCallback     1<<0
#define _HAL_SPI_EVENT_RxCpltCallback     1<<1
#define _HAL_SPI_EVENT_ERROR              1<<2
STATIC rt_event_t _hal_spi_event;

//ADX35采集启动事件，由gps_driver触发
#define _ADX355_START_EVENT               1<<0
rt_event_t adx355_start_event;

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
#define ADX355_SAMPRATE         200
#define ADX355_RECORD_SECOND    60
#define ADX355_DATA_MQ_BUFFER   2*ADX355_RECORD_SECOND*ADX355_SAMPRATE*ADX355_DATA_SIZE_EACH// 数据缓冲区大小 2分钟
STATIC uint8_t* adx355_data_buffer;//传感器数据缓冲区
uint32_t adx355_data_buffer_counter=0;
uint64_t adx355_data_total=0;

extern TIM_HandleTypeDef htim1;

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
#include "stdbool.h"
static void read_adx355_data_cb(uint8_t *buffer, uint8_t size, void *user_data){
    bool need_record = false;
    // 这里使用关中断的方式实现 锁机制
    register rt_base_t temp;
    temp = rt_hw_interrupt_disable();

    rt_memcpy(&adx355_data_buffer[adx355_data_buffer_counter], buffer, size);
    adx355_data_buffer_counter+=size;
    adx355_data_total++;
    if(adx355_data_buffer_counter % (ADX355_RECORD_SECOND*ADX355_SAMPRATE*ADX355_DATA_SIZE_EACH) == 0)
        need_record = true;
    rt_hw_interrupt_enable(temp); 
    if(need_record)
        rt_event_send(adx355_data_enough, _ADX355_DATA_ENOUGH);
}

#include "gps_driver.h"
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 硬件连接 ADX355: DRDY --- PG12
    if(GPIO_Pin == GPIO_PIN_12){
        __NOP();
        //启动SPI DMA传输
        read_reg_async(0x08, 9, read_adx355_data_cb, NULL);
    }else if(GPIO_Pin == GPIO_PIN_1){
        gps_driver_EXTI_handle();
    }
}


/**
 * @brief 线程，数据量够1分钟后，启动存SD卡
 */
#include "libmseed.h"
#include "ff.h"
static void
ms_record_handler_int (char *record, int reclen, void *ofp)
{
  FRESULT f_ret;
  UINT bw;
  f_ret = f_write ((FIL*)ofp, record, reclen, &bw);
  if(f_ret != FR_OK){
      rt_kprintf("f_write failed, error code is:[%d]\n", f_ret);
  }
  if(bw != reclen){
      rt_kprintf("ms_record_handler_int bw not equal reclen, bw is:[%d], reclen is:[%d]\n", bw, reclen);
  }
} /* End of ms_record_handler_int() */

/**
 * @brief 写miniseed文件，一次写一个通道，一个完整的miniseed需要写UD、EW、NS三个通道。写三次。
 * @param filename 文件名
 * @param timestr 时间戳，字符串类型，传递给ms_timestr2hptime
 * @param channel 通道，UD EW NS 三者之一
 * @param buffer  数据buffer
 * @param size    数据buffer大小
 */
STATIC MSRecord *msr = 0;
static int write_file_one_channel(char *filename, char* timestr, char *channel, uint32_t *buffer, uint32_t size){

        msr = msr_init(NULL);
        if(msr == NULL){
            rt_kprintf("msr_init failed!\n");
        }

        msr->samprate = ADX355_SAMPRATE;
        msr->numsamples = size;
        msr->starttime = ms_timestr2hptime(timestr);
        strcpy(msr->channel, channel);
        msr->datasamples = buffer; /* pointer to 32-bit integer data samples */
        strcpy(msr->station, "TYSD-");
        strcpy(msr->network, "NE");
        msr->dataquality = 'D';
        msr->encoding = 11;
        msr->byteorder = 1;
        msr->sampletype = 'i';
        msr->reclen = 4096;
        msr->sequence_number = 1;

        FRESULT f_ret;
        FIL fil;
        f_ret = f_open(&fil, filename,  FA_WRITE | FA_OPEN_APPEND);
        if(f_ret != FR_OK){
            rt_kprintf("f_open failed, error code is %d\r\n", f_ret);
            return -1;
        }
        int packedrecords = msr_pack(msr, &ms_record_handler_int, &fil, NULL, 1, 0);
        rt_kprintf("write %s, packedrecords:[%d]\n", filename, packedrecords);
        f_close(&fil);
        msr->datasamples = NULL;
        msr_free(&msr);

        return 0;
}

STATIC int _utils_struct_tm_2_string(struct tm t, char *buf, size_t size){
    if(size < 20){
        rt_kprintf("_utils_struct_tm_2_string size:[%d] error, can not less than 20\n");
        return -1;
    }
    int ret = snprintf(buf, size, "%04d-%02d-%02dT%02d:%02d:%02d", 
                    t.tm_year+1900, t.tm_mon+1, t.tm_mday,
                    t.tm_hour+8, t.tm_min, t.tm_sec);
    if(ret==0){
        rt_kprintf("_utils_struct_tm_2_string snprintf error, expect:[%d], but:[%d]\n", size, ret);
        return -1;
    }
    return 0;
}

STATIC time_t _utils_struct_tm_2_time_t(struct tm t){
    return mktime(&t);
}

STATIC int time_t_2_string(time_t t, char *buf, size_t size, char *format){
    if(size < 20){
        rt_kprintf("time_t_2_string size:[%d] error, can not less than 20\n");
        return -1;
    }
    struct tm *temp=NULL;
    temp = localtime(&t);

    int ret = strftime(buf, size, format, 
                    temp);
    if(ret==0){
        rt_kprintf("time_t_2_string snprintf error, expect:[%d], but:[%d]\n", size, ret);
        return -1;
    }
    return 0;
}

void adx355_driver_data_record_thread_entry(void *parameter){
    FRESULT f_ret;
    FATFS *fs;
    fs = rt_malloc(sizeof (FATFS));
    f_ret = f_mount(fs, "1:/", 1);
    if(f_ret != FR_OK){
        rt_kprintf("adx355_driver_data_record_thread_entry f_mount failed, error code is %d\r\n", f_ret);
    }

    dma_rx_buffer = rt_malloc(DMA_RX_TX_BUFFER_SIZE);
    dma_tx_buffer = rt_malloc(DMA_RX_TX_BUFFER_SIZE);
    if(dma_rx_buffer == NULL || dma_tx_buffer == NULL){
        rt_kprintf("dma_rx_buffer dma_tx_buffer malloc failed!");
    }

    rt_kprintf("starting init adx355...\n");
    /*
    * 读取4个ID，验证芯片基本信息
        1. ADI ID 正确值为 0xAD
        2. ADI MEMS ID 正确值为 0x1D
        3. 器件ID 正确值为 0xED
        4. 产品ID 正确值为 0x01
    */
    rt_kprintf("read IDs...\n");
    uint8_t IDs[4];
    int ret = read_reg_sync(0, 4, IDs);
    if(ret<0){
        rt_kprintf("read_reg failed, error code is[%d]\n",ret);
    }else{
        for(int i=0;i<4;i++){
        rt_kprintf("id[%d]:[%x]\n", i, IDs[i]);
        }
    }
    rt_kprintf("\n\n");

    rt_kprintf("reset adxl355\n");
    uint8_t RESET = 0x52;
    ret = write_reg_sync(0x2F, 1, &RESET);
    if(ret < 0){
        rt_kprintf("write_reg_sync RESET failed, error code is[%d]\n", ret);
    }

    rt_thread_mdelay(100);

    rt_kprintf("prepare to set Sync:EXT_SYNC to 10\n");
    uint8_t EXT_SYNC;
    ret = read_reg_sync(0x2b, 1, &EXT_SYNC);
    if(ret < 0){
        rt_kprintf("write_reg_sync EXT_SYNC failed, error code is[%d]\n", ret);
    }
    rt_kprintf("Sync:EXT_SYNC now is:[%d]\n",EXT_SYNC);
    rt_kprintf("writing reg...\n");
    CLEAR_BIT(EXT_SYNC, 1<<0);
    SET_BIT(EXT_SYNC, 1<<1);
    ret = write_reg_sync(0x2b, 1, &EXT_SYNC);
    if(ret < 0){
        rt_kprintf("write_reg_sync EXT_SYNC failed, error code is[%d]\n", ret);
    }
    rt_kprintf("write ok\n");
    ret = read_reg_sync(0x2b, 1, &EXT_SYNC);
    if(ret < 0){
        rt_kprintf("write_reg_sync EXT_SYNC failed, error code is[%d]\n", ret);
    }
    rt_kprintf("Sync:EXT_SYNC now is:[%d]\n",EXT_SYNC);
    rt_kprintf("\n\n");
    rt_kprintf("prepare to set POWER_CTL:STANDBY to 0\n");
    uint8_t STANDBY;
    ret = read_reg_sync(0x2d, 1, &STANDBY);
    if(ret < 0){
        rt_kprintf("write_reg_sync POWER_CTL failed, error code is[%d]\n", ret);
    }
    rt_kprintf("POWER_CTL:STANDBY now is:[%d]\n",STANDBY);
    rt_kprintf("writing reg...\n");
    CLEAR_BIT(STANDBY, 1<<0);
    ret = write_reg_sync(0x2d, 1, &STANDBY);
    if(ret < 0){
        rt_kprintf("write_reg_sync POWER_CTL failed, error code is[%d]\n", ret);
    }
    rt_kprintf("write ok\n");
    ret = read_reg_sync(0x2d, 1, &STANDBY);
    if(ret < 0){
        rt_kprintf("write_reg_sync POWER_CTL failed, error code is[%d]\n", ret);
    }
    rt_kprintf("POWER_CTL:STANDBY now is:[%d]\n",STANDBY);

    rt_thread_mdelay(1000);

    //等待gps ready
    rt_uint32_t e;
    rt_event_recv(adx355_start_event, _ADX355_START_EVENT, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,
                    RT_WAITING_FOREVER, &e);
    UNUSED(e);

    //开始采集数据
    HAL_TIM_Base_Start_IT(&htim1);
    rt_kprintf("gps ready， start sample...\r\n");
    struct tm gps_ready_time = get_gps_time();
    char gps_ready_time_str[32] = {0};
    time_t gps_ready_timestamp = _utils_struct_tm_2_time_t(gps_ready_time);
    ret = _utils_struct_tm_2_string(gps_ready_time,gps_ready_time_str, sizeof(gps_ready_time_str));
    if(ret < 0){
        rt_kprintf("gps get ready time failed!\n");
    }else{
        rt_kprintf("gps ready time: %s\n, timestamp:[%ld]\n", gps_ready_time_str,gps_ready_timestamp);
    }
    time_t now_timestamp = gps_ready_timestamp;
    uint32_t *buffer = rt_malloc(ADX355_RECORD_SECOND*ADX355_SAMPRATE*sizeof(uint32_t));

    while(1){
        rt_uint32_t e;
        rt_err_t rt_ret;
        rt_ret = rt_event_recv(adx355_data_enough, _ADX355_DATA_ENOUGH , RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &e);
        if(rt_ret != RT_EOK){
            rt_kprintf("adx355_driver_data_record_thread_entry rt_event_recv failed! error code is:[%d]\n", rt_ret);
        }
        UNUSED(e);

        /* 计算时间和文件名 */
        now_timestamp = now_timestamp + ADX355_RECORD_SECOND;
        char now_filename[32] = {0};
        //1:/20220125T152300.mseed
        int ret = time_t_2_string(now_timestamp-ADX355_RECORD_SECOND, now_filename, sizeof(now_filename),"1:/%Y%m%dT%H%M%S.mseed");
        if(ret<0){
            rt_kprintf("adx355_driver_data_record_thread_entry time_t_2_string failed,error code is:[%d]\n",ret);
            continue;
        }
        char now_timestr[32] = {0};
        //2022-01-25T15:23:00
        ret = time_t_2_string(now_timestamp-ADX355_RECORD_SECOND, now_timestr, sizeof(now_timestr),"%Y-%m-%dT%H:%M:%S");
        if(ret<0){
            rt_kprintf("adx355_driver_data_record_thread_entry time_t_2_string failed,error code is:[%d]\n",ret);
            continue;
        }

        rt_kprintf("ready to write mseed:[%s], starttime:[%s]\n", now_filename, now_timestr);
        
        rt_memset(buffer, 0, ADX355_RECORD_SECOND*ADX355_SAMPRATE*sizeof(uint32_t));
        uint32_t buffer_counter=0;
        if(buffer == NULL){
            rt_kprintf("adx355_driver_data_record_thread_entry buffer malloc failed\n");
            continue;
        }
        //copy x axis data to buffer
        for(int i=0;i<ADX355_RECORD_SECOND*ADX355_SAMPRATE*ADX355_DATA_SIZE_EACH;){
            buffer[buffer_counter] = (adx355_data_buffer[i] << 12);
            buffer[buffer_counter] |= (adx355_data_buffer[i+1] << 4);
            buffer[buffer_counter] |= (adx355_data_buffer[i+2] >> 4);

            buffer_counter++;
            i+=ADX355_DATA_SIZE_EACH;
        }
        write_file_one_channel(now_filename, now_timestr, "EW", buffer, buffer_counter);
        buffer_counter=0;

        for(int i=3;i<ADX355_RECORD_SECOND*ADX355_SAMPRATE*ADX355_DATA_SIZE_EACH;){
            buffer[buffer_counter] = (adx355_data_buffer[i] << 12);
            buffer[buffer_counter] |= (adx355_data_buffer[i+1] << 4);
            buffer[buffer_counter] |= (adx355_data_buffer[i+2] >> 4);

            buffer_counter++;
            i+=ADX355_DATA_SIZE_EACH;
        }

        write_file_one_channel(now_filename, now_timestr, "NS", buffer, buffer_counter);
        buffer_counter=0;

        for(int i=6;i<ADX355_RECORD_SECOND*ADX355_SAMPRATE*ADX355_DATA_SIZE_EACH;){
            buffer[buffer_counter] = (adx355_data_buffer[i] << 12);
            buffer[buffer_counter] |= (adx355_data_buffer[i+1] << 4);
            buffer[buffer_counter] |= (adx355_data_buffer[i+2] >> 4);

            buffer_counter++;
            i+=ADX355_DATA_SIZE_EACH;
        }
        write_file_one_channel(now_filename, now_timestr, "UD", buffer, buffer_counter);
        buffer_counter=0;


   

        // //debug打印第一条数据
        // rt_kprintf("x:[%d] y:[%d] z:[%d]\n", x, y,z);
        // rt_uint32_t x = adx355_data_buffer[0] << 12;
        // x |= adx355_data_buffer[1] << 4;
        // x |= adx355_data_buffer[2] >> 4;
        // rt_uint32_t y = adx355_data_buffer[3] << 12;
        // y |= adx355_data_buffer[4] << 4;
        // y |= adx355_data_buffer[5] >> 4;
        // rt_uint32_t z = adx355_data_buffer[6] << 12;
        // z |= adx355_data_buffer[7] << 4;
        // z |= adx355_data_buffer[8] >> 4;


        register rt_base_t temp;
        temp = rt_hw_interrupt_disable();

        //前一分钟数据已经保存完毕，所有数据前移
        memcpy(adx355_data_buffer, adx355_data_buffer+ADX355_RECORD_SECOND*ADX355_SAMPRATE*ADX355_DATA_SIZE_EACH, 
                adx355_data_buffer_counter-ADX355_RECORD_SECOND*ADX355_SAMPRATE*ADX355_DATA_SIZE_EACH);

        adx355_data_buffer_counter -=ADX355_RECORD_SECOND*ADX355_SAMPRATE*ADX355_DATA_SIZE_EACH;

        rt_hw_interrupt_enable(temp); 
    }
}



int adx355_driver_init(){
    _hal_spi_event = rt_event_create("hal_spi_event", RT_IPC_FLAG_PRIO);
    if(_hal_spi_event == NULL) {
        rt_kprintf("hal_spi_event create failed!\n");
        return -1;
    }

    adx355_start_event = rt_event_create("adx355_start_event", RT_IPC_FLAG_PRIO);
    if(adx355_start_event == NULL) {
        rt_kprintf("adx355_start_event create failed!\n");
        return -1;
    }

    adx355_data_enough = rt_event_create("adx355_data_enough", RT_IPC_FLAG_PRIO);
    if(adx355_data_enough == NULL) {
        rt_kprintf("adx355_data_enough create failed!\n");
        return -1;
    }

    adx355_data_buffer = rt_malloc(ADX355_DATA_MQ_BUFFER);
    if(adx355_data_buffer == NULL) {
        rt_kprintf("adx355_data_buffer create failed!\n");
        return -1;
    }

    return 0;
}

int adx355_driver_init_later(void){
    rt_err_t ret;
    rt_thread_t adx355_driver_data_record_thread_t = rt_thread_create("adx355_data_rec",
                                    adx355_driver_data_record_thread_entry, NULL, 
                                    4096, 2, 20);
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
