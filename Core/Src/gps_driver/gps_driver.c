
#include "rtthread.h"
#include "rthw.h"
#include "main.h"
#include "minmea.h"


#ifndef STATIC
#define STATIC static
#endif

#define _GPS_UART_DATA_MQ_SIZE   128
#define _GPS_UART_DATA_BUF_SIZE  128
STATIC rt_mq_t gps_uart_data_mq;

#define _GPS_NMEA_RMC_UPDATE     1<<0
STATIC rt_event_t gps_nmea_rmc_update_event;


extern UART_HandleTypeDef huart2;

STATIC struct minmea_date rmc_date={0};
STATIC struct minmea_time rmc_time={0};
uint8_t star_num=0;            // GPS卫星数量
STATIC uint64_t pulse_counter=0;

#define UBX_MSG_CLASS_NAV 0x01
#define UBX_MSG_ID_NAV_TIMEGPS 0x20
#define UBX_MSG_CLASS_AND_ID_NAV_TIMEGPS UBX_MSG_CLASS_NAV << 8 | UBX_MSG_ID_NAV_TIMEGPS

/**
 * UBX_NAV_TIMEGPS_VALID协议，其中valid代表：
 * Name Description
    towValid 1 = Valid GPS time of week (iTOW & fTOW, (see Time Validity section for details)
    weekValid 1 = Valid GPS week number (see Time Validity section for details)
    leapSValid 1 = Valid GPS leap seconds

    参考Ublox官方协议文档： https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf
 */
uint8_t UBX_NAV_TIMEGPS_VALID = 0;

/**
 * @brief 解析串口收到的数据,
 */
int data_parse_func(char* buf, uint32_t size){
    if(size < 2){
        rt_kprintf("data_parse_func error, size:[%d] is less than 2\n");
        return -1;
    }
    if(buf[0] == '$'){
        //NMEA数据
        struct minmea_sentence_rmc frame;
        switch (minmea_sentence_id(buf, false))
        {
        case MINMEA_SENTENCE_RMC:
                if (minmea_parse_rmc(&frame, buf)) {
                    // printf(INDENT_SPACES "$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
                    //         frame.latitude.value, frame.latitude.scale,
                    //         frame.longitude.value, frame.longitude.scale,
                    //         frame.speed.value, frame.speed.scale);
                    // printf(INDENT_SPACES "$xxRMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
                    //         minmea_rescale(&frame.latitude, 1000),
                    //         minmea_rescale(&frame.longitude, 1000),
                    //         minmea_rescale(&frame.speed, 1000));
                    // printf(INDENT_SPACES "$xxRMC floating point degree coordinates and speed: (%f,%f) %f\n",
                    //         minmea_tocoord(&frame.latitude),
                    //         minmea_tocoord(&frame.longitude),
                    //         minmea_tofloat(&frame.speed));
                    
                    //模拟自旋锁
                    register rt_base_t temp;
                    temp = rt_hw_interrupt_disable();

                    rmc_date = frame.date;
                    rmc_time = frame.time;

                    rt_hw_interrupt_enable(temp); 
                    rt_event_send(gps_nmea_rmc_update_event, _GPS_NMEA_RMC_UPDATE);
                }
                else {
                    rt_kprintf("$xxRMC sentence is not parsed\n");
                }
            break;
        case MINMEA_SENTENCE_GSV: {
        struct minmea_sentence_gsv frame;
        if (minmea_parse_gsv(&frame, buf))
        {
            // dev_dbg(dev, "$GSV: message %d of %d\n", frame.msg_nr,
            // frame.total_msgs); dev_dbg(dev, "$GSV: sattelites in view:
            // %d\n", frame.total_sats); int i;

            // for (i = 0; i < 4; i++)
            // dev_dbg(dev, "$GSV: sat nr %d, elevation: %d, azimuth: %d,
            // snr: %d dbm\n", frame.sats[i].nr, frame.sats[i].elevation,
            // frame.sats[i].azimuth,
            // frame.sats[i].snr);

            if (frame.msg_nr == 1)
            {
                /* clear */
                // status->start_list_len=0;
                // kfree(status->stars_list);

                star_num = frame.total_sats;
                // status->stars_list = kzalloc(sizeof(struct
                // star)*4*frame.total_msgs, GFP_KERNEL);
                // if(!status->stars_list){
                //     dev_err(dev, "malloc status.stars_list failed!");
                //     goto stars_list_failed;
                // }
            }

            // if(!status->stars_list){
            //     dev_err(dev, "malloc status.stars_list is NULL!");
            //     goto stars_list_failed;
            // }
            // for(i=0;i<4;i++){
            //     status->stars_list[i+4*(frame.msg_nr-1)].nr=frame.sats[i].nr;
            //     status->stars_list[i+4*(frame.msg_nr-1)].elevation=frame.sats[i].elevation;
            //     status->stars_list[i+4*(frame.msg_nr-1)].azimuth=frame.sats[i].azimuth;
            //     status->stars_list[i+4*(frame.msg_nr-1)].snr=frame.sats[i].snr;
            // }
            // stars_list_failed:
            break;
        }
    }
        
        default:
            break;
        }
    }
    else if((buf[0] == 0xB5) && (buf[1] == 0x62)){
        //Ublox数据
        //以下代码抄自 大摆
        unsigned short msg_class_msg_id = (buf[2] << 8) | buf[3];
        size_t data_len = (buf[5] << 8) | buf[4];
        UNUSED(data_len);
        const char * data = &buf[6];

        uint8_t leapS = data[10];
        switch (msg_class_msg_id)
        {
        case UBX_MSG_CLASS_AND_ID_NAV_TIMEGPS:
            // uint32_t iTOW = utils_get_uint32_t_little_endia(&data[0]);
            // uint32_t fTOW = utils_get_uint32_t_little_endia(&data[4]);
            // uint16_t week = utils_get_uint16_t_little_endia(&data[8]);
            // uint8_t valid = data[11];
            // uint32_t tAcc = utils_get_uint32_t_little_endia(&data[12]);

            // // dev_dbg(dev, "UBX_MSG_CLASS_AND_ID_NAV_TIMEGPS, iTOW:[%d]",iTOW);
            // status->timegps.iTOW = iTOW;
            // status->timegps.fTOW = fTOW;
            // status->timegps.week = week;
            // status->timegps.leapS = leapS;
            // status->timegps.valid = valid;
            // status->timegps.tAcc = tAcc;
            UBX_NAV_TIMEGPS_VALID = leapS;
            break;
        default:
            break;
        }
    }
    else{
        rt_kprintf("data_parse_func error, buf[0] unkonwn\n");
        return -1;
    }
    
    return 0;
}


/**
 * @brief gps主循环，循环处理收到的数据
 * @note 这里的实现逻辑是： 串口每收到一个字节就立即处理。
 * 
 * @note    
         * 串口收到的数据有两种格式：
         * 1.标准NMEA格式，帧头 '$' 帧尾 '\n'
         * 2.ublox私有协议格式， 帧头 '0xB5 0x62' 帧尾 无
        
 */ 
void gps_driver_main_loop(void *parameter){
    char buffer[_GPS_UART_DATA_BUF_SIZE]={0};
    uint32_t buffer_counter=0;

    bool lastIsUblox=false;

    while(1){
        char c;
        rt_err_t rt_ret;
        
        rt_ret = rt_mq_recv(gps_uart_data_mq, &c, 1, RT_WAITING_FOREVER);
        if(rt_ret != RT_EOK){
            rt_kprintf("gps_driver_main_loop rt_mq_recv failed, error code is:[%d]\n", rt_ret);
            continue;
        }


        /**
        * 串口收到的数据有两种格式：
        * 1.标准NMEA格式，帧头 '$' 帧尾 '\n'
        * 2.ublox私有协议格式， 帧头 '0xB5 0x62' 帧尾 无
        * 
        * 收到本帧头时，解析上一帧数据
        */
        switch (c)
        {
        case '$':
            data_parse_func(buffer, buffer_counter);
            rt_memset(buffer,0, _GPS_UART_DATA_BUF_SIZE);
            buffer_counter=0;
            break;
        
        case 0xB5:
            //收到0XB5还不能直接判断就是帧头，还必须进行下一帧
            lastIsUblox=true;
            buffer_counter++;
            break;
        
        case 0x62:
            if(lastIsUblox == true){
                lastIsUblox = false;
                data_parse_func(buffer, buffer_counter);
                rt_memset(buffer,0, _GPS_UART_DATA_BUF_SIZE);
                buffer_counter=0;
            }

        default:
            buffer_counter++;
            break;
        }
        buffer[buffer_counter] = c;
    }
}

int gps_driver_init_later(){
    gps_uart_data_mq = rt_mq_create("gps_uart_mq", 1, _GPS_UART_DATA_MQ_SIZE, RT_IPC_FLAG_FIFO);
    if(gps_uart_data_mq == NULL){
        rt_kprintf("gps_uart_data_mq create failed!\n");
        return -1;
    }

    gps_nmea_rmc_update_event = rt_event_create("gps_nmea_rmc_update", RT_IPC_FLAG_PRIO);
    if(gps_nmea_rmc_update_event == NULL){
        rt_kprintf("gps_uart_data_mq create failed!\n");
        return -1;
    }

    rt_thread_t gps_main_loop_thread_t = rt_thread_create("gps_main_loop", gps_driver_main_loop,
                                    NULL, 2048, 5, 20);
    if(gps_main_loop_thread_t == NULL){
        rt_kprintf("gps_main_loop_thread_t create failed!\n");
        return -1;
    }
    
    rt_thread_startup(gps_main_loop_thread_t);

    //手动开启串口接收中断位
    SET_BIT(huart2.Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);
    return 0;
}

/**
 * @brief 秒脉冲中断引脚处理函数
 * 
 * @note Core\Src\adxl355_driver\adx355_driver.c HAL_GPIO_EXTI_Callback中调用
 */
#define _ADX355_START_EVENT               1<<0
extern rt_event_t adx355_start_event;
void gps_driver_EXTI_handle(void){
    pulse_counter++;
    // if((pulse_counter > 10) && (UBX_NAV_TIMEGPS_VALID == 7))
    if(pulse_counter == 10 && rmc_time.seconds !=0)
        rt_event_send(adx355_start_event, _ADX355_START_EVENT);
}

uint64_t get_pulse_counter(void){
    return pulse_counter;
}

/**
 * @brief 返回GPRMC中获取到的时间,格式：2022-01-24T14:08:00
 */
struct tm get_gps_time(void){
    struct tm ret;
    ret.tm_year = rmc_date.year+2000-1900;
    ret.tm_mon = rmc_date.month-1;
    ret.tm_mday = rmc_date.day;
    ret.tm_hour = rmc_time.hours;
    ret.tm_min = rmc_time.minutes;
    ret.tm_sec = rmc_time.seconds;
    return ret;
}




/** 
 * @brief GPS数据串口接收中断处理函数
 */
void USART2_IRQHandler(void)
{
  if(huart2.Instance->ISR & USART_ISR_RXNE_RXFNE)
  {
    uint8_t data =  (uint8_t)(huart2.Instance->RDR & (uint8_t)0x00FF);
    rt_err_t error = rt_mq_send(gps_uart_data_mq, &data, 1);
    if(error != RT_EOK){
        __NOP();
    }
  }

  if(huart2.Instance->ISR & USART_ISR_ORE){
      __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF);
  }
}

// void USART2_IRQHandler(void)
// {
//   /* USER CODE BEGIN USART2_IRQn 0 */

//   /* USER CODE END USART2_IRQn 0 */
//   HAL_UART_IRQHandler(&huart2);
//   /* USER CODE BEGIN USART2_IRQn 1 */

//   /* USER CODE END USART2_IRQn 1 */
// }