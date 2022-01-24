
#include "rtthread.h"

#ifndef STATIC
#define STATIC static
#endif

#define _SDCARD_DATA_READY_   1<<0
STATIC rt_event_t sdcard_data_ready_event;

/**
 * @brief sd卡主循环，主要用来写miniseed文件 
 */
void sdcard_driver_main_loop_entry(void *parameter){
    while(1){
        rt_uint32_t e;
        rt_event_recv(sdcard_data_ready_event, _SDCARD_DATA_READY_, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,
                        RT_WAITING_FOREVER, &e);
        RT_UNUSED(e);
    }
}

int sdcard_driver_init_later(){
    sdcard_data_ready_event = rt_event_create("sdcard_data_ready_event", RT_IPC_FLAG_PRIO);
    if(sdcard_data_ready_event == NULL){
        rt_kprintf("sdcard_data_ready_event create failed!\n");
        return -1;
    }


    return 0;
}