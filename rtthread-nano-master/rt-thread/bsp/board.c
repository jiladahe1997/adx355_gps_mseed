/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-07-24     Tanek        the first version
 * 2018-11-12     Ernest Chen  modify copyright
 */
 
#include <stdint.h>
#include <rthw.h>
#include <rtthread.h>

#define _SCB_BASE       (0xE000E010UL)
#define _SYSTICK_CTRL   (*(rt_uint32_t *)(_SCB_BASE + 0x0))
#define _SYSTICK_LOAD   (*(rt_uint32_t *)(_SCB_BASE + 0x4))
#define _SYSTICK_VAL    (*(rt_uint32_t *)(_SCB_BASE + 0x8))
#define _SYSTICK_CALIB  (*(rt_uint32_t *)(_SCB_BASE + 0xC))
#define _SYSTICK_PRI    (*(rt_uint8_t  *)(0xE000ED23UL))

// Updates the variable SystemCoreClock and must be called 
// whenever the core clock is changed during program execution.
extern void SystemCoreClockUpdate(void);

// Holds the system core clock, which is the system clock 
// frequency supplied to the SysTick timer and the processor 
// core clock.
extern uint32_t SystemCoreClock;

static uint32_t _SysTick_Config(rt_uint32_t ticks)
{
    if ((ticks - 1) > 0xFFFFFF)
    {
        return 1;
    }
    
    _SYSTICK_LOAD = ticks - 1; 
    _SYSTICK_PRI = 0xFF;
    _SYSTICK_VAL  = 0;
    _SYSTICK_CTRL = 0x07;  
    
    return 0;
}

/*
MEMORY
{
DTCMRAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 128K
RAM_D1 (xrw)      : ORIGIN = 0x24000000, LENGTH = 512K
RAM_D2 (xrw)      : ORIGIN = 0x30000000, LENGTH = 288K
RAM_D3 (xrw)      : ORIGIN = 0x38000000, LENGTH = 64K
ITCMRAM (xrw)      : ORIGIN = 0x00000000, LENGTH = 64K
FLASH (rx
 */
#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
#define RT_HEAP_SIZE 511*1024
uint8_t rt_heap[RT_HEAP_SIZE]  __attribute__ ((section (".RT_HEAP"))) = {0};  
RT_WEAK void *rt_heap_begin_get(void)
{
    return rt_heap;
}

RT_WEAK void *rt_heap_end_get(void)
{
    return rt_heap + RT_HEAP_SIZE;
}
#endif

/**
 * This function will initial your board.
 */
#include "main.h"
#include "rtthread.h"
#include "adx355_driver.h"
static rt_mq_t usart1_mq = NULL;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
rt_mutex_t console_mutex;
static char * rt_log_buf2;
static struct rt_memheap _heap1;
static struct rt_memheap _heap2;
static struct rt_memheap _heap3;

void SystemClock_Config(void);
void rt_hw_board_init()
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    MX_DMA_Init();
    HAL_NVIC_DisableIRQ(DMA1_Stream2_IRQn);// 关闭串口DMA中断，手动处理
    MX_SPI2_Init(); 
    MX_TIM1_Init();

    /* System Clock Update */
    SystemCoreClockUpdate();
    
    /* System Tick Configuration */
    _SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);




    /* Call components board initial (use INIT_BOARD_EXPORT()) */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
    // rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
/* 
DTCMRAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 128K
RAM_D1 (xrw)      : ORIGIN = 0x24000000, LENGTH = 512K
RAM_D2 (xrw)      : ORIGIN = 0x30000000, LENGTH = 288K
RAM_D3 (xrw)      : ORIGIN = 0x38000000, LENGTH = 64K
ITCMRAM (xrw)      : ORIGIN = 0x00000000, LENGTH = 64K 
    rt_memheap_init(0x24000000, 0x24080000);
    rt_memheap_init(0x30000000, 0x30048000);
    rt_memheap_init(0x38000000, 0x38010000);
*/
    rt_memheap_init(&_heap3,
                    "heap3",
                    0x38000000,
                    (rt_uint32_t)0x38010000 - (rt_uint32_t)0x38000000);
    rt_memheap_init(&_heap2,
                    "heap2",
                    0x30000000,
                    (rt_uint32_t)0x30048000 - (rt_uint32_t)0x30000000);
    rt_memheap_init(&_heap1,
                    "heap1",
                    0x24000000,
                    (rt_uint32_t)0x24080000 - (rt_uint32_t)0x24000000);
#endif

    //初始化串口
    MX_USART1_UART_Init();
    //开启串口 
    //使用消息队列来传送串口接收数据
    usart1_mq = rt_mq_create("usart1_recv_mq", 1, 128, RT_IPC_FLAG_PRIO);
    if(usart1_mq == RT_NULL){
        rt_kprintf("rt_mq_create usart1_recv_mq failed, system halt\n");
        while(1);
    }
    console_mutex = rt_mutex_create("console_mutext", RT_IPC_FLAG_PRIO);
    if(console_mutex == RT_NULL){
        rt_kprintf("rt_mutex_create console_mutext failed, output may not right\n");
    }
    rt_log_buf2 = rt_malloc(RT_CONSOLEBUF_SIZE);
    if(rt_log_buf2 == NULL){
        rt_kprintf("rt_malloc rt_log_buf2 failed, system halt\n");
        while(1);
    }

    /* 手动开启串口中断位 */
    SET_BIT(huart1.Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);

    /* sd卡初始化 */
    //MX_SDMMC1_SD_Init();

    /* 项目驱动初始化 */
    int ret = adx355_driver_init();
    if(ret <0){
        rt_kprintf("adx355_driver_init failed, error code is:[%d]\n",ret);
        rt_kprintf("system halt\n");
        while(1);
    }
}

void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();
    HAL_IncTick();

    /* leave interrupt */
    rt_interrupt_leave();
}

/* 串口接收处理函数 */
void USART1_IRQHandler(void){

  if(huart1.Instance->ISR & USART_ISR_RXNE_RXFNE)
  {
    uint8_t data =  (uint8_t)(huart1.Instance->RDR & (uint8_t)0x00FF);
    rt_err_t error = rt_mq_send(usart1_mq, &data, 1);
    if(error != RT_EOK){
        __NOP();
    }
  }
}

char rt_hw_console_getchar(void){
    char ch = 0;
    rt_mq_recv(usart1_mq, &ch, 1, 500);
    return ch;
}

typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;
void rt_hw_console_output(const char *str)
{
    // rt_size_t i = 0, size = 0;
    // char a = '\r';

    // __HAL_UNLOCK(&huart1);

    // size = rt_strlen(str);
    // for (i = 0; i < size; i++)
    // {
    //     if (*(str + i) == '\n')
    //     {
    //         HAL_UART_Transmit(&huart1, (uint8_t *)&a, 1, 500);
    //     }
    //     HAL_UART_Transmit(&huart1, (uint8_t *)(str + i), 1, 500);
    // }
    size_t size = rt_strlen(str);
    size_t i = 0;
    for(size_t c=0;c<size;c++) {
        rt_log_buf2[i++] = str[c];
        if(str[c] == '\n') {
            rt_log_buf2[i++] = '\r';
        }
    }
    rt_log_buf2[i++] = '\0';

    if(rt_thread_self()!=0)
        rt_mutex_take(console_mutex, RT_WAITING_FOREVER);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)rt_log_buf2, rt_strlen(rt_log_buf2));
    volatile uint32_t *isr_reg  = &(((DMA_Base_Registers *)hdma_usart1_tx.StreamBaseAddress)->ISR);
    volatile uint32_t *ifcr_reg = &(((DMA_Base_Registers *)hdma_usart1_tx.StreamBaseAddress)->IFCR);
    while(((*isr_reg) & (DMA_FLAG_TCIF0_4 << (hdma_usart1_tx.StreamIndex & 0x1FU))) == 0U);
    *ifcr_reg = DMA_FLAG_HTIF0_4 << (hdma_usart1_tx.StreamIndex & 0x1FU);
    *ifcr_reg = (DMA_FLAG_TCIF0_4) << (hdma_usart1_tx.StreamIndex & 0x1FU);
    huart1.gState  = HAL_UART_STATE_READY;
    __HAL_UNLOCK(&hdma_usart1_tx);
    hdma_usart1_tx.State = HAL_DMA_STATE_READY;
    if(rt_thread_self()!=0)
      rt_mutex_release(console_mutex);
}