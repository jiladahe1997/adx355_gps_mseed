#ifndef __GPS_DRIVER_H__
#define __GPS_DRIVER_H__

void gps_driver_EXTI_handle(void);
uint64_t get_pulse_counter(void);
struct tm get_gps_time(void);
int gps_driver_init_later();

#endif