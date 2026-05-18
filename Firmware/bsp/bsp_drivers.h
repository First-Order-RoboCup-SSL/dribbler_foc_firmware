#ifndef __BSP_DRIVERS_H__
#define __BSP_DRIVERS_H__

#include "controller/types.h"

/* cpu related function */
void wdog_reload(void);
void cpu_reboot(void);

/* 1ms timer function */
void mc_sched_timer_init(int ms);

/* adc driver functions */
void adc_init(void);
void adc_start_convert(void);
void adc_stop_convert(void);
void adc_get_phase_curr_value(u8 chan, s16 *adc1, s16 *adc2);

s16 adc_get_dc_current_value(void);
s16 adc_get_dc_voltage_value(void);

/* pwm timer driver functions */
void pwm_timer_init(u32 half_period);
void pwm_enable_update_irq(bool enable);
void pwm_start_output(void);
void pwm_stop_output(void);
void pwm_update_duty(u16 a, u16 b, u16 c);
void pwm_enable_channel(void);
void pwm_disable_channel(void);

#endif /* __BSP_DRIVERS_H__ */
