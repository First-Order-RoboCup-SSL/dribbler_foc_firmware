#ifndef __SIM_BSP_H__
#define __SIM_BSP_H__

#include <stdint.h>
#include "controller/types.h"

#define CONFIG_DC_CURR_SAMPLE 1
#define CONFIG_DC_CURR_CALC  1

#define CONFIG_ADC_FULL_MAX   4095
#define CONFIG_ADC_REF_VOLTAGE   3.3F

#define CONFIG_CURRENT_SENSOR_CEOF 0.1954f
#define CONFIG_IBUS_SENSOR_CEOF 0.1954f
#define CONFIG_VBUS_SENSOR_CEOF (1.0f/CONFIG_ADC_FULL_MAX*CONFIG_ADC_REF_VOLTAGE*34.33f)

#define CONFIG_HW_MAX_DC_CURRENT 250.0f
#define CONFIG_HW_MAX_PHASE_CURR   400
#define CONFIG_HW_MAX_DC_VOL    120
#define CONFIG_HW_MIN_DC_VOL    70

#define CONFIG_PHASE_CURRENT_PHASE_AB 1

#define CONFIG_FOC_PWM_FREQ 10000
#define CONFIG_FOC_PWM_Period (6000)
#define CONFIG_FOC_PWM_Half_Period (CONFIG_FOC_PWM_Period/2)

/* 1: single sample, 0: double sample */
#define CONFIG_FOC_PWM_UPDATE_REPEAT 1
#define CONFIG_FOC_HW_DeadTime 0

#define CONFIG_SVM_MODULATION 0.92f

#define CONFIG_SIM_DISABLE_ADC_OFFSET 0

#define SystemCoreClockMHz 1

#define CONFIG_USE_MAG_ENCODER  0
#define CONFIG_ENCODER_ABI 0
#define CONFIG_ENCODER_PWM 0
#define CONFIG_ENCODER_TYPE 0

float sim_encoder_get_angle(void);
float sim_encoder_get_velocity(void);
void pwm_update_duty(u16 ta, u16 tb, u16 tc);
void pwm_enable_update_irq(bool enable);

extern s16 g_encoder_counter;
extern s16 g_encoder_dir;

static inline float _enc_count(void) {
    return g_encoder_counter;
}
static inline void _enc_set_count(u16 count) {
    (void)count;
}

static inline s16 enc_direction(void) {
    return g_encoder_dir;
}

static inline void encoder_abi_timer_init(u32 res) {
    (void)res;
}
static inline void encoder_pwm_timer_init(void) {
}
#define pwm_count() 0

static inline void __disable_irq(void) {
}

static inline void __enable_irq(void) {
}

static inline void __NOP(void) {
}

static inline s32 board_get_error(void) {
    return 0;
}

#endif /* __SIM_BSP_H__ */
