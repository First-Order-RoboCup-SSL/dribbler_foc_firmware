#include "bsp/stm32g431/bsp_g431.h"
#include "bsp/bsp_drivers.h"
#include "config.h"
#include "controller/utils_math.h"
#include "controller/types.h"

#if (CONFIG_MAG_ENCODER_TYPE == CONFIG_MAG_ENCODER_AS5600)
#include "drivers/encoder/as5600.h"
#endif

#include "main.h"
#include "tim.h"
#include "adc.h"
#include <stdbool.h>

volatile float g_vbus_voltage_dbg = 12.0f;

void wdog_reload(void) {
}

void cpu_reboot(void) {
    NVIC_SystemReset();
}

void adc_init(void) {
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADC_Start(&hadc1);
}

void adc_start_convert(void) {
    HAL_ADCEx_InjectedStart_IT(&hadc2);
}

void adc_stop_convert(void) {
    HAL_ADCEx_InjectedStop_IT(&hadc2);
    HAL_ADC_Stop(&hadc1);
}

void adc_get_phase_curr_value(uint8_t chan, int16_t *adc1, int16_t *adc2) {
    (void)chan;
    *adc1 = (int16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);  
    *adc2 = (int16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);  
}

// no bus sample hardware
int16_t adc_get_dc_voltage_value() { return 0; }
int16_t adc_get_dc_current_value() { return 0; }

#if (CONFIG_MAG_ENCODER_TYPE == CONFIG_MAG_ENCODER_AS5600)

#define MAG_ENC_VEL_LPF_ALPHA 0.05f
#define MAG_ENC_DT_SEC        (1.0f / (float)CONFIG_FOC_PWM_FREQ)

static enc_state_t s_enc;

void bsp_mag_encoder_init(void) {
    enc_state_init(&s_enc,
                   (u16)CONFIG_MOTOR_PARAMS_Poles,
                   0.0f,    /* offset applied in controller (g_mag_encoder_offset_dbg) */
                   1,       /* direction */
                   MAG_ENC_VEL_LPF_ALPHA);
}

void bsp_mag_encoder_update(void) {
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
    u16 raw = (u16)HAL_ADC_GetValue(&hadc1);
    as5600_update_from_raw(&s_enc, raw, (u16)CONFIG_ADC_FULL_MAX, MAG_ENC_DT_SEC);
}

float bsp_mag_encoder_get_elec_angle(void) { return enc_state_get_elec_angle(&s_enc); }
float bsp_mag_encoder_get_velocity(void)   { return enc_state_get_velocity(&s_enc); }

#else
void  bsp_mag_encoder_init(void)            {}
void  bsp_mag_encoder_update(void)          {}
float bsp_mag_encoder_get_elec_angle(void)  { return 0.0f; }
float bsp_mag_encoder_get_velocity(void)    { return 0.0f; }
#endif

volatile int g_hall_val_dbg = 0;
volatile int g_hall_idr_dbg = 0;    /* raw GPIOA IDR low 3 bits */
volatile int g_hall_val_min = 0xFF; 

void bsp_hall_encoder_irq_update(void) {
    int idr = (int)(GPIOA->IDR & 0x7u);
    g_hall_idr_dbg = idr;

    int hall = 0;
    if (HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin) == GPIO_PIN_SET) {
        hall |= 0x4;
    }
    if (HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin) == GPIO_PIN_SET) {
        hall |= 0x2;
    }
    if (HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin) == GPIO_PIN_SET) {
        hall |= 0x1;
    }

    g_hall_val_dbg = hall;
    if (hall < g_hall_val_min) g_hall_val_min = hall;
}

void bsp_hall_encoder_init(void) {
    bsp_hall_encoder_irq_update();
}

int bsp_hall_encoder_get_val(void) {
    return g_hall_val_dbg;
}

static bool s_tim3_pwm_started = false;

static inline void tim3_sync_start_on_tim1_update(void) {
    MODIFY_REG(TIM3->SMCR, TIM_SMCR_SMS | TIM_SMCR_TS,
               TIM_SLAVEMODE_TRIGGER | TIM_TS_ITR7);
    __HAL_TIM_DISABLE(&htim3);

    TIM3->CNT = 0U;
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE(&htim3);

    TIM1->CNT = 0U;
    TIM1->EGR = TIM_EGR_UG;
}

void pwm_timer_init(uint32_t half_period) {
    htim1.Instance->ARR = half_period;
    htim3.Instance->ARR = half_period;
    MODIFY_REG(TIM3->SMCR, TIM_SMCR_SMS | TIM_SMCR_TS,
               TIM_SLAVEMODE_TRIGGER | TIM_TS_ITR7);

    uint16_t mid = (uint16_t)(half_period / 2);
    TIM1->CCR1 = mid;
    TIM1->CCR2 = mid;
    TIM3->CCR1 = mid;
    TIM1->EGR  = TIM_EGR_UG;  
    TIM3->EGR  = TIM_EGR_UG;

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    s_tim3_pwm_started = false;

    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
}

void pwm_start_output(void) {
    if (!s_tim3_pwm_started) {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        tim3_sync_start_on_tim1_update();
        s_tim3_pwm_started = true;
    }
    __HAL_TIM_MOE_ENABLE(&htim1);
}

void pwm_stop_output(void) {
    if (s_tim3_pwm_started) {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        s_tim3_pwm_started = false;
    }
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    uint16_t mid = (uint16_t)(htim1.Instance->ARR / 2);
    TIM1->CCR1 = mid;
    TIM1->CCR2 = mid;
    TIM3->CCR1 = mid;
}

void pwm_update_duty(uint16_t a, uint16_t b, uint16_t c) {
    TIM1->CCR1 = a;
    TIM1->CCR2 = b;
    TIM3->CCR1 = c;
}

void pwm_enable_update_irq(bool enable) {
    (void)enable;   
}

void pwm_enable_channel(void) {
    if (!s_tim3_pwm_started) {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        tim3_sync_start_on_tim1_update();
        s_tim3_pwm_started = true;
    }
    __HAL_TIM_MOE_ENABLE(&htim1);
}

void pwm_disable_channel(void) {
    if (s_tim3_pwm_started) {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        s_tim3_pwm_started = false;
    }
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
}

void mc_sched_timer_init(int ms) {
    (void)ms;
    HAL_TIM_Base_Start_IT(&htim6);
}

int32_t board_get_error(void) {
    return 0;
}
