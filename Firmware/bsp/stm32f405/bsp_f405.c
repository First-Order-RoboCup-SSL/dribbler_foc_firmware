#include "bsp/stm32f405/bsp_f405.h"
#include "bsp/bsp_drivers.h"
#include "config.h"
#include "controller/utils_math.h"

#include "main.h"
#include "tim.h"
#include "adc.h"
#include "spi.h"
#include <stdbool.h>

#define DRV_NSCS_PORT  GPIOC
#define DRV_NSCS_PIN   GPIO_PIN_13

#define HALL_C_Pin       HALL_Z_Pin
#define HALL_C_GPIO_Port HALL_Z_GPIO_Port

volatile float g_vbus_voltage_dbg = 24.0f;

void wdog_reload(void) {
}

void cpu_reboot(void) {
    NVIC_SystemReset();
}

static void drv8303_enable(void) {
    HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_SET);
    HAL_Delay(2);
}

static void drv8303_disable(void) {
    HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_RESET);
}

volatile uint16_t g_drv_last_tx        = 0;
volatile uint16_t g_drv_last_rx        = 0;
volatile uint32_t g_drv_write_count    = 0;
volatile int      g_drv_last_hal_status = -1;

static void drv8303_write_reg(uint8_t addr, uint16_t data) {
    uint16_t frame = ((uint16_t)(addr & 0x0F) << 11) | (data & 0x07FF);
    uint16_t tx = frame;
    uint16_t rx = 0xAAAA;

    g_drv_last_tx = frame;

    HAL_GPIO_WritePin(DRV_NSCS_PORT, DRV_NSCS_PIN, GPIO_PIN_RESET);
    __asm__("nop; nop; nop; nop;");
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&tx,
                                                   (uint8_t *)&rx, 1, 10);
    __asm__("nop; nop; nop; nop;");
    HAL_GPIO_WritePin(DRV_NSCS_PORT, DRV_NSCS_PIN, GPIO_PIN_SET);

    g_drv_last_rx = rx;
    g_drv_last_hal_status = (int)st;
    g_drv_write_count++;
}

__attribute__((unused))
static uint16_t drv8303_read_reg(uint8_t addr) {
    uint16_t tx_cmd   = (1U << 15) | ((uint16_t)(addr & 0x0F) << 11) | 0x807F;
    uint16_t tx_dummy = 0xFFFF;
    uint16_t rx       = 0;

    HAL_GPIO_WritePin(DRV_NSCS_PORT, DRV_NSCS_PIN, GPIO_PIN_RESET);
    __asm__("nop; nop; nop; nop;");
    HAL_SPI_Transmit(&hspi3, (uint8_t *)&tx_cmd, 1, 10);
    __asm__("nop; nop; nop; nop;");
    HAL_GPIO_WritePin(DRV_NSCS_PORT, DRV_NSCS_PIN, GPIO_PIN_SET);

    __asm__("nop; nop; nop; nop; nop; nop; nop; nop;");

    HAL_GPIO_WritePin(DRV_NSCS_PORT, DRV_NSCS_PIN, GPIO_PIN_RESET);
    __asm__("nop; nop; nop; nop;");
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&tx_dummy, (uint8_t *)&rx, 1, 10);
    __asm__("nop; nop; nop; nop;");
    HAL_GPIO_WritePin(DRV_NSCS_PORT, DRV_NSCS_PIN, GPIO_PIN_SET);

    return rx & 0x07FF;  /* lower 11 bits = data */
}

/* DRV8303 Control Register 2 (0x03), GAIN bits at D3:D2 
 *   00 = 10 V/V (default)   -> data 0x000
 *   01 = 20 V/V              -> data 0x004
 *   10 = 40 V/V              -> data 0x008  
 *   11 = 80 V/V              -> data 0x00C
 */
volatile uint16_t g_drv_readback_03 = 0xDEAD;  
volatile uint16_t g_drv_readback_02 = 0xDEAD;  
volatile uint16_t g_drv_readback_00 = 0xDEAD;  

static void drv8303_configure(void) {
    HAL_GPIO_WritePin(DRV_NSCS_PORT, DRV_NSCS_PIN, GPIO_PIN_SET);
    HAL_Delay(10);  

    (void)drv8303_read_reg(0x00);
    (void)drv8303_read_reg(0x01);

    /* Set GAIN = 40 V/V */
    for (int i = 0; i < 5; ++i) {
        drv8303_write_reg(0x03, 0x008);
    }
    HAL_Delay(1);

    g_drv_readback_00 = drv8303_read_reg(0x00);
    g_drv_readback_02 = drv8303_read_reg(0x02);
    g_drv_readback_03 = drv8303_read_reg(0x03);
}

void adc_init(void) {
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
}

void adc_start_convert(void) {
    HAL_ADCEx_InjectedStart_IT(&hadc1);
}

void adc_stop_convert(void) {
    HAL_ADCEx_InjectedStop(&hadc1);
}

void adc_get_phase_curr_value(uint8_t chan, int16_t *adc1, int16_t *adc2) {
    (void)chan;
    *adc1 = (int16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    *adc2 = (int16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
}

int16_t adc_get_dc_voltage_value(void) { return 0; }
int16_t adc_get_dc_current_value(void) { return 0; }

void bsp_mag_encoder_update(void) {}
float bsp_mag_encoder_get_elec_angle(void) { return 0.0f; }
float bsp_mag_encoder_get_velocity(void)   { return 0.0f; }

volatile int g_hall_val_dbg = 0;
volatile int g_hall_val_min = 0xFF;

static inline int hall_read_raw(void) {
    int hall = 0;
    if (HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin) == GPIO_PIN_SET) hall |= 0x4;
    if (HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin) == GPIO_PIN_SET) hall |= 0x2;
    if (HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin) == GPIO_PIN_SET) hall |= 0x1;
    return hall;
}

void bsp_hall_encoder_irq_update(void) {
    int hall = hall_read_raw();
    g_hall_val_dbg = hall;
    if (hall < g_hall_val_min) g_hall_val_min = hall;
}

void bsp_hall_encoder_init(void) {
    bsp_hall_encoder_irq_update();
}

int bsp_hall_encoder_get_val(void) {
    int hall = hall_read_raw();
    g_hall_val_dbg = hall;
    return hall;
}

void pwm_timer_init(uint32_t half_period) {
    htim1.Instance->ARR = half_period;

    if (half_period >= 5) {
        TIM1->CCR4 = half_period - 5;
    } else {
        TIM1->CCR4 = 0;
    }

    uint16_t mid = (uint16_t)(half_period / 2);
    TIM1->CCR1 = mid;
    TIM1->CCR2 = mid;
    TIM1->CCR3 = mid;
    TIM1->EGR  = TIM_EGR_UG;

    HAL_TIM_PWM_Start(&htim1,    TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,    TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,    TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_OC_Start(&htim1,     TIM_CHANNEL_4);

    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);

    drv8303_enable();
    drv8303_configure();   
}

void pwm_start_output(void) {
    __HAL_TIM_MOE_ENABLE(&htim1);
}

void pwm_stop_output(void) {
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    uint16_t mid = (uint16_t)(htim1.Instance->ARR / 2);
    TIM1->CCR1 = mid;
    TIM1->CCR2 = mid;
    TIM1->CCR3 = mid;
}

void pwm_update_duty(uint16_t a, uint16_t b, uint16_t c) {
    TIM1->CCR1 = a;
    TIM1->CCR2 = b;
    TIM1->CCR3 = c;
}

void pwm_enable_update_irq(bool enable) {
    (void)enable;
}

void pwm_enable_channel(void) {
    __HAL_TIM_MOE_ENABLE(&htim1);
}

void pwm_disable_channel(void) {
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
}

void mc_sched_timer_init(int ms) {
    (void)ms;
    HAL_TIM_Base_Start_IT(&htim6);
}

int32_t board_get_error(void) {
    return 0;
}

__attribute__((unused)) static void *const _bsp_keepalive[] = {
    (void *)drv8303_disable,
};
