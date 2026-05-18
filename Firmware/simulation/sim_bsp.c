#include "bsp/bsp.h"
#include "controller/mc_config.h"
#include "controller/types.h"
#include "tmwtypes.h"
#include <stdint.h>

extern float g_motor_angle;
extern float g_motor_velocity;
extern u16   g_pwm_duty_out[3];
extern bool  g_pwm_update_irq;
extern bool  g_pwm_chan_enabled;
extern bool  g_adc_started;
extern s16   g_phase_current_adc[2];
extern s16   g_vbus_current_adc;
extern s16   g_vbus_voltage_adc;

void wdog_reload(void) {
}

float sim_encoder_get_angle(void) {
    return g_motor_angle;
}

float sim_encoder_get_velocity(void) {
    return g_motor_velocity;
}

/* === timer === */
void pwm_timer_init(u32 half_period) {
    (void)half_period;
}

void mc_sched_timer_init(int ms) {
    (void)ms;
}

void pwm_enable_update_irq(bool enable) {
    g_pwm_update_irq = enable;
}

void pwm_start_output(void) {
    g_pwm_chan_enabled = true;
    g_pwm_duty_out[0] = mc_conf()->pwm_half_period / 2;
    g_pwm_duty_out[1] = mc_conf()->pwm_half_period / 2;
    g_pwm_duty_out[2] = mc_conf()->pwm_half_period / 2;
}

void pwm_stop_output(void) {
    g_pwm_chan_enabled = false;
    g_pwm_duty_out[0] = 0;
    g_pwm_duty_out[1] = 0;
    g_pwm_duty_out[2] = 0;
}

void pwm_update_duty(u16 a, u16 b, u16 c) {
    if (!g_pwm_chan_enabled) {
        g_pwm_duty_out[0] = mc_conf()->pwm_half_period / 2;
        g_pwm_duty_out[1] = mc_conf()->pwm_half_period / 2;
        g_pwm_duty_out[2] = mc_conf()->pwm_half_period / 2;
    } else {
        g_pwm_duty_out[0] = a;
        g_pwm_duty_out[1] = b;
        g_pwm_duty_out[2] = c;
    }
}

void pwm_enable_channel(void) {
    g_pwm_chan_enabled = true;
}

void pwm_disable_channel(void) {
    g_pwm_chan_enabled = false;
}

/* == adc == */
void adc_init(void) {
}

void adc_start_convert(void) {
    g_adc_started = true;
}

void adc_stop_convert(void) {
    g_adc_started = false;
}

void adc_get_phase_curr_value(uint8_t chan, int16_t *adc1, int16_t *adc2) {
    (void)chan;
    *adc1 = g_phase_current_adc[0];
    *adc2 = g_phase_current_adc[1];
}

int16_t adc_get_dc_current_value(void) {
    return g_vbus_current_adc;
}

int16_t adc_get_dc_voltage_value(void) {
    return g_vbus_voltage_adc;
}
