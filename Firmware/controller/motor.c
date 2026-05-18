#include "motor.h"
#include "mc_config.h"
#include "config.h"

extern s16 g_phase_current_adc[2];
extern s16 g_vbus_voltage_adc;

volatile u8 g_openloop_daxis_lock_dbg = 0u;  /* 1 = Vd lock, 0 = Vq mode */

motor_t g_motor[1];

static void motor_update_samples(motor_t *motor) {
    controller_t *ctrl = controller(motor);

    motor_update_encoder(motor);
    ctrl->encoder_angle = motor_get_encoder_angle(motor);
    ctrl->encoder_velocity = motor_get_encoder_velocity(motor);

    float iabc[3] = {0};
    hw_sample_get_phacurr(&motor->samples, iabc);
    ctrl->phase_curr_A = iabc[0];
    ctrl->phase_curr_B = iabc[1];
    ctrl->phase_curr_C = iabc[2];

    ctrl->foc.vdc = hw_sample_get_vbus_vol(&motor->samples);
}

void motor_init(motor_t *motor) {
    memset(motor, 0, sizeof(motor_t));
    mc_conf_init();
    motor_init_encoder(motor);
    pwm_timer_init(mc_conf()->pwm_half_period);
    hw_sample_init(&motor->samples);
    hw_sample_get_phacurr_offset(&motor->samples);
    contrl_init(controller(motor));
    mc_sched_timer_init((int)(1000.0f / CONFIG_SPEED_CTRL_FREQ));
    adc_start_convert();
}

bool motor_start(motor_t *motor, u8 mode) {
    if (motor->b_start) {
        return true;
    }
    if (board_get_error()) {
        return false;
    }
    contrl_start(controller(motor), mode);
    motor->b_start = true;
    pwm_start_output();
    return true;
}

bool motor_stop(motor_t *motor) {
    if (!motor->b_start) {
        return true;
    }
    motor->velocity_command = 0;
    motor->torque_command = 0;
    pwm_stop_output();
    contrl_stop(controller(motor));
    motor->b_start = false;
    return true;
}

void timer_up_irq_handler(motor_t *motor) {
    (void)motor;
}

void motor_adc_compute_duty(motor_t *motor, u16 *ta, u16 *tb, u16 *tc) {
    controller_t *ctrl = controller(motor);
    foc_t *foc = &ctrl->foc;

    if (hw_sample_calc_phacurr_offset(&motor->samples)) {
        u16 half = mc_conf()->pwm_half_period >> 1;
        *ta = half;
        *tb = half;
        *tc = half;
        return;
    }
    motor_update_samples(motor);
    if (!motor->b_start) {
        *ta = 0;
        *tb = 0;
        *tc = 0;
        return;
    }

    contrl_get_phase_duty(ctrl);

    *ta = (u16)(foc->duty_norm[0] * mc_conf()->pwm_half_period);
    *tb = (u16)(foc->duty_norm[1] * mc_conf()->pwm_half_period);
    *tc = (u16)(foc->duty_norm[2] * mc_conf()->pwm_half_period);
}

/* ADC current sampling interrupt, call FOC core functions */
void motor_adc_irq_handler(motor_t *motor) {
    u16 ta, tb, tc;
    motor_adc_compute_duty(motor, &ta, &tb, &tc);
    if (!motor->b_start) {
        return;
    }
    pwm_update_duty(ta, tb, tc);
}

/* slow part */
void mc_sched_irq_handler(motor_t *motor) {
    motor->mode_running = contrl_update_mode(controller(motor));
    if (motor->mode_running == CTRL_MODE_TORQUE) {
        contrl_set_target_torque(controller(motor), motor->torque_command);
    }
    else if (motor->mode_running == CTRL_MODE_VELOCITY) {
        contrl_set_target_velocity(controller(motor), motor->velocity_command);
    }
    else if (motor->mode_running == CTRL_MODE_CURRENT) {
        contrl_set_target_idq(controller(motor), motor->currdq_command[0], motor->currdq_command[1]);
    }
    else {
        if (g_openloop_daxis_lock_dbg) {
            contrl_set_target_vdq(controller(motor), g_openloop_vf_offset_dbg, 0.0f);
        } else {
            float abs_omega = (g_openloop_omega_e_dbg >= 0.0f)
                              ? g_openloop_omega_e_dbg : -g_openloop_omega_e_dbg;
            float vq = g_openloop_vf_gain_dbg * abs_omega + g_openloop_vf_offset_dbg;
            if (vq > g_openloop_vq_limit_dbg) { vq = g_openloop_vq_limit_dbg; }
            if (g_openloop_omega_e_dbg < 0.0f) { vq = -vq; }
            contrl_set_target_vdq(controller(motor), 0.0f, vq);
        }
    }
    contrl_update_target(controller(motor));
}
