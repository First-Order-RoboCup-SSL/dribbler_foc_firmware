#include "controller.h"
#include "mc_config.h"
#include "config.h"
#include "bsp/bsp.h"
#if defined(STM32G431xx)
#  include "stm32g4xx.h"
#elif defined(STM32F405xx)
#  include "stm32f4xx.h"
#endif

/* debug vars for bring-up */
volatile float g_mag_encoder_offset_dbg = CONFIG_MAG_ENCODER_OFFSET_RAD;  /* 5.66 for 2804+AS5600 */
volatile u8    g_openloop_use_encoder_dbg = 0u;
volatile float g_openloop_theta_dbg = 0.0f;
volatile float g_openloop_omega_e_dbg = 0.0f;
volatile u8    g_openloop_use_runtime_dt_dbg = 1u;
volatile float g_openloop_dt_dbg = 0.0f;

volatile float g_openloop_vf_gain_dbg   = 0.0f; 
volatile float g_openloop_vf_offset_dbg = 0.09f;  
volatile float g_openloop_vq_limit_dbg  = 1.5f;   

static float s_openloop_theta_e = 0.0f;
static bool s_openloop_dwt_inited = false;
static bool s_openloop_dt_valid = false;
static uint32_t s_openloop_last_cyc = 0u;

static inline float wrap_0_2pi(float angle) {
    while (angle >= M_2PI) {
        angle -= M_2PI;
    }
    while (angle < 0.0f) {
        angle += M_2PI;
    }
    return angle;
}

static inline void openloop_dwt_init_once(void) {
    if (s_openloop_dwt_inited) {
        return;
    }
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_openloop_dwt_inited = true;
}

static inline float openloop_get_dt(float fallback_ts) {
    if (!g_openloop_use_runtime_dt_dbg) {
        s_openloop_dt_valid = false;
        g_openloop_dt_dbg = fallback_ts;
        return fallback_ts;
    }

    openloop_dwt_init_once();

    uint32_t now_cyc = DWT->CYCCNT;
    float dt = fallback_ts;

    if (s_openloop_dt_valid) {
        uint32_t dcyc = now_cyc - s_openloop_last_cyc;
        if ((dcyc > 0u) && (SystemCoreClock > 0u)) {
            dt = (float)dcyc / (float)SystemCoreClock;
        }
    } else {
        s_openloop_dt_valid = true;
    }
    s_openloop_last_cyc = now_cyc;
    dt = clamp_f(dt, 0.25f * fallback_ts, 4.0f * fallback_ts);
    g_openloop_dt_dbg = dt;
    return dt;
}

void contrl_init(controller_t *ctrl) {
    memset(ctrl, 0, sizeof(controller_t));
    s_openloop_theta_e = 0.0f;
    s_openloop_dt_valid = false;
    g_openloop_dt_dbg = mc_conf()->ts;

    line_ramp_init(&ctrl->id_target, 10);
    line_ramp_init(&ctrl->iq_target, 10);
    line_ramp_init(&ctrl->vd_target, 300);
    line_ramp_init(&ctrl->vq_target, 300);
    line_ramp_init(&ctrl->vel_target, 100);
    line_ramp_init(&ctrl->torque_target, 50);

    ctrl->mode_request = ctrl->mode_running = CTRL_MODE_OPEN;
    ctrl->foc.ts = mc_conf()->ts;

    ctrl->foc.pi_con_id.kp = mc_conf()->control_params.pid[pid_id].kp;
    ctrl->foc.pi_con_id.ki = mc_conf()->control_params.pid[pid_id].ki;
    ctrl->foc.pi_con_id.ts = mc_conf()->ts;

    ctrl->foc.pi_con_iq.kp = mc_conf()->control_params.pid[pid_iq].kp;
    ctrl->foc.pi_con_iq.ki = mc_conf()->control_params.pid[pid_iq].ki;
    ctrl->foc.pi_con_iq.ts = mc_conf()->ts;

    ctrl->pi_con_velocity.ts = mc_conf()->slow_ts;
    ctrl->pi_con_velocity.kp = mc_conf()->control_params.pid[pid_vel].kp;
    ctrl->pi_con_velocity.ki = mc_conf()->control_params.pid[pid_vel].ki;
    pi_set_saturate(&ctrl->pi_con_velocity,
                    mc_conf()->board_params.phase_curr_max,
                    -mc_conf()->board_params.phase_curr_max);

    foc_init(&ctrl->foc);
}

void contrl_get_phase_duty(controller_t *ctrl){
    foc_t *foc = &ctrl->foc;
    if ((ctrl->mode_running == CTRL_MODE_OPEN) && (g_openloop_use_encoder_dbg == 0u)) {
        /* angle generator */
        float dt = openloop_get_dt(foc->ts);
        s_openloop_theta_e += g_openloop_omega_e_dbg * dt;
        s_openloop_theta_e = wrap_0_2pi(s_openloop_theta_e);
        foc->theta_e = s_openloop_theta_e + g_openloop_theta_dbg;
    } else {
#if CONFIG_THETA_INVERT
        foc->theta_e = -ctrl->encoder_angle;
#else
        foc->theta_e = ctrl->encoder_angle;
#endif
        s_openloop_dt_valid = false;
        g_openloop_dt_dbg = foc->ts;
    }
    foc->theta_e = wrap_0_2pi(foc->theta_e);
#if CONFIG_USE_MAG_ENCODER
    foc->theta_e += g_mag_encoder_offset_dbg;
    foc->theta_e = wrap_0_2pi(foc->theta_e);
#endif

    clark(ctrl->phase_curr_A, ctrl->phase_curr_B, ctrl->phase_curr_C, &ctrl->curr_alpha, &ctrl->curr_beta);
    foc->theta_e_sin = sinf(foc->theta_e);
    foc->theta_e_cos = cosf(foc->theta_e);
    park(ctrl->curr_alpha, ctrl->curr_beta, foc->theta_e_sin, foc->theta_e_cos, &foc->id_mea, &foc->iq_mea);

    foc->b_openloop = (ctrl->mode_running == CTRL_MODE_OPEN);
    foc->m_velocity = ctrl->encoder_velocity;

    foc_update_svpwm(foc);
}

void contrl_update_target(controller_t *ctrl) {
    if (!ctrl->b_start) {
        return;
    }

    float id_set = 0, iq_set = 0;
    float vd_set = 0, vq_set = 0;
    float torque = 0;
    float vel_target = 0, vel_err = 0;

    if (ctrl->mode_running == CTRL_MODE_CURRENT) {
        id_set = line_ramp_step(&ctrl->id_target);
        iq_set = line_ramp_step(&ctrl->iq_target);
    }
    else if (ctrl->mode_running == CTRL_MODE_TORQUE) {
        torque = line_ramp_step(&ctrl->torque_target);
        contrl_torque_to_idq(ctrl, torque, &id_set, &iq_set);
    }
    else if (ctrl->mode_running == CTRL_MODE_VELOCITY) {
        vel_target = line_ramp_step(&ctrl->vel_target);
        vel_err = vel_target - ctrl->encoder_velocity;
        torque = pi_run(&ctrl->pi_con_velocity, vel_err, 0);
        contrl_torque_to_idq(ctrl, torque, &id_set, &iq_set);
    }
    else {
        vd_set = line_ramp_step(&ctrl->vd_target);
        vq_set = line_ramp_step(&ctrl->vq_target);
    }

    ctrl->foc.id_set = id_set;
    ctrl->foc.iq_set = iq_set;
    ctrl->foc.vd_set = vd_set;
    ctrl->foc.vq_set = vq_set;
}

bool contrl_enable(controller_t *ctrl, bool start) {
    if (ctrl->b_start == start) {
        return true;
    }

    pi_reset(&ctrl->pi_con_velocity_limit, 0);
    pi_reset(&ctrl->pi_con_velocity, 0);

    ctrl->mode_request = CTRL_MODE_OPEN;
    ctrl->mode_running = CTRL_MODE_OPEN;

    foc_init(&ctrl->foc);
    s_openloop_theta_e = 0.0f;
    s_openloop_dt_valid = false;
    g_openloop_dt_dbg = mc_conf()->ts;

    ctrl->b_start = start;
    return true;
}

bool contrl_request_mode(controller_t *ctrl, u8 mode) {
    if (mode > CTRL_MODE_CURRENT) {
        return false;
    }
    ctrl->mode_request = mode;
    return true;
}

u8 contrl_update_mode(controller_t *ctrl) {
    u8 mode_last = ctrl->mode_running;
    if (!ctrl->b_start) {
        ctrl->mode_running = CTRL_MODE_OPEN;
    } else {
        ctrl->mode_running = ctrl->mode_request;
    }
    if (mode_last != ctrl->mode_running) {
        /* Avoid carrying current-loop integral state across mode switches. */
        pi_reset(&ctrl->foc.pi_con_id, 0.0f);
        pi_reset(&ctrl->foc.pi_con_iq, 0.0f);

        if (ctrl->mode_running == CTRL_MODE_CURRENT) {
            line_ramp_reset(&ctrl->id_target, 0);
            line_ramp_reset(&ctrl->iq_target, 0);
        } else if (ctrl->mode_running == CTRL_MODE_TORQUE) {
            line_ramp_reset(&ctrl->torque_target, 0);
        } else if (ctrl->mode_running == CTRL_MODE_VELOCITY) {
            line_ramp_reset(&ctrl->vel_target, 0);
        } else {
            line_ramp_reset(&ctrl->vd_target, 0);
            line_ramp_reset(&ctrl->vq_target, 0);
        }
    }
    return ctrl->mode_running;
}
