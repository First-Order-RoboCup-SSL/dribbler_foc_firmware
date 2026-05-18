/* run time configuratbles */
#include "mc_config.h"
#include "config.h"
#include "bsp/bsp.h"

mc_config_t _mc_conf;

void mc_conf_init(void) {
    _mc_conf.pwm_freq = CONFIG_FOC_PWM_FREQ;
    _mc_conf.pwm_period = CONFIG_FOC_PWM_Period;
    _mc_conf.pwm_half_period = _mc_conf.pwm_period / 2;
    _mc_conf.ts = 1.0f / _mc_conf.pwm_freq;
#if CONFIG_FOC_PWM_UPDATE_REPEAT == 0
    _mc_conf.ts = _mc_conf.ts / 2.0f;
#endif
    _mc_conf.slow_ts = 1.0f / (float)CONFIG_SPEED_CTRL_FREQ;

    _mc_conf.motor_params.poles = CONFIG_MOTOR_PARAMS_Poles;
    _mc_conf.motor_params.ld = CONFIG_MOTOR_PARAMS_Ld;
    _mc_conf.motor_params.lq = CONFIG_MOTOR_PARAMS_Lq;
    _mc_conf.motor_params.rs = CONFIG_MOTOR_PARAMS_Rs;
    _mc_conf.motor_params.flux = CONFIG_MOTOR_PARAMS_Flux;
    _mc_conf.motor_params.J = CONFIG_MOTOR_PARAMS_J;
    _mc_conf.motor_params.max_rpm = CONFIG_MOTOR_PARAMS_MAX_RPM;
    _mc_conf.motor_params.encoder_offset = CONFIG_MOTOR_ENCODER_OFFSET;

    _mc_conf.board_params.vbus_voltage_max = CONFIG_HW_MAX_DC_VOL;
    _mc_conf.board_params.vbus_voltage_min = CONFIG_HW_MIN_DC_VOL;
    _mc_conf.board_params.vbus_curr_max = CONFIG_HW_MAX_DC_CURRENT;
    _mc_conf.board_params.phase_curr_max = CONFIG_HW_MAX_PHASE_CURR;

    _mc_conf.control_params.curr_loop_bandwith = (s16)CONFIG_CURRENT_LOOP_BANDWIDTH_HZ;
    _mc_conf.control_params.enc_pll_bandwith = 0;

    float wc = M_2PI * CONFIG_CURRENT_LOOP_BANDWIDTH_HZ;
    _mc_conf.control_params.pid[pid_id].kp = _mc_conf.motor_params.ld * wc;
    _mc_conf.control_params.pid[pid_id].ki = _mc_conf.motor_params.rs * wc;
    _mc_conf.control_params.pid[pid_iq].kp = _mc_conf.motor_params.lq * wc;
    _mc_conf.control_params.pid[pid_iq].ki = _mc_conf.motor_params.rs * wc;

    _mc_conf.control_params.pid[pid_vel].kp = CONFIG_VELOCITY_PID_KP;
    _mc_conf.control_params.pid[pid_vel].ki = CONFIG_VELOCITY_PID_KI;
   
    _mc_conf.hall_params.hall_table[0] = CONFIG_HALL_TAB_0;
    _mc_conf.hall_params.hall_table[1] = CONFIG_HALL_TAB_1;
    _mc_conf.hall_params.hall_table[2] = CONFIG_HALL_TAB_2;
    _mc_conf.hall_params.hall_table[3] = CONFIG_HALL_TAB_3;
    _mc_conf.hall_params.hall_table[4] = CONFIG_HALL_TAB_4;
    _mc_conf.hall_params.hall_table[5] = CONFIG_HALL_TAB_5;
    _mc_conf.hall_params.hall_table[6] = CONFIG_HALL_TAB_6;
    _mc_conf.hall_params.hall_table[7] = CONFIG_HALL_TAB_7;
    _mc_conf.hall_params.hall_interp_erpm = CONFIG_HALL_INTERP_ERPM;
}
