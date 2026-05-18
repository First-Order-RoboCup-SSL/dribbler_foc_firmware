/* glue layer between Simulink and algorithm */
#include "simstruc.h"
#include "tmwtypes.h"
#include "config.h"
#include "bsp/bsp.h"
#include "controller/motor.h"
#include "controller/mc_config.h"
#include "math/fast_math.h"

int_T nInPortNums = 3;
int_T nInPortNumWidth[3] = {4, 2, 3};
int_T nOutPortNums = 10;
int_T nOutPortNumWidth[10] = {4, 2, 3, 2, 5, 2, 5, 2, 2, 1};

int_T simGetInPortNums(void) {
    return nInPortNums;
}

int_T simGetInPortNumWidth(int port) {
    return nInPortNumWidth[port];
}

int_T simGetOutPortNums(void) {
    return nOutPortNums;
}

int_T simGetOutPortNumWidth(int port) {
    return nOutPortNumWidth[port];
}

#define IN_PORTS(p)             *ssGetInputPortRealSignalPtrs(S, p)
#define OUT_PORTS(p)            ssGetOutputPortRealSignal(S, p)

#define in_adc_curr_A           IN_PORTS(0)[0]
#define in_adc_curr_B           IN_PORTS(0)[1]
#define in_adc_ibus             IN_PORTS(0)[2]
#define in_adc_vbus             IN_PORTS(0)[3]

#define in_enc_count            IN_PORTS(1)[0]
#define in_enc_dir              IN_PORTS(1)[1]

#define in_mot_vel              IN_PORTS(2)[0]
#define in_mot_angle            IN_PORTS(2)[1]
#define in_sim_clk              IN_PORTS(2)[2]

/* Outputs */
#define out_pwm_dutyA           OUT_PORTS(0)[0]
#define out_pwm_dutyB           OUT_PORTS(0)[1]
#define out_pwm_dutyC           OUT_PORTS(0)[2]
#define out_pwm_enable          OUT_PORTS(0)[3]

#define out_foc_vel             OUT_PORTS(1)[0]
#define out_foc_angle           OUT_PORTS(1)[1]

#define out_foc_phase_curr      OUT_PORTS(2)
#define out_foc_vdq_ramped      OUT_PORTS(3)
#define out_dbg_sched           OUT_PORTS(4)
#define out_foc_idiq_mea        OUT_PORTS(5)
#define out_dbg_mode            OUT_PORTS(6)
#define out_foc_alphabeta       OUT_PORTS(7)
#define out_foc_pi_max          OUT_PORTS(8)
#define out_dbg_timestep        OUT_PORTS(9)[0]

s16   g_encoder_dir = 1;
s16   g_encoder_counter = 0;
float g_motor_angle;
float g_motor_velocity;
u16   g_pwm_duty_out[3];
bool  g_pwm_update_irq = false;
bool  g_pwm_chan_enabled = false;
bool  g_adc_started = false;
static u32 mc_sched_count = 0;
s16   g_phase_current_adc[2];
s16   g_vbus_current_adc;
s16   g_vbus_voltage_adc;
u32   cpu_cycle_count = 0;
u32   SystemCoreClock = 2000000;
float g_sim_dt = 0.0f;

void sim_sfunc_init(SimStruct *S) {
    UNUSED_ARG(S);
    motor_init(motor(0));
    // motor(0)->vdq_command[0] = 1.0f;
    // motor(0)->vdq_command[1] = mc_conf()->board_params.vbus_voltage_max * 0.05f;
    // motor(0)->vdq_command[1] = 0.0f;
    // motor(0)->velocity_command = 5.0f;
    motor(0)->currdq_command[0] = 0.0f;
    motor(0)->currdq_command[1] = 3.0f;
    motor_start(motor(0), CTRL_MODE_CURRENT);
}

void sim_sfunc_terminate(SimStruct *S) {
    UNUSED_ARG(S);
}

void sim_sfunc_core(SimStruct *S) {
    g_phase_current_adc[0] = (s16)in_adc_curr_A;
    g_phase_current_adc[1] = (s16)in_adc_curr_B;
    g_vbus_current_adc = (s16)in_adc_ibus;
    g_vbus_voltage_adc = (s16)in_adc_vbus;

    cpu_cycle_count = (u32)(ssGetT(S) * SystemCoreClock);
    g_motor_angle = (float)in_mot_angle;
    g_motor_velocity = (float)in_mot_vel;
    g_encoder_counter = (s16)in_enc_count;
    g_encoder_dir = (s16)in_enc_dir;

    if (!ssIsMajorTimeStep(S)) {
        g_adc_started = false;
        return;
    }
    static real_T t_prev = -1.0;
    real_T t_now = ssGetT(S);
    if (t_prev >= 0.0) {
        g_sim_dt = (float)(t_now - t_prev);
    }
    t_prev = t_now;

    g_adc_started = true;
    motor_adc_irq_handler(motor(0));

    u32 slow_to_pwm_cnt = (CONFIG_FOC_PWM_FREQ / CONFIG_SPEED_CTRL_FREQ);
#if CONFIG_FOC_PWM_UPDATE_REPEAT == 0
    slow_to_pwm_cnt *= 2;
#endif
    if (++mc_sched_count == slow_to_pwm_cnt) {
        mc_sched_count = 0;
        mc_sched_irq_handler(motor(0));
    }
}

void sim_sfunc_output(SimStruct *S) {
    controller_t *ctrl = controller(motor(0));

    out_pwm_dutyA = g_pwm_duty_out[0];
    out_pwm_dutyB = g_pwm_duty_out[1];
    out_pwm_dutyC = g_pwm_duty_out[2];
    out_pwm_enable = g_pwm_chan_enabled;

    out_foc_vel = ctrl->encoder_velocity;
    out_foc_angle = ctrl->encoder_angle;

    out_foc_phase_curr[0] = ctrl->phase_curr_A;
    out_foc_phase_curr[1] = ctrl->phase_curr_B;
    out_foc_phase_curr[2] = ctrl->phase_curr_C;
    out_foc_vdq_ramped[0] = ctrl->foc.vd_out;
    out_foc_vdq_ramped[1] = ctrl->foc.vq_out;

    out_dbg_sched[0] = g_adc_started ? 1.0f : 0.0f;
    out_dbg_sched[1] = (float)mc_sched_count;
    out_dbg_sched[2] = ctrl->foc.id_set;
    out_dbg_sched[3] = ctrl->foc.iq_set;
    out_dbg_sched[4] = fast_atan2_f(ctrl->curr_beta, ctrl->curr_alpha);

    out_foc_idiq_mea[0] = ctrl->foc.id_mea;
    out_foc_idiq_mea[1] = ctrl->foc.iq_mea;

    out_dbg_mode[0] = (float)(ctrl->mode_request + 1); // 1..4
    out_dbg_mode[1] = (float)(ctrl->mode_running + 1); // 1..4
    out_dbg_mode[2] = ctrl->b_start ? 1.0f : 0.0f;
    out_dbg_mode[3] = motor(0)->currdq_command[0];
    out_dbg_mode[4] = motor(0)->currdq_command[1];

    out_foc_alphabeta[0] = ctrl->curr_alpha;
    out_foc_alphabeta[1] = ctrl->curr_beta;

    out_foc_pi_max[0] = ctrl->foc.pi_con_id.max;
    out_foc_pi_max[1] = ctrl->foc.pi_con_iq.max;

    out_dbg_timestep = g_sim_dt;
}
