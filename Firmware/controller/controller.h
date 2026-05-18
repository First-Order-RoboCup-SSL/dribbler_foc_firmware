#ifndef _CONTROLLER_H__
#define _CONTROLLER_H__

#include "foc.h"
#include "lineramp.h"
#include "pi.h"

/* control modes */
#define CTRL_MODE_OPEN      ((uint8_t)0U)
#define CTRL_MODE_VELOCITY  ((uint8_t)1U)
#define CTRL_MODE_TORQUE    ((uint8_t)2U)
#define CTRL_MODE_CURRENT   ((uint8_t)3U)

typedef struct {
    bool    b_start;
    u8 mode_request;
    u8 mode_running;
    foc_t   foc;
    pi_t    pi_con_velocity;
    pi_t    pi_con_velocity_limit;
    float   encoder_angle;
    float   encoder_velocity;

    float   phase_curr_A;
    float   phase_curr_B;
    float   phase_curr_C;
    float   curr_alpha;
    float   curr_beta;

    lineramp_t id_target;
    lineramp_t iq_target;
    lineramp_t vel_target;
    lineramp_t vd_target;
    lineramp_t vq_target;
    lineramp_t torque_target;

} controller_t;

extern volatile float g_mag_encoder_offset_dbg;
extern volatile uint8_t g_openloop_use_encoder_dbg;
extern volatile float g_openloop_theta_dbg;
extern volatile float g_openloop_omega_e_dbg;
extern volatile uint8_t g_openloop_use_runtime_dt_dbg;
extern volatile float g_openloop_dt_dbg;
extern volatile float g_openloop_vf_gain_dbg;
extern volatile float g_openloop_vf_offset_dbg;
extern volatile float g_openloop_vq_limit_dbg;

void contrl_init(controller_t *ctrl);
void contrl_get_phase_duty(controller_t *ctrl);
void contrl_update_target(controller_t *ctrl);
bool contrl_enable(controller_t *ctrl, bool start);
bool contrl_request_mode(controller_t *ctrl, u8 mode);
u8   contrl_update_mode(controller_t *ctrl);

static inline bool contrl_start(controller_t *ctrl, u8 mode) {
    if (contrl_enable(ctrl, true)) { 
        return contrl_request_mode(ctrl, mode); 
    }
    return false;
}

static inline bool contrl_stop(controller_t *ctrl) {
    return contrl_enable(ctrl, false);
}

static inline bool contrl_is_start(controller_t *ctrl) {
    return ctrl->b_start;
}

static inline void contrl_torque_to_idq(controller_t *ctrl, float torque, float *id, float *iq) {
    // use the id=0 controller ?
    *id = 0;
    *iq = torque;
}

static inline void contrl_set_target_torque(controller_t *ctrl, float torque) {
	line_ramp_set_target(&ctrl->torque_target, torque);
}

static inline void contrl_set_target_velocity(controller_t *ctrl, float vel) {
	line_ramp_set_target(&ctrl->vel_target, vel);
}

static inline void contrl_set_target_idq(controller_t *ctrl, float id, float iq) {
	line_ramp_set_target(&ctrl->id_target, id);
	line_ramp_set_target(&ctrl->iq_target, iq);
}

static inline void contrl_set_target_vdq(controller_t *ctrl, float vd, float vq) {
	line_ramp_set_target(&ctrl->vd_target, vd);
	line_ramp_set_target(&ctrl->vq_target, vq);
}

#endif /* _CONTROLLER_H__ */
