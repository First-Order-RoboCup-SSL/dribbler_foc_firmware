#ifndef _MOTOR_H__
#define _MOTOR_H__

#include "types.h"
#include "controller.h"
#include "hw_samples.h"
#include "utils_math.h"
#include "bsp/bsp.h"

#define PLOT_MAX_IDS 4

typedef struct {
    bool  b_start;
    u8    mode_running;
    float torque_command;
    float velocity_command;
    float vdq_command[2];
    float currdq_command[2];
    u8    plot_ids[PLOT_MAX_IDS];

    hw_samples_t samples;
    controller_t controller;
    hall_state_t hall;
} motor_t;

void motor_init(motor_t *motor);
bool motor_start(motor_t *motor, u8 mode);
bool motor_stop(motor_t *motor);
void motor_adc_irq_handler(motor_t *motor);
void motor_adc_compute_duty(motor_t *motor, u16 *ta, u16 *tb, u16 *tc);
void timer_up_irq_handler(motor_t *motor);
void mc_sched_irq_handler(motor_t *motor);

extern motor_t g_motor[1];
extern volatile u8 g_openloop_daxis_lock_dbg;

static inline motor_t *motor(int index) {
    return &g_motor[index];
}

static inline controller_t *controller(motor_t *motor) {
    return &motor->controller;
}

static inline void motor_set_target_velocity(motor_t *motor, float velocity) {
    motor->velocity_command = velocity;
}

static inline void motor_set_target_torque(motor_t *motor, float torque) {
    motor->torque_command = torque;
}

static inline void motor_set_target_vdq(motor_t *motor, float vd, float vq) {
    motor->vdq_command[0] = vd;
    motor->vdq_command[1] = vq;
}

static inline void motor_set_target_idq(motor_t *motor, float id, float iq) {
    motor->currdq_command[0] = id;
    motor->currdq_command[1] = iq;
}

/* encoder hooks (sim uses sim_encoder_get_*) */
static inline void motor_init_encoder(motor_t *motor) {
    (void)motor;
#if (CONFIG_USE_HALL_ENCODER)
    bsp_hall_encoder_init();
    foc_hall_init(&motor->hall);
#elif (CONFIG_USE_MAG_ENCODER)
#endif
}

static inline void motor_update_encoder(motor_t *motor) {
#if (CONFIG_USE_HALL_ENCODER)
    float ts = controller(motor)->foc.ts;
    int hall_val = bsp_hall_encoder_get_val();
    foc_hall(ts, &motor->hall, hall_val);
    float delta = rads_angle_diff(motor->hall.ang_est_limited_rad,
                                  motor->hall.ang_est_prev_rad);
    float vel_raw = delta / ts;
    motor->hall.vel_est_filtered_rads += 0.05f * (vel_raw - motor->hall.vel_est_filtered_rads);
    motor->hall.ang_est_prev_rad = motor->hall.ang_est_limited_rad;
#elif (CONFIG_USE_MAG_ENCODER)
    (void)motor;
    bsp_mag_encoder_update();
#endif
}

static inline float motor_get_encoder_velocity(motor_t *motor) {
#if (CONFIG_USE_HALL_ENCODER)
    return motor->hall.vel_est_filtered_rads;
#elif (CONFIG_USE_MAG_ENCODER)
    (void)motor;
    return bsp_mag_encoder_get_velocity();
#else
    (void)motor;
    return sim_encoder_get_velocity();
#endif
}

static inline float motor_get_encoder_angle(motor_t *motor) {
#if (CONFIG_USE_HALL_ENCODER)
    return motor->hall.ang_est_limited_rad;
#elif (CONFIG_USE_MAG_ENCODER)
    (void)motor;
    return bsp_mag_encoder_get_elec_angle();
#else
    (void)motor;
    return sim_encoder_get_angle();
#endif
}

#endif /* _MOTOR_H__ */
