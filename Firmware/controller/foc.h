#ifndef _FOC_H__
#define _FOC_H__

#include "pi.h"
typedef struct {
    int   ang_code_prev;
    float ang_est_rad;
    float ang_est_limited_rad;
    float ang_est_prev_rad;
    float vel_est_filtered_rads;
    float dt_last;
    float dt_now;
} hall_state_t;

typedef struct {
    float ts;
    float vdc;
    float theta_e;
    float theta_e_sin;
    float theta_e_cos;
    float id_set;
    float iq_set;

    float duty_norm[3];
    float m_velocity;
    float e_velocity;
    float e_velocity_filted;
    float id_mea;
    float iq_mea;
    float vd_set;            
    float vq_set;
    float vd_out;
    float vq_out;
    float valpha_out;
    float vbeta_out;
    int   b_openloop;
    pi_t  pi_con_id;
    pi_t  pi_con_iq;

    uint8_t sector;
} foc_t;

void foc_init(foc_t *foc);
void foc_update_svpwm(foc_t *foc);
void foc_hall_init(hall_state_t *hall);
float foc_hall(float dt, hall_state_t *hall, int hall_val);

#endif /* _FOC_H__ */