#ifndef _MC_CONFIG_H__
#define _MC_CONFIG_H__

#include "foc.h"
#include "lineramp.h"
#include "types.h"
typedef enum {
    pid_id,
    pid_iq,
    pid_vel,
    pid_max_id,
} pid_enum_t;

typedef struct {
    u8 poles;
    float ld;
    float lq;
    float rs;
    float flux;
    float J;
    s16 max_rpm;
    float encoder_offset;
} motor_params_t; // motor parameter

typedef struct {
	s16 vbus_voltage_max;    //if higher than max/lower than mean, motor force stop and can not start
	s16 vbus_voltage_min;   
	s16 vbus_curr_max;
	s16 phase_curr_max;      //max phase current
} board_params_t;

typedef struct {
	float kp;
	float ki;
} pid_params_t;

typedef struct {	
	s16 curr_loop_bandwith;
	s16 enc_pll_bandwith;
	pid_params_t pid[pid_max_id];
} contrl_params_t;

typedef struct {
	u8    hall_table[8];
	float hall_interp_erpm;
} hall_params_t;

typedef struct {
	float pwm_freq;
	float ts;
	float slow_ts;
	u16 pwm_period;
	u16 pwm_half_period;
	motor_params_t motor_params;
	board_params_t board_params;
	contrl_params_t control_params;
	hall_params_t hall_params;
} mc_config_t;

extern mc_config_t _mc_conf;

void mc_conf_init(void);

static inline mc_config_t *mc_conf(void) {
	return &_mc_conf;
}

#endif
