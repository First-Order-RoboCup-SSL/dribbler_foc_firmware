#ifndef _PI_Contrller_H__
#define _PI_Contrller_H__

#include "types.h"

typedef struct {
	float  kp;
	float  ki;
	float  integral;
	float  max; // physical limits
	float  min;
	float  ts;
}pi_t;

static inline void pi_set_params(pi_t *pi, float kp, float ki) {
	pi->kp = kp;
	pi->ki = ki;
}

static inline void pi_set_saturate(pi_t *pi, float max, float min) {
	pi->max = max;
	pi->min = min;
}

static inline void pi_reset(pi_t *pi, float init) {
	pi->integral = (init);
}

static inline float pi_run(pi_t *pi, float err, float ff) {
	float kp_err = err * pi->kp;
	float integral_term = err * pi->ts * pi->ki; // = Ki*Ts*ei
	float out, out_presat; // pre-saturation output for back calculation

	pi->integral += integral_term; 
	out = pi->integral + kp_err + ff;
	out_presat = out;

    // integral clamping + back calculation
	if (out > pi->max) { // when output saturates in +ve direction...
		out = pi->max;
		if (kp_err >= pi->max) { // ...and big sat
			pi->integral = 0; // ...clamp
		}else { // ...and small sat
			pi->integral += (out - out_presat); //...back-calculate
		}
	}else if (out < pi->min) {
		out = pi->min;
		if (kp_err <= pi->min) {
			pi->integral = 0;
		}else {
			pi->integral += (out - out_presat);
		}
	}
	return out;
}

#endif 
