#ifndef _LINE_RAMP__
#define _LINE_RAMP__
#include "utils_math.h"
#include "config.h"

typedef struct {
    float target; 
    float interpolation;
    float step;    
    float time;    
    float time_dec; 
    float min_step;
} lineramp_t;

/* delta is change of target */
static inline void line_ramp_calc_step(lineramp_t *line, float delta) {
    if (delta > 0) {
        line->step = delta / line->time;
    } 
    else if (delta < 0) {
        float delta_abs = abs_f(delta);
        line->step = delta_abs / line->time_dec;
    }
    else {
        line->step = 0;
    }
    if ((line->step != 0) && (line->min_step > 0) && (line->step < line->min_step)) {
        line->step = line->min_step;
    }
}

static inline void line_ramp_set_target(lineramp_t *line, float target) {
    if (line->target != target) {
        line->target = target; // record new target
        float delta = (line->target - line->interpolation); //
        line_ramp_calc_step(line, delta);
        step_towards(&line->interpolation, line->target, line->step);
    }
}

static inline void line_ramp_init(lineramp_t *line, float time) {
    line->target = 0;
    line->interpolation = 0;
    line->step = 0;
    line->min_step = 0;
    line->time_dec = line->time = time;
}

static inline void line_ramp_reset(lineramp_t *line, float target) {
    line->target = target;
    line->interpolation = target;
}

static inline bool line_ramp_set_dectime(lineramp_t *line, float time) {
    float old = line->time_dec;
    line->time_dec = time;
    return old != time;
}

static inline bool line_ramp_set_acctime(lineramp_t *line, float time) {
    float old = line->time;
	line->time = time;
	return old != time;
}

static inline bool line_ramp_set_time(lineramp_t *line, float time) {
    bool change = line_ramp_set_dectime(line, time);
    change |= line_ramp_set_acctime(line, time);
    return change;
}

static inline void line_ramp_set_minstep(lineramp_t *line, float step) {
	line->min_step = step;
}

static inline float line_ramp_get_target(lineramp_t *line) {
	return line->target;
}

static inline float line_ramp_get_interp(lineramp_t *line) {
	return line->interpolation;
}

static inline void line_ramp_update(lineramp_t *line) {
	float delta = (line->target - line->interpolation);
	line_ramp_calc_step(line, delta);
}

static inline float line_ramp_step(lineramp_t *line) {
#if CONFIG_DISABLE_RAMP
    line->interpolation = line->target;
    return line->interpolation;
#else
	step_towards(&line->interpolation, line->target, line->step);
	return line->interpolation;
#endif
}

#endif /* _LINE_RAMP__ */
