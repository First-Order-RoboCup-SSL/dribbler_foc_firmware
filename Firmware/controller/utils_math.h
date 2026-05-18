#ifndef _UTILS_MATH_H__
#define _UTILS_MATH_H__

#include "controller/types.h"

#undef M_PI
#define M_PI (3.14159265f)
#define M_2PI  (2*M_PI)

// Constants
#define ONE_BY_SQRT3			(0.57735026919f) // 1/sqrt(3)
#define TWO_BY_SQRT3			(ONE_BY_SQRT3 * 2.0f)
#define SQRT3_BY_2				(0.86602540378f)
#define SQRT2_BY_SQRT3          (0.8164966f)
#define	TWO_BY_THREE            (0.66667f)
#define SQRT3                   (1.732f)

#define UTILS_IS_NAN(x)		((x) != (x))
#define UTILS_NAN_ZERO(x)	(x = UTILS_IS_NAN(x) ? 0.0F : x)

#ifdef USE_MEX_CMD
#include <math.h>
#define fast_sin_f sinf
#define fast_cos_f cosf
#define fast_sqrt_f sqrtf
#else
#include <math.h>
extern float arm_sin_f32(float x);
extern float arm_cos_f32(float x);
#define fast_sin_f arm_sin_f32
#define fast_cos_f arm_cos_f32
#define fast_sqrt_f sqrtf
#endif

static inline void fast_sin_cos_f(float angle, float *s, float *c){
    *s = fast_sin_f(angle);
    *c = fast_cos_f(angle);
}

#define SQ_f(x) ((x)*(x))
#define NORM2_f(x,y)		(fast_sqrt_f(x*x + y*y))
#define norm_angle_rad(a) {while (a >= M_2PI) a-=M_2PI;while (a < 0) a +=M_2PI;};
#define DIV_2(x) ((x)/2.0f)


#define abs_f(in) fabs(in)
#define sign_f(in) (in<0?-1.0f:1.0f)

/* sqrt(x^2 - y^2) */
#define sqrtsub2_f(x,y)       (fast_sqrt_f(SQ_f(x) - SQ_f(y)))

#define lowpass_filter(value, sample, filter_constant)	(value = (sample - value) * filter_constant + value)

#define rpm_2_rads(v, poles) ((v) * M_PI * poles / 30.0f)
#define rpm_2_w_hz(v, poles) ((v)*(poles)/60.0f)
#define rads_2_rpm(v, poles) ((v) * 30.0f / (M_PI * poles))
#define rads_2_degree(r) ((r) / M_PI * 180.0f)
#define angle_from_We_Ts(w, t) ((w) * (t))
#define angle_from_Hz_Ts(h, t) ((h) * (t) * M_2PI)
#define hz_2_w_rads(v) ((v) * M_2PI)
#define w_rads_2_hz(v) ((v) / M_2PI)
#define lpf_param(hz, ts) (hz_2_w_rads(hz) * ts)

#define rads_2_erpm(v) ((v) * 30.0f / M_PI)
#define erpm_2_rad(v) ((v) * M_PI / 30.0f)

#define _min(a,b) ((a)>(b)?(b):(a))
#define _max(x, y) ((x)>(y)?(x):(y))
#define _abs(x) ((x)>0?(x):-(x))

static inline void utils_norm_angle_rad(float *angle) {
    while (*angle < -M_PI) { *angle += M_2PI; }
    while (*angle >= M_PI) { *angle -= M_2PI; }
}

static inline void clark(float A, float B, float C, float *pAlpha, float *pBeta){
	*pAlpha = A;
	*pBeta = ONE_BY_SQRT3 * (B - C);
}

static inline void park(float alpha, float beta, float s, float c, float *pId, float *pIq) {
	*pId = alpha * c + beta * s;
	*pIq = - (alpha * s) + (beta * c);
}

static inline void clark_inv(float alpha, float beta, float *ABC){
	ABC[0] = alpha;
    float beta_temp = beta * SQRT3_BY_2;
	ABC[1] = -(alpha * 0.5f) + beta_temp;
	ABC[2] = -(alpha * 0.5f) - beta_temp;
}

static inline void park_inv(float id, float iq, float s, float c, float *pAlpha, float *pBeta) {

	*pAlpha = (id * c) - (iq * s);
	*pBeta  = (id * s) + (iq * c);
}

static inline float rads_angle_diff(float angle1, float angle2) {
	float diff = angle1 - angle2;
	while (diff > M_PI) {
		diff -= M_2PI;
	}
	while (diff < -M_PI) {
        diff += M_2PI;
	}
	return diff;
}

static inline void step_towards(float *value, float goal, float step) {
    if (*value < goal) {
        if ((*value + step) < goal) {
            *value += step;
        } else {
            *value = goal;
        }
    } else if (*value > goal) {
        if ((*value - step) > goal) {
            *value -= step;
        } else {
            *value = goal;
        }
    }
}

static inline bool step_towards_s16(s16 *value, s16 goal, s16 step) {
    if (*value < goal) {
        if ((*value + step) < goal) {
            *value += step;
        } else {
            *value = goal;
        }
    } else if (*value > goal) {
        if ((*value - step) > goal) {
            *value -= step;
        } else {
            *value = goal;
        }
    }
	return (*value == goal);
}

/* if x not in [in_min, in_max], output will in [..., out1] and [out2, ...] */
static inline float map_unlimit(float x, float in_min, float in_max, float out1, float out2) {
	return (x - in_min) * (out2 - out1) / (in_max - in_min) + out1;
}

/* output must be limited to [out1, out2] */
static inline float map_limited(float x, float in_min, float in_max, float out1, float out2) {
	if (x < in_min) {
		return out1;
	}else if (x > in_max) {
		return out2;
	}
	return map_unlimit(x, in_min, in_max, out1, out2);
}

static inline float clamp_f(float v, float minv, float maxv) {
	if (v < minv) {
		return minv;
	}else if (v > maxv) {
		return maxv;
	}
	return v;
}

// See
// http://math.stackexchange.com/questions/297768/how-would-i-create-a-exponential-ramp-function-from-0-0-to-1-1-with-a-single-val
static inline float throttle_curve(float x, float alpha) {
	return x / (1.0f + alpha * (1.0f - x));
}

/**
 * Fast atan2
 *
 * See based on https://math.stackexchange.com/a/1105038/81278
 *
 * @param y
 * y
 *
 * @param x
 * x
 *
 * @return
 * The angle in radians
 */
static inline float fast_atan2_f(float y, float x) {
    // a := min (|x|, |y|) / max (|x|, |y|)
    float abs_y = _abs(y);
    float abs_x = _abs(x);
    // inject FLT_MIN in denominator to avoid division by zero
    float a = _min(abs_x, abs_y) / (_max(abs_x, abs_y) + 1e-20f);
    // s := a * a
    float s = a * a;
    // r := ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a
    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    // if |y| > |x| then r := 1.57079637 - r
    if (abs_y > abs_x)
        r = 1.57079637f - r;
    // if x < 0 then r := 3.14159274 - r
    if (x < 0.0f)
        r = 3.14159274f - r;
    // if y < 0 then r := -r
    if (y < 0.0f)
        r = -r;

    return r;
}

#endif /* _UTILS_MATH_H__ */