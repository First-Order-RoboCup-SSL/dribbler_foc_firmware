#ifndef _DRV_ENCODER_COMMON_H_
#define _DRV_ENCODER_COMMON_H_

#include "controller/types.h"

typedef struct {
    u16   pole_pairs;
    float offset_rad;
    s8    direction;
    float vel_lpf_alpha;

    bool  first;
    float elec_angle;
    float elec_velocity;
    float prev_elec_angle;
} enc_state_t;

void enc_state_init(enc_state_t *st,
                    u16   pole_pairs,
                    float offset_rad,
                    s8    direction,
                    float vel_lpf_alpha);

/* dt_sec : time elapsed since the previous successful feed.
 *          The driver uses it to compute electrical velocity. */
void enc_state_feed_mech_rad(enc_state_t *st, float angle_mech_rad, float dt_sec);

static inline float enc_state_get_elec_angle(const enc_state_t *st) {
    return st->elec_angle;
}

static inline float enc_state_get_velocity(const enc_state_t *st) {
    return st->elec_velocity;
}

#endif /* _DRV_ENCODER_COMMON_H_ */
