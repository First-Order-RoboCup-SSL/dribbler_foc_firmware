#include "drivers/encoder/encoder_common.h"
#include "controller/utils_math.h"

void enc_state_init(enc_state_t *st,
                    u16   pole_pairs,
                    float offset_rad,
                    s8    direction,
                    float vel_lpf_alpha) {
    st->pole_pairs      = pole_pairs ? pole_pairs : 1U;
    st->offset_rad      = offset_rad;
    st->direction       = (direction >= 0) ? 1 : -1;
    st->vel_lpf_alpha   = vel_lpf_alpha;

    st->first           = true;
    st->elec_angle      = 0.0f;
    st->elec_velocity   = 0.0f;
    st->prev_elec_angle = 0.0f;
}

void enc_state_feed_mech_rad(enc_state_t *st, float angle_mech_rad, float dt_sec) {
    float angle_mech = angle_mech_rad - st->offset_rad;
    if (st->direction < 0) {
        angle_mech = -angle_mech;
    }
    float angle_elec = angle_mech * (float)st->pole_pairs;
    norm_angle_rad(angle_elec);

    if (st->first || dt_sec <= 0.0f) {
        st->first           = false;
        st->prev_elec_angle = angle_elec;
        st->elec_angle      = angle_elec;
        return;
    }

    float delta = angle_elec - st->prev_elec_angle;
    if (delta >  M_PI)  delta -= M_2PI;
    if (delta < -M_PI)  delta += M_2PI;

    float vel_raw     = delta / dt_sec;
    st->elec_velocity = st->elec_velocity
                        + st->vel_lpf_alpha * (vel_raw - st->elec_velocity);
    st->prev_elec_angle = angle_elec;
    st->elec_angle      = angle_elec;
}
