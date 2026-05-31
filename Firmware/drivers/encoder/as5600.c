#include "drivers/encoder/as5600.h"
#include "controller/utils_math.h"

void as5600_update_from_raw(enc_state_t *st, u16 raw, u16 raw_full, float dt_sec) {
    if (raw_full == 0U) {
        return;
    }
    float angle_mech = ((float)raw / (float)raw_full) * M_2PI;
    enc_state_feed_mech_rad(st, angle_mech, dt_sec);
}
