#include "drivers/encoder/as5048a_pwm.h"
#include "controller/utils_math.h"

bool as5048a_pwm_update_from_capture(enc_state_t *st, u32   t_high_ticks, u32   t_period_ticks, float tim_clk_hz) {
    if (t_period_ticks == 0U || t_high_ticks > t_period_ticks || tim_clk_hz <= 0.0f) {
        return false;
    }

    float duty       = (float)t_high_ticks / (float)t_period_ticks;
    float chip_high  = duty * (float)AS5048A_PWM_PERIOD_CLKS;

    /* Drop invalid/error frames instead of forcing angle to zero. */
    if (chip_high < (float)(AS5048A_PWM_INIT_CLKS + AS5048A_PWM_ERROR_CLKS + 1U)) {
        return false;
    }

    s32 raw = (s32)(chip_high + 0.5f)
              - (s32)AS5048A_PWM_INIT_CLKS
              - (s32)AS5048A_PWM_ERROR_CLKS;

    if (raw < 0) {
        return false;
    }
    if (raw >= (s32)AS5048A_PWM_DATA_RANGE) raw = (s32)AS5048A_PWM_DATA_RANGE - 1;

    float angle_mech = ((float)raw / (float)AS5048A_PWM_DATA_RANGE) * M_2PI;
    float dt_sec     = (float)t_period_ticks / tim_clk_hz;
    enc_state_feed_mech_rad(st, angle_mech, dt_sec);
    return true;
}
