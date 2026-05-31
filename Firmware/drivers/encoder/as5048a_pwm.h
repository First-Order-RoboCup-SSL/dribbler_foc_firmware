#ifndef _DRV_AS5048A_PWM_H_
#define _DRV_AS5048A_PWM_H_

#include "drivers/encoder/encoder_common.h"

#define AS5048A_PWM_PERIOD_CLKS  4119U
#define AS5048A_PWM_INIT_CLKS    12U
#define AS5048A_PWM_ERROR_CLKS   4U
#define AS5048A_PWM_DATA_RANGE   4096U

bool as5048a_pwm_update_from_capture(enc_state_t *st,
                                     u32   t_high_ticks,
                                     u32   t_period_ticks,
                                     float tim_clk_hz);

#endif /* _DRV_AS5048A_PWM_H_ */
