#ifndef _DRV_AS5048A_PWM_H_
#define _DRV_AS5048A_PWM_H_

#include "drivers/encoder/encoder_common.h"

/* AS5048A in PWM output mode (datasheet rev 1.0):
 *   T_period(chip clk) = 4119
 *   T_init  (high)     = 12   (fixed start pulse)
 *   T_data  (high)     = pos  (0..4095, encodes angle)
 *   T_error (high)     = 4    (fixed error indicator)
 *
 * Measured by host TIM input capture:
 *   t_high_ticks    = total high width of one PWM frame
 *   t_period_ticks  = full period (rising edge to rising edge)
 *   tim_clk_hz      = host TIM tick frequency (e.g. 84e6 on F405)
 *
 * Returns true if the capture pair was valid and the state was advanced.
 */

#define AS5048A_PWM_PERIOD_CLKS  4119U
#define AS5048A_PWM_INIT_CLKS    12U
#define AS5048A_PWM_ERROR_CLKS   4U
#define AS5048A_PWM_DATA_RANGE   4096U

bool as5048a_pwm_update_from_capture(enc_state_t *st,
                                     u32   t_high_ticks,
                                     u32   t_period_ticks,
                                     float tim_clk_hz);

#endif /* _DRV_AS5048A_PWM_H_ */
