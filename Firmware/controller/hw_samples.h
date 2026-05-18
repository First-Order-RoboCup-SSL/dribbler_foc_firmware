#ifndef _HW_SAMPLES_H__
#define _HW_SAMPLES_H__

#include "controller/types.h"

typedef struct current_sample {
    s32   adc_offset_a;
    s32   adc_offset_b;
    s32   adc_offset_c;
    s32   adc_offset_ibus;
    s32   sample_count;
    s32   sample_skip;
    bool  b_get_offset;
} hw_samples_t;

void hw_sample_init(hw_samples_t *ph_curr);
void hw_sample_get_phacurr(hw_samples_t *ph_curr, float *iABC);
void hw_sample_get_phacurr_offset(hw_samples_t *ph_curr);
bool hw_sample_calc_phacurr_offset(hw_samples_t *ph_curr);
bool hw_sample_check_phacurr_offset(hw_samples_t *ph_curr);
bool hw_sample_get_vbuscurr_offset(hw_samples_t *samples);
float hw_sample_get_vbus_vol(hw_samples_t *samples);
float hw_sample_get_vbus_curr(hw_samples_t *samples);

#endif /* _HW_SAMPLES_H__ */
