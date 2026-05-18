#include "bsp/bsp_drivers.h"
#include "bsp/bsp.h"
#include "controller/hw_samples.h"

#define NB_OFFSET_SAMPLES_SHIFT (7)
#define NB_OFFSET_SAMPLES (1 << NB_OFFSET_SAMPLES_SHIFT)

static s32 round_div(s32 v1, s32 a) {
    s32 mod = v1 % a;
    if (mod * 2 >= a) {
        return (v1 / a + 1);
    }
    return v1 / a;
}

void hw_sample_init(hw_samples_t *ph_curr) {
    adc_init();
    ph_curr->b_get_offset = false;
    ph_curr->adc_offset_a = CONFIG_ADC_FULL_MAX >> 1;
    ph_curr->adc_offset_b = CONFIG_ADC_FULL_MAX >> 1;
    ph_curr->adc_offset_c = CONFIG_ADC_FULL_MAX >> 1;
}

void hw_sample_get_phacurr_offset(hw_samples_t *ph_curr) {
#if CONFIG_SIM_DISABLE_ADC_OFFSET
    ph_curr->adc_offset_a = CONFIG_ADC_FULL_MAX >> 1;
    ph_curr->adc_offset_b = CONFIG_ADC_FULL_MAX >> 1;
    ph_curr->adc_offset_c = CONFIG_ADC_FULL_MAX >> 1;
    ph_curr->b_get_offset = false;
    return;
#endif
    ph_curr->adc_offset_a = 0;
    ph_curr->adc_offset_b = 0;
    ph_curr->adc_offset_c = 0;
    ph_curr->sample_count = NB_OFFSET_SAMPLES;
    ph_curr->sample_skip = 16;
    ph_curr->b_get_offset = true;
}

bool hw_sample_calc_phacurr_offset(hw_samples_t *ph_curr) {
#if CONFIG_SIM_DISABLE_ADC_OFFSET
    (void)ph_curr;
    return false;
#endif
    if (!ph_curr->b_get_offset) {
        return false;
    }

    s16 phase_current1 = 0;
    s16 phase_current2 = 0;
    adc_get_phase_curr_value(0xFF, &phase_current1, &phase_current2);

    if (ph_curr->sample_skip > 0) {
        ph_curr->sample_skip--;
        return true;
    }

    if (ph_curr->sample_count > 0) {
        ph_curr->sample_count--;
#if CONFIG_PHASE_CURRENT_PHASE_AB
        ph_curr->adc_offset_a += phase_current1;
        ph_curr->adc_offset_b += phase_current2;
        if (ph_curr->sample_count == 0) {
            ph_curr->adc_offset_a = round_div(ph_curr->adc_offset_a, NB_OFFSET_SAMPLES);
            ph_curr->adc_offset_b = round_div(ph_curr->adc_offset_b, NB_OFFSET_SAMPLES);
        }
#elif CONFIG_PHASE_CURRENT_PHASE_AC
        ph_curr->adc_offset_a += phase_current1;
        ph_curr->adc_offset_c += phase_current2;
        if (ph_curr->sample_count == 0) {
            ph_curr->adc_offset_a = round_div(ph_curr->adc_offset_a, NB_OFFSET_SAMPLES);
            ph_curr->adc_offset_c = round_div(ph_curr->adc_offset_c, NB_OFFSET_SAMPLES);
        }
#else
        ph_curr->adc_offset_b += phase_current1;
        ph_curr->adc_offset_c += phase_current2;
        if (ph_curr->sample_count == 0) {
            ph_curr->adc_offset_b = round_div(ph_curr->adc_offset_b, NB_OFFSET_SAMPLES);
            ph_curr->adc_offset_c = round_div(ph_curr->adc_offset_c, NB_OFFSET_SAMPLES);
        }
#endif
    } else {
        ph_curr->b_get_offset = false;
    }
    return true;
}

bool hw_sample_check_phacurr_offset(hw_samples_t *ph_curr) {
#if CONFIG_PHASE_CURRENT_PHASE_AB
    if ((ph_curr->adc_offset_a > (CONFIG_ADC_FULL_MAX >> 1) + CONFIG_ADC_OFFSET_TOLERANCE) ||
        (ph_curr->adc_offset_b > (CONFIG_ADC_FULL_MAX >> 1) + CONFIG_ADC_OFFSET_TOLERANCE)) {
        return true;
    }
    if ((ph_curr->adc_offset_a < (CONFIG_ADC_FULL_MAX >> 1) - CONFIG_ADC_OFFSET_TOLERANCE) ||
        (ph_curr->adc_offset_b < (CONFIG_ADC_FULL_MAX >> 1) - CONFIG_ADC_OFFSET_TOLERANCE)) {
        return true;
    }
#elif CONFIG_PHASE_CURRENT_PHASE_BC
    if ((ph_curr->adc_offset_c > (CONFIG_ADC_FULL_MAX >> 1) + CONFIG_ADC_OFFSET_TOLERANCE) ||
        (ph_curr->adc_offset_b > (CONFIG_ADC_FULL_MAX >> 1) + CONFIG_ADC_OFFSET_TOLERANCE)) {
        return true;
    }
    if ((ph_curr->adc_offset_c < (CONFIG_ADC_FULL_MAX >> 1) - CONFIG_ADC_OFFSET_TOLERANCE) ||
        (ph_curr->adc_offset_b < (CONFIG_ADC_FULL_MAX >> 1) - CONFIG_ADC_OFFSET_TOLERANCE)) {
        return true;
    }
#elif CONFIG_PHASE_CURRENT_PHASE_AC
    if ((ph_curr->adc_offset_c > (CONFIG_ADC_FULL_MAX >> 1) + CONFIG_ADC_OFFSET_TOLERANCE) ||
        (ph_curr->adc_offset_a > (CONFIG_ADC_FULL_MAX >> 1) + CONFIG_ADC_OFFSET_TOLERANCE)) {
        return true;
    }
    if ((ph_curr->adc_offset_c < (CONFIG_ADC_FULL_MAX >> 1) - CONFIG_ADC_OFFSET_TOLERANCE) ||
        (ph_curr->adc_offset_a < (CONFIG_ADC_FULL_MAX >> 1) - CONFIG_ADC_OFFSET_TOLERANCE)) {
        return true;
    }
#endif
    return false;
}

#define adc_to_current(adc) ((adc) * CONFIG_CURRENT_SENSOR_CEOF * CONFIG_CURRENT_POLARITY)

void hw_sample_get_phacurr(hw_samples_t *ph_curr, float *iABC) {
    s16 phase1 = 0;
    s16 phase2 = 0;
    adc_get_phase_curr_value(0xFF, &phase1, &phase2);

#if CONFIG_PHASE_CURRENT_PHASE_AB
    s16 adc_ia = (phase1 - (s16)ph_curr->adc_offset_a);
    s16 adc_ib = (phase2 - (s16)ph_curr->adc_offset_b);
    iABC[0] = adc_to_current(adc_ia);
    iABC[1] = adc_to_current(adc_ib);
    iABC[2] = -(iABC[0] + iABC[1]);
#elif CONFIG_PHASE_CURRENT_PHASE_AC
    s16 adc_ia = (phase1 - (s16)ph_curr->adc_offset_a);
    s16 adc_ic = (phase2 - (s16)ph_curr->adc_offset_c);
    iABC[0] = adc_to_current(adc_ia);
    iABC[2] = adc_to_current(adc_ic);
    iABC[1] = -(iABC[0] + iABC[2]);
#else
    s16 adc_ib = (phase1 - (s16)ph_curr->adc_offset_b);
    s16 adc_ic = (phase2 - (s16)ph_curr->adc_offset_c);
    iABC[1] = adc_to_current(adc_ib);
    iABC[2] = adc_to_current(adc_ic);
    iABC[0] = -(iABC[1] + iABC[2]);
#endif
}

bool hw_sample_get_vbuscurr_offset(hw_samples_t *samples) {
    (void)samples;
    return true;
}

float hw_sample_get_vbus_curr(hw_samples_t *samples) {
    (void)samples;
    return 0.0f;
}

float hw_sample_get_vbus_vol(hw_samples_t *samples) {
    (void)samples;
    return (g_vbus_voltage_dbg > 1.0f) ? g_vbus_voltage_dbg : 1.0f;
}
