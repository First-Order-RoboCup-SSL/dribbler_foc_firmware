/* Algorithm layer with core computational blocks */
#include "foc.h"
#include "pi.h"
#include "bsp/bsp.h"
#include "mc_config.h"
#include "config.h"
#include "utils_math.h"

#define SECTOR_1  0u
#define SECTOR_2  1u
#define SECTOR_3  2u
#define SECTOR_4  3u
#define SECTOR_5  4u
#define SECTOR_6  5u

u8 svpwm_get_duty(float alpha, float beta, float *duty) {
	float vabs = fabsf(alpha) + fabsf(beta);
	if (vabs < 1e-6f) {
		duty[0] = 0.5f;
		duty[1] = 0.5f;
		duty[2] = 0.5f;
		return 0xFF;
	}

	uint8_t sector = 0xFF;
	float   tA = 0.5f, tB = 0.5f, tC = 0.5f;
	float   X, Y, Z;

	if (beta >= 0) {
		if (alpha >= 0) {
			if ((ONE_BY_SQRT3 * beta) > alpha) {
				sector = SECTOR_2;
			} else {
				sector = SECTOR_1;
			}
		} else {
			if ((-ONE_BY_SQRT3 * beta) > alpha) {
				sector = SECTOR_3;
			} else {
				sector = SECTOR_2;
			}
		}
	} else {
		if (alpha >= 0) {
			if ((-ONE_BY_SQRT3 * beta) > alpha) {
				sector = SECTOR_5;
			} else {
				sector = SECTOR_6;
			}
		} else {
			if ((ONE_BY_SQRT3 * beta) > alpha) {
				sector = SECTOR_4;
			} else {
				sector = SECTOR_5;
			}
		}
	}

	X = (SQRT3 * beta);
	Y = 1.5f * alpha + (SQRT3_BY_2 * beta);
	Z = -1.5f * alpha + (SQRT3_BY_2 * beta);
	switch(sector) {
		case SECTOR_1: // 3
		{	
            float T4 = -Z;
			float T6 = X;
		#if CONFIG_PWM_HIGH_FIRST
			tC = 0.5f * (1.0f - T4 - T6); 
        	tB = tC + T6;
        	tA = tB + T4;
		#else
			tA = 0.5f * (1.0f - T4 - T6);
			tB = tA + T4;
			tC = tB + T6;
		#endif
			break;
		}
		case SECTOR_2: // 1
		{
			float T6 = Y;
			float T2 = Z;
		#if CONFIG_PWM_HIGH_FIRST
			tC = 0.5f * (1.0f - T6 - T2);
        	tA = tC + T6;
        	tB = tA + T2;
		#else
			tB = 0.5f * (1.0f - T6 - T2);
			tA = tB + T2;
			tC = tA + T6;
		#endif
			break;
		}
		case SECTOR_3: // 5
		{
			float T2 = X;
			float T3 = -Y;
		#if CONFIG_PWM_HIGH_FIRST
			tA = 0.5f * (1.0f - T2 - T3);
			tC = tA + T3;
			tB = tC + T2;
		#else
			tB = 0.5f * (1.0f - T2 - T3);
			tC = tB + T2;
			tA = tC + T3;
		#endif
			break;
		}
		case SECTOR_4: // 4
		{
			float T1 = -X;
			float T3 = Z;
		#if CONFIG_PWM_HIGH_FIRST
			tA = 0.5f * (1.0f - T1 - T3);
			tB = tA + T3;
			tC = tB + T1;
		#else
			tC = 0.5f * (1.0f - T1 - T3);
			tB = tC + T1;
			tA = tB + T3;
		#endif
			break;
		}
		case SECTOR_5: // 6
		{
			float T1 = -Y;
			float T5 = -Z;
		#if CONFIG_PWM_HIGH_FIRST
			tB = 0.5f * (1.0f - T1 - T5);
			tA = tB + T5;
			tC = tA + T1;
		#else
			tC = 0.5f * (1.0f - T1 - T5);
			tA = tC + T1;
			tB = tA + T5;
		#endif
			break;
		}					
		case SECTOR_6: // 2
		{
			float T4 = Y;
			float T5 = -X;
		#if CONFIG_PWM_HIGH_FIRST
			tB = 0.5f * (1.0f - T4 - T5);
			tC = tB + T5;
			tA = tC + T4;
		#else
			tA = 0.5f * (1.0f - T4 - T5);
			tC = tA + T4;
			tB = tC + T5;
		#endif
			break;
		}
		default:
			break;
	}	
	duty[0] = clamp_f(tA, 0, 1.0f);
	duty[1] = clamp_f(tB, 0, 1.0f);
	duty[2] = clamp_f(tC, 0, 1.0f);
	return sector;
}


void foc_init(foc_t *foc) {
    pi_reset(&foc->pi_con_id, 0);
    pi_reset(&foc->pi_con_iq, 0);
    foc->e_velocity_filted = 0;
    foc->id_set = 0;
    foc->iq_set = 0;
    foc->vd_set = 0;
    foc->vq_set = 0;
}

void foc_hall_init(hall_state_t *hall) {
	hall->ang_code_prev = -1;
	hall->ang_est_rad = 0.0f;
	hall->ang_est_limited_rad = 0.0f;
	hall->ang_est_prev_rad = 0.0f;
	hall->vel_est_filtered_rads = 0.0f;
	hall->dt_last = 1.0f;  // safe non-zero initial value
	hall->dt_now = 0.0f;
}

void foc_update_svpwm(foc_t *foc) {
    float s = sinf(foc->theta_e);
    float c = cosf(foc->theta_e);
    if (foc->vdc <= 0.0f) {
        foc->duty_norm[0] = 0;
        foc->duty_norm[1] = 0;
        foc->duty_norm[2] = 0;
        return;
    }
    float max_mag = foc->vdc * (ONE_BY_SQRT3 * CONFIG_SVM_MODULATION);
    if (!foc->b_openloop) {
        float max_vd = max_mag;
        foc->pi_con_id.max = max_vd;
        foc->pi_con_id.min = -max_vd;

        float err = foc->id_set - foc->id_mea;
        foc->vd_out = pi_run(&foc->pi_con_id, err, 0);

        float max_vq = sqrtsub2_f(max_vd, foc->vd_out);
        foc->pi_con_iq.max = max_vq;
        foc->pi_con_iq.min = -max_vq;

        err = foc->iq_set - foc->iq_mea;
        foc->vq_out = pi_run(&foc->pi_con_iq, err, 0);
    } 
    else {
        float max_vd = max_mag;
        foc->vd_out = clamp_f(foc->vd_set, -max_vd, max_vd);
        float max_vq = sqrtsub2_f(max_vd, foc->vd_out);
        foc->vq_out = clamp_f(foc->vq_set, -max_vq, max_vq);
    }

    park_inv(foc->vd_out, foc->vq_out, s, c, &foc->valpha_out, &foc->vbeta_out);
    float valpha_uni = foc->valpha_out / foc->vdc;
    float vbeta_uni = foc->vbeta_out / foc->vdc;

    foc->sector = svpwm_get_duty(valpha_uni, vbeta_uni, &foc->duty_norm[0]);
}

float foc_hall(float dt, hall_state_t *hall, int hall_val) {
	hall->dt_now += dt;

	int ang_code = mc_conf()->hall_params.hall_table[hall_val];

	if (ang_code < 201) {
		float ang_sample_rad = ((float)ang_code / 200.0f) * M_2PI;
		if (hall->ang_code_prev < 0) {
			// Initial valid sample
			hall->ang_code_prev = ang_code;
			hall->ang_est_rad = ang_sample_rad;
			hall->ang_est_limited_rad = ang_sample_rad;
		}
		// Transition made
		else if (ang_code != hall->ang_code_prev) {
			int diff = ang_code - hall->ang_code_prev;
			if (diff > 100) {
				diff -= 200;
			} else if (diff < -100) {
				diff += 200;
			}

			if (sign_f(diff) == sign_f(hall->dt_last)) {
				if (diff > 0) {
					hall->dt_last = hall->dt_now;
				} else {
					hall->dt_last = -hall->dt_now;
				}
			} else {
				hall->dt_last = -hall->dt_last;
			}
			hall->dt_now = 0.0f;

			// Middle of new and old angle
			int ang_avg = hall->ang_code_prev + diff / 2;
			if (ang_avg < 0) ang_avg += 200;
			ang_avg %= 200;

			hall->ang_est_rad = ((float)ang_avg / 200.0f) * M_2PI;
		}

		hall->ang_code_prev = ang_code;

		// Speed estimate from hall timing
		float speed_est_rads = 0.0f;
		float speed_est_rpm_abs = 0.0f;
		if (fabsf(hall->dt_last) > 1e-6f) {
			speed_est_rads = (M_PI / 3.0f) / hall->dt_last;
			speed_est_rpm_abs = fabsf(rads_2_erpm(speed_est_rads));
		}

		float rpm_threshold = rads_2_erpm((M_PI / 3.0f) / fmaxf(
				fmaxf(fabsf(hall->dt_now), fabsf(hall->dt_last)), 1e-6f));

		/* No Interpolation at low speed */
		if (fabsf(rpm_threshold) < mc_conf()->hall_params.hall_interp_erpm) {
			hall->ang_est_rad = ang_sample_rad;
		} else {
			// Interpolate
			float diff = rads_angle_diff(hall->ang_est_rad, ang_sample_rad);
			if (fabsf(diff) < (M_2PI / 12.0f) || sign_f(diff) != sign_f(speed_est_rads)) {
				hall->ang_est_rad += speed_est_rads * dt;
			} else {
				hall->ang_est_rad -= diff * 0.01f;
			}
		}

		norm_angle_rad(hall->ang_est_rad);

		// Rate limiting
		float angle_step = (fmaxf(speed_est_rpm_abs,
				mc_conf()->hall_params.hall_interp_erpm) / 60.0f)
				* M_2PI * dt * 1.5f;
		float angle_diff = rads_angle_diff(hall->ang_est_rad,
				hall->ang_est_limited_rad);

		if (fabsf(angle_diff) < angle_step) {
			hall->ang_est_limited_rad = hall->ang_est_rad;
		} else {
			hall->ang_est_limited_rad += angle_step * sign_f(angle_diff);
		}
		norm_angle_rad(hall->ang_est_limited_rad);
	}

	return hall->ang_est_limited_rad;
}