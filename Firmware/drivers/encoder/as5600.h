#ifndef _DRV_AS5600_H_
#define _DRV_AS5600_H_

#include "drivers/encoder/encoder_common.h"

void as5600_update_from_raw(enc_state_t *st, u16 raw, u16 raw_full, float dt_sec);

#endif /* _DRV_AS5600_H_ */
