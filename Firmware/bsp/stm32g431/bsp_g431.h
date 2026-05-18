#ifndef __BSP_G431_H__
#define __BSP_G431_H__

#include <stdint.h>

#define INA240_GAIN         50.0f      
#define RSENSE_OHM          0.005f

#define CONFIG_CURRENT_SENSOR_CEOF \
    (3.3f / 4095.0f / INA240_GAIN / RSENSE_OHM)

/* adc */
#define CONFIG_ADC_FULL_MAX         4095
#define CONFIG_ADC_REF_VOLTAGE      3.3f

#define CONFIG_ADC_OFFSET_TOLERANCE 80         
#define CONFIG_SIM_DISABLE_ADC_OFFSET 0        

/* pwm */
#define CONFIG_FOC_PWM_FREQ         40000
#define CONFIG_FOC_PWM_Period       4250      
#define CONFIG_FOC_PWM_UPDATE_REPEAT 1         
#define CONFIG_SVM_MODULATION       0.92f

/* cs */
#define CONFIG_PHASE_CURRENT_PHASE_AB   0
#define CONFIG_PHASE_CURRENT_PHASE_AC   0
#define CONFIG_PHASE_CURRENT_PHASE_BC   1  
#define CONFIG_PHASE_CURRENT_PHASE_ABC  0
#ifdef CONFIG_CURRENT_POLARITY
#undef CONFIG_CURRENT_POLARITY
#endif
#define CONFIG_CURRENT_POLARITY     -1.0f   /* overrides */

#define CONFIG_HW_MAX_DC_VOL        12
#define CONFIG_HW_MIN_DC_VOL        10          
#define CONFIG_HW_MAX_DC_CURRENT    10        
#define CONFIG_HW_MAX_PHASE_CURR    10         

#define CONFIG_USE_MAG_ENCODER      0
#define CONFIG_USE_HALL_ENCODER     1
#define CONFIG_ENCODER_ABI          0
#define CONFIG_ENCODER_ABS          0
#define CONFIG_ENCODER_PWM          0
#define CONFIG_ENCODER_TYPE         0
#define CONFIG_ENCODER_CC_INVERT    0


#define SystemCoreClockMHz          170
#define CONFIG_BOARD_COM_UART       1
#define CONFIG_BOARD_COM_CAN        0
#define CONFIG_UART_TX_BUFFER_SIZE  4096
#define CONFIG_UART_RX_BUFFER_SIZE  512

int32_t board_get_error(void);

void  bsp_mag_encoder_update(void);
float bsp_mag_encoder_get_elec_angle(void);
float bsp_mag_encoder_get_velocity(void);
void  bsp_hall_encoder_init(void);
void  bsp_hall_encoder_irq_update(void);
int   bsp_hall_encoder_get_val(void);

extern volatile float g_vbus_voltage_dbg;

#endif /* __BSP_G431_H__ */
