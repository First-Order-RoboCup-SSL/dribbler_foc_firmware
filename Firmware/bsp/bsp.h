#ifndef __BSP_H__
#define __BSP_H__

#include <stdint.h>

#if defined(SIMULATION)
#  include "simulation/sim_bsp.h"
#elif defined(STM32G431xx)
#  include "bsp/stm32g431/bsp_g431.h"
#elif defined(STM32F405xx)
#  include "bsp/stm32f405/bsp_f405.h"
#endif

#include "bsp/bsp_drivers.h"

#define CONFIG_ENCODER_MPS 1
#define CONFIG_ENCODER_MT  2
#define CONFIG_ENCODER_AMS 3

#define ENCODER_TIMER_DIR_DOWN 1
#define ENCODER_TIMER_DIR_UP 2

#define INVALD_PIO 0xFFFFFFFF

/* dq-axis current loop decouple */
#ifndef CONFIG_CURRENT_LOOP_DECOUPE
#define CONFIG_CURRENT_LOOP_DECOUPE 0
#endif

/* phase current sampling selection */
#ifndef CONFIG_PHASE_CURRENT_PHASE_AB
#define CONFIG_PHASE_CURRENT_PHASE_AB 0
#endif
#ifndef CONFIG_PHASE_CURRENT_PHASE_AC
#define CONFIG_PHASE_CURRENT_PHASE_AC 0
#endif
#ifndef CONFIG_PHASE_CURRENT_PHASE_BC
#define CONFIG_PHASE_CURRENT_PHASE_BC 0
#endif
#ifndef CONFIG_PHASE_CURRENT_PHASE_ABC
#define CONFIG_PHASE_CURRENT_PHASE_ABC 0
#endif

#ifndef CONFIG_ADC_OFFSET_TOLERANCE
#define CONFIG_ADC_OFFSET_TOLERANCE 50
#endif

#ifndef CONFIG_CURRENT_POLARITY
#define CONFIG_CURRENT_POLARITY 1.0f
#endif

#ifndef CONFIG_DC_CURR_POLARITY
#define CONFIG_DC_CURR_POLARITY 1.0f
#endif

#if (!CONFIG_DC_CURR_SAMPLE)
#undef CONFIG_DC_CURR_CALC
#define CONFIG_DC_CURR_CALC 1
#endif

#ifndef CONFIG_ENCODER_CC_INVERT
#define CONFIG_ENCODER_CC_INVERT 0
#endif

#ifndef CONFIG_ENCODER_ABI
#define CONFIG_ENCODER_ABI 0
#endif

#ifndef CONFIG_ENCODER_ABS
#define CONFIG_ENCODER_ABS 0
#endif

#ifndef CONFIG_UART_TX_BUFFER_SIZE
#define CONFIG_UART_TX_BUFFER_SIZE 4096
#endif
#ifndef CONFIG_UART_RX_BUFFER_SIZE
#define CONFIG_UART_RX_BUFFER_SIZE 512
#endif

#ifndef CONFIG_BOARD_COM_UART
#define CONFIG_BOARD_COM_UART 0
#endif

#ifndef CONFIG_BOARD_COM_CAN
#define CONFIG_BOARD_COM_CAN 0
#endif

#endif /* __BSP_H__ */
