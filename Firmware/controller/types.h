#ifndef _OS_Type_H__
#define _OS_Type_H__
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifndef AT32FXXX
#ifndef APM32F035
#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif
#endif
#endif
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u24;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s24;
typedef int32_t s32;
typedef int64_t s64;

typedef signed char	int8;
typedef signed short int16;
typedef signed int int32;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;

#define MOT_DIR_NEGATIVE ((int8_t)-1)
#define MOT_DIR_POSITIVE ((int8_t)1)

#define ADC_PHASE_AB 1
#define ADC_PHASE_BC 2
#define ADC_PHASE_AC 3

#define MAX_S16 (0x7FFF)
#define MAX_U16 (0xFFFF)
#define MAX_U8  (0xFF)

#define NO_VALID_CURRENT 800000.0f

typedef u8 (*read_io_value)(void);
typedef void (*hw_call_back)(void *, unsigned char *data, int len);

#endif /* _OS_Type_H__ */

