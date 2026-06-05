#ifndef _CONFIG_H__
#define _CONFIG_H__

#ifndef CONFIG_DQ_LOOP_RAMP_COUNT
#define CONFIG_DQ_LOOP_RAMP_COUNT 8
#endif

#ifndef CONFIG_SPEED_CTRL_FREQ
#define CONFIG_SPEED_CTRL_FREQ 1000
#endif

#ifndef CONFIG_SIM_STEPS_PER_PWM
#define CONFIG_SIM_STEPS_PER_PWM 200
#endif

#ifndef CONFIG_DISABLE_RAMP
#define CONFIG_DISABLE_RAMP 0
#endif
/* 1 = high first (PWM_MODE1), 0 = low first */
#ifndef CONFIG_PWM_HIGH_FIRST
#define CONFIG_PWM_HIGH_FIRST 1
#endif

#define CONFIG_MOTOR_DRIBBLER    1
#define CONFIG_MOTOR_GIMBAL_2804 0

#if     CONFIG_MOTOR_DRIBBLER
#define CONFIG_MOTOR_PARAMS_Poles        1
#define CONFIG_MOTOR_PARAMS_Ld           50e-6f
#define CONFIG_MOTOR_PARAMS_Lq           50e-6f
#define CONFIG_MOTOR_PARAMS_Rs           0.3f
#define CONFIG_MOTOR_PARAMS_Flux         0.000254f
#define CONFIG_MOTOR_PARAMS_MAX_RPM      60000
#define CONFIG_CURRENT_LOOP_BANDWIDTH_HZ 500.0f
#elif   CONFIG_MOTOR_GIMBAL_2804
#define CONFIG_MOTOR_PARAMS_Poles        7
#define CONFIG_MOTOR_PARAMS_Ld           0.86e-3f
#define CONFIG_MOTOR_PARAMS_Lq           0.86e-3f
#define CONFIG_MOTOR_PARAMS_Rs           2.55f
#define CONFIG_MOTOR_PARAMS_Flux         0.0035f
#define CONFIG_MOTOR_PARAMS_MAX_RPM       2600
#define CONFIG_CURRENT_LOOP_BANDWIDTH_HZ 200.0f
#endif

#ifndef CONFIG_MOTOR_PARAMS_J
#define CONFIG_MOTOR_PARAMS_J 0.0020f
#endif
#ifndef CONFIG_MOTOR_ENCODER_OFFSET
#define CONFIG_MOTOR_ENCODER_OFFSET 0.0f
#endif

#define CONFIG_ROBOT_INDEX_A 0
#define CONFIG_ROBOT_INDEX_B 0
#define CONFIG_ROBOT_INDEX_C 1
#define CONFIG_ROBOT_INDEX_D 0

#if     CONFIG_ROBOT_INDEX_A
#define CONFIG_MAG_ENCODER_OFFSET_RAD 1.59f 
#define CONFIG_THETA_INVERT           0
#define CONFIG_DRIBBLER_IQ            -1.7f
#elif   CONFIG_ROBOT_INDEX_B
#define CONFIG_MAG_ENCODER_OFFSET_RAD 1.3f
#define CONFIG_THETA_INVERT           1 
#define CONFIG_DRIBBLER_IQ            -1.8f
#elif   CONFIG_ROBOT_INDEX_C
#define CONFIG_MAG_ENCODER_OFFSET_RAD -1.3f 
#define CONFIG_THETA_INVERT           1 
#define CONFIG_DRIBBLER_IQ            -1.9f
#elif   CONFIG_ROBOT_INDEX_D
#define CONFIG_MAG_ENCODER_OFFSET_RAD 0.0f /* not calibrated */
#define CONFIG_THETA_INVERT           1 // DANGER, not calibrated
#define CONFIG_DRIBBLER_IQ            -1.6f
#else
#error "no robot index"
#endif


/* Magnetic encoder device selection */
#define CONFIG_MAG_ENCODER_AS5600       1  
#define CONFIG_MAG_ENCODER_AS5048A_PWM  2  

#ifndef CONFIG_MAG_ENCODER_TYPE
#  if defined(STM32G431xx)
#    define CONFIG_MAG_ENCODER_TYPE CONFIG_MAG_ENCODER_AS5600
#  elif defined(STM32F405xx)
#    define CONFIG_MAG_ENCODER_TYPE CONFIG_MAG_ENCODER_AS5048A_PWM
#  else
#    define CONFIG_MAG_ENCODER_TYPE CONFIG_MAG_ENCODER_AS5600
#  endif
#endif

#ifndef CONFIG_OPENLOOP_VF_OFFSET
#define CONFIG_OPENLOOP_VF_OFFSET 0.02f   /* hall calibration: ~67mA on dribbler */
#endif
#ifndef CONFIG_OPENLOOP_VQ_LIMIT
#define CONFIG_OPENLOOP_VQ_LIMIT 6.0f
#endif
#ifndef CONFIG_CURRENT_POLARITY
#define CONFIG_CURRENT_POLARITY 1.0f /* 1 or -1 */
#endif
#ifndef CONFIG_VELOCITY_PID_KP
#define CONFIG_VELOCITY_PID_KP 0.01f
#endif
#ifndef CONFIG_VELOCITY_PID_KI
#define CONFIG_VELOCITY_PID_KI 0.001f
#endif

#ifndef CONFIG_HALL_INTERP_ERPM
#define CONFIG_HALL_INTERP_ERPM 500.0f
#endif

#ifndef CONFIG_HALL_TAB_0
#define CONFIG_HALL_TAB_0 255
#endif
#ifndef CONFIG_HALL_TAB_1
#define CONFIG_HALL_TAB_1 0
#endif
#ifndef CONFIG_HALL_TAB_2      
#define CONFIG_HALL_TAB_2 133
#endif
#ifndef CONFIG_HALL_TAB_3      
#define CONFIG_HALL_TAB_3 166
#endif
#ifndef CONFIG_HALL_TAB_4      
#define CONFIG_HALL_TAB_4 66
#endif
#ifndef CONFIG_HALL_TAB_5      
#define CONFIG_HALL_TAB_5 33
#endif
#ifndef CONFIG_HALL_TAB_6     
#define CONFIG_HALL_TAB_6 100
#endif
#ifndef CONFIG_HALL_TAB_7      
#define CONFIG_HALL_TAB_7 255
#endif

#endif /* _CONFIG_H__ */
