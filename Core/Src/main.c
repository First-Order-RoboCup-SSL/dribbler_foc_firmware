/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "controller/motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* External start input (PA0) must be stable for this long before it is acted
 * on, to reject glitches/EMI on the single control line. */
#define EXT_START_DEBOUNCE_MS 5u
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* In normal operation the motor is initialised once at boot and start/stop is
 * driven by the external PA0 line (see main loop). These debug variables remain
 * for Ozone bring-up: set g_dbg_motor_mode (0=OPEN,1=VELOCITY,2=TORQUE,3=CURRENT)
 * and the matching target, e.g. g_dbg_id_target / g_dbg_iq_target in CURRENT. */
/* Control source: 0 = PA0 external line drives start/mode/setpoint (production
 * default); 1 = Ozone override, PA0 is ignored and the g_dbg_* variables below
 * are driven manually from the debugger. Flip this in Ozone to take control. */
volatile uint8_t g_dbg_override    = 0u;
/* Diagnostics readable in Ozone: g_reset_cause latches RCC->CSR at boot;
 * g_loop_count is a free-running heartbeat (no reset => it keeps climbing). */
volatile uint32_t g_reset_cause    = 0u;
volatile uint32_t g_loop_count     = 0u;
/* NOTE: g_dbg_init was removed — init now happens at boot, so the variable was
 * unused and the linker (--gc-sections) stripped it. An unused symbol resolves
 * to address 0x00000000, and writing it from Ozone pokes a null pointer, which
 * faults the debug access and drops the J-Link. Do not re-add it unused. */
volatile uint8_t g_dbg_motor_start = 0u;
volatile uint8_t g_dbg_daxis_lock  = 0u;
volatile uint8_t g_dbg_motor_mode  = 0u;   /* CTRL_MODE_OPEN */
volatile float   g_dbg_id_target   = 0.0f;
volatile float   g_dbg_iq_target   = 0.0f;
/* Velocity target in rad/s electrical. For dribbler (pp=1): 100 rad/s ≈ 955 RPM.
 * Used when g_dbg_motor_mode = 1 (CTRL_MODE_VELOCITY). */
volatile float   g_dbg_velocity_target = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* Latch the reset cause so Ozone can tell a power/brownout reset apart from a
   * pure debug-link drop. Read g_reset_cause after a "disconnect":
   *   bit 25 BORRSTF / bit 27 PORRSTF set -> brownout/power reset (supply sag)
   *   bit 28 SFTRSTF                 set -> software reset
   *   bit 29 IWDGRSTF / 30 WWDGRSTF  set -> watchdog
   * If the target did NOT reset at all (g_loop_count kept climbing), the CPU
   * stayed alive and only the J-Link link dropped -> EMI on the SWD lines. */
  g_reset_cause = RCC->CSR;
  __HAL_RCC_CLEAR_RESET_FLAGS();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_SPI3_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* Initialise the motor once here, with the gate driver still disabled and the
   * machine at rest. No torque is produced until motor_start() (gate stays off),
   * and this quiescent state is the ideal moment for phase-current offset
   * calibration done inside motor_init(). */
  motor_init(motor(0));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static uint8_t s_last_mode = 0xFFu;

  /* Debounced external start input (PA0). */
  static uint8_t  s_cmd_on   = 0u;   /* debounced start command */
  static uint8_t  s_raw_prev = 0u;   /* last raw level */
  static uint32_t s_raw_since = 0u;  /* tick when the raw level last changed */
  static uint8_t  s_seen_low = 0u;   /* a low must be seen before the first start */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    g_loop_count++;   /* heartbeat: if this keeps climbing after a "disconnect",
                       * the CPU never reset and only the SWD link dropped (EMI). */

    /* Production: external MCU start/enable on PA0 drives the g_dbg_* controls.
     * A stable high => start in CURRENT mode, low => stop. The level is debounced
     * to reject glitches/EMI, and a low must be observed at least once before the
     * first start, so a line that is already high at power-up cannot auto-spin the
     * motor. When g_dbg_override is set (Ozone), PA0 is ignored and the g_dbg_*
     * variables are left untouched for manual control from the debugger. */
    if (!g_dbg_override) {
        uint8_t  raw = (HAL_GPIO_ReadPin(EXT_START_GPIO_Port, EXT_START_Pin) == GPIO_PIN_SET) ? 1u : 0u;
        uint32_t now = HAL_GetTick();
        if (raw != s_raw_prev) {
            s_raw_prev  = raw;
            s_raw_since = now;
        } else if ((now - s_raw_since) >= EXT_START_DEBOUNCE_MS) {
            if (raw == 0u) { s_seen_low = 1u; }
            s_cmd_on = (raw && s_seen_low) ? 1u : 0u;
        }

        g_dbg_motor_mode  = CTRL_MODE_CURRENT;
        g_dbg_motor_start = s_cmd_on;
        g_dbg_iq_target   = s_cmd_on ? -1.6f : 0.0f;
    }

    if (g_dbg_motor_start == 0u && motor(0)->b_start) {
        motor_stop(motor(0));
    }
    if (g_dbg_motor_start == 1u && !motor(0)->b_start) {
        motor_start(motor(0), g_dbg_motor_mode);
    }

    /* live mode switching while motor is running */
    if (motor(0)->b_start && g_dbg_motor_mode != s_last_mode) {
        contrl_request_mode(&motor(0)->controller, g_dbg_motor_mode);
        s_last_mode = g_dbg_motor_mode;
    }

    /* current-mode setpoints (used by mc_sched at 1 kHz) */
    motor(0)->currdq_command[0] = g_dbg_id_target;
    motor(0)->currdq_command[1] = g_dbg_iq_target;

    /* velocity-mode setpoint (used by mc_sched when mode=CTRL_MODE_VELOCITY) */
    motor(0)->velocity_command = g_dbg_velocity_target;

    g_openloop_daxis_lock_dbg = (g_dbg_daxis_lock != 0u) ? 1u : 0u;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
