/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : FSAE DV Testbed — tricycle robot CAN command/feedback
  *
  * CAN interface (fsae_dv_testbed.dbc):
  *   RX  0x200  DV_Command   — TargetSteeringAngle (±20 °, 0.1°/count) +
  *                              TargetMotorSpeed (0–1000 PWM duty)
  *   TX  0x201  DV_Feedback  — EncoderCount (32-bit, 50 Hz)
  *
  * Hardware:
  *   TIM2 CH1 (PA0)       — steering servo PWM (50 Hz)
  *   TIM3 CH1 (PC6)       — drive motor PWM via L298 (2 kHz)
  *   TIM4 CH1/2 (PD12/13) — quadrature encoder (encoder mode)
  *   CAN1 (PD0/PD1)       — bxCAN, 500 kbps
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* USER CODE BEGIN PD */

/* ---- Configurable limits — tune these without touching any other code ---- */
#define CMD_STEER_MIN_DEG       (-20.0f)   /* hardware travel limit, degrees  */
#define CMD_STEER_MAX_DEG       ( 20.0f)
#define CMD_STEER_NEUTRAL_DEG   (  0.0f)   /* boot / watchdog default         */

#define CMD_SPEED_MIN           0u         /* raw PWM duty, maps to Drive_Motor_Control() */
#define CMD_SPEED_MAX           1000u

#define WATCHDOG_TIMEOUT_MS     100u       /* ms: safe-state if RX lapses     */
#define FEEDBACK_PERIOD_MS      20u        /* ms: encoder broadcast = 50 Hz   */

/* ---- CAN message IDs (must match fsae_dv_testbed.dbc) ---- */
#define CAN_ID_DV_COMMAND       0x200u
#define CAN_ID_DV_FEEDBACK      0x201u

/* ---- Steering servo CCR calibration (TIM2, period = 10000) ---- */
#define SERVO_CCR_AT_NEG20      690u
#define SERVO_CCR_AT_POS20      840u

/* ---- Steering slew rate limiter -------------------------------------------
 * Max rate the servo is allowed to move in software.  The ISR only updates
 * g_steer_target; the main loop moves the actual CCR toward it at this rate.
 * Increase to make steering snappier; decrease for smoother, slower motion.
 * At 0.15 f: full ±20° sweep takes ~267 ms.  At 0.40 f it takes ~100 ms. */
#define STEER_SLEW_RATE_DEG_PER_MS  0.15f

/* USER CODE END PD */

/* USER CODE BEGIN PV */

static CAN_TxHeaderTypeDef sTxHeader;
static CAN_RxHeaderTypeDef sRxHeader;
static uint8_t  sTxData[8];
static uint8_t  sRxData[8];
static uint32_t sTxMailbox;

/* Watchdog state — written in CAN ISR, read in main loop */
static volatile uint32_t g_last_cmd_ms  = 0;
static volatile uint8_t  g_cmd_received = 0;

/* Steering rate limiter — ISR writes target; main loop slews actual toward it */
static volatile float g_steer_target = CMD_STEER_NEUTRAL_DEG;

/* Encoder: monotonic 32-bit count built from 16-bit TIM4 counter.
 * Written in TIM4 IC ISR, read (snapshot) in main loop.
 * A 32-bit aligned read on Cortex-M is atomic; no critical section needed. */
static volatile uint32_t g_encoder_count = 0;
static volatile uint32_t g_enc_overflow  = 0;
static volatile uint32_t g_enc_prev      = 0;

/* USER CODE END PV */

/* Private function prototypes */
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static void CAN_Config(void);
static void Steering_Set(float deg);
static void Motor_Set(uint16_t speed);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* TIM4 encoder input-capture ISR — called on every encoder edge.
 * Detects 16-bit wrap-around by checking if the counter went backward,
 * then accumulates a monotonic 32-bit position. */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance != TIM4) return;
    uint32_t cur = __HAL_TIM_GET_COUNTER(htim);
    if (cur < g_enc_prev) {
        g_enc_overflow += 65536u;
    }
    g_enc_prev = cur;
    g_encoder_count = g_enc_overflow + cur;
}

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Initialize peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();   /* steering servo PWM  */
  MX_TIM3_Init();   /* drive motor PWM     */
  MX_TIM4_Init();   /* quadrature encoder  */

  /* USER CODE BEGIN 2 */

  CAN_Config();

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
      Error_Handler();

  /* Start PWM outputs before touching CCR registers */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /* Start encoder at boot so feedback is valid from the first broadcast */
  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);

  /* Apply safe state at boot */
  Steering_Set(CMD_STEER_NEUTRAL_DEG);
  Motor_Set(CMD_SPEED_MIN);

  /* Pre-fill the TX header — fields never change */
  sTxHeader.StdId              = CAN_ID_DV_FEEDBACK;
  sTxHeader.DLC                = 8;
  sTxHeader.IDE                = CAN_ID_STD;
  sTxHeader.RTR                = CAN_RTR_DATA;
  sTxHeader.TransmitGlobalTime = DISABLE;

  uint32_t last_feedback_ms = 0;
  uint32_t last_steer_ms   = 0;
  float    steer_actual    = CMD_STEER_NEUTRAL_DEG;  /* tracks the CCR, main-loop only */

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint32_t now = HAL_GetTick();

      /* ---- Safety watchdog ------------------------------------------------
       * Policy: if a command has been received at least once and the last
       * valid command is older than WATCHDOG_TIMEOUT_MS, zero motor speed
       * while holding the last steering angle.
       *
       * Rationale: zeroing speed stops the robot immediately if the laptop
       * disconnects or ROS crashes.  Holding steering (rather than snapping
       * to neutral) avoids an abrupt jerk if the robot is cornering when
       * comms drop; it is a deliberate design choice — see DBC signal docs.
       * The watchdog re-arms automatically on the next valid 0x200 frame. */
      if (g_cmd_received && (now - g_last_cmd_ms) > WATCHDOG_TIMEOUT_MS) {
          Motor_Set(CMD_SPEED_MIN);
          /* steering target is left unchanged — slew loop continues holding it */
      }

      /* ---- Steering slew rate limiter ------------------------------------
       * Move steer_actual toward g_steer_target by at most
       * STEER_SLEW_RATE_DEG_PER_MS per millisecond elapsed.
       * The ISR only updates g_steer_target; all CCR writes happen here. */
      {
          uint32_t dt_ms  = now - last_steer_ms;
          last_steer_ms   = now;
          float target    = g_steer_target;   /* single volatile read */
          float delta     = target - steer_actual;
          float max_step  = STEER_SLEW_RATE_DEG_PER_MS * (float)dt_ms;
          if      (delta >  max_step) delta =  max_step;
          else if (delta < -max_step) delta = -max_step;
          steer_actual   += delta;
          Steering_Set(steer_actual);
      }

      /* ---- Encoder feedback at 50 Hz -------------------------------------*/
      if ((now - last_feedback_ms) >= FEEDBACK_PERIOD_MS) {
          last_feedback_ms = now;

          uint32_t count = g_encoder_count;   /* atomic 32-bit snapshot */
          sTxData[0] = (uint8_t)(count);
          sTxData[1] = (uint8_t)(count >> 8);
          sTxData[2] = (uint8_t)(count >> 16);
          sTxData[3] = (uint8_t)(count >> 24);
          sTxData[4] = 0; sTxData[5] = 0; sTxData[6] = 0; sTxData[7] = 0;

          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

          HAL_CAN_AddTxMessage(&hcan1, &sTxHeader, sTxData, &sTxMailbox);
      }

    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/* ---- CAN RX FIFO0 interrupt callback ------------------------------------
 * Kept short: read frame → decode → clamp → apply → refresh watchdog.
 * No blocking calls, no printf, no HAL_Delay. */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &sRxHeader, sRxData) != HAL_OK)
        return;

    if (sRxHeader.StdId != CAN_ID_DV_COMMAND) return;

    /* TargetSteeringAngle — bytes 0–1, signed int16, little-endian
     * Scaling: physical_deg = raw * 0.1
     * Examples: raw -200 = -20.0°  raw 0 = 0°  raw +200 = +20.0° */
    int16_t raw_steer = (int16_t)((uint16_t)sRxData[0] | ((uint16_t)sRxData[1] << 8));
    float steer_deg = raw_steer * 0.1f;

    /* TargetMotorSpeed — bytes 2–3, unsigned int16, little-endian
     * Scaling: 1 count = 1 PWM duty unit (0–1000, maps to Drive_Motor_Control) */
    uint16_t speed = (uint16_t)sRxData[2] | ((uint16_t)sRxData[3] << 8);

    /* Store steering target — main loop slews actual CCR toward it.
     * Motor is applied immediately (no ramp needed). */
    if (steer_deg < CMD_STEER_MIN_DEG) steer_deg = CMD_STEER_MIN_DEG;
    if (steer_deg > CMD_STEER_MAX_DEG) steer_deg = CMD_STEER_MAX_DEG;
    g_steer_target = steer_deg;
    Motor_Set(speed);

    /* Refresh watchdog */
    g_last_cmd_ms  = HAL_GetTick();
    g_cmd_received = 1;
}

/* Set steering servo to the given angle in degrees.
 * Clamped to [CMD_STEER_MIN_DEG, CMD_STEER_MAX_DEG].
 * CCR is linearly interpolated from the calibration points at the top. */
static void Steering_Set(float deg) {
    if (deg < CMD_STEER_MIN_DEG) deg = CMD_STEER_MIN_DEG;
    if (deg > CMD_STEER_MAX_DEG) deg = CMD_STEER_MAX_DEG;
    float ccr = SERVO_CCR_AT_NEG20
        + (deg - CMD_STEER_MIN_DEG)
          / (CMD_STEER_MAX_DEG - CMD_STEER_MIN_DEG)
          * (float)(SERVO_CCR_AT_POS20 - SERVO_CCR_AT_NEG20);
    htim2.Instance->CCR1 = (uint16_t)ccr;
}

/* Set drive motor PWM duty, clamped to [CMD_SPEED_MIN, CMD_SPEED_MAX]. */
static void Motor_Set(uint16_t speed) {
    if (speed > CMD_SPEED_MAX) speed = CMD_SPEED_MAX;
    htim3.Instance->CCR1 = speed;
}

/* CAN filter + start.  Pass-all mask; ID filtering done in the callback. */
static void CAN_Config(void) {
    CAN_FilterTypeDef f = {0};
    f.FilterBank           = 0;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;   /* mask 0 = accept all IDs */
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) Error_Handler();
    if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();
}

/* USER CODE END 4 */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
