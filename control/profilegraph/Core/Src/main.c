/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ADXL.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	STATE_IDLE = 0,
	STATE_STARTUP,
	STATE_RUNNING,
	STATE_STOPPING,
	STATE_ERROR,
} State;

State currentState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DT (0.01f)
#define KP (2.0f)
#define KI (0.0f)
#define DIS_OFF_MAX_LEFT (-67)
#define DIS_OFF_MAX_RIGHT (67)
#define STEERING_ANGLE_CENTER (0.0f)
#define STEERING_ANGLE_MAX_LEFT (-20.0f)
#define STEERING_ANGLE_MAX_RIGHT (20.0f)
#define SERVO_CCR_AT_CENTER 765
#define SERVO_CCR_AT_NEG20  690
#define SERVO_CCR_AT_POS20  840

#define DRIVE_MOTOR_MAX_SPEED 1000
#define DRIVE_MOTOR_MIN_SPEED 0

#define ENCODER_PULSES_PER_WHEEL_TURN_26RPM 2387.0
#define DRIVE_WHEEL_CIRCUMFERENCE_METER 0.2042

#define DIS_OFF_DEFAULT (-100)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

ADXL_InitTypeDef ADXL;
adxlStatus adxlSts;
int32_t accelData[3];
float accelData_g[3];

int8_t dis_off;
uint8_t S_surveyDistanceSet = 0;
uint8_t S_heightThreashold = 255;
uint16_t S_wheelBase = 0;
uint8_t S_startSurvey = 0;
float integral_global;
float pidOutput_global;
float steerAngle_global;
float length = 6.0f;
float theta = 0.0f;
//float theta_deg = 0.0f;
float height_diff = 0.0f;
int16_t height_diff_send = 0;

uint32_t current_encoder_value  = 0;
uint32_t encoder_position = 0;
uint32_t total_count = 0;
uint32_t extended_counter = 0;
uint32_t prev_encoder_value = 0;

float C_drivenDistance = 0;
uint8_t start_delay = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void CAN_Config(void);
void Steering_Servo_Position(int8_t steeringAngle);
void Steering_Servo_Control(int8_t offsetVal);
void Drive_Motor_Control(uint16_t speed);
void Drive_Motor_Start(float C_drivenDistance);
void C_transverseHeight(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

//	static uint32_t encoder_position = 0;
//	static uint32_t overflow_counter = 0;
//	static uint32_t encoder_temp = 0;

    // Read the current value from the timer counter
	current_encoder_value  = __HAL_TIM_GET_COUNTER(htim);

    // Detect overflow: if the current value is less than the previous value,
    // it means the counter has wrapped around from 65535 back to 0.
	if (current_encoder_value < prev_encoder_value) {
	        extended_counter += 65536; // add one full count cycle (0x10000)
	}

    prev_encoder_value = current_encoder_value;

//	if (current_encoder_value  >= 65531) {
//		extended_counter += 65536;
//		overflow_counter = extended_counter - 1;
//	} else {
		total_count = extended_counter + current_encoder_value;
//	}
	encoder_position = total_count/4;
	C_drivenDistance = (float) (encoder_position/ENCODER_PULSES_PER_WHEEL_TURN_26RPM) * DRIVE_WHEEL_CIRCUMFERENCE_METER;
}
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_MspInit(&hcan1);
  CAN_Config();

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
	  Error_Handler();
  }

  ADXL.StandbyMode = ADXL_MODE_MEASUREMENT;
  ADXL.TempMode = ADXL_TEMP_OFF;
  ADXL.DataReadyMode = ADXL_DRDY_ON;
  ADXL.IntMode = ADXL_INT_ACTIVELOW;
  ADXL.Range = ADXL_RANGE_2G;

  TxHeader.StdId = 0x101;
  TxHeader.DLC = 5;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.TransmitGlobalTime = DISABLE;

  TxData[0] = 0x00;
  TxData[1] = 0x00;
//  TxData[2] = 0x00;
//  TxData[3] = 0x00;
//  TxData[4] = 0x00;
//  TxData[5] = 0x00;


  ADXL_Init(&ADXL);
  ADXL_Measure(ON);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);

  htim2.Instance->CCR1 = SERVO_CCR_AT_CENTER;

  State nextState = STATE_IDLE;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch (nextState){
	  	  case STATE_IDLE:
	  		  currentState = STATE_IDLE;
	  		  dis_off = DIS_OFF_DEFAULT;
	  		  Drive_Motor_Control(DRIVE_MOTOR_MIN_SPEED);
//	  		  Drive_Motor_Control(DRIVE_MOTOR_MAX_SPEED);
	  		  Steering_Servo_Position(STEERING_ANGLE_CENTER);
	  		  current_encoder_value = __HAL_TIM_SET_COUNTER(&htim4, 0);
	  		  if (S_startSurvey){
	  			  nextState = STATE_STARTUP;
	  		  }
	  		  break;

	  	  case STATE_STARTUP:
	  		  currentState = STATE_STARTUP;
	  		  if (dis_off != (DIS_OFF_DEFAULT)){
	  			  nextState = STATE_RUNNING;
	  		  }
	  		  break;

	  	  case STATE_RUNNING:
	  		  currentState = STATE_RUNNING;
	  		  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
	  		  C_transverseHeight();
	  		  Steering_Servo_Control(dis_off);
			  Drive_Motor_Start(S_surveyDistanceSet);
			  //Add can message broadcast
			  if (C_drivenDistance >= S_surveyDistanceSet){
				  nextState = STATE_STOPPING;
			  }
	  		  break;

	  	  case STATE_STOPPING:
	  		  currentState = STATE_STOPPING;
	  		  Drive_Motor_Control(DRIVE_MOTOR_MIN_SPEED);
	  		  HAL_TIM_Encoder_Stop_IT(&htim4, TIM_CHANNEL_ALL);
	  		  C_drivenDistance = 0;
	  		  current_encoder_value  = 0;
	  		  encoder_position = 0;
	  		  total_count = 0;
	  		  extended_counter = 0;
	  		  prev_encoder_value = 0;
	  		  if (!S_startSurvey){
	  			  nextState = STATE_IDLE;
	  		  }
	  		  break;

	  	  case STATE_ERROR:
	  		  currentState = STATE_ERROR;
	  		  break;

	  	  default:
	  		  nextState = STATE_ERROR;
	  		  break;
	  }

//	  if (S_startSurvey && (dis_off != (-100))){
//		  if (!start_delay) {
//			  for (int speed = 0; speed < 1000; speed++){
//				HAL_Delay(1);
//			}
//			start_delay = 1;
//		  }
//		  Drive_Motor_Start(S_surveyDistanceSet);
//	  }
//	  Steering_Servo_Control(dis_off);
//	  C_transverseHeight();
//	  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
//	  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//	  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
//	  knobRotation_P = Knob_Rotation_Percent()*100;

	  //180 deg -> CCR = 125, 0 deg -> CCR = 25
//	  htim2.Instance->CCR1 = SERVO_CCR_AT_NEG20;
//	  HAL_Delay(500);
//	  htim2.Instance->CCR1 = SERVO_CCR_AT_CENTER;
//	  HAL_Delay(500);
//	  htim2.Instance->CCR1 = SERVO_CCR_AT_POS20;
//	  HAL_Delay(500);
//	  htim2.Instance->CCR1 = SERVO_CCR_AT_CENTER;
//	  HAL_Delay(500);
//	  htim2.Instance->CCR1 = 1000;
//	  HAL_Delay(2000);
//	  htim2.Instance->CCR1 = 1250;
//	  if (dirction == 0){
//	 		  for (int i = DIS_OFF_MAX_LEFT; i < DIS_OFF_MAX_RIGHT; i+=1){
//	 			 Steering_Servo_Control(i);
//	 			 HAL_Delay(10);
//	 		  }
//	 		  dirction = 1;
//	 	  } else {
//	 		  for (int i = DIS_OFF_MAX_RIGHT; i > DIS_OFF_MAX_LEFT; i-=1){
//	 			 Steering_Servo_Control(i);
//	 			 HAL_Delay(10);
//	 		  }
//	 		  dirction = 0;
//	 	  }

//	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

//	  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	  HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN */
void C_transverseHeight(void) {
    float accelData_g[3];
    ADXL_getAccelFloat(accelData_g);
    float accel_x = accelData_g[0];
    float accel_z = accelData_g[2];

    theta = atanf(accel_x / accel_z);
    //theta_deg =  theta * (180.0f / 3.14)
    height_diff = length * sinf(theta);
    height_diff_send = height_diff * 10;

    TxData[2] = (height_diff_send) & 0xFF;
    TxData[3] = ((height_diff_send) >> 8) & 0xFF;
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}
/* USER CODE END  */

/* USER CODE BEGIN 4 */
static void CAN_Config(void)
{
	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterBank = 13;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000; //Only ID 0x284 and 0x285 can pass through
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 0;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}

	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

	//Get Rx message
	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		Error_Handler();
	}

	if (RxHeader.StdId == 0x123) {
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		dis_off = RxData[0]*(-1);
	} else if (RxHeader.StdId == 0x102) {
		S_surveyDistanceSet = RxData[0];
		S_heightThreashold = RxData[1];
		S_wheelBase = RxData[2] | (RxData[3] << 8);
		S_startSurvey = RxData[4] & 0x01;
	}

}

void Steering_Servo_Position(int8_t steeringAngle){
	if (steeringAngle < STEERING_ANGLE_MAX_LEFT) {
		steeringAngle = STEERING_ANGLE_MAX_LEFT;   // clamp to -20°
	}
	else if (steeringAngle > STEERING_ANGLE_MAX_RIGHT) {
		steeringAngle = STEERING_ANGLE_MAX_RIGHT;  // clamp to +20°
	}
	//Linear interpolation from steering angle to ccr value
	float ccrValue = SERVO_CCR_AT_NEG20
		+ ( (steeringAngle - STEERING_ANGLE_MAX_LEFT)
			/ (STEERING_ANGLE_MAX_RIGHT - STEERING_ANGLE_MAX_LEFT) )
		  * (SERVO_CCR_AT_POS20 - SERVO_CCR_AT_NEG20);

	//Write to the timer’s CCR register (cast to uint16_t)
	htim2.Instance->CCR1 = (uint16_t) ccrValue;
}

void Steering_Servo_Control(int8_t offsetVal){
	//Clamp dis_off to valid range
	static float integral = 0.0f;     // integral term (accumulated error)

	if (offsetVal != -100){
		if (offsetVal < DIS_OFF_MAX_LEFT) {
			offsetVal = DIS_OFF_MAX_LEFT;
		} else if (offsetVal >= DIS_OFF_MAX_RIGHT) {
			offsetVal = DIS_OFF_MAX_RIGHT;
		}

		float error = offsetVal;   // setpoint is zero offset
		integral += error * DT;           // integrate

		// PID output = KP*error + KI*integral + KD*derivative
		float pidOutput = (KP * error) + (KI * integral);

		//Linear interpolation from dis_off to steering angle
		float steerAngle = STEERING_ANGLE_MAX_LEFT
			+ ( (float)(pidOutput - DIS_OFF_MAX_LEFT)
				/ (float)(DIS_OFF_MAX_RIGHT - DIS_OFF_MAX_LEFT) )
			  * ( STEERING_ANGLE_MAX_RIGHT - STEERING_ANGLE_MAX_LEFT );

		Steering_Servo_Position(steerAngle);
		integral_global = integral;
		pidOutput_global = pidOutput;
		steerAngle_global = steerAngle;
	}
}

void Drive_Motor_Control(uint16_t speed){
	//Clamp input speed
	if (speed >= DRIVE_MOTOR_MIN_SPEED && speed <= DRIVE_MOTOR_MAX_SPEED){
		  htim3.Instance->CCR1 = speed;
	} else {
		  htim3.Instance->CCR1 = DRIVE_MOTOR_MIN_SPEED;
	}
}

void Drive_Motor_Start(float drivenDistance){
	static uint8_t fullSpeed = 0;
	//Speed ramp up
	if (!fullSpeed) {
		for (int speed = 100; speed < DRIVE_MOTOR_MAX_SPEED; speed += 2){
			Drive_Motor_Control(speed);
			HAL_Delay(1);
		}
		fullSpeed = 1;
	} else if (C_drivenDistance >= drivenDistance){
		Drive_Motor_Control(DRIVE_MOTOR_MIN_SPEED);
	} else if (C_drivenDistance < drivenDistance){
		Drive_Motor_Control(DRIVE_MOTOR_MAX_SPEED);
	}
}

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

#ifdef  USE_FULL_ASSERT
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
