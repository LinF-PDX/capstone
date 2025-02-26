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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DIS_OFF_MAX_LEFT (-67)
#define DIS_OFF_MAX_RIGHT (67)
#define STEERING_ANGLE_MAX_LEFT (-20.0f)
#define STEERING_ANGLE_MAX_RIGHT (20.0f)
#define SERVO_CCR_AT_CENTER 752
#define SERVO_CCR_AT_NEG20  678
#define SERVO_CCR_AT_POS20  830

#define DRIVE_MOTOR_MAX_SPEED 1000
#define DRIVE_MOTOR_MIN_SPEED 0
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
uint8_t dirction = 0;
uint8_t knobRotation_P = 0;
uint8_t deviceAddr = 0;
int8_t dis_off = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void CAN_Config(void);
void Steering_Servo_Control(int8_t offsetVal);
void Drive_Motor_Control(uint16_t speed);
void Drive_Motor_Start(float C_drivenDistance);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float Knob_Rotation_Percent(void) {
	uint16_t ADC_VAL;

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	ADC_VAL = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return (float)ADC_VAL/4095; //returns ADC percentage ranges from 0-1
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

  CAN_Config();

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
	  Error_Handler();
  }

  ADXL.StandbyMode = ADXL_MODE_MEASUREMENT;
  ADXL.TempMode = ADXL_TEMP_OFF;
  ADXL.DataReadyMode = ADXL_DRDY_ON;
  ADXL.IntMode = ADXL_INT_ACTIVELOW;
  ADXL.Range = ADXL_RANGE_2G;

  TxHeader.StdId = 0x123;
  TxHeader.DLC = 8;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.TransmitGlobalTime = DISABLE;

  TxData[0] = 0x01;
  TxData[1] = 0x02;
  TxData[2] = 0x03;
  TxData[3] = 0x04;
  TxData[4] = 0x05;
  TxData[5] = 0x06;
  TxData[6] = 0x07;
  TxData[7] = 0x08;


  ADXL_Init(&ADXL);
//  ADXL_Measure(ON);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  htim2.Instance->CCR1 = SERVO_CCR_AT_CENTER;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Drive_Motor_Start(1);
//	  Steering_Servo_Control(dis_off);
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
//	 		  for (int i = 250; i < 1250; i+=10){
//	 			  htim2.Instance->CCR1 = i;
//	 			  HAL_Delay(5);
//	 		  }
//	 		  dirction = 1;
//	 	  } else {
//	 		  for (int i = 1250; i > 250; i-=10){
//	 			  htim2.Instance->CCR1 = i;
//	 			  HAL_Delay(5);
//	 		  }
//	 		  dirction = 0;
//	 	  }

//	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//	  ADXL_getAccelRaw(accelData);
//	  ADXL_getAccelFloat(accelData_g);

//	  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//	  HAL_Delay(500);
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
		dis_off = RxData[0];
	}

}

void Steering_Servo_Control(int8_t offsetVal){
	//Clamp dis_off to valid range
	if (offsetVal != 100){
		if (offsetVal < DIS_OFF_MAX_LEFT) {
			offsetVal = DIS_OFF_MAX_LEFT;
		} else if (offsetVal >= DIS_OFF_MAX_RIGHT) {
			offsetVal = DIS_OFF_MAX_RIGHT;
		}

		//Linear interpolation from dis_off to steering angle
		float steerAngle = STEERING_ANGLE_MAX_LEFT
			+ ( (float)(offsetVal - DIS_OFF_MAX_LEFT)
				/ (float)(DIS_OFF_MAX_RIGHT - DIS_OFF_MAX_LEFT) )
			  * ( STEERING_ANGLE_MAX_RIGHT - STEERING_ANGLE_MAX_LEFT );

		//Linear interpolation from steering angle to ccr value
		float ccrValue = SERVO_CCR_AT_NEG20
			+ ( (steerAngle - STEERING_ANGLE_MAX_LEFT)
				/ (STEERING_ANGLE_MAX_RIGHT - STEERING_ANGLE_MAX_LEFT) )
			  * (SERVO_CCR_AT_POS20 - SERVO_CCR_AT_NEG20);

		//Write to the timer’s CCR register (cast to uint16_t)
		htim2.Instance->CCR1 = (uint16_t) ccrValue;
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

void Drive_Motor_Start(float C_drivenDistance){
	static uint8_t fullSpeed = 0;
	if (!fullSpeed) {
		for (int speed = 100; speed < 1000; speed += 2) {
			Drive_Motor_Control(speed);
			HAL_Delay(1);
		}
		fullSpeed = 1;
	} else {
		Drive_Motor_Control(1000);
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
