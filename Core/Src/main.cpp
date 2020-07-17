/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU9250.h"
#include "mpu_porting_api.h"

#include "bmp280.h"
#include "bmp_porting_api.h"

//#define UART_DEBUGGING
#ifdef UART_DEBUGGING
#include <stdio.h>
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct control_variables {
	float p_c, i_c, d_c; 	// PID constants
	float output, integral, derivative, bias;
	float error;
	float error_prior, integral_prior;
	float current_state;
	uint8_t clamp;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ALTP						(1)
#define ALTI						(0)
#define ALTD						(0)

#define YAWP						(1)
#define YAWI						(0)
#define YAWD						(0)

#define PITCHP					(1)
#define PITCHI					(0)
#define PITCHD					(0)

#define ROLLP						(1)
#define ROLLI						(0)
#define ROLLD						(0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void UpdateMotorSpeeds(struct control_variables *altitude, struct control_variables *yaw, struct control_variables *pitch, struct control_variables *roll);
void CalculatePIDControlOutput(struct control_variables *ctrl, float commanded_state, int iteration_time);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char log_buffer[128];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	MPU9250 mpu9250;
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
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

#ifdef UART_DEBUGGING
  sprintf(log_buffer, "Power on\r\n");
  ConsoleLog(log_buffer);
#endif
  HAL_Delay(1000);
  uint8_t whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250

  if (whoami == 0x71)
  {
  	mpu9250.resetMPU9250();
		mpu9250.calibrateMPU9250(gyroBias, accelBias);
		wait(2);
		mpu9250.initMPU9250();
		mpu9250.initAK8963(magCalibration);
		wait(2);
  }
  else
  {
  	while(1);
  }

  mpu9250.getAres(); // Get accelerometer sensitivity
	mpu9250.getGres(); // Get gyro sensitivity
	mpu9250.getMres(); // Get magnetometer sensitivity
	magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
	magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
	magbias[2] = +125.;  // User environmental x-axis correction in milliGauss


	struct bmp280_dev bmp;
	struct bmp280_config conf;
	struct bmp280_uncomp_data ucomp_data;
	float altitude, altitudeOffset;
	uint32_t pres32;
	int32_t temp;

	/* Map the delay function pointer with the function responsible for implementing the delay */
	bmp.delay_ms = BMP_delay_ms;

	/* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77)) */
	bmp.dev_id = BMP280_I2C_ADDR_PRIM << 1;

	/* Select the interface mode as I2C */
	bmp.intf = BMP280_I2C_INTF;

	/* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
	bmp.read = BMP_i2c_reg_read;
	bmp.write = BMP_i2c_reg_write;

	bmp280_init(&bmp);

	/* Always read the current settings before writing, especially when
	 * all the configuration is not modified
	 */
	bmp280_get_config(&conf, &bmp);

	conf.filter = BMP280_FILTER_COEFF_16;
	conf.os_pres = BMP280_OS_16X;
	conf.os_temp = BMP280_OS_4X;
	conf.odr = BMP280_ODR_0_5_MS;

	bmp280_set_config(&conf, &bmp);
	bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);


	altitudeOffset = 0.0;

	HAL_Delay(1000);

	// Determine the altitude offset
	for (int i = 0; i < 100; i ++)
	{
		bmp280_get_uncomp_data(&ucomp_data, &bmp);
		bmp280_get_comp_temp_32bit(&temp, ucomp_data.uncomp_temp, &bmp);
		bmp280_get_comp_pres_64bit(&pres32, ucomp_data.uncomp_press, &bmp);

		altitudeOffset += 44330 * (1.0 - pow((float)(pres32 >> 8) / 1006.1, 0.1903)) * 3.28084;
	}

	altitudeOffset /= 100;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	// PID pseudocode found in http://robotsforroboticists.com/pid-control/
	struct control_variables altitude_ctrl;
	struct control_variables yaw_ctrl;
	struct control_variables pitch_ctrl;
	struct control_variables roll_ctrl;

	// Altitude control init
	altitude_ctrl.p_c = ALTP;
	altitude_ctrl.i_c = ALTI;
	altitude_ctrl.d_c = ALTD;
	altitude_ctrl.error_prior = 0;
	altitude_ctrl.error = 0;
	altitude_ctrl.bias = 0;

	// Yaw control init
	yaw_ctrl.p_c = YAWP;
	yaw_ctrl.i_c = YAWI;
	yaw_ctrl.d_c = YAWD;
	yaw_ctrl.error_prior = 0;
	yaw_ctrl.error = 0;
	yaw_ctrl.bias = 0;

	// Pitch and roll control init
	pitch_ctrl.p_c = roll_ctrl.p_c = PITCHP;
	pitch_ctrl.i_c = roll_ctrl.i_c = PITCHI;
	pitch_ctrl.d_c = roll_ctrl.d_c = PITCHD;
	pitch_ctrl.error_prior = roll_ctrl.error_prior = 0;
	pitch_ctrl.integral_prior = roll_ctrl.integral_prior = 0;
	pitch_ctrl.error = roll_ctrl.error = 0;
	pitch_ctrl.bias = roll_ctrl.bias = 0;

  while (1)
  {
  	if(mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  	{
  		mpu9250.readAccelData(accelCount);  // Read the x/y/z adc values
			// Now we'll calculate the accleration value into actual g's
			ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
			ay = (float)accelCount[1]*aRes - accelBias[1];
			az = (float)accelCount[2]*aRes - accelBias[2];

			mpu9250.readGyroData(gyroCount);  // Read the x/y/z adc values
			// Calculate the gyro value into actual degrees per second
			gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
			gy = (float)gyroCount[1]*gRes - gyroBias[1];
			gz = (float)gyroCount[2]*gRes - gyroBias[2];

			mpu9250.readMagData(magCount);  // Read the x/y/z adc values
			// Calculate the magnetometer values in milliGauss
			// Include factory calibration per data sheet and user environmental corrections
			mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
			my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];
			mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];
  	}
  	else
  	{
  		continue;
  	}

  	mpu9250.MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);

  	tempCount = mpu9250.readTempData();  // Read the adc values
  	temperature = ((float) tempCount) / 333.87f + 21.0f; // Temperature in degrees Centigrade

  	yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
		pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
		roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
		pitch *= 180.0f / PI;
		yaw   *= 180.0f / PI;
		yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
		roll  *= 180.0f / PI;

		// Update current_state's in control variables
		altitude_ctrl.current_state = altitude;
		yaw_ctrl.current_state = yaw;
		pitch_ctrl.current_state = pitch;
		roll_ctrl.current_state = roll;


		bmp280_get_uncomp_data(&ucomp_data, &bmp);
		bmp280_get_comp_temp_32bit(&temp, ucomp_data.uncomp_temp, &bmp);
		bmp280_get_comp_pres_64bit(&pres32, ucomp_data.uncomp_press, &bmp);

		altitude = 44330 * (1.0 - pow((float)(pres32 >> 8) / 1006.1, 0.1903)) * 3.28084;
#ifdef UART_DEBUGGING
		sprintf((char*)log_buffer, "Altitude: %f\r\n", altitude - altitudeOffset);
		ConsoleLog((char*)log_buffer);
#endif

		int iteration_time = 1;
		CalculatePIDControlOutput(&altitude_ctrl, 10, iteration_time);
		CalculatePIDControlOutput(&yaw_ctrl, 0, iteration_time);
		CalculatePIDControlOutput(&pitch_ctrl, 0, iteration_time);
		CalculatePIDControlOutput(&roll_ctrl, 0, iteration_time);

		UpdateMotorSpeeds(&altitude_ctrl, &yaw_ctrl, &pitch_ctrl, &roll_ctrl);
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

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void UpdateMotorSpeeds(struct control_variables *altitude, struct control_variables *yaw, struct control_variables *pitch, struct control_variables *roll)
{
	/* TODO:
	 * Real motor mixing algorithm based on the drone geometry should be applied here.
	 * PID controls also have to be tuned and implemented.
	 * Over-current protection algorithm.
	 * Vertical thrust preservation - means increasing overall thrust to maintain height
	 * while pitching or rolling.
	 * Cap overall current draw.
	 * Cap individual motor voltage. Priority: thrust, roll, pitch, then yaw.
	 * Anti-windup for the integrator and low pass filter for the differentiator
	*/

	int thrust = altitude->output;
	float percentagePitch = pitch->output / 90 / 2;
	float percentageRoll = roll->output / 90 / 2;
	float percentageYaw = yaw->output / 90 / 2;
	percentageYaw = 0;


	float p1 = (1000 - thrust) * -1 * (percentagePitch - percentageYaw - percentageRoll) + thrust;
	float p2 = (1000 - thrust) * -1 * (percentageYaw + percentageRoll + percentagePitch) + thrust;
	float p3 = (1000 - thrust) * -1 * (percentageRoll - percentageYaw - percentagePitch) + thrust;
	float p4 = (1000 - thrust) * -1 * (percentageYaw - percentageRoll - percentagePitch) + thrust;

	// Propeller angular velocities cannot be nagative.
	htim2.Instance->CCR4 = p1 > 0 ? p1 : 0;
	htim2.Instance->CCR2 = p2 > 0 ? p2 : 0;
	htim2.Instance->CCR3 = p3 > 0 ? p3 : 0;
	htim2.Instance->CCR1 = p4 > 0 ? p4 : 0;

}

void CalculatePIDControlOutput(struct control_variables *ctrl, float commanded_state, int iteration_time)
{
	/* TODO:
	 * Add derivative noise filter
	 * Implement integral clamping conditions
	 */
	ctrl->error = commanded_state - ctrl->current_state;

	if (ctrl->clamp == 0)
		ctrl->integral = ctrl->integral_prior + ctrl->error * iteration_time;

	ctrl->derivative = (ctrl->error - ctrl->error_prior) / iteration_time;
	ctrl->output = (ctrl->p_c * ctrl->error) + (ctrl->i_c * ctrl->integral) + (ctrl->d_c * ctrl->derivative) + ctrl->bias;

	ctrl->error_prior = ctrl->error;
	ctrl->integral_prior = ctrl->integral;
}
/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
