/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

struct MP92_Commands
{
	uint8_t	SELF_TEST_X_GY;		/*	0	Gyroscope Self-Test Registers	*/
	uint8_t	SELF_TEST_Y_GY;		/*	1	Gyroscope Self-Test Registers	*/
	uint8_t	SELF_TEST_Z_GY;		/*	2	Gyroscope Self-Test Registers	*/
	uint8_t	SELF_TEST_X_AC;		/*	13	Accelerometer Self-Test Registers	*/
	uint8_t	SELF_TEST_Y_AC;		/*	14	Accelerometer Self-Test Registers	*/
	uint8_t	SELF_TEST_Z_AC;		/*	15	Accelerometer Self-Test Registers	*/
	uint8_t	XG_OFFSET_H;		/*	19	Gyro Offset Registers	*/
	uint8_t	XG_OFFSET_L;		/*	20	Gyro Offset Registers	*/
	uint8_t	YG_OFFSET_H;		/*	21	Gyro Offset Registers	*/
	uint8_t	YG_OFFSET_L;		/*	22	Gyro Offset Registers	*/
	uint8_t	ZG_OFFSET_H;		/*	23	Gyro Offset Registers	*/
	uint8_t	ZG_OFFSET_L;		/*	24	Gyro Offset Registers	*/
	uint8_t	SMPLRT_DIV;			/*	25	Sample Rate Divide	*/
	uint8_t	CONFIG;				/*	26	Configuration	*/
	uint8_t	GYRO_CONFIG;		/*	27	Gyroscope Configuration	*/
	uint8_t	ACCEL_CONFIG;		/*	28	Accelerometer Configuration	*/
	uint8_t	ACCEL_CONFIG_2;		/*	29	Accelerometer Configuration 	*/
	uint8_t	LP_ACCEL_ODR;		/*	30	Low Power Accelerometer ODR Contro	*/
	uint8_t	WOM_THR;			/*	31	Wake-on Motion Threshold	*/
	uint8_t	FIFO_EN;			/*	35	FIFO Enable	*/
	uint8_t	I2C_MST_CTRL;		/*	36	I2C Master Control	*/
	uint8_t	I2C_SLV0_ADDR;		/*	37		*/
	uint8_t	I2C_SLV0_REG;		/*	38		*/
	uint8_t	I2C_SLV0_CTRL;		/*	39		*/
	uint8_t	I2C_SLV1_ADDR;		/*	40		*/
	uint8_t	I2C_SLV1_REG;		/*	41		*/
	uint8_t	I2C_SLV1_CTRL;		/*	42		*/
	uint8_t	I2C_SLV2_ADDR;		/*	43		*/
	uint8_t	I2C_SLV2_REG;		/*	44		*/
	uint8_t	I2C_SLV2_CTRL;		/*	45		*/
	uint8_t	I2C_SLV3_ADDR;		/*	46		*/
	uint8_t	I2C_SLV3_REG;		/*	47		*/
	uint8_t	I2C_SLV3_CTRL;		/*	48		*/
	uint8_t	I2C_SLV4_ADDR;		/*	49		*/
	uint8_t	I2C_SLV4_REG;		/*	50		*/
	uint8_t	I2C_SLV4_DO;		/*	51		*/
	uint8_t	I2C_SLV4_CTRL;		/*	52		*/
	uint8_t	I2C_SLV4_DI;		/*	53		*/
	uint8_t	I2C_MST_STATUS;		/*	54	I2C Master Status	*/
	uint8_t	INT_PIN_CFG;		/*	55	INT Pin / Bypass Enable Configuration	*/
	uint8_t	INT_ENABLE;			/*	56	Interrupt Enable	*/
	uint8_t	INT_STATUS;			/*	58	Interrupt Status	*/
	uint8_t	ACCEL_XOUT_H;		/*	59	Accelerometer Measurements	*/
	uint8_t	ACCEL_XOUT_L;		/*	60	Accelerometer Measurements	*/
	uint8_t	ACCEL_YOUT_H;		/*	61	Accelerometer Measurements	*/
	uint8_t	ACCEL_YOUT_L;		/*	62	Accelerometer Measurements	*/
	uint8_t	ACCEL_ZOUT_H;		/*	63	Accelerometer Measurements	*/
	uint8_t	ACCEL_ZOUT_L;		/*	64	Accelerometer Measurements	*/
	uint8_t	TEMP_OUT_H;			/*	65	Temperature Measuremen	*/
	uint8_t	TEMP_OUT_L;			/*	66	Temperature Measuremen	*/
	uint8_t	GYRO_XOUT_H;		/*	67	Gyroscope Measurements	*/
	uint8_t	GYRO_XOUT_L;		/*	68	Gyroscope Measurements	*/
	uint8_t	GYRO_YOUT_H;		/*	69	Gyroscope Measurements	*/
	uint8_t	GYRO_YOUT_L;		/*	70	Gyroscope Measurements	*/
	uint8_t	GYRO_ZOUT_H;		/*	71	Gyroscope Measurements	*/
	uint8_t	GYRO_ZOUT_L;		/*	72	Gyroscope Measurements	*/
	uint8_t	EXT_SENS_DATA_00;	/*	73	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_01;	/*	74	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_02;	/*	75	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_03;	/*	76	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_04;	/*	77	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_05;	/*	78	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_06;	/*	79	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_07;	/*	80	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_08;	/*	81	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_09;	/*	82	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_10;	/*	83	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_11;	/*	84	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_12;	/*	85	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_13;	/*	86	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_14;	/*	87	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_15;	/*	88	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_16;	/*	89	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_17;	/*	90	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_18;	/*	91	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_19;	/*	92	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_20;	/*	93	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_21;	/*	94	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_22;	/*	95	External Sensor Data	*/
	uint8_t	EXT_SENS_DATA_23;	/*	96	External Sensor Data	*/
	uint8_t	I2C_SLV0_DO;		/*	99	I2C Slave 0 Data Out	*/
	uint8_t	I2C_SLV1_DO;		/*	100	I2C Slave 1 Data Out	*/
	uint8_t	I2C_SLV2_DO;		/*	101	I2C Slave 2 Data Out	*/
	uint8_t	I2C_SLV3_DO;		/*	102	I2C Slave 3 Data Out	*/
	uint8_t	I2C_MST_DELAY_CTRL;	/*	103	I2C Master Delay Control	*/
	uint8_t	SIGNAL_PATH_RESET;	/*	104	Signal Path Reset	*/
	uint8_t	MOT_DETECT_CTRL;	/*	105	Accelerometer Interrupt Control	*/
	uint8_t	USER_CTRL;			/*	106	User Control	*/
	uint8_t	PWR_MGMT_1;			/*	107	Power Management 1	*/
	uint8_t	PWR_MGMT_2;			/*	108	Power Management 2	*/
	uint8_t	FIFO_COUNTH;		/*	114	FIFO Count Registers	*/
	uint8_t	FIFO_COUNTL;		/*	115	FIFO Count Registers	*/
	uint8_t	FIFO_R_W;			/*	116	FIFO Read Write	*/
	uint8_t	WHO_AM_I;			/*	117	Who Am I -> 0x71	*/
	uint8_t	XA_OFFSET_H;		/*	119	AccelerometerOffset Registers	*/
	uint8_t	XA_OFFSET_L;		/*	120	AccelerometerOffset Registers	*/
	uint8_t	YA_OFFSET_H;		/*	122	AccelerometerOffset Registers	*/
	uint8_t	YA_OFFSET_L;		/*	123	AccelerometerOffset Registers	*/
	uint8_t	ZA_OFFSET_H;		/*	125	AccelerometerOffset Registers	*/
	uint8_t	ZA_OFFSET_L;		/*	126	AccelerometerOffset Registers	*/
} MP92cmd = {
		0x00,0x01,0x02,0x0D,0x0E,0x0F,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x23,
		0x24,0x25,0x26,0x27,0x28,0x29,0x2A,0x2B,0x2C,0x2D,0x2E,0x2F,0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,
		0x38,0x3A,0x3B,0x3C,0x3D,0x3E,0x3F,0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x4B,0x4C,
		0x4D,0x4E,0x4F,0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x5B,0x5C,0x5D,0x5E,0x5F,0x60,
		0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x6B,0x6C,0x72,0x73,0x74,0x75,0x77,0x78,0x7A,0x7B,0x7D,0x7E
};

enum MP92BitNum {
	D0, D1, D2, D3, D4, D5, D6, D7
};

#define MP92_ADR (0x68 << 1)
#define MAG_ADR (0x0C << 1)
#define BMP_ADR (0x76 << 1) // or 0x77

struct GY91_Data
{
	uint8_t id;
	int16_t ax,ay,az;
	int16_t gx,gy,gz;
	int16_t mx,my,mz;

	uint8_t raw[14];
	uint8_t mag[7];

	uint8_t mag_status;
}GY91;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

void i2cWrite(uint8_t adr, uint8_t reg, uint8_t data) {
	uint8_t i2c_tx[2];
	i2c_tx[0] = reg;
	i2c_tx[1] = data;
	HAL_I2C_Master_Transmit(&hi2c1, adr, i2c_tx, 2, 1000);
}

void i2cRead(uint8_t adr, uint8_t reg, uint8_t *data, int16_t count) {
	HAL_I2C_Master_Transmit(&hi2c1, adr, &reg, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, adr, data, count, 100);
}

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  i2cWrite(MP92_ADR, MP92cmd.INT_PIN_CFG, (1 << D1));
  i2cWrite(MAG_ADR, 0x0A, 0x16);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  /* USER CODE END WHILE */

	  i2cRead(MP92_ADR, MP92cmd.WHO_AM_I, &GY91.id, 1);
	  i2cRead(MP92_ADR, MP92cmd.ACCEL_XOUT_H, (uint8_t*)&GY91.raw, 14);

	  //i2cRead(MAG_ADR, 0x02, (uint8_t*)&GY91.mag_status, 1);
	  //i2cRead(BMP_ADR, 0xD0, (uint8_t*)&GY91.mag_status, 1);

	  i2cRead(MAG_ADR, 0x03, (uint8_t*)&GY91.mag, 7);



	  // Accelerometer
	  GY91.ax=-(GY91.raw[0]<<8 | GY91.raw[1]);
	  GY91.ay=-(GY91.raw[2]<<8 | GY91.raw[3]);
	  GY91.az= (GY91.raw[4]<<8 | GY91.raw[5]);

	  // Gyroscope
	  GY91.gx=-(GY91.raw[8]<<8 | GY91.raw[9]);
	  GY91.gy=-(GY91.raw[10]<<8 | GY91.raw[11]);
	  GY91.gz= (GY91.raw[12]<<8 | GY91.raw[13]);

	  // Magnetometer
	  GY91.mx=-(GY91.mag[3]<<8 | GY91.mag[2]);
	  GY91.my=-(GY91.mag[1]<<8 | GY91.mag[0]);
	  GY91.mz=-(GY91.mag[5]<<8 | GY91.mag[4]);

	  HAL_Delay(10);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
