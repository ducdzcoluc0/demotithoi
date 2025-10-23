/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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
#include <stdbool.h>
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int i=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern TIM_HandleTypeDef htim1;
int vitri_trong = 5, vitri_coxe = 0;
uint8_t goc1 = 0;
uint8_t goc2 = 0;
char M[32];

bool TrangThai_ViTri[] = {false, false, false, false, false};
bool TrangThai_ViTri_Truoc[] = {false, false, false, false, false};

volatile bool xevao = false, xera = false;

#define SLAVE_ADDRESS_LCD 0x4E // dia chi i2c 
#define Tong_ViTri 5

void Lcd_Ghi_Lenh (char lenh)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (lenh&0xf0);
	data_l = ((lenh<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
	//HAL_I2C_Master_Transmit_IT(&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4);
}

void Lcd_Ghi_Dulieu (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1
	data_t[1] = data_u|0x09;  //en=0, rs=1
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
	//HAL_I2C_Master_Transmit_IT(&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4);
}

void lcd_init (void)
{
	Lcd_Ghi_Lenh (0x01); 
	HAL_Delay(50);
	Lcd_Ghi_Lenh (0x06); 	
	HAL_Delay(50);
	Lcd_Ghi_Lenh (0x0c); // bat tat con tro
	HAL_Delay(50);
	Lcd_Ghi_Lenh (0x02); 
	HAL_Delay(50);
	Lcd_Ghi_Lenh (0x80);
	HAL_Delay(50);
	Lcd_Ghi_Lenh (0x0E);
}

void Lcd_Ghi_Chuoi (char *str)
{
	while (*str) Lcd_Ghi_Dulieu (*str++);
}

void Lcd_xoa_manhinh (void)
{
	Lcd_Ghi_Lenh (0x01); //xoa man hinh
	HAL_Delay(2);
}
void LCD_Cursor(uint8_t dong, uint8_t cot) 
{
	uint8_t vitri;
	if (dong == 0) 
	{
		vitri = 0x80 + cot;
	} else 
	{
		vitri = 0xC0 + cot;
	}
	Lcd_Ghi_Lenh(vitri);
}

void TrangThai_CamBien() 
{
	// dat lai gia tri 
	vitri_coxe = 0;
	// doc trang thai hien tai
	bool TrangThai_HienTai[5];
	TrangThai_HienTai[0] = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET);
	TrangThai_HienTai[1] = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET);
	TrangThai_HienTai[2] = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_RESET);
	TrangThai_HienTai[3] = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET);
	TrangThai_HienTai[4] = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET);
	
	// kiem tra trang thai cam bien
	for (int i = 0; i < 5; i++) 
	{
		TrangThai_ViTri[i] = TrangThai_HienTai[i];
		if (TrangThai_HienTai[i]) 
		{
			vitri_coxe++;
		}
	}
	
	vitri_trong = Tong_ViTri - vitri_coxe;
	
	// tranh tinh sai so luong xe
	if (vitri_coxe < 0) vitri_coxe = 0;
	if (vitri_coxe > Tong_ViTri) vitri_coxe = Tong_ViTri;
	if (vitri_trong < 0) vitri_trong = 0;
	if (vitri_trong > Tong_ViTri) vitri_trong = Tong_ViTri;
}

// doi goc sang gia tri pwm
uint32_t goc_sang_pwm(uint8_t goc) 
{

	float pulse_width_ms = 1.0 + (goc / 180.0); // 1.0ms to 2.0ms
  return (uint32_t)(pulse_width_ms * 1000);
}
// set goc cho servo
void set_goc_servo(uint8_t kenh, uint8_t goc) 
{
	if (goc > 180) goc = 180;
    
  uint32_t ccr_gt = goc_sang_pwm(goc);
	
	if (kenh == 1) 
	{
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, ccr_gt);
	} 
	else if (kenh == 2) 
	{
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, ccr_gt);
	}
}

void HienThi_TrangThai() 
{
	Lcd_xoa_manhinh();
	
	// hien thi trang thai vi tri 
	LCD_Cursor(0,0);
	Lcd_Ghi_Chuoi("S1:");
	Lcd_Ghi_Chuoi(TrangThai_ViTri[0] ? "F" : "E");
	
	LCD_Cursor(0,5);
	Lcd_Ghi_Chuoi("S2:");
	Lcd_Ghi_Chuoi(TrangThai_ViTri[1] ? "F" : "E");
	
	LCD_Cursor(0,10);
	Lcd_Ghi_Chuoi("S3:");
	Lcd_Ghi_Chuoi(TrangThai_ViTri[2] ? "F" : "E");
	
	LCD_Cursor(1,0);
	Lcd_Ghi_Chuoi("S4:");
	Lcd_Ghi_Chuoi(TrangThai_ViTri[3] ? "F" : "E");
	
	LCD_Cursor(1, 5);
	Lcd_Ghi_Chuoi("S5:");
	Lcd_Ghi_Chuoi(TrangThai_ViTri[4] ? "F" : "E");
	
	
	LCD_Cursor(1,10);
	if (vitri_trong > 0) 
	{
		sprintf(M, "SoXe:%d", vitri_coxe);
		Lcd_Ghi_Chuoi(M);
	}
	else
	{
		LCD_Cursor(1, 10);
		Lcd_Ghi_Chuoi("Full!");
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 
{
	static uint32_t LanNgatCuoi = 0;
	
	if (HAL_GetTick() - LanNgatCuoi < 200) return;
	LanNgatCuoi = HAL_GetTick();
	
	if (GPIO_Pin == GPIO_PIN_2 && vitri_trong > 0) xevao = true;
  if (GPIO_Pin == GPIO_PIN_6) xera = true;
	
}
void DieuKhienCong() 
{
	if (xevao) 
	{
		set_goc_servo(1, 0);
    HAL_Delay(3500);
    set_goc_servo(1, 180);
    xevao = false;
	}
	if (xera) 
	{
		set_goc_servo(2, 0);
    HAL_Delay(3500);
    set_goc_servo(2, 180);
    xera = false;
	}
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	set_goc_servo(1, 180);
	set_goc_servo(2, 180);
	
	lcd_init();
	HAL_Delay(20);
	Lcd_xoa_manhinh();
	HAL_Delay(20);
	Lcd_Ghi_Chuoi("Hello");
	HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		TrangThai_CamBien();
		
    HienThi_TrangThai();

		DieuKhienCong();
		
    HAL_Delay(50);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : CB6_Pin CB7_Pin */
  GPIO_InitStruct.Pin = CB6_Pin|CB7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CB5_Pin CB1_Pin CB2_Pin CB3_Pin
                           CB4_Pin */
  GPIO_InitStruct.Pin = CB5_Pin|CB1_Pin|CB2_Pin|CB3_Pin
                          |CB4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
