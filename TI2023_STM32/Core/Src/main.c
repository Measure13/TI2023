/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AD9833.h"
#include "USART_HMI.h"
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

/* USER CODE BEGIN PV */
bool volatile conv_done = false;
uint16_t adc1_values[ADC_DATA_NUM + 4];
uint16_t adc2_values[ADC_DATA_NUM + 4];
uint16_t adc3_values[ADC_DATA_NUM + 4];
uint32_t quadrant_time_stamp[4];

static uint16_t total_steps = 50;
static uint16_t steps = 15;
static uint16_t P_skip_num = 0;
static float MICROPHONE[4][2] = { {HALF_SQUARE, HALF_SQUARE}, 
                                  {-HALF_SQUARE, HALF_SQUARE}, 
                                  {-HALF_SQUARE, -HALF_SQUARE}, 
                                  {HALF_SQUARE, -HALF_SQUARE}};
static float POINT_DIST[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static float CORRECT_POINT_DIST[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static float G_VECTOR[4][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}};
uint32_t quadrant_time_stamp[4] = {0, 0, 0, 0};
static float pix, piy;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static inline float dist(float p1x, float p1y, float p2x, float p2y);
static inline float norm(float px, float py);
static void Gradient_descent(uint8_t step);
static void Gradient_descent_wrapper(void);
static void Quadrant_Lattice_Indexing(void);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_DisableIRQ(EXTI1_IRQn);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
  HAL_NVIC_DisableIRQ(EXTI2_IRQn);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
  HAL_NVIC_DisableIRQ(EXTI3_IRQn);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
  HAL_NVIC_DisableIRQ(EXTI4_IRQn);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
  AD9833_Init();
  UARTHMI_Forget_It();
  UARTHMI_Reset();
  HAL_TIM_Base_Start_IT(&htim2);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_values, ADC_DATA_NUM + 4);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_values, ADC_DATA_NUM + 4);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3_values, ADC_DATA_NUM + 4);
  HAL_Delay(150);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (sweep_freq)
    {
      for (uint16_t i = 15; i < 21; ++i)
      {
        AD9833_Default_Set(i * 1000);
        HAL_Delay(1000);
      }
      sweep_freq = false;
    }
    else if (sound_trace)
    {
      __HAL_TIM_SetCounter(&htim5, 0);
      HAL_TIM_Base_Start(&htim5);
      HAL_NVIC_EnableIRQ(EXTI1_IRQn);
      HAL_NVIC_EnableIRQ(EXTI2_IRQn);
      HAL_NVIC_EnableIRQ(EXTI3_IRQn);
      HAL_NVIC_EnableIRQ(EXTI4_IRQn);
      while (1)
      {
        if ((!((__NVIC_GetEnableIRQ(EXTI1_IRQn)) | (__NVIC_GetEnableIRQ(EXTI2_IRQn)) | (__NVIC_GetEnableIRQ(EXTI3_IRQn)) | (__NVIC_GetEnableIRQ(EXTI4_IRQn)))))
        {
          Quadrant_Lattice_Indexing();
          HAL_Delay(500);
          break;
        }
        else if (sweep_freq | magnet_trace)
        {
          break;
        }
      }
      sound_trace = false;
      HAL_TIM_Base_Stop(&htim5);
    }
    else if (magnet_trace)
    {
      magnet_trace = false;
    }
    
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
static inline float dist(float p1x, float p1y, float p2x, float p2y)
{
  return sqrtf((p1x - p2x) * (p1x - p2x) + (p1y - p2y) * (p1y - p2y));
}

static inline float norm(float px, float py)
{
  return sqrtf(px * px + py * py);
}

static void Gradient_descent(uint8_t step)
{
  float sum = 0.0f, temp = 0.0f;
  float gradients_list[4][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}};
  for (uint8_t i = 0; i < 4; ++i)
  {
    POINT_DIST[i] = (dist(pix, piy, MICROPHONE[i][0], MICROPHONE[i][1]) - dist(pix, piy, MICROPHONE[P_skip_num][0], MICROPHONE[P_skip_num][1])) - CORRECT_POINT_DIST[i] / CLK_FREQ * V_BOARD;
    sum += fabsf(POINT_DIST[i]);
  }
  for (uint8_t i = 0; i < 4; ++i)
  {
    POINT_DIST[i] /= sum;
    temp = (steps + sum / 20.0f) * (1.0f - logf(1.0f + expf(step / 5.0f - 10.0f)) / 0.65f);
    gradients_list[i][0] = G_VECTOR[i][0] * POINT_DIST[i] * temp;
    gradients_list[i][1] = G_VECTOR[i][1] * POINT_DIST[i] * temp;
    pix += gradients_list[i][0];
    piy += gradients_list[i][1];
  }
  
}

static void Gradient_descent_wrapper(void)
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    G_VECTOR[i][0] = MICROPHONE[i][0] - MICROPHONE[P_skip_num][0];
    G_VECTOR[i][1] = MICROPHONE[i][1] - MICROPHONE[P_skip_num][1];
    if (i != P_skip_num)
    {
      G_VECTOR[i][0] /= norm(G_VECTOR[i][0], G_VECTOR[i][1]) * sqrtf(3.0f);
      G_VECTOR[i][1] /= norm(G_VECTOR[i][0], G_VECTOR[i][1]) * sqrtf(3.0f);
    }
  }
  for (uint8_t i = 0; i < total_steps; ++i)
  {
    Gradient_descent(i);
  }
}

static void Quadrant_Lattice_Indexing(void)
{
  int8_t x_index, y_index;
  uint32_t min = quadrant_time_stamp[0];
  for (uint8_t i = 1; i < 4; ++i)
  {
    if (quadrant_time_stamp[i] < min)
    {
      P_skip_num = i;
      min = quadrant_time_stamp[i];
    }
  }
  pix = MICROPHONE[P_skip_num][0] * LENGTH / HALF_SQUARE / 2;
  piy = MICROPHONE[P_skip_num][1] * LENGTH / HALF_SQUARE / 2;
  for (uint8_t i = 0; i < 4; ++i)
  {
    CORRECT_POINT_DIST[i] = (float)(quadrant_time_stamp[i] - min) * V_BOARD / CLK_FREQ;
  }
  Gradient_descent_wrapper();
  pix += WIDTH / 2;
  piy = -piy;
  piy += LENGTH / 2;
  x_index = (uint8_t)(pix / LATTICE_6_UNIT) + 1;
  y_index = (uint8_t)(piy / LATTICE_6_UNIT) + 1;
  printf("page 0\xff\xff\xff");
  printf("fill %d,%d,%d,%d,BLUE\xff\xff\xff", x_index * LATTICE_6_SQUARE, y_index * LATTICE_6_SQUARE, LATTICE_6_SQUARE, LATTICE_6_SQUARE);
}

void Configuration_Init(void)
{
  HAL_TIM_Base_Stop_IT(&htim5);
  __HAL_TIM_SetCounter(&htim5, 0);
  memset(quadrant_time_stamp, 0x00000000, sizeof(uint32_t) * 4);
  __NVIC_EnableIRQ(EXTI0_IRQn);
  __NVIC_EnableIRQ(EXTI1_IRQn);
  __NVIC_EnableIRQ(EXTI2_IRQn);
  __NVIC_EnableIRQ(EXTI3_IRQn);
  __NVIC_EnableIRQ(TIM5_IRQn);
  htim5.State = HAL_TIM_STATE_BUSY;
  __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE(&htim5);
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
