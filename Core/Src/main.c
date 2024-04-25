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

/**
 * TODO https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
 * https://deepbluembedded.com/how-to-receive-uart-serial-data-with-stm32-dma-interrupt-polling/
 */
#include <profile.h>
#include <string.h>
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "command.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	BTN_RELEASED = 0,
	BTN_DEBOUNCING,
	BTN_PRESSED,	// waiting for release
} BTN_StatusTypeDef;




/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// 2 for S
// 1 for C
// 0 for floating
#define SET_CHAN(ccfg, cname)							do {					\
	if (ccfg == '2')															\
	{																			\
		HAL_GPIO_WritePin(cname##C_GPIO_Port, cname##C_Pin, GPIO_PIN_RESET);	\
		HAL_GPIO_WritePin(cname##S_GPIO_Port, cname##S_Pin, GPIO_PIN_SET);		\
	}																			\
	else if (ccfg == '1')														\
	{																			\
		HAL_GPIO_WritePin(cname##S_GPIO_Port, cname##S_Pin, GPIO_PIN_RESET);    \
		HAL_GPIO_WritePin(cname##C_GPIO_Port, cname##C_Pin, GPIO_PIN_SET);	    \
	}																			\
	else																		\
	{																			\
		HAL_GPIO_WritePin(cname##S_GPIO_Port, cname##S_Pin, GPIO_PIN_RESET);    \
		HAL_GPIO_WritePin(cname##C_GPIO_Port, cname##C_Pin, GPIO_PIN_RESET);	\
	} } while (0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

osThreadId uxTaskHandle;
osThreadId profileTaskHandle;
osMessageQId requestQueueHandle;
/* USER CODE BEGIN PV */

uint8_t UART2_rxbuf[12] = {0};
const uint8_t hello[] = "\r\nhello\r\n> ";

static BTN_StatusTypeDef btn_status = BTN_PRESSED;
int btn_index = -1;
int btn_count = 0;

static bool on_status = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
void StartUxTask(void const * argument);
extern void StartProfileTask(void const * argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * Write char to uart in blocking mode
 */

static void UART2_write(const char * str)
{
	int len = strlen(str);
	if (len == 0)
	{
		return;
	}

	HAL_UART_Transmit(&huart2, (const uint8_t *)str, len, HAL_MAX_DELAY);
}

void print_line(const char *str)
{
	UART2_write(str);
	UART2_write("\r\n");
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart2, hello, sizeof(hello)/sizeof(hello[0]), 100);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of requestQueue */
  osMessageQDef(requestQueue, 1, profile_t);
  requestQueueHandle = osMessageCreate(osMessageQ(requestQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of uxTask */
  osThreadDef(uxTask, StartUxTask, osPriorityNormal, 0, 256);
  uxTaskHandle = osThreadCreate(osThread(uxTask), NULL);

  /* definition and creation of profileTask */
  osThreadDef(profileTask, StartProfileTask, osPriorityAboveNormal, 0, 512);
  profileTaskHandle = osThreadCreate(osThread(profileTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, D9S_Pin|D9C_Pin|D8S_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, D8C_Pin|D7S_Pin|C4S_Pin|C4C_Pin
                          |C3S_Pin|C3C_Pin|C2S_Pin|C2C_Pin
                          |C1S_Pin|A8S_Pin|A8C_Pin|A7S_Pin
                          |A7C_Pin|A6S_Pin|A6C_Pin|A5S_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, D7C_Pin|D6S_Pin|D6C_Pin|D5S_Pin
                          |D5C_Pin|D4S_Pin|D4C_Pin|D3S_Pin
                          |D3C_Pin|A1S_Pin|A1C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D2S_Pin|D2C_Pin|D1S_Pin|D1C_Pin
                          |C9S_Pin|C9C_Pin|A5C_Pin|A4S_Pin
                          |A4C_Pin|A3S_Pin|A3C_Pin|A2S_Pin
                          |A2C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, C8S_Pin|C8C_Pin|C7S_Pin|C7C_Pin
                          |C6S_Pin|C6C_Pin|C5S_Pin|C5C_Pin
                          |B3S_Pin|B3C_Pin|B2S_Pin|B2C_Pin
                          |B1S_Pin|B1C_Pin|A9S_Pin|A9C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, C1C_Pin|B9S_Pin|B9C_Pin|B8S_Pin
                          |B5C_Pin|B4S_Pin|B4C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, B8C_Pin|B7S_Pin|B7C_Pin|B6S_Pin
                          |B6C_Pin|B5S_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SW0_Pin SW9_Pin SW8_Pin SW7_Pin */
  GPIO_InitStruct.Pin = SW0_Pin|SW9_Pin|SW8_Pin|SW7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW6_Pin SW5_Pin */
  GPIO_InitStruct.Pin = SW6_Pin|SW5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SW4_Pin SW3_Pin */
  GPIO_InitStruct.Pin = SW4_Pin|SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW2_Pin SW1_Pin */
  GPIO_InitStruct.Pin = SW2_Pin|SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : D9S_Pin D9C_Pin D8S_Pin */
  GPIO_InitStruct.Pin = D9S_Pin|D9C_Pin|D8S_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : D8C_Pin D7S_Pin C4S_Pin C4C_Pin
                           C3S_Pin C3C_Pin C2S_Pin C2C_Pin
                           C1S_Pin A8S_Pin A8C_Pin A7S_Pin
                           A7C_Pin A6S_Pin A6C_Pin A5S_Pin */
  GPIO_InitStruct.Pin = D8C_Pin|D7S_Pin|C4S_Pin|C4C_Pin
                          |C3S_Pin|C3C_Pin|C2S_Pin|C2C_Pin
                          |C1S_Pin|A8S_Pin|A8C_Pin|A7S_Pin
                          |A7C_Pin|A6S_Pin|A6C_Pin|A5S_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : D7C_Pin D6S_Pin D6C_Pin D5S_Pin
                           D5C_Pin D4S_Pin D4C_Pin D3S_Pin
                           D3C_Pin A1S_Pin A1C_Pin */
  GPIO_InitStruct.Pin = D7C_Pin|D6S_Pin|D6C_Pin|D5S_Pin
                          |D5C_Pin|D4S_Pin|D4C_Pin|D3S_Pin
                          |D3C_Pin|A1S_Pin|A1C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : D2S_Pin D2C_Pin D1S_Pin D1C_Pin
                           C9S_Pin C9C_Pin A5C_Pin A4S_Pin
                           A4C_Pin A3S_Pin A3C_Pin A2S_Pin
                           A2C_Pin */
  GPIO_InitStruct.Pin = D2S_Pin|D2C_Pin|D1S_Pin|D1C_Pin
                          |C9S_Pin|C9C_Pin|A5C_Pin|A4S_Pin
                          |A4C_Pin|A3S_Pin|A3C_Pin|A2S_Pin
                          |A2C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : C8S_Pin C8C_Pin C7S_Pin C7C_Pin
                           C6S_Pin C6C_Pin C5S_Pin C5C_Pin
                           B3S_Pin B3C_Pin B2S_Pin B2C_Pin
                           B1S_Pin B1C_Pin A9S_Pin A9C_Pin */
  GPIO_InitStruct.Pin = C8S_Pin|C8C_Pin|C7S_Pin|C7C_Pin
                          |C6S_Pin|C6C_Pin|C5S_Pin|C5C_Pin
                          |B3S_Pin|B3C_Pin|B2S_Pin|B2C_Pin
                          |B1S_Pin|B1C_Pin|A9S_Pin|A9C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : C1C_Pin B9S_Pin B9C_Pin B8S_Pin
                           B5C_Pin B4S_Pin B4C_Pin */
  GPIO_InitStruct.Pin = C1C_Pin|B9S_Pin|B9C_Pin|B8S_Pin
                          |B5C_Pin|B4S_Pin|B4C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : B8C_Pin B7S_Pin B7C_Pin B6S_Pin
                           B6C_Pin B5S_Pin */
  GPIO_InitStruct.Pin = B8C_Pin|B7S_Pin|B7C_Pin|B6S_Pin
                          |B6C_Pin|B5S_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static uint32_t BTN_KeyScan()
{
	uint32_t idr, code = 0;

	idr = GPIOA->IDR;
	if (!(idr & SW0_Pin))	// active low!
	{
		code |= (1UL << 0);
	}

	if (!(idr & SW7_Pin))
	{
		code |= (1UL << 7);
	}

	if (!(idr & SW8_Pin))
	{
		code |= (1UL << 8);
	}

	if (!(idr & SW9_Pin))
	{
		code |= (1UL << 9);
	}

	idr = GPIOB->IDR;
	if (!(idr & SW3_Pin))
	{
		code |= (1UL << 3);
	}

	if (!(idr & SW4_Pin))
	{
		code |= (1UL << 4);
	}

	idr = GPIOC->IDR;
	if (!(idr & SW5_Pin))
	{
		code |= (1UL << 5);
	}
	if (!(idr & SW6_Pin))
	{
		code |= (1UL << 6);
	}

	idr = GPIOF->IDR;
	if (!(idr & SW1_Pin))
	{
		code |= (1UL << 1);
	}
	if (!(idr & SW2_Pin))
	{
		code |= (1UL << 2);
	}

	return code;
}

/* return key index if single key pressed in given key code.
 * return -1 if no key is pressed.
 * return -2 if multiple keys are pressed.
 * this is a level detector.
 */
static int BTN_SingleKeyPress(uint32_t code)
{
	int i, index, count = 0;
	for (i = 0; i < 32; i++)
	{
		if (code & (1UL << i))
		{
			index = i;
			count++;
		}
	}

	if (count == 0)
	{
		return -1;
	}
	else if (count == 1)
	{
		return index;
	}
	else
	{
		return -2;
	}
}

/* return key index if single key pressed down.
 * return -1 if no key down detected (may or may not be pressed)
 * this is an edge detector.
 */
static int BTN_SingleKeyDown()
{
	uint32_t code = BTN_KeyScan();
	int index = BTN_SingleKeyPress(code);
	switch (btn_status)
	{
	case BTN_RELEASED:
		if (index < 0)
		{
			// do nothing
		}
		else
		{
			btn_status = BTN_DEBOUNCING;
			btn_index = index;
			btn_count = 1;
		}
		break;
	case BTN_DEBOUNCING:
		if (index == -2) // multiple keys pressed?
		{
			btn_status = BTN_PRESSED;
			btn_index = -1;
			btn_count = 0;
		}
		else if (index == -1) // no keys pressed
		{
			btn_status = BTN_RELEASED;
			btn_index = -1;
			btn_count = 0;
		}
		else if (index == btn_index)
		{
			btn_count++;
			if (btn_count > 3)
			{
				btn_status = BTN_PRESSED;
				btn_index = -1;
				btn_count = 0;
				return index;
			}
			// otherwise, status and index unchanged
		}
		else // another key pressed in debouncing?
		{
			btn_status = BTN_PRESSED;
			btn_index = index;
			btn_count = 1;
		}
		break;
	case BTN_PRESSED:	// no change in btn_index and btn_count in this state
		if (index == -1)
		{
			btn_status = BTN_RELEASED;
			// btn_index = -1;
			// btn_count = 0;
		}
		break;
	}
	return -1;
}

int detect_single_keydown(void)
{
	return BTN_SingleKeyDown();
}

static void unset_chan()
{
	SET_CHAN(0, A1);
	SET_CHAN(0, A2);
	SET_CHAN(0, A3);
	SET_CHAN(0, A4);
	SET_CHAN(0, A5);
	SET_CHAN(0, A6);
	SET_CHAN(0, A7);
	SET_CHAN(0, A8);
	SET_CHAN(0, A9);

	SET_CHAN(0, B1);
	SET_CHAN(0, B2);
	SET_CHAN(0, B3);
	SET_CHAN(0, B4);
	SET_CHAN(0, B5);
	SET_CHAN(0, B6);
	SET_CHAN(0, B7);
	SET_CHAN(0, B8);
	SET_CHAN(0, B9);

	SET_CHAN(0, C1);
	SET_CHAN(0, C2);
	SET_CHAN(0, C3);
	SET_CHAN(0, C4);
	SET_CHAN(0, C5);
	SET_CHAN(0, C6);
	SET_CHAN(0, C7);
	SET_CHAN(0, C8);
	SET_CHAN(0, C9);

	SET_CHAN(0, D1);
	SET_CHAN(0, D2);
	SET_CHAN(0, D3);
	SET_CHAN(0, D4);
	SET_CHAN(0, D5);
	SET_CHAN(0, D6);
	SET_CHAN(0, D7);
	SET_CHAN(0, D8);
	SET_CHAN(0, D9);
}

static void blink_delay(void)
{
	vTaskDelay(100);
}

static void CLI_CMD_Blink(EmbeddedCli *cli, char *args, void *context)
{
	if (on_status == true)
	{
		UART2_write("this command works only when switches are off.");
		return;
	}

	unset_chan();

	SET_CHAN('2', A1); blink_delay();
	SET_CHAN('2', A2); blink_delay();
	SET_CHAN('2', A3); blink_delay();
	SET_CHAN('2', A4); blink_delay();
	SET_CHAN('2', A5); blink_delay();
	SET_CHAN('2', A6); blink_delay();
	SET_CHAN('2', A7); blink_delay();
	SET_CHAN('2', A8); blink_delay();
	SET_CHAN('2', A9); blink_delay();

	SET_CHAN('2', B1); blink_delay();
	SET_CHAN('2', B2); blink_delay();
	SET_CHAN('2', B3); blink_delay();
	SET_CHAN('2', B4); blink_delay();
	SET_CHAN('2', B5); blink_delay();
	SET_CHAN('2', B6); blink_delay();
	SET_CHAN('2', B7); blink_delay();
	SET_CHAN('2', B8); blink_delay();
	SET_CHAN('2', B9); blink_delay();

	SET_CHAN('2', C1); blink_delay();
	SET_CHAN('2', C2); blink_delay();
	SET_CHAN('2', C3); blink_delay();
	SET_CHAN('2', C4); blink_delay();
	SET_CHAN('2', C5); blink_delay();
	SET_CHAN('2', C6); blink_delay();
	SET_CHAN('2', C7); blink_delay();
	SET_CHAN('2', C8); blink_delay();
	SET_CHAN('2', C9); blink_delay();

	SET_CHAN('2', D1); blink_delay();
	SET_CHAN('2', D2); blink_delay();
	SET_CHAN('2', D3); blink_delay();
	SET_CHAN('2', D4); blink_delay();
	SET_CHAN('2', D5); blink_delay();
	SET_CHAN('2', D6); blink_delay();
	SET_CHAN('2', D7); blink_delay();
	SET_CHAN('2', D8); blink_delay();
	SET_CHAN('2', D9); blink_delay();

	SET_CHAN('1', A1); blink_delay();
	SET_CHAN('1', A2); blink_delay();
	SET_CHAN('1', A3); blink_delay();
	SET_CHAN('1', A4); blink_delay();
	SET_CHAN('1', A5); blink_delay();
	SET_CHAN('1', A6); blink_delay();
	SET_CHAN('1', A7); blink_delay();
	SET_CHAN('1', A8); blink_delay();
	SET_CHAN('1', A9); blink_delay();

	SET_CHAN('1', B1); blink_delay();
	SET_CHAN('1', B2); blink_delay();
	SET_CHAN('1', B3); blink_delay();
	SET_CHAN('1', B4); blink_delay();
	SET_CHAN('1', B5); blink_delay();
	SET_CHAN('1', B6); blink_delay();
	SET_CHAN('1', B7); blink_delay();
	SET_CHAN('1', B8); blink_delay();
	SET_CHAN('1', B9); blink_delay();

	SET_CHAN('1', C1); blink_delay();
	SET_CHAN('1', C2); blink_delay();
	SET_CHAN('1', C3); blink_delay();
	SET_CHAN('1', C4); blink_delay();
	SET_CHAN('1', C5); blink_delay();
	SET_CHAN('1', C6); blink_delay();
	SET_CHAN('1', C7); blink_delay();
	SET_CHAN('1', C8); blink_delay();
	SET_CHAN('1', C9); blink_delay();

	SET_CHAN('1', D1); blink_delay();
	SET_CHAN('1', D2); blink_delay();
	SET_CHAN('1', D3); blink_delay();
	SET_CHAN('1', D4); blink_delay();
	SET_CHAN('1', D5); blink_delay();
	SET_CHAN('1', D6); blink_delay();
	SET_CHAN('1', D7); blink_delay();
	SET_CHAN('1', D8); blink_delay();
	SET_CHAN('1', D9); blink_delay();

	SET_CHAN('0', A1); blink_delay();
	SET_CHAN('0', A2); blink_delay();
	SET_CHAN('0', A3); blink_delay();
	SET_CHAN('0', A4); blink_delay();
	SET_CHAN('0', A5); blink_delay();
	SET_CHAN('0', A6); blink_delay();
	SET_CHAN('0', A7); blink_delay();
	SET_CHAN('0', A8); blink_delay();
	SET_CHAN('0', A9); blink_delay();

	SET_CHAN('0', B1); blink_delay();
	SET_CHAN('0', B2); blink_delay();
	SET_CHAN('0', B3); blink_delay();
	SET_CHAN('0', B4); blink_delay();
	SET_CHAN('0', B5); blink_delay();
	SET_CHAN('0', B6); blink_delay();
	SET_CHAN('0', B7); blink_delay();
	SET_CHAN('0', B8); blink_delay();
	SET_CHAN('0', B9); blink_delay();

	SET_CHAN('0', C1); blink_delay();
	SET_CHAN('0', C2); blink_delay();
	SET_CHAN('0', C3); blink_delay();
	SET_CHAN('0', C4); blink_delay();
	SET_CHAN('0', C5); blink_delay();
	SET_CHAN('0', C6); blink_delay();
	SET_CHAN('0', C7); blink_delay();
	SET_CHAN('0', C8); blink_delay();
	SET_CHAN('0', C9); blink_delay();

	SET_CHAN('0', D1); blink_delay();
	SET_CHAN('0', D2); blink_delay();
	SET_CHAN('0', D3); blink_delay();
	SET_CHAN('0', D4); blink_delay();
	SET_CHAN('0', D5); blink_delay();
	SET_CHAN('0', D6); blink_delay();
	SET_CHAN('0', D7); blink_delay();
	SET_CHAN('0', D8); blink_delay();
	SET_CHAN('0', D9); blink_delay();
}

//CliCommandBinding cli_cmd_blink_binding = {
//		"blink",
//		"Switch on/off all ports (without simultaneous signal/common on, for test only)",
//		false,
//		NULL,
//		CLI_CMD_Blink
//};
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUxTask */
/**
  * @brief  Function implementing the uxTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUxTask */
__weak void StartUxTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /*
  cli_config = embeddedCliDefaultConfig();
  cli = embeddedCliNew(cli_config);
  cli -> writeChar = cli_writeChar;

  embeddedCliAddBinding(cli, cli_cmd_blink_binding);
  embeddedCliAddBinding(cli, cli_cmd_list_binding);
  embeddedCliAddBinding(cli, cli_cmd_define_binding);

  for(;;)
  {
	uint8_t c;
	HAL_StatusTypeDef status = HAL_UART_Receive(&huart2, &c, 1, 10);
	if (status == HAL_OK)
	{
	  embeddedCliReceiveChar(cli, c);
	  embeddedCliProcess(cli);
	}

	int key = BTN_SingleKeyDown();
	if (key >= 0)
	{
		const uint8_t digit[] = "0123456789";
		HAL_UART_Transmit(&huart2, (uint8_t*)"key:", 4, 10);
		HAL_UART_Transmit(&huart2, &digit[key], 1, 10);
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 10);

		do_profile_by_key(key);
	}

	// wait 10ms
	vTaskDelay(10);
  } */
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
