/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"
#include "usbd_cdc_if.h"
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef struct {
	uint8_t opcode;
	uint8_t arg1;
	uint8_t arg2;
} PlotCmd;
typedef enum {
	OP_RESET,
	OP_MOVE,
	OP_MOVE_PEN,
	OP_START_STORING,
	OP_STOP_STORING,
	OP_RUN_STORED,
	OP_LED,
	OP_STOP
} PlotCmdOpcode;
typedef enum {
	STATE_OPCODE,
	STATE_ARG1,
	STATE_ARG2
} PlotCmdParseState;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define STEPS_PER_MM 80
#define UART_RX_BUF_SIZE 64
#define MAX_STORED_PLOT_CMDS 16
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t VCP_RxBuffer[VCP_RX_BUFFER_SIZE];
volatile uint32_t VCP_RxWriteIndex = 0; // Written by USB ISR
volatile uint32_t VCP_RxReadIndex = 0;  // Read by application
volatile int targetPosStepsX = 0;
volatile int targetPosStepsY = 0;
volatile int posStepsX = 0;
volatile int posStepsY = 0;
volatile uint8_t rxBuf[UART_RX_BUF_SIZE];
volatile uint16_t rxBufIdx = 0;
volatile bool gotCommand = false;
bool storingPlotCmds = false;
uint32_t storingPlotCmdsIdx = 0;
PlotCmd plotCmdBuf[MAX_STORED_PLOT_CMDS] = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int mmToSteps(int mm)
{
	return mm * STEPS_PER_MM;
}

int stepsToMm(int steps)
{
	return steps / STEPS_PER_MM;
}

void USB_DEVICE_MasterHardReset(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12, 0);
	HAL_Delay(300);
}

// VCP = virtual com port
int VCP_ReadChar(uint8_t *pChar)
{
  if (VCP_RxReadIndex != VCP_RxWriteIndex) // Check if buffer is not empty
  {
    *pChar = VCP_RxBuffer[VCP_RxReadIndex];
    VCP_RxReadIndex = (VCP_RxReadIndex + 1) % VCP_RX_BUFFER_SIZE;
    return 1; // Character read successfully
  }
  return 0; // Buffer is empty
}

bool ParsePlotCmdByte(PlotCmd* cmd, PlotCmdParseState* state, uint8_t newByte)
{
	bool cmdCompleted = false;
	switch (*state)
	{
	case STATE_OPCODE:
		cmd->opcode = newByte;
		*state = STATE_ARG1;
		break;
	case STATE_ARG1:
		cmd->arg1 = newByte;
		*state = STATE_ARG2;
		break;
	case STATE_ARG2:
		cmd->arg2 = newByte;
		cmdCompleted = true;
		*state = STATE_OPCODE;
		break;
	default:
		break;
	}
	return cmdCompleted;
}

// returns false if overflowed (command storage isn't big enough)
bool storePlotCmd(PlotCmd cmd)
{
	if (storingPlotCmdsIdx > MAX_STORED_PLOT_CMDS)
	{
		storingPlotCmdsIdx = 0;
	}
	plotCmdBuf[storingPlotCmdsIdx] = cmd;
	storingPlotCmdsIdx++;
	bool overflowed = false;
	if (storingPlotCmdsIdx > MAX_STORED_PLOT_CMDS)
	{
		overflowed = true;
		storingPlotCmdsIdx = 0;
	}
	return !overflowed;
}

void DoPlotCmd(PlotCmd cmd)
{
	if (cmd.opcode == OP_STOP_STORING)
	{
		storingPlotCmds = false;
	}
	if (storingPlotCmds)
	{
		storePlotCmd(cmd);
		return;
	}
	switch (cmd.opcode)
	{
	case OP_LED:
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, cmd.arg1 != 0);
		break;
	case OP_MOVE:
		targetPosStepsX = mmToSteps(cmd.arg1);
		targetPosStepsY = mmToSteps(cmd.arg2);
		break;
	case OP_STOP:
		targetPosStepsX = posStepsX;
		targetPosStepsY = posStepsY;
		break;
	case OP_START_STORING:
		storingPlotCmds = true;
		break;
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
  USB_DEVICE_MasterHardReset();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
  {
	Error_Handler();
  }
  targetPosStepsX = 100;
  targetPosStepsY = 100;
  const char msg[] = "****** Incrediplotter v0.2 ******\r\n";
  CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
  const char hi[] = "hello world";
  CDC_Transmit_FS((uint8_t*)hi, strlen(hi));
  const char hi2[] = "another message";
  CDC_Transmit_FS((uint8_t*)hi2, strlen(hi2));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static char txBuf[100];
  static PlotCmd currentPlotCmd = { 0 };
  static PlotCmdParseState plotCmdParseState = STATE_OPCODE;
  while (1)
  {
//	sprintf(txBuf, "ugh %u\r\n", count);
//	count++;
//	CDC_Transmit_FS((uint8_t*)txBuf, strlen(txBuf));
//	HAL_Delay(100);
	uint8_t rxByte = 0;
	bool cmdCompleted = false;
	if (VCP_ReadChar(&rxByte))
	{
		//sprintf(txBuf, "Got 0x%02X\r\n", rxByte);
		//CDC_Transmit_FS((uint8_t*)txBuf, strlen(txBuf));
		PlotCmdParseState stateBefore = plotCmdParseState;
		cmdCompleted = ParsePlotCmdByte(&currentPlotCmd, &plotCmdParseState, rxByte);
		sprintf(txBuf, "Rx: %02X\r\nbefore: %u | after: %u\r\ncmdCompleted: %u\r\n---\r\n",
				rxByte, stateBefore, plotCmdParseState, cmdCompleted);
		CDC_Transmit_FS((uint8_t*)txBuf, strlen(txBuf));
	}
	if (cmdCompleted) {
		cmdCompleted = false;
		DoPlotCmd(currentPlotCmd);
	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void Periodic()
{
	bool shouldStepX = posStepsX != targetPosStepsX;
	bool dirX = targetPosStepsX > posStepsX;
	if (shouldStepX)
	{
		//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, !dirX); // note: flipped compared to y
		HAL_GPIO_WritePin(X_STEP_GPIO_Port, X_STEP_Pin, true);
		volatile int delayer = 10;
		while (delayer > 0) { delayer--; }
		HAL_GPIO_WritePin(X_STEP_GPIO_Port, X_STEP_Pin, false);
		if (dirX) {
			posStepsX++;
		} else {
			posStepsX--;
		}
	}
	bool shouldStepY = posStepsY != targetPosStepsY;
	bool dirY = targetPosStepsY > posStepsY;
	if (shouldStepY)
	{
		HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, dirY);
		HAL_GPIO_WritePin(Y_STEP_GPIO_Port, Y_STEP_Pin, true);
		volatile int delayer = 10;
		while (delayer > 0) { delayer--; }
		HAL_GPIO_WritePin(Y_STEP_GPIO_Port, Y_STEP_Pin, false);
		if (dirY) {
			posStepsY++;
		} else {
			posStepsY--;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
		Periodic();
	}
}

//uint8_t CDC_DataReceivedHandler(const uint8_t *Buf, uint32_t len)
//{
//	for (uint32_t i = 0; i < len; i++)
//	{
//		CDC_Transmit((const uint8_t*)"Got: ", 5);
//		CDC_Transmit(&Buf[i], 1);
//	}
//	return CDC_RX_DATA_HANDLED;
//
//}
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
