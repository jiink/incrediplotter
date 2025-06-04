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
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
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
	uint8_t arg3;
	uint8_t arg4;
} PlotCmd;
typedef enum {
	OP_RESET,
	OP_MOVE,
	OP_PEN_DOWN,
	OP_PEN_UP,
	OP_SET_PEN_DOWN,
	OP_SET_PEN_UP,
	OP_MOVE_PEN,
	OP_HOME,
	OP_START_STORING,
	OP_STOP_STORING,
	OP_RUN_STORED,
	OP_SHOW_STORED,
	OP_LED,
	OP_STOP,
	OP_SET_X_MAX_STEPS,
	OP_SET_Y_MAX_STEPS,
	OP_SET_STEPS_PER_MM,
	OP_DEBUG,
	OP_MAX
} PlotCmdOpcode;
const uint8_t opcodeArgCounts[OP_MAX] = {
    [OP_RESET]         = 0,
    [OP_MOVE]          = 2, // x y
	[OP_PEN_DOWN] = 0,
	[OP_PEN_UP] = 0,
	[OP_SET_PEN_DOWN] = 1,
	[OP_SET_PEN_UP] = 1,
	[OP_MOVE_PEN] = 1,
	[OP_HOME] = 0,
	[OP_START_STORING] = 0,
	[OP_STOP_STORING] = 0,
	[OP_RUN_STORED] = 0,
	[OP_SHOW_STORED] = 0,
	[OP_LED] = 1,
	[OP_STOP] = 0,
	[OP_SET_X_MAX_STEPS] = 2, // high low
	[OP_SET_Y_MAX_STEPS] = 0, // high low
	[OP_SET_STEPS_PER_MM] = 0,
	[OP_DEBUG] = 1
};
typedef enum {
	STATE_OPCODE,
	STATE_ARG1,
	STATE_ARG2,
	STATE_ARG3,
	STATE_ARG4
} PlotCmdParseState;
typedef struct {
	int32_t x;
	int32_t y;
} Vec2i;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define UART_BUF_SIZE 64
#define PLOT_CMD_BUF_SIZE 1024
#define DELIMITER 0xFF
#define CMD_ACK_MSG "OK\r\n"
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t VCP_RxBuffer[VCP_RX_BUFFER_SIZE];
volatile uint32_t VCP_RxWriteIndex = 0; // Written by USB ISR
volatile uint32_t VCP_RxReadIndex = 0;  // Read by application
volatile Vec2i targetPosSteps = {0, 0};
volatile Vec2i posSteps = {0, 0};
volatile uint8_t rxBuf[UART_BUF_SIZE];
volatile uint16_t rxBufIdx = 0;
volatile bool gotCommand = false;
bool storingPlotCmds = false;
uint32_t storingPlotCmdsIdx = 0;
uint8_t plotCmdBuf[PLOT_CMD_BUF_SIZE] = { 0 };
volatile Vec2i bresDiff = {0, 0};
volatile Vec2i bresDir = {0, 0};
volatile int32_t bresErr = 0;
volatile bool bresLineActive = false;
int stepsPerMm = 41;
Vec2i posMaxSteps = {6960, 6960};
int penUpPulseWidth = 10;
int penDownPulseWidth = 18;
bool debugPrints = false;
volatile bool justFinishedCmd = false;
bool runningStoredPlotCmds = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DoPlotCmd(PlotCmd cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int mmToSteps(int mm)
{
	return mm * stepsPerMm;
}

int stepsToMm(int steps)
{
	return steps / stepsPerMm;
}

void Log(const char* fmt, ...)
{
    if (fmt == NULL) {
        return;
    }
    static char log_buffer[UART_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(log_buffer, UART_BUF_SIZE, fmt, args);
    va_end(args);
    if (len < 0) {
        const char* error_msg = "Log format error\n";
        CDC_Transmit_FS((uint8_t*)error_msg, strlen(error_msg));
        return;
    }
    if (len > 0) {
        uint16_t chars_to_send = (len < UART_BUF_SIZE) ? (uint16_t)len : (uint16_t)(UART_BUF_SIZE - 1);

        if (chars_to_send > 0) {
            CDC_Transmit_FS((uint8_t*)log_buffer, chars_to_send);
        }

        if (len >= UART_BUF_SIZE) {
            const char* truncation_msg = "...\n";
            CDC_Transmit_FS((uint8_t*)truncation_msg, strlen(truncation_msg));
        }
    }
}

uint8_t GetOpcodeArgCount(PlotCmdOpcode opcode)
{
    if ((unsigned)opcode < OP_MAX) {
        return opcodeArgCounts[opcode];
    } else {
        return 0xFF;
    }
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

void SetPenServoPulseWidth(int microseconds)
{
	if (microseconds <= 0)
	{
		// disable the servo
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
		return;
	}
	if (microseconds < 500)
	{
		microseconds = 500;
	}
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, microseconds);
}

bool ParsePlotCmdByte(PlotCmd* cmd, PlotCmdParseState* state, uint8_t newByte)
{
	if (newByte == DELIMITER)
	{
		*state = STATE_OPCODE;
		return false;
	}
	bool cmdCompleted = false;
	uint8_t argCount = GetOpcodeArgCount(cmd->opcode);
	switch (*state)
	{
	case STATE_OPCODE:
		cmd->opcode = newByte;
		argCount = GetOpcodeArgCount(cmd->opcode);
		if (argCount == 0) {
			*state = STATE_OPCODE;
			cmdCompleted = true;
		} else {
			*state = STATE_ARG1;
		}
		break;
	case STATE_ARG1:
		cmd->arg1 = newByte;
		if (argCount == 1) {
			*state = STATE_OPCODE;
			cmdCompleted = true;
		} else {
			*state = STATE_ARG2;
		}
		break;
	case STATE_ARG2:
		cmd->arg2 = newByte;
		if (argCount == 2) {
			*state = STATE_OPCODE;
			cmdCompleted = true;
		} else {
			*state = STATE_ARG3;
		}
		break;
	case STATE_ARG3:
		cmd->arg3 = newByte;
		if (argCount == 3) {
			*state = STATE_OPCODE;
			cmdCompleted = true;
		} else {
			*state = STATE_ARG4;
		}
		break;
	case STATE_ARG4:
		cmd->arg4 = newByte;
		*state = STATE_OPCODE;
		cmdCompleted = true;
		break;
	default:
		*state = STATE_OPCODE;
		break;
	}
	return cmdCompleted;
}

// returns false if overflowed  (command storage isn't big enough)
// or invalid opcode
bool StorePlotCmd(PlotCmd cmd)
{
	if (storingPlotCmdsIdx > PLOT_CMD_BUF_SIZE - 1)
	{
		Log("STORAGE: OVERFLOW! (0)\r\n");
		return false;
	}
	if (cmd.opcode >= OP_MAX)
	{
		Log("STORAGE: INVALID OPCODE!\r\n");
		return false;
	}
	uint8_t numArgs = GetOpcodeArgCount(cmd.opcode);
	uint8_t numBToStore = 1 + numArgs;
	if (storingPlotCmdsIdx + numBToStore > PLOT_CMD_BUF_SIZE - 1)
	{
		Log("STORAGE: OVERFLOW! (1)\r\n");
		return false;
	}
	plotCmdBuf[storingPlotCmdsIdx] = cmd.opcode;
	switch (numArgs)
	{
	case 0:
		break;
	case 1:
		plotCmdBuf[storingPlotCmdsIdx + 1] = cmd.arg1;
		break;
	case 2:
		plotCmdBuf[storingPlotCmdsIdx + 1] = cmd.arg1;
		plotCmdBuf[storingPlotCmdsIdx + 2] = cmd.arg2;
		break;
	case 3:
		plotCmdBuf[storingPlotCmdsIdx + 1] = cmd.arg1;
		plotCmdBuf[storingPlotCmdsIdx + 2] = cmd.arg2;
		plotCmdBuf[storingPlotCmdsIdx + 3] = cmd.arg3;
		break;
	case 4:
		plotCmdBuf[storingPlotCmdsIdx + 1] = cmd.arg1;
		plotCmdBuf[storingPlotCmdsIdx + 2] = cmd.arg2;
		plotCmdBuf[storingPlotCmdsIdx + 3] = cmd.arg3;
		plotCmdBuf[storingPlotCmdsIdx + 4] = cmd.arg4;
		break;
	default:
		break;
	}
	storingPlotCmdsIdx += numBToStore;
	bool overflowed = false;
	if (storingPlotCmdsIdx > PLOT_CMD_BUF_SIZE)
	{
		overflowed = true;
		storingPlotCmdsIdx = 0;
	}
	return !overflowed;
}

void LogPlotCmdStorage()
{
	int argCounter = 0;
	for (int i = 0; i < storingPlotCmdsIdx; i++)
	{
		uint8_t b = plotCmdBuf[i];
		Log("%02X ", b);
		if (argCounter == 0)
		{
			argCounter = GetOpcodeArgCount(b);
			if (argCounter == 0) {
				Log("\r\n");
			}
		} else {
			argCounter--;
			if (argCounter == 0) {
				Log("\r\n");
			}
		}
	}
}

// Called when new target position is set
void SetupLineMovement(int x, int y)
{
	targetPosSteps.x = x;
	targetPosSteps.y = y;
	if (targetPosSteps.x > posMaxSteps.x) {
		targetPosSteps.x = posMaxSteps.x;
	}
	if (targetPosSteps.y > posMaxSteps.y) {
		targetPosSteps.y = posMaxSteps.y;
	}
	if (posSteps.x == targetPosSteps.x && posSteps.y == targetPosSteps.y)
	{
		justFinishedCmd = true;
		Log(CMD_ACK_MSG);
		bresLineActive = false;
		return;
	}
	bresDiff.x = abs(targetPosSteps.x - posSteps.x);
	bresDir.x = (posSteps.x < targetPosSteps.x) ? 1 : -1;
	bresDiff.y = abs(targetPosSteps.y - posSteps.y);
	bresDir.y = (posSteps.y < targetPosSteps.y) ? 1 : -1;
	if (bresDiff.x > bresDiff.y)
	{
		bresErr = (2 * bresDiff.y) - bresDiff.x;
	} else {
		bresErr = (2 * bresDiff.x) - bresDiff.y;
	}
	bresLineActive = true;
	// X needs to be flipped because of the wiring at the present time
	HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, bresDir.x > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, bresDir.y > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Stop()
{
	targetPosSteps.x = posSteps.x;
	targetPosSteps.y = posSteps.y;
	bresLineActive = false;
}

// true on success
bool RunStoredPlotCmds(uint8_t* plotCmdBuf, size_t maxIdx)
{
	if (runningStoredPlotCmds)
	{
		return false;
	}
	runningStoredPlotCmds = true;
	size_t idx = 0;
	while (idx < maxIdx) {
		PlotCmd cmdToRun = { 0 };
		cmdToRun.opcode = plotCmdBuf[idx];
		idx++;
		if (cmdToRun.opcode >= OP_MAX)
		{
			Log("PLAYBACK: INVALID OPCODE!\r\n");
			runningStoredPlotCmds = false;
			return false;
		}
		uint8_t numArgs = GetOpcodeArgCount(cmdToRun.opcode);
		if (idx + numArgs > maxIdx)
		{
			Log("PLAYBACK: OVERRUN!\r\n");
			runningStoredPlotCmds = false;
			return false;
		}
		if (numArgs > 0) {
			cmdToRun.arg1 = plotCmdBuf[idx++];
		}
		if (numArgs > 1) {
			cmdToRun.arg2 = plotCmdBuf[idx++];
		}
		if (numArgs > 2) {
			cmdToRun.arg3 = plotCmdBuf[idx++];
		}
		if (numArgs > 3) {
			cmdToRun.arg4 = plotCmdBuf[idx++];
		}
		justFinishedCmd = false;
		DoPlotCmd(cmdToRun);
		int c = 0;
		while (!justFinishedCmd)
		{
			HAL_Delay(1);
			c++;
			if (c > 8000)
			{
				Log("Timed out.\r\n");
				break;
			}
		}
		justFinishedCmd = false;
	}
	runningStoredPlotCmds = false;
	Log("Done.\r\n");
	return true;
}

void LogPlotCmd(PlotCmd cmd)
{
	Log("Op: %02X args: %02X %02X %02X %02X\r\n",
		cmd.opcode, cmd.arg1, cmd.arg2, cmd.arg3, cmd.arg4);
}

void DoPlotCmd(PlotCmd cmd)
{
	if (cmd.opcode >= OP_MAX)
	{
		Log("INVALID OPCODE!\r\n");
		return;
	}
	if (debugPrints)
	{
		LogPlotCmd(cmd);
	}
	if (cmd.opcode == OP_STOP_STORING)
	{
		Log("Storing stopped.");
		storingPlotCmds = false;
	}
	if (storingPlotCmds)
	{
		StorePlotCmd(cmd);
		return;
	}
	bool immediateReply = true;
	switch (cmd.opcode)
	{
	case OP_MOVE:
		SetupLineMovement(mmToSteps(cmd.arg1), mmToSteps(cmd.arg2));
		immediateReply = false;
		break;
	case OP_PEN_DOWN:
		SetPenServoPulseWidth(penDownPulseWidth);
		break;
	case OP_PEN_UP:
		SetPenServoPulseWidth(penUpPulseWidth);
		break;
	case OP_SET_PEN_DOWN:
		penDownPulseWidth = (int)cmd.arg1 * 100;
		break;
	case OP_SET_PEN_UP:
		penUpPulseWidth = (int)cmd.arg1 * 100;
		break;
	case OP_MOVE_PEN:
		SetPenServoPulseWidth((int)cmd.arg1 * 100);
		break;
	case OP_HOME:
		// todo
		break;
	case OP_START_STORING:
		storingPlotCmdsIdx = 0;
		storingPlotCmds = true;
		Log("Storing started.");
		break;
	case OP_STOP_STORING:
		// Handled before this switch statement.
		break;
	case OP_RUN_STORED:
		RunStoredPlotCmds(plotCmdBuf, storingPlotCmdsIdx);
		break;
	case OP_SHOW_STORED:
		LogPlotCmdStorage();
		break;
	case OP_LED:
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, cmd.arg1 != 0);
		break;
	case OP_STOP:
		Stop();
		break;
	case OP_SET_X_MAX_STEPS:
		posMaxSteps.x = (cmd.arg1 << 8) | cmd.arg2;
		break;
	case OP_SET_Y_MAX_STEPS:
		posMaxSteps.y = (cmd.arg1 << 8) | cmd.arg2;
		break;
	case OP_SET_STEPS_PER_MM:
		stepsPerMm = cmd.arg1;
		break;
	case OP_DEBUG:
		debugPrints = (bool)cmd.arg1;
		if (debugPrints)
		{
			Log("Debug prints on.");
		} else {
			Log("Debug prints off.");
		}
		break;
	default:
		break;
	}
	if (immediateReply)
	{
		justFinishedCmd = true;
		Log(CMD_ACK_MSG);
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
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4))
  {
	Error_Handler();
  }
  targetPosSteps.x = 0;
  targetPosSteps.y = 0;
  const char msg[] = "****** Incrediplotter v0.2 ******\r\n";
  CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
  const char hi[] = "hello world";
  CDC_Transmit_FS((uint8_t*)hi, strlen(hi));
  const char hi2[] = "another message";
  CDC_Transmit_FS((uint8_t*)hi2, strlen(hi2));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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
		if (debugPrints)
		{
			Log("Rx: %02X\r\nbefore: %u | after: %u\r\ncmdCompleted: %u\r\n---\r\n",
					rxByte, stateBefore, plotCmdParseState, cmdCompleted);
		}
	}
	if (cmdCompleted) {
		cmdCompleted = false;
		DoPlotCmd(currentPlotCmd);
	}
	if (justFinishedCmd)
	{
		justFinishedCmd = false;
		Log(CMD_ACK_MSG);
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
	if (!bresLineActive)
	{
		return;
	}
	if (posSteps.x == targetPosSteps.x &&
		posSteps.y == targetPosSteps.y)
	{
		justFinishedCmd = true;
		bresLineActive = false;
		return;
	}
	bool stepX = false;
	bool stepY = false;
	if (bresDiff.x > bresDiff.y)
	{ // Gentle slope
		if (bresErr >= 0)
		{
			stepY = true;
			bresErr -= 2 * bresDiff.x;
		}
		stepX = true;
		bresErr += 2 * bresDiff.y;
	} else { // Steep slope
		if (bresErr >= 0)
		{
			stepX = true;
			bresErr -= 2 * bresDiff.y;
		}
		stepY = true;
		bresErr += 2 * bresDiff.x;
	}
	if (posSteps.x + bresDir.x < 0 ||
			posSteps.x + bresDir.x > posMaxSteps.x)
	{
		stepX = false;
	}
	if (posSteps.y + bresDir.y < 0 ||
				posSteps.y + bresDir.y > posMaxSteps.y)
	{
		stepY = false;
	}
	if (stepX)
	{
		posSteps.x += bresDir.x;
		HAL_GPIO_WritePin(X_STEP_GPIO_Port, X_STEP_Pin, GPIO_PIN_SET);
		volatile int delayer = 10; while (delayer > 0) { delayer--; }
		HAL_GPIO_WritePin(X_STEP_GPIO_Port, X_STEP_Pin, GPIO_PIN_RESET);
	}
	if (stepY)
	{

		posSteps.y += bresDir.y;
		HAL_GPIO_WritePin(Y_STEP_GPIO_Port, Y_STEP_Pin, GPIO_PIN_SET);
		volatile int delayer = 10; while (delayer > 0) { delayer--; }
		HAL_GPIO_WritePin(Y_STEP_GPIO_Port, Y_STEP_Pin, GPIO_PIN_RESET);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
		Periodic();
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
