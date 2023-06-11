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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "stdbool.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int Xung = 1920;			//Xung = Xung/vong * 4 (Mode TI1 AND TI2)
float Ts = 0.05;			//Fs = (APB1 Timer Clocks) / (PSC + 1) = ... (KHz)
											//Ts = (1/Fs) * (Counter Period + 1) = ... (ms)

int BUFF_SIZE = 18;		//So byte trong 1 frame truyen
int ARR, CCR;

float setpoint, Kp, Ki, Kd;
float P_part, I_part, D_part;
float speed, speed_last, position, position_last;
float PID_output, PID_output_last;
float error, error_last, pre_error_last;

char Encoder_read[8];
char* endPtr;

int16_t counter_speed = 0, counter_speed_last = 0, counter_position = 0;

uint8_t nTxData[18];
uint8_t Tx_Index = 0;

uint8_t nRxData[18];
uint8_t Rx_Index = 0;
uint8_t Rx_data;
uint8_t strCommand[4];
uint8_t strOpt[3];
uint8_t strData[8];

uint8_t subData[8];

uint8_t STX[] = {0x02};													
uint8_t ETX[] = {0x03};													
uint8_t ACK[] = {0x06};													
uint8_t SYN[] = {0x16};

uint8_t LOAD[] = {0x4C, 0x4F, 0x41, 0x44};
uint8_t DEMO[] = {0x44, 0x45, 0x4D, 0x4F};
uint8_t STOP[] = {0x53, 0x54, 0x4F, 0x50};

uint8_t SPE[] = {0x53, 0x50, 0x45};
uint8_t POS[] = {0x50, 0x4F, 0x53};

uint8_t optSetpoint[] = {0x4F, 0x53, 0x70};
uint8_t optKp[] = {0x4F, 0x4B, 0x70};
uint8_t optKi[] = {0x4F, 0x4B, 0x69};
uint8_t optKd[] = {0x4F, 0x4B, 0x64};

uint8_t strMode[3];
uint8_t strSetpoint[8];
uint8_t strKp[8];
uint8_t strKi[8];
uint8_t strKd[8];

bool bDataAvailable = false;
bool bStart = false;
bool bRx_flag = false;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

bool StrCompare(uint8_t *pBuff, uint8_t *Sample, uint8_t nSize)
{
	for (int i=0; i< nSize; i++)
	{
		if (pBuff[i] != Sample[i])
		{
			return false;
		}
	}
	return true;
}
uint8_t *subString(uint8_t *s, int pos, int index)
{
	memset(subData,0,sizeof(subData));
	int j = pos;
	int k = pos + index;
	for (int i = pos; i < k; i++)
	{
		subData[i-j] = s[i];
	}
	return subData;
}
bool WriteComm(uint8_t *pBuff, uint8_t nSize)
{
	return HAL_UART_Transmit(&huart1, pBuff, nSize, 1000);
}
void PWM_Set_Duty(int Duty)
{
	if(Duty > 0)
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET); // IN1 = 1
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET); // IN2 = 0
	}
	else if(Duty < 0)
	{
		Duty = Duty*(-1);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET); // IN1 = 0
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET); // IN2 = 1
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET); // IN1 = 0
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET); // IN2 = 0
	}
	ARR = TIM1->ARR;
	CCR = Duty * (htim1.Instance->ARR)/100;
	htim1.Instance->CCR1 = (int)CCR;
}
void Serial_Process(void)
{
	Tx_Index = 0;
	if(StrCompare(strCommand, LOAD, 4))
	{
		if(StrCompare(strOpt, SPE, 3))
		{
			memcpy(nTxData + Tx_Index, STX, 1);
			Tx_Index +=1;
			memcpy(nTxData + Tx_Index, (uint8_t *)strCommand, 4);
			Tx_Index +=4;
			memcpy(nTxData + Tx_Index, (uint8_t *)strOpt, 3);
			Tx_Index +=3;
			memcpy(nTxData + Tx_Index, (uint8_t *)strData, 8);
			Tx_Index +=8;
			memcpy(nTxData + Tx_Index, ACK, 1);
			Tx_Index +=1;
			memcpy(nTxData + Tx_Index, ETX, 1);

			WriteComm(nTxData, BUFF_SIZE);
			
			memcpy(strMode, strOpt, 3);
		}
		else if(StrCompare(strOpt, POS, 3))
		{
			memcpy(nTxData + Tx_Index, STX, 1);
			Tx_Index +=1;
			memcpy(nTxData + Tx_Index, (uint8_t *)strCommand, 4);
			Tx_Index +=4;
			memcpy(nTxData + Tx_Index, (uint8_t *)strOpt, 3);
			Tx_Index +=3;
			memcpy(nTxData + Tx_Index, (uint8_t *)strData, 8);
			Tx_Index +=8;
			memcpy(nTxData + Tx_Index, ACK, 1);
			Tx_Index +=1;
			memcpy(nTxData + Tx_Index, ETX, 1);

			WriteComm(nTxData, BUFF_SIZE);
			
			memcpy(strMode, strOpt, 3);
		}
		else if(StrCompare(strOpt, optSetpoint , 3))
		{
			memcpy(strSetpoint, strData, 8);
			setpoint = atoi((char *)strSetpoint);
		}
		else if(StrCompare(strOpt, optKp, 3))
		{
			memcpy(strKp, strData, 8);
			Kp = strtof((char *)strKp, &endPtr);
		}
		else if(StrCompare(strOpt, optKi, 3))
		{
			memcpy(strKi, strData, 8);
			Ki = strtof((char *)strKi, &endPtr);
		}
		else if(StrCompare(strOpt, optKd, 3))
		{
			memcpy(strKd, strData, 8);
			Kd = strtof((char *)strKd, &endPtr);
		}
	}
	else if(StrCompare(strCommand, DEMO, 4))
	{
		memcpy(nTxData + Tx_Index, STX, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, (uint8_t *)strCommand, 4);
		Tx_Index +=4;
		memcpy(nTxData + Tx_Index, (uint8_t *)strOpt, 3);
		Tx_Index +=3;
		memcpy(nTxData + Tx_Index, (uint8_t *)strData, 8);
		Tx_Index +=8;
		memcpy(nTxData + Tx_Index, ACK, 1);
		Tx_Index +=1;
		memcpy(nTxData + Tx_Index, ETX, 1);

		WriteComm(nTxData, BUFF_SIZE);
		
		bStart = true;
	}
	else if(StrCompare(strCommand, STOP, 4))
	{
		bStart = false;
	}
}
bool ReadComm(uint8_t *pBuff, uint8_t nSize)
{
	if ((pBuff[0] == STX[0]) && (pBuff[17] == ETX[0]))
	{
		memcpy(strCommand, subString(pBuff, 1, 4), 4);
		memcpy(strOpt, subString(pBuff, 5, 3), 3);
		memcpy(strData, subString(pBuff, 8, 8), 8);
		
		bDataAvailable = true;
	}
	else
	{
		bDataAvailable = false;
	}
	return bDataAvailable;
}
void PID_Speed_Process(void)
{	
	speed = (float)counter_speed*60.0f/(Xung*Ts);
	error = setpoint - speed;
	
	P_part = error;
	I_part += error*Ts;
	D_part = (error - error_last)/Ts;
	
	PID_output = Kp*P_part + Ki*I_part + Kd*D_part;
	
	if(PID_output > 99) PID_output = 99;
	else if(PID_output < -99) PID_output = -99;
	
	speed_last = speed;
	pre_error_last = error_last;
	error_last = error;
	PID_output_last = PID_output;
	
	PWM_Set_Duty((int)PID_output);
}
void PID_Position_Process(void)
{	
	position = (float)counter_position*360/Xung;
	error = setpoint - position;
	
	P_part = error;
	I_part += error*Ts;
	D_part = (error - error_last)/Ts;
	
	PID_output = Kp*P_part + Ki*I_part + Kd*D_part;
	
	if(PID_output > 90) PID_output = 90;
	else if(PID_output < -90) PID_output = -90;
	
	pre_error_last = error_last;
	error_last = error;
	PID_output_last = PID_output;
	
	PWM_Set_Duty((int)PID_output);
}
void Encoder(void)
{
	if(bStart == true)
	{
		counter_speed = htim2.Instance->CNT - counter_speed_last;
		counter_speed_last = htim2.Instance->CNT;
		counter_position += counter_speed;
	}
	else if(bStart == false)
	{
		htim2.Instance->CNT = 0;
		counter_speed = 0;
		counter_speed_last = 0;
		counter_position = 0;
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim3.Instance)
	{		
		if(bStart == true)
		{
			Encoder();
			memset(Encoder_read,0,sizeof(Encoder_read));
			memset(strData,0,sizeof(strData));
			if(StrCompare(strMode,SPE,3) == true)
			{
				sprintf(Encoder_read,"%d",counter_speed);
				memcpy(strData,Encoder_read,8);
				Serial_Process();
			}
			else if(StrCompare(strMode,POS,3) == true)
			{
				sprintf(Encoder_read,"%d",counter_position);
				memcpy(strData,Encoder_read,8);
				Serial_Process();
			}
		}
		else if(bStart == false)
		{
			memset(Encoder_read,0,sizeof(Encoder_read));
			memset(strData,0,sizeof(strData));
		}
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart1.Instance)
	{
		if(Rx_data != 0x03)
			nRxData[Rx_Index++] = Rx_data;
		else if(Rx_data == 0x03)
		{
			nRxData[17] = Rx_data;
			bRx_flag = true;
		}
		HAL_UART_Receive_IT(&huart1,&Rx_data,1);
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1,&Rx_data,1);	
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(bRx_flag == true)
		{
			ReadComm(nRxData, BUFF_SIZE);
			bRx_flag = false;
			Rx_Index = 0;
		}
		if(bDataAvailable == true)
		{
			Serial_Process();
			bDataAvailable = false;
		}
		if(bStart == true)
		{
			if(StrCompare(strMode,SPE,3) == true)
			{
				PID_Speed_Process();
			}
			else if(StrCompare(strMode,POS,3) == true)
			{
				PID_Position_Process();
			}
		}
		else
		{	
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET); // IN1 = 0
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET); // IN2 = 0
			
			TIM2->CNT = 0;
			counter_speed = 0;
			counter_speed_last = 0;
			counter_position = 0;
	
			error = 0;
			error_last = 0;
			pre_error_last = 0;
			
			PID_output = 0;
			PID_output_last = 0;
	
			P_part = 0;
			I_part = 0;
			D_part = 0;
	
			speed = 0;
			position = 0;
			
			ARR = 0;
			CCR = 0;
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
