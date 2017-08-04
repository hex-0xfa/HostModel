/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define RXBUFFERSIZE 256
uint8_t aRxBuffer[RXBUFFERSIZE];
uint8_t Device_Address = 0xA1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int  fputc(int ch, FILE *f);
void Myscanf(char* stringPointer);                          //把输入的字符串复制到字符串 inputString 中
void I2CResponse(char* stringPointer);                      //根据字符串的内容对寄存器做读写操作并给予反馈
void ResetToNULL(char* stringPointer, int Length);          //清空字符串  
void I2C_Read_Mode(char* stringPointer);                    //读取模式
void I2C_Write_Mode(char* stringPointer);                   //写入模式
void I2C_Memmap_Mode(void);                                 //Memmap 模式
void I2C_Init_LD_BM_Mode(void);
void I2C_Address_Change(char* stringPointer);
void I2C_ModSet(void);
void I2C_ModNSet(void);
void Module_Reset(void);
void IsModluePresent(void);
void IsintL(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  char inputString[32] = {""};
	printf("\n------------------------------------------------------------------------------------------------------------\n");
	printf("Welcoome to the debug mode for GN2010, input command from the terminal according to the format given below\n");
	printf("Read a register : r(space)(register address in decimal)(space)\n");
	printf("Write a register : w(space)(register address in decimal)(space)(written value in hexadecimal)(space)\n");
	printf("Print the Memory Map: m\n");
	printf("Initiate: i\n");
	printf("Change the memory address: a(space)(register address in hexadecimal)(space)\n");
	printf("------------------------------------------------------------------------------------------------------------\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		Myscanf(inputString);
		I2CResponse(inputString);
		ResetToNULL(inputString,32);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ResetL_GPIO_Port, ResetL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ModSetL_Pin|LPMode_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ResetL_Pin */
  GPIO_InitStruct.Pin = ResetL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ResetL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ModSetL_Pin LPMode_Pin */
  GPIO_InitStruct.Pin = ModSetL_Pin|LPMode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IntL_Pin ModPrsL_Pin */
  GPIO_InitStruct.Pin = IntL_Pin|ModPrsL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)
{
   HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 2);
	 return ch;
}
void Myscanf(char* stringPointer)
{
	printf("->");
	int i = 0;
	uint8_t u;
	while(1)
	{
	  while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_RXNE)!=SET);
		 HAL_UART_Receive(&huart2, &u, 1, 2);
		 if(u == 0x0D)
		 {
			 break;                 //回车键未被保存
		 }
		 else if(u == 0x08)
		 {
			 i--;
		 }
		 else
		 {
			 printf("%c",u);
			 stringPointer[i]= u;
			 i++;
		 }
	 }
   stringPointer[i] = 0x00;
}
void I2CResponse(char* stringPointer)
{
	if(stringPointer[0]=='r')
	{
		I2C_Read_Mode(stringPointer);
	}
	else if(stringPointer[0]=='w')
	{
	  I2C_Write_Mode(stringPointer);
	}
	else if((stringPointer[0] == 'm') && (stringPointer[1] == 'm'))
	{
		I2C_Memmap_Mode();
	}
	else if(stringPointer[0] == 'i')\
	{
		I2C_Init_LD_BM_Mode();
	}
	else if(stringPointer[0] == 'a')
	{
		I2C_Address_Change(stringPointer);
	}
		else if((stringPointer[0] == 'd') && (stringPointer[1] == 's'))
	{
		I2C_ModSet();
	}
	else if((stringPointer[0] == 'd') && (stringPointer[1] == 'n'))
	{
		I2C_ModNSet();
	}
	else if((stringPointer[0] == 's') && (stringPointer[1] == 'e'))
	{
		Module_Reset();
	}
	else if(stringPointer[0] == 't')
	{
		IsintL();
	}
	else
	{
		printf("Unlawful command\n");
	}
}
void ResetToNULL(char* stringPointer, int Length)
{
	int j = 0;
	while(j < Length)
	{
		stringPointer[j] = 0x00;
		j++;
	}
}
void I2C_Read_Mode(char* stringPointer)
{
	uint8_t regValue = 0x00;
	int ten = 1;
	uint16_t regAddress = 0;
	char numberString[6] = {""};
  int i = 2;
	while(1)
	{
		if(stringPointer[i] == 0x00)
		{
			numberString[i-2] = 0x00;
			break;
		}
		else                       //以后可以加一个输入保护措施
		{
			numberString[i-2] = stringPointer[i];
		}
		i++;
	}
	i--;
	while(i >= 2)
	{
		regAddress = regAddress + (numberString[i-2]-0x30)*ten;
		ten = ten*10;
		i--;
	}
  uint8_t status = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)Device_Address,regAddress,I2C_MEMADD_SIZE_8BIT,&regValue,1,1000);
	if(status == 0)
	{
	  printf("Value of register %d is 0x%02X\n", regAddress, regValue);
	}
	else
	{
		printf("Read Error.\n");
	}
}
void I2C_Write_Mode(char* stringPointer)
{
  uint8_t regValue = 0x00;
	int ten = 1;
	uint16_t regAddress = 0;
	char numberString[6] = {""};
  int i = 2;
	while(1)
	{
		if(stringPointer[i] == 0x20)
		{
			numberString[i-2] = 0x00;
			break;
		}
		else                       //以后可以加一个输入保护措施
		{
			numberString[i-2] = stringPointer[i];
		}
		i++;
	}
	int j = i + 1;
	i--;
	while(i >= 2)
	{
		regAddress = regAddress + (numberString[i-2]-0x30)*ten;
		ten = ten*10;
		i--;
	}
	int k;
	int hex = 16;
	for(k = 0; k<2; k++)
	{
	  if((stringPointer[j+k] >= 0x30)&&(stringPointer[j+k] <= 0x39))
		 {
			 regValue = regValue + (stringPointer[j+k]-0x30)*hex;
		 }
		 else if((stringPointer[j+k] >= 0x41)&&(stringPointer[j+k] <= 0x46))
		 {
			 regValue = regValue + (stringPointer[j+k]-55)*hex;
		 }
		 else
		 {
			 printf("unlawful command\n");
		 }
		 hex = hex / 16;
	}
  uint8_t status = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)Device_Address,(uint16_t)regAddress,I2C_MEMADD_SIZE_8BIT,&regValue,1,1000);
	if(status == 0)
	{
	  printf("Value of register %d has been changed to 0x%02X\n", regAddress, regValue);
	}
	else
	{
		printf("Write Error.\n");
	}
}
/**
  * @}
*/ 
void I2C_Memmap_Mode(void)
{
		uint8_t status = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)Device_Address,(uint16_t)0,I2C_MEMADD_SIZE_8BIT,aRxBuffer,RXBUFFERSIZE,1000);
		if(status != 0)
		{
			printf("Read Error.\n");
			return;
		}
	  printf("Memory Map:\n");
	  printf("    0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F   \n");
	  int i = 0;
	  int j = 0;
    while(i <= 15)                        //Changed
		{
			printf("%X   ",i);
			j = 0;
	    while(j <= 15)
			{
	       printf("%02X  ",aRxBuffer[i*16+j]);
				 j++;
      }
			printf("\n");
			i++;
		}
}
void I2C_Init_LD_BM_Mode(void)
{
	 uint8_t a[8]= {0x30, 0x11, 0x40, 0x00,0x04,0x05,0x00,0x01};
   HAL_I2C_Mem_Write(&hi2c1, (uint16_t)Device_Address,14,I2C_MEMADD_SIZE_8BIT,a,1,1000);
	 HAL_I2C_Mem_Write(&hi2c1, (uint16_t)Device_Address,100,I2C_MEMADD_SIZE_8BIT,a+1,1,1000);
	 HAL_I2C_Mem_Write(&hi2c1, (uint16_t)Device_Address,113,I2C_MEMADD_SIZE_8BIT,a+2,1,1000);
	 HAL_I2C_Mem_Write(&hi2c1, (uint16_t)Device_Address,106,I2C_MEMADD_SIZE_8BIT,a+3,1,1000);
	 HAL_I2C_Mem_Write(&hi2c1, (uint16_t)Device_Address,107,I2C_MEMADD_SIZE_8BIT,a+4,1,1000);
	 HAL_I2C_Mem_Write(&hi2c1, (uint16_t)Device_Address,107,I2C_MEMADD_SIZE_8BIT,a+5,1,1000);
	 HAL_I2C_Mem_Write(&hi2c1, (uint16_t)Device_Address,87,I2C_MEMADD_SIZE_8BIT,a+6,1,1000);
	 HAL_I2C_Mem_Write(&hi2c1, (uint16_t)Device_Address,88,I2C_MEMADD_SIZE_8BIT,a+7,1,1000);
	 printf("Initilization Finished\n");;
}

void I2C_Address_Change(char* stringPointer)
{
	int devAddress = 0;
	int k;
	int hex = 16;
	for(k = 0; k<2; k++)
	{
	  if((stringPointer[k+2] >= 0x30)&&(stringPointer[k+2] <= 0x39))
		 {
			 devAddress = devAddress + (stringPointer[k+2]-0x30)*hex;
		 }
		 else if((stringPointer[k+2] >= 0x41)&&(stringPointer[k+2] <= 0x46))
		 {
			 devAddress = devAddress + (stringPointer[k+2]-55)*hex;
		 }
		 else
		 {
			 printf("unlawful command\n");
		 }
		 hex = hex / 16;
	}
	Device_Address = devAddress;
	printf("The address has been changed to 0x%02X\n",Device_Address);
}
void I2C_ModSet(void)
{
	HAL_GPIO_WritePin(ModSetL_GPIO_Port, ModSetL_Pin, GPIO_PIN_RESET);
	printf("Module has been selected\n");
  return;
}
void I2C_ModNSet(void)
{
	HAL_GPIO_WritePin(ModSetL_GPIO_Port, ModSetL_Pin, GPIO_PIN_SET);
	printf("Module has been deselected\n");
  return;
}
void Module_Reset(void)
{
	HAL_GPIO_WritePin(ResetL_GPIO_Port, ResetL_Pin, GPIO_PIN_RESET);
	HAL_Delay(2);
	HAL_GPIO_WritePin(ResetL_GPIO_Port, ResetL_Pin, GPIO_PIN_SET);
	printf("Reset finished.\n");
}
void IsintL(void)
{
	uint8_t u =HAL_GPIO_ReadPin(IntL_GPIO_Port, IntL_Pin);
	if(u == 0)
	{
		printf("interrupt asserted\n");
	}
	else
	{
		printf("interrupt deasserted\n");
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
