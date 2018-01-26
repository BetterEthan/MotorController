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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void USER_PWM_SetDutyRatio(TIM_HandleTypeDef *htim,uint32_t Channel,uint8_t value);

void ResetAB_Pin_High(void);
void ResetAB_Pin_Low(void);

static void ADS8684_Init(void);
static void ADS8684_RST(void);
static void ADS8684_STDBY(void);
static void ADS8684_Range_Select(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


extern _Bool countTim;
float count = 0;
_Bool flag = 0;
int percentPWM = 10;
int testt = 0;
int SYSCLOCK = 0;
int HCLKFRQ = 0;
int PCLK1 = 0;
int PCLK2 = 0;
int OTW_INFO = 99;
int FAULT_INFO = 99;



/*-------------- ADS8684 reg define --------------*/
uint8_t Received_DATA0[4] = {0x00,0x00,0x00,0x00};
uint8_t Received_DATA1[4] = {0x00,0x00,0x00,0x00};
uint8_t Received_DATA2[4] = {0x00,0x00,0x00,0x00};
uint8_t Received_DATA3[4] = {0x00,0x00,0x00,0x00};
uint8_t Received_DATA4[4] = {0x00,0x00,0x00,0x00};
uint8_t Received_DATA5[4] = {0x00,0x00,0x00,0x00};
uint8_t Received_DATA6[4] = {0x00,0x00,0x00,0x00};
uint8_t Received_DATA7[4] = {0x00,0x00,0x00,0x00};
uint8_t STORE_DATA0[400];
uint8_t STORE_DATA1[400];
uint8_t STORE_DATA2[400];
uint8_t STORE_DATA3[400];
uint8_t STORE_DATA4[400];
uint8_t STORE_DATA5[400];
uint8_t STORE_DATA6[400];
uint8_t STORE_DATA7[400];

uint8_t STDBY[2] = {130,0};
uint8_t RST[2] = {133,0};
uint8_t Range_Select0[2] = {11,6};
uint8_t Range_Select1[2] = {13,6};

uint8_t RD_AUTO_SEQ_EN[3] = {2,0,0};
uint8_t rece_AUTO_SEQ_EN[3] = {0,0,0};
	
uint8_t WR_AUTO_SEQ_EN[3] = {3,1,0};
	
uint8_t RD_Feature_Select[3] = {6,0,0};
uint8_t rece_Feature_Select[3] = {0,0,0};
	
uint8_t WR_Feature_Select[2] = {7,40};

uint8_t MAN_CH0[2] = {192,0};
uint8_t MAN_CH1[2] = {196,0};
uint8_t MAN_CH2[2] = {200,0};
uint8_t MAN_CH3[2] = {204,0};
uint8_t AUTO_CH[2] = {160,0};
uint8_t NO_OP[4] = {0,0,0,0};
uint8_t Received_DATA[4] = {0,0,0,0};
uint8_t flag1 = 0;
float voltage1 = 0.0f;
float voltage2 = 0.0f;
float voltage3 = 0.0f;
float voltage0 = 0.0f;
float Data0=0;
float Data1=0;
float Data2=0;
float Data3=0;
float Data4=0;
float Data5=0;
float Data6=0;
float Data7=0;
float bit = 0.00007812; //2*10.24/(2^16);
float bit1 = 6553.0799; //1/(0.5*2*5/32768);
float bit3 = 0.0003052f; //0.5*4*5/32768;
float bit2 = 0.000078125f; //5.12/2^16;
uint8_t WR_ch0_Shift[3] ;     //在ch0写入
uint8_t WR_ch1_Shift[3];      //在ch1写入
uint8_t WR_ch2_Shift[3];   //在ch2写入
uint8_t WR_ch3_Shift[3];    //在ch3写入
uint8_t WR_ch4_Shift[3] ;     //在ch0写入
uint8_t WR_ch5_Shift[3];      //在ch1写入
uint8_t WR_ch6_Shift[3];   //在ch2写入
uint8_t WR_ch7_Shift[3];    //在ch3写入
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_SPI3_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	
	//AD采集初始化程序
	ADS8684_Init();
	ADS8684_RST();
  ADS8684_Range_Select();
	
//	ResetAB_Pin_Low();

//	HAL_Delay(10000);
//	ResetAB_Pin_High();
//	HAL_Delay(1000);
//	testt = Set_AB_Voltage(4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		//10ms控制周期  时间由TIM2控制
		while(!countTim);
		countTim = 0;
		
		

		
		count = count + 0.1f;
		if(count > 10) 
		{
			percentPWM += 10;
			if(percentPWM > 99) percentPWM = 10;
			count = 0;
			
			flag = !flag;

				HAL_GPIO_WritePin(TEST_LIGHT_GPIO_Port,TEST_LIGHT_Pin,flag);
//			testt = Set_AB_Voltage(17);
			
			USER_PWM_SetDutyRatio(&htim3,TIM_CHANNEL_1,59);
			USER_PWM_SetDutyRatio(&htim3,TIM_CHANNEL_2,40);
//			percentPWM = 90;
////		USER_PWM_SetDutyRatio(&htim3,TIM_CHANNEL_2,count);  
//		 printf("%d\r\n",1);
		}
			
			SYSCLOCK = HAL_RCC_GetSysClockFreq();;
			HCLKFRQ = HAL_RCC_GetHCLKFreq();
			PCLK1 = HAL_RCC_GetPCLK1Freq();
			PCLK2 = HAL_RCC_GetPCLK2Freq();
		

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 839;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|RESET_CD_Pin|TEST_LIGHT_Pin|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 RESET_CD_Pin TEST_LIGHT_Pin PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|RESET_CD_Pin|TEST_LIGHT_Pin|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : FAULT_Pin OTW_Pin */
  GPIO_InitStruct.Pin = FAULT_Pin|OTW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

		//AD接收程序，再tim2的中断里调用。周期10ms
    if (htim->Instance == htim2.Instance)
    {
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);	
					HAL_SPI_Transmit(&hspi3, AUTO_CH, 2 ,100); 		
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);	
					HAL_SPI_TransmitReceive(&hspi3, NO_OP, Received_DATA0,4,100); 		
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
							   
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);	
					HAL_SPI_TransmitReceive(&hspi3, NO_OP, Received_DATA1,4,100); 		
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
								
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);	
					HAL_SPI_TransmitReceive(&hspi3, NO_OP, Received_DATA2,4,100); 		
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
					
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);	
					HAL_SPI_TransmitReceive(&hspi3, NO_OP, Received_DATA3,4,100); 		
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

          voltage0 =  (float)((Received_DATA0[2]<<8) + Received_DATA0[3])*bit;
					voltage1 =  (float)((Received_DATA1[2]<<8) + Received_DATA1[3])*bit;		
					voltage2 =  -10.24 + (float)((Received_DATA2[2]<<8) + Received_DATA2[3])*bit;			
					voltage3 =  -10.24 + (float)((Received_DATA3[2]<<8) + Received_DATA3[3])*bit;		

    }

		
}


/*------------------ ADS8684 Init ---------------------*/
/* 进入STDBY模式
   对autoseq先读后写再读  应该是读FF  写1 读1
	 对feature先读后写再度  应该是读0   写40 读40
*/
void ADS8684_Init(void)
{
//	printf("This is ADS8684 SPI Init\r\n");
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, STDBY, 2, 1000);   //进入STDBY模式
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_Delay(100);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);	
	HAL_SPI_TransmitReceive(&hspi3, RD_AUTO_SEQ_EN, rece_AUTO_SEQ_EN,4,1000);   //读autoseq 默认 ff
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	
//	printf("receive data1 is %d\r\n",rece_AUTO_SEQ_EN[0]);
//	printf("receive data2 is %d\r\n",rece_AUTO_SEQ_EN[1]);
//	printf("receive data3 is %d\r\n",rece_AUTO_SEQ_EN[2]);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, WR_AUTO_SEQ_EN, 3, 1000);    //写autoseq   最后一位1
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);	
	HAL_Delay(1000);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi3, RD_AUTO_SEQ_EN, rece_AUTO_SEQ_EN,4,1000);   //读autoseq 默认 ff
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
//  printf("write receive data1 is %d\r\n",rece_AUTO_SEQ_EN[0]);
//	printf("write receive data2 is %d\r\n",rece_AUTO_SEQ_EN[1]);
//	printf("write receive data3 is %d\r\n",rece_AUTO_SEQ_EN[2]);

	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi3, RD_Feature_Select, rece_Feature_Select,4,1000); //读feature  默认00
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
//  HAL_Delay(1000);
	
//	printf("receive data4 is %d\r\n",rece_Feature_Select[0]);
//	printf("receive data5 is %d\r\n",rece_Feature_Select[1]);
//	printf("receive data6 is %d\r\n",rece_Feature_Select[2]);
	
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, WR_Feature_Select, 4, 1000);    //写feature 40
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);	
//	HAL_Delay(1000);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi3, RD_Feature_Select, rece_Feature_Select,4,1000); // 读feature 应该是40
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
//	HAL_Delay(1000);

//	printf("receive data7 is %d\r\n",rece_Feature_Select[0]);
//	printf("receive data8 is %d\r\n",rece_Feature_Select[1]);
//	printf("receive data9 is %d\r\n",rece_Feature_Select[2]);
//	
//	printf("ADS8684 Intial is Done\r\n");
}

/*------------------ ADS8684 RST ---------------------*/
void ADS8684_RST(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, RST, 2, 1000);   //进入RST模式
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_Delay(100);
}


void ADS8684_STDBY(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, STDBY, 2, 1000);   //进入STDBY模式
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_Delay(100);
}

void ADS8684_Range_Select(void)
{
	//设置1、2通道为0~5.12V的输入电压范围
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, Range_Select0, 2, 1000);   //选择ch范围
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, Range_Select1, 2, 1000);   //选择ch范围
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_Delay(100);
}


/** 
  * @brief  调整PWM占空比 
  * @param  value为占空比 value=50 即占空比为50% 
  * @retval None 
  */  
void USER_PWM_SetDutyRatio(TIM_HandleTypeDef *htim,uint32_t Channel,uint8_t value)  
{  
    TIM_OC_InitTypeDef sConfigOC;  
      
    uint32_t period=htim->Init.Period+1;  
    uint32_t pluse=(value * period)/100;  
      
    sConfigOC.OCMode = TIM_OCMODE_PWM1;  
    sConfigOC.Pulse = pluse;  
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;  
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;  
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, Channel);  
    HAL_TIM_PWM_Start(htim, Channel);     
}



//将RESET_AB引脚拉高，高电平为工作状态
void ResetAB_Pin_High(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
}



//将RESET_AB引脚拉低，低电平为重启状态
void ResetAB_Pin_Low(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
}



#define MAX_VALUE 24.0f
#define MIN_VALUE -24.0f
//设置AB端电压函数
int Set_AB_Voltage(float value)
{
	//占空比
	int percent = 0;
	
	percent = (int)(50.0f - value / MAX_VALUE * 50.0f); 
	
	//进行限幅处理
	if(percent > 99)
		percent = 99;
	else if(percent < 2)
		percent = 2;
	
	USER_PWM_SetDutyRatio(&htim3,TIM_CHANNEL_1,percent);
	
	return percent;
}



#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */

  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;

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
