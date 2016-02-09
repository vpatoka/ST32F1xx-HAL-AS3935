/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

#include <string.h>
#include "AS3935_I2C.h"

/*
Booting After power on the demo board, the
austriamicrosystems logo and the firmware version
will be displayed.


Antenna Tuning After the startup the antenna of the AS3935 is
automatically tuned. The tuning uses the internal
array of capacitors to achieve 500kHz as resonance
frequency. The resonance frequency and the
internal capacitance are shown on the LCD.


RCOs Calibration After the antenna tuning the internal RC-Oscillators
are calibrated. The SRCO is calibrated to 32kHz
and the TRCO is calibrated to 1.1MHz. Both
frequencies are displayed.


USB connected The demo board can be connected via USB. As
soon as the USB bus is connected the sensor is
turned off and all settings of the AS3935 can be
saved in the GUI. If the demo board is power cycled
when the USB plug is already connected, the LCO
and RCO calibration is not executed, in order to
allow the user to do those calibrations via the GUI.

Listening Mode After the calibration, the Lightning sensor is set in
listening mode. No storm is within detection range.


Lightning
This symbol
indicates a lightning
– the text give
further information.
Lightning has been detected. The distance
estimation and movement of the head of the storm
is shown afterwards.
The distance to the head of the storm gets closer.
The distance to the head of the storm gets farther.

Noise Floor
Detected
Continuous noise is jamming the AS3935 AFE;
during this time the sensor cannot detect the
presence of lightning activities.


Disturber
Detected
Disturbers have been received by the AS3935 and
rejected by the disturber rejection embedded
algorithm. 
*/

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

static RTC_TimeTypeDef stimestructureget;
static RTC_DateTypeDef sdatestructureget;

/* Captured Values */
__IO uint32_t               uwIC2Value1 = 0;        // Prev. value 81920 ==> TIM1 [9830400 Mhz] count number for 120hz. [9830400/120]
__IO uint32_t               uwIC2Value2 = 0;        // Current value
__IO uint32_t               uwDiffCapture = 0;
__IO uint8_t                uhCaptureIndex = 0; 
__IO uint8_t                uhLevent = 0; 


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void Error_Handler(void);

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

// AS3935 structure
AS3935_HandleTypeDef	as3935;

// Lightening sensor AS3935
void AS3935_Setup(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
    uint8_t i, j, k = 0;
    uint32_t uwFrequency, nrgy_val;
    uint8_t ls, lso = 7;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  
    printf("\n\r\n\r# Board has started with %d Hz Sytem Clock\n\r", SystemCoreClock);
    
    
    // Initilize AS3935 Lightening Module
    AS3935_Setup();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	if(uhLevent) {
		uhLevent = 0;
        // 0 = unknown src, 1 = lightning detected, 2 = disturber, 3 = Noise level too high
        ls = AS3935_GetInterruptSrc(&as3935);
        if(ls == 1 || lso != ls) {
            lso = ls;
            switch(ls) {
                case 0:
                    printf("# AS3935: Unknown source of Interrupt\n\r");
                    break;
                case 1:
                    AS3935_GetLightningDistKm(&as3935);
                    printf("# AS3935: Lightning has deteccted. Distance: %u\n\r", AS3935_GetLightningDistKm(&as3935));
                    nrgy_val = AS3935_GetStrikeEnergyRaw(&as3935);
                    printf("# AS3935: Strike Energy: 0x%X\n\r",  nrgy_val);
                    break;
                case 2:
                    printf("# AS3935: Some source of disturbance nearby\n\r");
                    break;
                case 3:
                    printf("# AS3935: RF noise level is too high\n\r");
                    break;
            }    
        }
	}       


    HAL_Delay(300);
    HAL_GPIO_TogglePin(GPIOB, LED_A_Pin);

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

}

/* I2C2 init function */
void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c2);

}

/* RTC init function */
void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  HAL_RTC_Init(&hrtc);

  sTime.Hours = 9;
  sTime.Minutes = 21;
  sTime.Seconds = 48;

  HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BIN);

  DateToUpdate.WeekDay = RTC_WEEKDAY_SATURDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 31;
  DateToUpdate.Year = 0;

  HAL_RTC_SetDate(&hrtc, &DateToUpdate, FORMAT_BIN);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_IC_Init(&htim1);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  HAL_TIM_Encoder_Init(&htim2, &sConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : AS3935_SI_Pin */
  GPIO_InitStruct.Pin = AS3935_SI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(AS3935_SI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_LED_Pin SPI_RST_Pin LED_B_Pin LED_A_Pin */
  GPIO_InitStruct.Pin = SPI_LED_Pin|SPI_RST_Pin|LED_B_Pin|LED_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_NCS_Pin SPI_DC_Pin */
  GPIO_InitStruct.Pin = SPI_NCS_Pin|SPI_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
    while(1) {
        /* Turn LED [PA3] on */
        HAL_GPIO_TogglePin(GPIOB, LED_B_Pin);
        HAL_Delay(150);
    }
}

/**
  * @brief  I2C error callbacks.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
    printf("# I2C Transfer failure\n\r");
    Error_Handler();
}


/**
  * @brief  Tx Transfer completed callback.
  * @param  I2cHandle: I2C handle. 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
    /* Toggle LED1: Transfer in transmission process is correct */
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
    /* Toggle LED1: Transfer in reception process is correct */
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
}


/**
  * @brief  Conversion complete callback in non blocking mode 
  * @param  htim : hadc handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {

        if(uhCaptureIndex == 0) {
            // Get the 1st Input Capture value
            uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            uhCaptureIndex = 1;
        } else if(uhCaptureIndex == 1) {
                // Get the 2nd Input Capture value
                uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); 

                // Capture computation
                if (uwIC2Value2 > uwIC2Value1) {
                    uwDiffCapture = (uwIC2Value2 - uwIC2Value1); 
                } else {
                    if (uwIC2Value2 < uwIC2Value1) {
                        uwDiffCapture = ((0xFFFF - uwIC2Value1) + uwIC2Value2); 
                    } else {
                        // If capture values are equal, we have reached the limit of frequency  measures
                        Error_Handler();
                    }
                }
                // Frequency computation: for this project TIMx (TIM1) is clocked by APB1Clk
                //uwFrequency = HAL_RCC_GetPCLK1Freq() / uwDiffCapture;
                uhCaptureIndex = 0;
        }
        uhLevent = 1;	// Lighting has detected
    } // TIM1 CH1
}




void AS3935_Setup(void)
{
    uint8_t cap = AS3935_CAPACITANCE;

	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);

    printf("# AS3935 Lightning Sensor, SEN-39001-R01: beginning boot procedure....\n\r");

    // Enable I2C
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

    // Init AS3934 I2C port
    as3935.as3935_i2c_address=AS3935_I2C_ADDRESS;
    as3935.as3935_i2c_timeout=AS3935_I2C_TIMEOUT;
    as3935.i2c.Instance=I2C2;
    as3935.i2c.Init.ClockSpeed=400000;

    //AS3935_DeInit(&as3935);          // set registers to default  

    // setup for the the I2C library: (enable pullups, set speed to 400kHz)
    if(AS3935_Init(&as3935) != AS3935_OK) {
        printf("# AS3935: ERROR with I2C Bus\n\r");
        Error_Handler();
    } else {
        printf("# AS3935: I2C Bus is OK\n\r");
    }

    //check for r-m-w problem
    if((I2C_CR1_STOP & hi2c2.Instance->CR1) != (I2C_CR1_STOP & hi2c2.Instance->CR1)) {
        printf("# I2C Read-Modify-Write trap\n\r");
    }

    // Contol checkup for the communication between of MCU and AS3935
    if(AS3935_CheckUp(&as3935) != AS3935_OK) {
        Error_Handler();
    }

    // Capture Lighting Sersor Events on TIM1
    // Tim1Frequency = ((9830400 / Prescaler) + (Capture / 2)) / Capture; 120Hz --> 81920 pulses
    if(HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1) != HAL_OK ) {
        printf("# Cannot start AS3935 Strike Detection\n\r");
        Error_Handler();
    } else {
        printf("# AS3935 Strike Detection has been activated on MCU\n\r");
    }

    cap = AS3935_AutoCal(&as3935);

    // now update sensor cal for your application and power up chip
    AS3935_ManualCal(&as3935, cap, AS3935_OUTDOORS, AS3935_DIST_EN);
                                 // AS3935_ManualCal Parameters:
                                 //   --> capacitance, in pF (marked on package)
                                 //   --> indoors/outdoors (AS3935_INDOORS:0 / AS3935_OUTDOORS:1)
                                 //   --> disturbers (AS3935_DIST_EN:1 / AS3935_DIST_DIS:2)
                                 // function also powers up the chip
                  
    // enable interrupt (hook IRQ pin to Arduino Uno/Mega interrupt input: 0 -> pin 2, 1 -> pin 3 )
    //attachInterrupt(0, AS3935_ISR, RISING);
    AS3935_PrintAllRegs(&as3935);


}


/* USER CODE END 4 */

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
