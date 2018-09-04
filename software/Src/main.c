
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "systick.h"
#include "presets.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim16;
DMA_HandleTypeDef hdma_tim16_ch1_up;

/* USER CODE BEGIN PV */

#define nLEDS       10

#define MAX_LIGHT   0x80    // sviesos diodo MAX ryskumas

#define TMR_COUNTER 60  //60 -> 1.25us
#define LOW_LVL     18  // loginis 0
#define HIGH_LVL    44  // loginis 1
#define RST_BYTES   46

#define DMA_BUFFER_SIZE   (RST_BYTES + nLEDS*24 + 1)

enum {FWD, BACK};


typedef struct{
    const uint32_t*     Body;       // pointeris i objekto forma/ilgi ir spalva
    uint8_t             Lenght;
    uint8_t             CurrentSpeed;      // speed    (0-7)
    uint8_t             CurrentDirection;  // kriptys  (FORWARD/BACK)
    int16_t             CurrentPosition;   // nuo pirmo sviesos diodo iki nLEDS. nurodo objekto pradzia. int todel, kad objekto pozicija gali buti uz buferio ribu, bet dar reikia rodyti objekto "uodega"
}Meteo_TypeDef;


Meteo_TypeDef MeteoBlue = {
    .Body = meteo_blue,
    .Lenght = 10,
    .CurrentSpeed = 0,
    .CurrentDirection = FWD,
    .CurrentPosition = 0
};




/* Private variables ---------------------------------------------------------*/
uint8_t dma_buffer[DMA_BUFFER_SIZE];   // taimerio PULSE reiksme vienam sviesdiodziui

FlagStatus SendToLedsRequired = RESET;
FlagStatus BufferIsFilled = RESET;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM16_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void WS2812B_Init(void);
void WS2812B_LedsOff(void);
void WS2812B_FillDMABuffer(const uint32_t* pdata);
void WS2812B_Set_RGB(uint16_t led, uint8_t red, uint8_t green, uint8_t blue);
uint8_t* WS2812B_GetLedBuferPointer( uint16_t led);

void WS2812B_ShowMeteo(Meteo_TypeDef* meteo);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    static uint32_t delay = 0;
    static uint8_t n = 0, val = 0x01;
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
  MX_DMA_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

    UserTimersInit();

    StartUserPeriodicTimer(USER_TIMER3, 40);
    StartUserPeriodicTimer(USER_TIMER4, 1000);

    /* inicializuojam buferi */
    WS2812B_Init();

    /* isvalom buferi ir gesinam ledus */
    WS2812B_LedsOff();

    //WS2812B_Set_RGB(1, 0x00, 0xFF, 0x00);   //B
    //WS2812B_Set_RGB(2, 0xFF, 0x00, 0x00);   //R
    //WS2812B_Set_RGB(3, 0x00, 0x00, 0xFF); //G
    //WS2812B_Set_RGB(10, 0xFF, 0xFF, 0xFF);//


    //WS2812B_FillDMABuffer(meteo_red);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

        if( delay < HAL_GetTick() ){

            delay = HAL_GetTick() + 30;

            //WS2812B_FillDMABuffer(preset1+n*nLEDS);
            //if(++n > 19) n = 0;



            WS2812B_ShowMeteo(&MeteoBlue);



            SendToLedsRequired = SET;
        }


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

        /* siunciam buferi */
        if(SendToLedsRequired == SET){
            SendToLedsRequired = RESET;
            HAL_TIM_PWM_Start_DMA(&htim16, TIM_CHANNEL_1, (uint32_t*)dma_buffer, DMA_BUFFER_SIZE );
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 60;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim16);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD5_Pin|LD2_Pin|LD7_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD5_Pin LD2_Pin LD7_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD5_Pin|LD2_Pin|LD7_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*  */
void WS2812B_Init(void){

    uint16_t i = 0;

    /* isvalom buferi */
    while(i < DMA_BUFFER_SIZE){
       dma_buffer[i] = 0;
       i++;
    }
}


/* buferio uzpildymas. priima pointeri i preseto buferi */
void WS2812B_FillDMABuffer(const uint32_t* pdata){

    uint16_t i = 0, j = 0;
    uint32_t color = 0;
    uint8_t* pdmabuf = dma_buffer+RST_BYTES;

    BufferIsFilled = RESET;

    /* ruosiam buferi */
    while(i < nLEDS){

        color = *(pdata+i);

        while(j < 24){
            *(pdmabuf+j+24*i) = ( (color & 0x800000) == 0 ) ? LOW_LVL : HIGH_LVL;
            color = color<<1;
            j++;
        }

        j = 0;
        i++;
    }

    BufferIsFilled = SET;
}

/*  */
void WS2812B_Set_RGB(uint16_t led, uint8_t red, uint8_t green, uint8_t blue){

    uint8_t i = 0;
    uint32_t color = (green<<16) | (red<<8) | blue;
    uint8_t* pdata = WS2812B_GetLedBuferPointer(led);    // pointeris i reikiama ledo buferi

    while(i < 24){
        *(pdata+i) = ( (color & 0x800000) == 0 ) ? LOW_LVL : HIGH_LVL;
        color = color<<1;
        i++;
    }
}



void WS2812B_ShowMeteo(Meteo_TypeDef* meteo) {

    uint8_t i = 0, j = 0;
    uint32_t color = 0;
    uint8_t* pdmabuf = dma_buffer + RST_BYTES;


    meteo->CurrentDirection = BACK;

    do {

        color = *( meteo->Body + i);

        if(meteo->CurrentDirection == FWD) {

            j = 0;

            while(j < 24){
                *(pdmabuf+j+24*i) = ( (color & 0x800000) == 0 ) ? LOW_LVL : HIGH_LVL;
                color = color<<1;
                j++;
            }

        } else {


        }

    } while(++i < meteo->Lenght);
}





/* grazina pasirinkto ledo 3 baitu pozicija ledu buferyje */
uint8_t* WS2812B_GetLedBuferPointer( uint16_t led){
    return dma_buffer + (RST_BYTES + 24*(led-1) );
}

/*  */
void WS2812B_LedsOff(void){

    uint16_t i = 0;
    uint8_t* pdata = dma_buffer + RST_BYTES;

    while(i < 24*nLEDS){
        *(pdata+i) = LOW_LVL;
        i++;
    }

    HAL_TIM_PWM_Start_DMA(&htim16, TIM_CHANNEL_1, (uint32_t*)dma_buffer, DMA_BUFFER_SIZE );
    HAL_Delay(10);
}


/* SYSTICK callback funkcija */
void HAL_SYSTICK_Callback(void)
{
    SysTimeCounterUpdate();

    //SendToLedsRequired = SET;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
