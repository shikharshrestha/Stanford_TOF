/**
  ******************************************************************************
  * @file    main.c 
  * @author  Shikhar Shrestha
  * @version V1.0
  * @date    7/29/2015
  * @brief   Main program body
  ******************************************************************************
	**/
	
 /*Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4_discovery.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ddsclock 300000000 //in MHz
#define timeout 100 //For SPI

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef SpiHandle;
GPIO_InitTypeDef GPIO_InitStruct;
static unsigned char init1[2] = {0x00,0xF0};
static unsigned char init2[4] = {0x01,0xD0,0x00,0x00};
static unsigned char init3[4] = {0x03,0x00,0x03,0x01};
static unsigned char chan[2] = {0x00,0xF0};
static unsigned char chan0[2] = {0x00,0x10};
static unsigned char chan1[2] = {0x00,0x20};
static unsigned char chan2[2] = {0x00,0x40};
static unsigned char chan3[2] = {0x00,0x80};
static unsigned char data[5] = {0x04,0x00,0x00,0x00,0x00}; //Use to shutdown channel
static unsigned char data0[5] = {0x04,0x11,0x11,0x11,0x11}; //20MHz
static unsigned char data1[5] = {0x04,0x0F,0x5C,0x28,0xF6}; //0xF6
static unsigned char phase[3] = {0x05,0x00,100};

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
HAL_StatusTypeDef HAL_SPI_TransmitNew(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
void ioupdate(void);
void resetDDS(void);
void initDDS(void);
void setfreqDDS(unsigned char channel,unsigned char freq);
void getftw(uint32_t freq); //Function to Calculate Frequency Tuning Word

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user 
             can eventually implement his proper time base source (a general purpose 
             timer for example or other time source), keeping in mind that Time base 
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
             handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 168 MHz */
  SystemClock_Config();


  /* Add your application code here
     */
  printf("\nAll Systems Initialized!");
	printf("\n...Running Host Application Code...\n");
	BSP_PB_Init(BUTTON_KEY,BUTTON_MODE_EXTI);
 /* Configure LED3, LED4, LED5 and LED6 */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED6);
	
	/*Configure GPIO pin : PB11 for I/O Update*/
	__HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
 /*Configure GPIO pin : PD8 for Slave Select*/
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	 /*Configure GPIO pin : PD9 for Master Reset*/
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	 /*Configure GPIO pin : PA1 for External Interrupts from ILLUM_EN*/
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/* Enable and set External Interrupt to the highest priority */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	
	
  /* Set the SPI parameters */
  SpiHandle.Instance               = SPI2;
  
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  SpiHandle.Init.Direction         = SPI_DIRECTION_1LINE;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_2EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
	SpiHandle.Init.Mode 						 = SPI_MODE_MASTER;
  
	//Initialize DDS for Host-Control
	initDDS();
	printf("\nDDS Initialized for Host-Control.");
	BSP_LED_On(LED3);
	setfreqDDS(0,1);
	
	
	/* Infinite loop */
  while (1)
  {
		/*
		for (unsigned char i =0;i<=255;i++){
			phase[3] = i;
			printf("\nCurrent Value = %d",i);
			HAL_Delay(1000);
		}
  		//printf("\n%d",HAL_GetTick());
		*/
  }
	
}

void ioupdate(void){
   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);
   for(unsigned char i =0;i<10;i++)
	 ;;
	 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
	 return;
}

void resetDDS(void){
   HAL_GPIO_WritePin(GPIOD,GPIO_PIN_9,GPIO_PIN_SET);
	 //Need a pulse atleast .266us wide
	 HAL_Delay(5);
	 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_9,GPIO_PIN_RESET);
	 return;
}

void initDDS(void){
	HAL_SPI_Init(&SpiHandle);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_SET); //Pulling Up Slave Select
	HAL_Delay(10);
	resetDDS();
	HAL_Delay(10);
	ioupdate();
	HAL_Delay(10);
	HAL_SPI_TransmitNew(&SpiHandle,init1,2,timeout);
	ioupdate();
	HAL_Delay(10);
	HAL_SPI_TransmitNew(&SpiHandle,init2,4,timeout);
	ioupdate();
	HAL_Delay(10);
	HAL_SPI_TransmitNew(&SpiHandle,init3,4,timeout);
	ioupdate();

}

void setfreqDDS(unsigned char channel,unsigned char freq){
	
	switch (channel){
		case 0:
		HAL_SPI_TransmitNew(&SpiHandle,chan0,2,timeout);
    break;
    case 1:
		HAL_SPI_TransmitNew(&SpiHandle,chan1,2,timeout);
    break;
		case 2:
		HAL_SPI_TransmitNew(&SpiHandle,chan2,2,timeout);
    break;
		case 3:
		HAL_SPI_TransmitNew(&SpiHandle,chan3,2,timeout);
    break;
	  default:
   	HAL_SPI_TransmitNew(&SpiHandle,chan,2,timeout);
		break;
	  }
	
	switch (freq){
		case 0:
		HAL_SPI_TransmitNew(&SpiHandle,data0,5,timeout);
    break;
    case 1:
		HAL_SPI_TransmitNew(&SpiHandle,data1,5,timeout);
    break;
		default:
		HAL_SPI_TransmitNew(&SpiHandle,data,5,timeout);
    break;
			
	}
	
	HAL_SPI_TransmitNew(&SpiHandle,phase,3,timeout);

	ioupdate();
	
}

void getftw(uint32_t freq){
	 //TBD
	
}


HAL_StatusTypeDef HAL_SPI_TransmitNew(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout){
   HAL_StatusTypeDef returnvar;
   HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_RESET);
	 returnvar = HAL_SPI_Transmit(hspi,pData,Size,Timeout);
	 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_SET);
	 return returnvar;

}


/**
  * @brief  Interrupt Callbacks
  * @param  None
  * @retval None
  */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	
	
	if (GPIO_Pin==GPIO_PIN_0){
	while (BSP_PB_GetState(BUTTON_KEY) != RESET);
	BSP_LED_Toggle(LED5);
	printf("\nButton Press Detected");
	return;
	}
	
	if (GPIO_Pin==GPIO_PIN_1){
	 
	 if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)){
		
		//printf("rising");
		BSP_LED_On(LED4);	
		setfreqDDS(1,1); 
	  setfreqDDS(0,1);
	 }
	 else{
	
		//printf("\nfalling");
		BSP_LED_Off(LED4);	
		setfreqDDS(1,4); //Should switch off channel by defaulting on the case  
		setfreqDDS(0,4);
	 }		 
		 
		
	}
	
}





/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 


