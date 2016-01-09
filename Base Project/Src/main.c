/**
  ******************************************************************************
  * @file    main.c 
  * @author  Shikhar Shrestha
  * @version V1.0
  * @date    7/29/2015
  * @brief   Main program body
  ******************************************************************************
	**/
	//Using LED 4 & 5 for Modulation Control
	//LED 6 for USB
	//LED 3 for General System Status
	
 /*Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4_discovery.h"
#include "tm_stm32f4_usb_vcp.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ddsclock 300000000 //in MHz
#define timeout 100 //For SPI
#define exposure 1970//in usec

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef SpiHandle;
GPIO_InitTypeDef GPIO_InitStruct;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim1;
static unsigned char quadcount = 0; //Keeps track of current quad
static unsigned char click = 0;  //Keeps track of User Button press events (Debugging only)
static unsigned char PhaseSteppingAuto = 1; //Automatic Phase stepping true if 1
static unsigned char FreqSteppingAuto = 0;//Automatic Frequency stepping to do Doppler [Adhoc, revise code!]


static unsigned char modulation_state = 0; //0 means not modulating
static unsigned char init1[2] = {0x00,0xF0};
static unsigned char init2[4] = {0x01,0xD0,0x00,0x00};
static unsigned char init3[4] = {0x03,0x00,0x03,0x04};  //Auto Clear Phase Accumulator
static unsigned char chan[2] = {0x00,0xF0};
static unsigned char chan0[2] = {0x00,0x10}; //Sensor Demodulation
static unsigned char chan1[2] = {0x00,0x20}; //ILLUM Modulation
static unsigned char chan2[2] = {0x00,0x40};
static unsigned char chan3[2] = {0x00,0x80};
static unsigned char data0[5] = {0x04,0x00,0x00,0x00,0x00}; //Use to shutdown channel - Default
static unsigned char data1[5] = {0x04,0x19,0x99,0x99,0x9A}; //30MHz {0x04,0x0F,0x5C,0x28,0xF6} - For 18MHz

//Channel-Level Frequency Settings
static unsigned char fch0[5] = {0x04,0x19,0x99,0x99,0x9A};
static unsigned char fch1[5] = {0x04,0x19,0x99,0x99,0x9A};
static unsigned char fch2[5] = {0x04,0x19,0x99,0x99,0x9A};
static unsigned char fch3[5] = {0x04,0x19,0x99,0x99,0x9A};

//Channel-Level Phase Settings
static unsigned char pch0[3] = {0x05,0x00,0x00};
static unsigned char pch1[3] = {0x05,0x00,0x00};
static unsigned char pch2[3] = {0x05,0x00,0x00};
static unsigned char pch3[3] = {0x05,0x00,0x00};

//static unsigned char idlesetting[5] = {0x0A,0x00,0x00,0x00,0x00}; //Sets 0Hz in Channel Word 2
static unsigned char phase0[3] = {0x05,0x00,0x00}; //In-Phase
static unsigned char phase1[3] = {0x05,0x10,0x00}; //90deg-Phase
static unsigned char phase2[3] = {0x05,0x20,0x00}; //180deg-Phase
static unsigned char phase3[3] = {0x05,0x30,0x00}; //270deg-Phase

//Doppler freq stepping variables
static unsigned char fhom[5] = {0x04,0x0F,0x5C,0x28,0xF6};
static unsigned char fhet[5] = {0x04,0x0F,0x5C,0x3E,0x7E};


//USB Commands
static uint8_t cmd[11] = "";
#define commandsize 11

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
void resetphase(void); //Function to set the channels at 0 Phase
void setphase(void); //Function to set the channels at desired Phase
void incrementphase(void); //Increments phase of sensor demodulation by 90deg
void modulation_on(void); //Performs all setup to turn on modulation
void modulation_off(void); //Performs all setup to turn off modulation
unsigned char parse_usb(uint8_t *cmd,uint16_t size); //Parses the USB Command and sets frequency/phase values
void incrementfreq(void); //Frequency stepping function
void initializesync(void); //Sets up timer to generate camera synchronization signal at 15-30 Hz
void startsyncpulse(void); //Starts sending out the synchronization pulse
void stopsyncpulse(void); //Stops sending out the synchronization pulses
void inituscounter(void); //Initializes a usec counter
void startcounter(void); //Starts the usec counter with defined period
void stopcounter(void); //Stops the usec counter
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
	
	
int main(void)
{ 
	/*Variable Declarations*/
	 uint8_t c;
	 uint16_t size = 0; 
	 char ACK[7]="ACK\n";
	 char NACK[7]="NACK\n";
	 unsigned char status = 0;
  
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
  BSP_LED_Init(LED4); //Line used for enabling Gating Board 2
  BSP_LED_Init(LED5); //Line used for enabling Gating Board 1
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
	
	 /*Configure GPIO pin : PA2 for External Interrupts from ILLUM_EN*/
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
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
  resetphase();
	modulation_off();
	ioupdate();
	printf("\nDDS Initialized for Host-Control.");
	printf("\nDDS Channel Setup Complete");
	
	//Initialize Timer for generating sync pulse and exposure counting
	initializesync();
	inituscounter();
	printf("\nSync Pulse Initialized");
	
	/* Initialize USB VCP */    
  TM_USB_VCP_Init();
	printf("\nUSB Communication Setup Complete - Host Ready!"); 
	
	////This Block makes the Host respond to ILLUM_EN
	/* Enable and set External Interrupt to the highest priority */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	////It should come at the end of all critical initialization
	
	//Start Pulsing VD_IN to start capture of frames
  startsyncpulse();
	
	/* Infinite loop */
  while (1)
  {
		 /* USB configured OK, drivers OK */
        if (TM_USB_VCP_GetStatus() == TM_USB_VCP_CONNECTED) {
            BSP_LED_On(LED3); //Status - All Good!
					//Only process USB when modulation not in progress  
					if (modulation_state==0){
            /* If something arrived at VCP */
						if(TM_USB_VCP_Getc(&c)==TM_USB_VCP_DATA_OK){
             /* Return data back */
						   if (c=='M'){
								 printf("\n");
								 //Take 10 Bytes in after receiving M
								 size = 0;
								 while((TM_USB_VCP_Getc(&cmd[size]) == TM_USB_VCP_DATA_OK)&&(size < commandsize))
								 {
									 printf("%c",cmd[size]);
									 size++;
								 }
								 printf("%d",size);
								 status = parse_usb(cmd,size); 
								 //Block checks is the parsing went correct
									 if (status==1){
										 TM_USB_VCP_Puts(ACK); 
										 BSP_LED_On(LED6);
									 }
									 else{
										TM_USB_VCP_Puts(NACK);										 
										BSP_LED_Off(LED6);	
									}									
               }
							 else{
							   BSP_LED_Off(LED6);
							   TM_USB_VCP_Puts(NACK);	 
								 
								 		
							}
					}}
        } else {
            /* USB not OK */
             BSP_LED_Off(LED3);
        }
  }
	
}


unsigned char parse_usb(uint8_t *cmd,uint16_t size){
	
	//Check size of cmd packet
	if (size!=11)
		return 0;
	
	//Check Delimiters
	if ((cmd[0]!=0xFF)||(cmd[9]!=0xFF))
		return 0;
	
	
	//Check if the command is for Doppler mode else process block as before
	if (cmd[8]==2){
		switch (cmd[1]){
				case 0:
			  fhom[1] = cmd[2];
				fhom[2] = cmd[3];
				fhom[3] = cmd[4];
				fhom[4] = cmd[5];		
				printf("\nChannel 0");
				break;
				
				case 1:
			  fhet[1] = cmd[2];
				fhet[2] = cmd[3];
				fhet[3] = cmd[4];
				fhet[4] = cmd[5];
				printf("\nChannel 1");
				break;
			}
		FreqSteppingAuto = 1;
		PhaseSteppingAuto = 0;
    printf("\nDoppler Mode Activated");			
		}
  else{
	
	//Extract Channel, FTW, PTW and AutoPhase Stepping Command
	switch (cmd[1]){
				case 0:
			  fch0[1] = cmd[2];
				fch0[2] = cmd[3];
				fch0[3] = cmd[4];
				fch0[4] = cmd[5];
				
				if (cmd[8]==0){
					pch0[1] = cmd[6];
					pch0[2] = cmd[7];
				}
				printf("\nChannel 0");
				break;
				case 1:
			  fch1[1] = cmd[2];
				fch1[2] = cmd[3];
				fch1[3] = cmd[4];
				fch1[4] = cmd[5];
				if (cmd[8]==0){
					pch1[1] = cmd[6];
					pch1[2] = cmd[7];
				}
				printf("\nChannel 1");
				break;
				case 2:
				fch2[1] = cmd[2];
				fch2[2] = cmd[3];
				fch2[3] = cmd[4];
				fch2[4] = cmd[5];
				if (cmd[8]==0){
					pch2[1] = cmd[6];
					pch2[2] = cmd[7];
				
				}
				printf("\nChannel 2");
				break;	
				case 3:
				fch3[1] = cmd[2];
				fch3[2] = cmd[3];
				fch3[3] = cmd[4];
				fch3[4] = cmd[5];
				if (cmd[8]==0){
					pch3[1] = cmd[6];
					pch3[2] = cmd[7];
				}
				printf("\nChannel 3");
				break;				
			}
	
	if (cmd[8]==0){
		PhaseSteppingAuto = 0;
		printf("\nAuto-Stepping OFF, Phases can be set!");
	}else
		PhaseSteppingAuto = 1;
	
}
	
	return 1;
}

void inituscounter(void){
	
	//Sets up a micro-second timer
	TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 168;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = exposure;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);
	
}

void startcounter(void){
	__HAL_TIM_SET_COUNTER(&htim1,0);
	HAL_TIM_Base_Start_IT(&htim1);
}

void stopcounter(void){
	HAL_TIM_Base_Stop_IT(&htim1);
}


void initializesync(void){
	
	//Sets up TIM8 for sync pulse generation
	/* TIM8 init function */
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 65535;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 160; //Sets the 30Hz pulse ~31.27Hz when = 80 //Right now ~16Hz
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim8);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig);

  HAL_TIM_OC_Init(&htim8);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10; 
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);
  
}

void startsyncpulse(void){
	//Starts pulsing for VD_IN
	HAL_TIM_Base_Start_IT(&htim8);
	HAL_TIM_OC_Start_IT(&htim8,TIM_CHANNEL_1);
}
	
void stopsyncpulse(void){
	//Stops pulsing for VD_IN
	HAL_TIM_Base_Stop_IT(&htim8);
	HAL_TIM_OC_Stop_IT(&htim8,TIM_CHANNEL_1);
	
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
  return;
}

void setfreqDDS(unsigned char channel,unsigned char freq){
	
	switch (channel){
		case 0:
		HAL_SPI_TransmitNew(&SpiHandle,chan0,2,timeout);
		switch (freq){
				case 0:
				HAL_SPI_TransmitNew(&SpiHandle,data0,5,timeout);
				break;
				case 1:
				HAL_SPI_TransmitNew(&SpiHandle,fch0,5,timeout);
				HAL_SPI_TransmitNew(&SpiHandle,pch0,3,timeout);
				break;
				default:
				HAL_SPI_TransmitNew(&SpiHandle,data0,5,timeout);
				break;			
			}
    break;
			
    case 1:
		HAL_SPI_TransmitNew(&SpiHandle,chan1,2,timeout);
		switch (freq){
				case 0:
				HAL_SPI_TransmitNew(&SpiHandle,data0,5,timeout);
				break;
				case 1:
				HAL_SPI_TransmitNew(&SpiHandle,fch1,5,timeout);
				HAL_SPI_TransmitNew(&SpiHandle,pch1,3,timeout);
				break;
				default:
				HAL_SPI_TransmitNew(&SpiHandle,data0,5,timeout);
				break;			
			}
    break;
			
		case 2:
		HAL_SPI_TransmitNew(&SpiHandle,chan2,2,timeout);
		switch (freq){
				case 0:
				HAL_SPI_TransmitNew(&SpiHandle,data0,5,timeout);
				break;
				case 1:
				HAL_SPI_TransmitNew(&SpiHandle,fch2,5,timeout);
				HAL_SPI_TransmitNew(&SpiHandle,pch2,3,timeout);
				break;
				default:
				HAL_SPI_TransmitNew(&SpiHandle,data0,5,timeout);
				break;			
			}
    break;
			
		case 3:
		HAL_SPI_TransmitNew(&SpiHandle,chan3,2,timeout);
		switch (freq){
				case 0:
				HAL_SPI_TransmitNew(&SpiHandle,data0,5,timeout);
				break;
				case 1:
				HAL_SPI_TransmitNew(&SpiHandle,fch3,5,timeout);
				HAL_SPI_TransmitNew(&SpiHandle,pch3,3,timeout);
				break;
				default:
				HAL_SPI_TransmitNew(&SpiHandle,data0,5,timeout);
				break;			
			}
    break;
	  default:
   	HAL_SPI_TransmitNew(&SpiHandle,chan,2,timeout);
		HAL_SPI_TransmitNew(&SpiHandle,data0,5,timeout);
		break;
	  }
	
	 
	
}


void resetphase(void){

	  HAL_SPI_TransmitNew(&SpiHandle,chan0,2,timeout);
		HAL_SPI_TransmitNew(&SpiHandle,phase0,3,timeout);
	  HAL_SPI_TransmitNew(&SpiHandle,chan1,2,timeout);
		HAL_SPI_TransmitNew(&SpiHandle,phase0,3,timeout);
		
  
}


void setphase(void){

	  HAL_SPI_TransmitNew(&SpiHandle,chan0,2,timeout);
		HAL_SPI_TransmitNew(&SpiHandle,pch0,3,timeout);
	  HAL_SPI_TransmitNew(&SpiHandle,chan1,2,timeout);
		HAL_SPI_TransmitNew(&SpiHandle,pch1,3,timeout);
		
  
}




void incrementphase(void){

	if (quadcount ==3)
		quadcount = 0;
	else
		quadcount++;
	
	switch (quadcount){
		case 0:
			pch0[1] = phase0[1];
		  pch0[2] = phase0[2];
		break;
		case 1:
			pch0[1] = phase1[1];
		  pch0[2] = phase1[2];
		break;
		case 2:
			pch0[1] = phase2[1];
		  pch0[2] = phase2[2];
		break;
		case 3:
			pch0[1] = phase3[1];
		  pch0[2] = phase3[2];
		break;

	}
	printf("\nquad = %d",quadcount);
}



void incrementfreq(void){
	if (quadcount ==3)
		quadcount = 0;
	else
		quadcount++;
	
	switch (quadcount){
		case 0:
			fch0[1] = fhom[1];
		  fch0[2] = fhom[2];
		  fch0[3] = fhom[3];
		  fch0[4] = fhom[4];
		 	pch0[1] = phase0[1];
		  pch0[2] = phase0[2];
		
			fch1[1] = fhom[1];
		  fch1[2] = fhom[2];
		  fch1[3] = fhom[3];
		  fch1[4] = fhom[4];
			pch1[1] = phase0[1];
		  pch1[2] = phase0[2];
		break;
		case 1:
			fch1[1] = fhet[1];
		  fch1[2] = fhet[2];
		  fch1[3] = fhet[3];
		  fch1[4] = fhet[4];
		  pch0[1] = phase0[1];
		  pch0[2] = phase0[2];
			
		  fch0[1] = fhom[1];
		  fch0[2] = fhom[2];
		  fch0[3] = fhom[3];
		  fch0[4] = fhom[4];
		  pch1[1] = phase0[1];
		  pch1[2] = phase0[2];
		break;
		case 2:
			fch0[1] = fhom[1];
		  fch0[2] = fhom[2];
		  fch0[3] = fhom[3];
		  fch0[4] = fhom[4];
		 	pch0[1] = phase2[1];
		  pch0[2] = phase2[2];
		
			fch1[1] = fhom[1];
		  fch1[2] = fhom[2];
		  fch1[3] = fhom[3];
		  fch1[4] = fhom[4];
			pch1[1] = phase0[1];
		  pch1[2] = phase0[2];
		break;
		case 3:
			fch1[1] = fhet[1];
		  fch1[2] = fhet[2];
		  fch1[3] = fhet[3];
		  fch1[4] = fhet[4];
		  pch0[1] = phase2[1];
		  pch0[2] = phase2[2];
			
		  fch0[1] = fhom[1];
		  fch0[2] = fhom[2];
		  fch0[3] = fhom[3];
		  fch0[4] = fhom[4];
		  pch1[1] = phase0[1];
		  pch1[2] = phase0[2];
		break;

	}
	
	printf("\nquad = %d",quadcount);
	
}

void getftw(uint32_t freq){
	 //TBD
	
}

void modulation_on(void){
	modulation_state = 1;
	setfreqDDS(0,1);
	setfreqDDS(1,1);
	setfreqDDS(2,1);
	setfreqDDS(3,1);
	ioupdate();
	
	//Hacked wait block 
	//Allows DDS waveforms to stabilize
	for(unsigned int i =0;i<400;i++)
  ;;
	
	BSP_LED_On(LED5);  //Turns on Gating Board 1	
	BSP_LED_On(LED4);	 //Turns on Gating Board 2
  BSP_LED_Off(LED6); //Clears the USB indicator LED
	return;
	
}

void modulation_off(void){
	modulation_state = 0;	
	BSP_LED_Off(LED5);  //Turns on Gating Board 1	 
	BSP_LED_Off(LED4);	//Turns on Gating Board 2
	setfreqDDS(0,0);
	setfreqDDS(1,0);
	setfreqDDS(2,0);
	setfreqDDS(3,0);
	ioupdate();	
	return;
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
	
	/*
	if (GPIO_Pin==GPIO_PIN_0){
	while (BSP_PB_GetState(BUTTON_KEY) != RESET)
	{};
		
 	click++;
		
	if (click==1){		
	  modulation_on(); 
	}
	
	if (click==2){
	  modulation_off();
		if (PhaseSteppingAuto==1)
			incrementphase();	 //Stepping Phase for next exposure		

    if (FreqSteppingAuto==1)
			incrementfreq(); //Stepping Frew for next exposure		
  	click=0;
	}
		
	printf("\nButton Press Detected");
	return;
	}
	*/
	
	
	if (GPIO_Pin==GPIO_PIN_2){
	 
	 if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)){
		//Rising Edge Detected
	  modulation_on();
		startcounter(); //Starts ticking the clock for exposure		 
	
	 }
	 else{	
		//Falling Edge Detected 	 	  
   
	 }
	}
	
	
}

//TIM1 Callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	//Handle TIM1 update for exposure control
	if (htim->Instance==TIM1){
	stopcounter();
	
	modulation_off(); 
	
	if (PhaseSteppingAuto==1)
			incrementphase();	 //Stepping Phase for next exposure		
		
	if (FreqSteppingAuto==1)
			incrementfreq(); //Stepping Freq for next exposure		 	  	 
  }
	
	//Handle TIM8 update for quad offset correction
  if (htim->Instance==TIM8){
	quadcount = 0;	
	//printf("\nFrame Triggered");	
	}

}

//TIM8 Callback
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){
	//Handle TIM8 update for quad offset correction
  if (htim->Instance==TIM8){

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


