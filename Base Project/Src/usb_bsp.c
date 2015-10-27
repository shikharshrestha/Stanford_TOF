/**
  ******************************************************************************
  * @file    usb_bsp.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file is responsible to offer board support package and is 
  *          configurable by user.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usb_bsp.h"
#include "usbd_conf.h"
#include "tm_stm32f4_usb_vcp.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_gpio_ex.h"
#include "defines.h"

#ifndef USB_VCP_NVIC_PRIORITY
#define USB_VCP_NVIC_PRIORITY			0x01
#endif

#ifndef USB_VCP_NVIC_SUBPRIORITY
#define USB_VCP_NVIC_SUBPRIORITY		0x01
#endif

extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler(USB_OTG_CORE_HANDLE *pdev);

/**
* @brief  USB_OTG_BSP_Init
*         Initilizes BSP configurations
* @param  None
* @retval None
*/

void USB_OTG_BSP_Init(USB_OTG_CORE_HANDLE *pdev) {
  GPIO_InitTypeDef GPIO_InitStructure;   
#ifdef USE_USB_OTG_FS
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStructure.Pin = 	GPIO_PIN_11;

	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH ;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP ;
	GPIO_InitStructure.Pull =  GPIO_NOPULL ;
	GPIO_InitStructure.Alternate = GPIO_AF10_OTG_FS; 
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);  

	GPIO_InitStructure.Pin = 	GPIO_PIN_12;

	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH ;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP ;
	GPIO_InitStructure.Pull =  GPIO_NOPULL ;
	GPIO_InitStructure.Alternate = GPIO_AF10_OTG_FS; 
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);  
	
#ifndef USB_VCP_DISABLE_VBUS
	// Configure  VBUS Pin
	GPIO_InitStructure.Pin = GPIO_PIN_9;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT ;
	GPIO_InitStructure.Pull = GPIO_NOPULL  ;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);    
#endif

#ifndef USB_VCP_DISABLE_ID
	// Configure ID pin
	GPIO_InitStructure.Pin = GPIO_PIN_10;
	GPIO_InitStructure.Mode =  GPIO_MODE_AF_OD ;
	GPIO_InitStructure.Pull = GPIO_PULLUP;  
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	GPIO_InitStructure.Alternate = GPIO_AF10_OTG_FS; 
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);  
#endif

	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_USB_OTG_FS_CLK_ENABLE();
  
	#endif


}

/**
* @brief  USB_OTG_BSP_EnableInterrupt
*         Enabele USB Global interrupt
* @param  None
* @retval None
*/
void USB_OTG_BSP_EnableInterrupt(USB_OTG_CORE_HANDLE *pdev) {

#ifdef USE_USB_OTG_FS

	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_1);
	HAL_NVIC_SetPriority(OTG_FS_IRQn,USB_VCP_NVIC_PRIORITY,USB_VCP_NVIC_SUBPRIORITY + 2);
	HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
	
	
#endif
}
/**
* @brief  USB_OTG_BSP_uDelay
*         This function provides delay time in micro sec
* @param  usec : Value of delay required in micro sec
* @retval None
*/
void USB_OTG_BSP_uDelay (const uint32_t usec) {
	uint32_t count = 0;
	const uint32_t utime = (120 * usec / 7);
	
	do
	{
		if ( ++count > utime ) {
			return ;
		}
	} while (1);
}


/**
* @brief  USB_OTG_BSP_mDelay
*          This function provides delay time in milli sec
* @param  msec : Value of delay required in milli sec
* @retval None
*/
void USB_OTG_BSP_mDelay (const uint32_t msec) {
	USB_OTG_BSP_uDelay(msec * 1000);   
}


#ifdef USE_USB_OTG_FS

void OTG_FS_WKUP_IRQHandler(void) {
	__HAL_GPIO_EXTI_CLEAR_IT(USB_OTG_FS_WAKEUP_EXTI_LINE);
}

void OTG_FS_IRQHandler(void) {
	USBD_OTG_ISR_Handler (&USB_OTG_dev);
}

#else
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
extern uint32_t USBD_OTG_EP1IN_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
extern uint32_t USBD_OTG_EP1OUT_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);

void OTG_HS_WKUP_IRQHandler(void) {
	EXTI_ClearITPendingBit(EXTI_Line20);
}

void OTG_HS_IRQHandler(void) {
	USBD_OTG_ISR_Handler(&USB_OTG_dev);
}

void OTG_HS_EP1_IN_IRQHandler(void) {
	USBD_OTG_EP1IN_ISR_Handler(&USB_OTG_dev);
}

void OTG_HS_EP1_OUT_IRQHandler(void) {
	USBD_OTG_EP1OUT_ISR_Handler(&USB_OTG_dev);
}

#endif

