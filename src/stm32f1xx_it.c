/**
 ******************************************************************************
 * @file    stm32f1xx_it.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    11-February-2014
 * @brief   Main Interrupt Service Routines.
 *          This file provides template for all exceptions handler and
 *          peripherals interrupt service routine.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
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
#include "stm32f1xx_it.h"
#include "global.h"

/** @addtogroup IO_Toggle
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M Processor Exceptions Handlers                          */
/******************************************************************************/


/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void)
{
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void)
{
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void)
{
	/* Go to infinite loop when Usage Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void)
{
}


/******************************************************************************/
/*                 STM32F1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_md.s).                                            */
/******************************************************************************/

/**
 * @brief  This function handles PPP interrupt request.
 * @param  None
 * @retval None
 */
/*void PPP_IRQHandler(void)
{
}*/

/**
 * @}
 */
void USART1_IRQHandler(void)
{
	u8 byte = 0;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		byte = USART_ReceiveData(USART1);
		USART_ClearFlag(USART1, USART_IT_RXNE);

	}
}

/**
  * @brief  This function handles USART2.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	u8 byte = 0;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		byte = USART_ReceiveData(USART2);
		USART_ClearFlag(USART2, USART_IT_RXNE);

		if(byte == '@'){
			ser2Cnt=0;
			queueSer2Send.buffer[ser2Cnt]=byte;
		}else{
			queueSer2Send.buffer[++ser2Cnt]=byte;
			if(byte == '!'){
				queueSer2Send.buffer[ser2Cnt+1]='\0';
				if(xQueueSer2In != NULL)
				{
					xQueueSendFromISR(xQueueSer2In, &queueSer2Send, &xHigherPriorityTaskWoken);
					if (xHigherPriorityTaskWoken)
					{
						portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // força a troca de contexto afim de minimizar latencia
					}
				}

			}
		}
	}
}

/**
 * @brief  This function handles TIM2.
 * @param  None
 * @retval None
 */
void TIM2_IRQHandler()
{
	// T = (360*2)/72e6 = 10us
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}


/**
 * @brief  This function handles External line 0 interrupt request.
 * @param  None
 * @retval None
 *
 */
void EXTI0_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		//UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

		//eint=1;

		//taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
		/* Clear the  EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);

		if(xSemaphoreBin1 != NULL){
			xSemaphoreGiveFromISR(xSemaphoreBin1, &xHigherPriorityTaskWoken);//Libera o semaforo
			if (xHigherPriorityTaskWoken)
			{
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // força a troca de contexto afim de minimizar latencia
			}
		}
	}
}

/**
 * @brief  This function handles EXTI1.
 * @param  None
 * @retval None
 */
void EXTI1_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
	{

		EXTI_ClearITPendingBit(EXTI_Line1);

		if(xSemaphoreBin2 != NULL){
			xSemaphoreGiveFromISR(xSemaphoreBin2, &xHigherPriorityTaskWoken); //Libera o semaforo
			if (xHigherPriorityTaskWoken)
			{
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			}
		}
	}
}

/**
 * @brief  This function handles EXTI3.
 * @param  None
 * @retval None
 */
void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line3) != RESET)
	{

		EXTI_ClearITPendingBit(EXTI_Line3);
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
