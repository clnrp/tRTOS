#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#define VAR_GLOBAL // to prevent multiple definition
#include "global.h"

#define delay_ms(x) vTaskDelay(configTICK_RATE_HZ/1000*x)   // atrasa x milisegundos
#define delay_100ms(x) vTaskDelay(configTICK_RATE_HZ/10*x)  // atrasa x 100 milisegundos
#define delay_s(x) vTaskDelay(configTICK_RATE_HZ*x)         // atrasa x segundos

#define T1_TASK_PRIORITY    ( tskIDLE_PRIORITY + 3 )
#define T1_STACK_SIZE       configMINIMAL_STACK_SIZE + 200
#define T2_TASK_PRIORITY    ( tskIDLE_PRIORITY + 3 )
#define T2_STACK_SIZE       configMINIMAL_STACK_SIZE + 200
#define T3_TASK_PRIORITY    ( tskIDLE_PRIORITY + 1 )
#define T3_STACK_SIZE       configMINIMAL_STACK_SIZE + 200
#define T4_TASK_PRIORITY    ( tskIDLE_PRIORITY + 3 )
#define T4_STACK_SIZE       configMINIMAL_STACK_SIZE + 200
#define T5_TASK_PRIORITY    ( tskIDLE_PRIORITY + 1 )
#define T5_STACK_SIZE       configMINIMAL_STACK_SIZE + 200

void Hardware_Config(void);
void RCC_Config(void);
void USART2_Config(void);
void EXTI_Config(void);
void TIM2_Config(void);
void PWM_TIM3_Config(void);
void PWM_TIM4_Config(void);
void usart_sendData(USART_TypeDef* usart, char* buffer);

/*
 * o take decrementa 1 do semaforo
 * e o give adiciona 1
 * o valor inicial é 0
*/

static void prvTask1( void *pvParameters )
{
	char str[50];

	xSemaphoreBin1 = xSemaphoreCreateBinary(); // semáforo binario 0 ou 1
	xSemaphoreBin2 = xSemaphoreCreateBinary();
	xSemaphoreCnt1 = xSemaphoreCreateCounting(10, 0); // semáforo com contagem máxima de 10 e um contagem inicial de 0
	xSemaphoreMut1 = xSemaphoreCreateMutex();
	for(;;){

		if(xSemaphoreBin2 != NULL){
			if( xSemaphoreTake( xSemaphoreBin2, (TickType_t) 10 ) == pdTRUE )
			{
				sprintf(str,"task1\r\n");
				usart_sendData(USART2, str);
				//xSemaphoreGive(xSemaphoreBin1);
				xSemaphoreGive(xSemaphoreCnt1);
				xSemaphoreGive(xSemaphoreCnt1);
			}
		}

		delay_s(1);
	}
}

static void prvTask2( void *pvParameters )
{
	char str[50];
	int cnt_task3 = 0;
    for(;;){
    	if(xSemaphoreBin1 != NULL){
    		if( xSemaphoreTake( xSemaphoreBin1, (TickType_t) 10 ) == pdTRUE )
    		{
    			sprintf(str,"bin - task2\r\n");
    			usart_sendData(USART2, str);
    			if(cnt_task3++ > 5){
    				cnt_task3 = 0;
    				vTaskPrioritySet(task3Handle, tskIDLE_PRIORITY + 4);
    			}
    		}
    	}

    	if(xSemaphoreCnt1 != NULL){
    		if( xSemaphoreTake( xSemaphoreCnt1, (TickType_t) 10 ) == pdTRUE )
    		{
    			sprintf(str,"cnt - task2\r\n");
    			usart_sendData(USART2, str);
    		}
    	}

    	if(xSemaphoreMut1 != NULL){
    		if( xSemaphoreTake( xSemaphoreMut1, (TickType_t) 10 ) == pdTRUE )
    		{
    			sprintf(str,"mut - task2\r\n");
    			usart_sendData(USART2, str);
    			delay_s(5);
        		xSemaphoreGive(xSemaphoreMut1);
    		}else{
    			sprintf(str,"mut fail- task2\r\n");
    			usart_sendData(USART2, str); // a demora ao liberar o recurso, levaria a inversão de prioridade com tarefa 2
    			// para não ocorrer inversão de prioridade a tarefa 5 vai herdar a prioridade da tarefa 2
    		}
    	}

    	delay_s(2);
	}
}

static void prvTask3( void *pvParameters )
{
	static int t1_cnt=0;
	int priority0, priority;
	char str[50];
    for(;;){

    	if(xSemaphoreBin1 != NULL){ // Starvation, task 2 tem privilégio maior
    		if( xSemaphoreTake( xSemaphoreBin1, (TickType_t) 10 ) == pdTRUE )
    		{
    			t1_cnt+=1;
    			sprintf(str,"bin - task3 - %i\r\n",t1_cnt);
    			usart_sendData(USART2, str);

    			priority = uxTaskPriorityGet(task3Handle); // prioridade atual
    			priority0 = uxTaskPriorityGet(NULL); // prioridade na criação
    			if(priority0 != priority){ // voltar para a prioridade na criação
    				vTaskPrioritySet(task3Handle, priority0);
    			}
    		}
    	}

    	if(xSemaphoreCnt1 != NULL){
    		if( xSemaphoreTake( xSemaphoreCnt1, (TickType_t) 10 ) == pdTRUE )
    		{
    			sprintf(str,"cnt - task3 - %i\r\n",t1_cnt);
    			usart_sendData(USART2, str);
    		}
    	}

    	delay_s(1);
	}
}

static void prvTask4( void *pvParameters )
{
	xQueueSer2In = xQueueCreate(10, sizeof(xMsgQueue1)); // até 10 elementos
    for(;;){

    	if(xQueueSer2In != NULL)
    	{
    		if( xQueueReceive(xQueueSer2In, &queueSer2Receive,(TickType_t) 10))
    		{
    			usart_sendData(USART2, "Received!\r\n");
    			usart_sendData(USART2, queueSer2Receive.buffer);
    		}
    	}

    	delay_s(1);
	}
}

static void prvTask5( void *pvParameters )
{
	char str[50];
	int i,priv;
    for(;;){

    	if(xSemaphoreMut1 != NULL){
    		if( xSemaphoreTake( xSemaphoreMut1, (TickType_t) 10 ) == pdTRUE )
    		{
    			sprintf(str,"mut - task5\r\n");
    			usart_sendData(USART2, str);
    			delay_s(2); // a demora ao liberar o recurso, levaria a inversão de prioridade com tarefa 2
    			// para não ocorrer inversão de prioridade a tarefa 5 vai herdar a prioridade da tarefa 2
        		xSemaphoreGive(xSemaphoreMut1);
    		}else{
    			sprintf(str,"mut fail- task5\r\n");
    			usart_sendData(USART2, str);
    		}
    	}
    	delay_ms(700);
	}
}

int main(void)
{
	Hardware_Config();
	//EXTI_GenerateSWInterrupt(EXTI_Line0);
	eint=0;
	ser2Cnt = 0;

	xTaskCreate( prvTask1, "T1", T1_STACK_SIZE, NULL, T1_TASK_PRIORITY, NULL );
	xTaskCreate( prvTask2, "T2", T2_STACK_SIZE, NULL, T2_TASK_PRIORITY, NULL );
	xTaskCreate( prvTask3, "T3", T3_STACK_SIZE, NULL, T3_TASK_PRIORITY, &task3Handle );
	xTaskCreate( prvTask4, "T4", T4_STACK_SIZE, NULL, T4_TASK_PRIORITY, NULL );
	xTaskCreate( prvTask5, "T5", T5_STACK_SIZE, NULL, T5_TASK_PRIORITY, NULL );

	vTaskStartScheduler();
}

void Hardware_Config(void)
{
	RCC_Config();
	USART2_Config();
	TIM2_Config(); // simple timer
	EXTI0_Config();
	EXTI_Config(); // external interrupt
	PWM_TIM3_Config();
}

void RCC_Config(void)
{
	/* CLOCK */
	RCC_DeInit();                                            // Start with the clocks in their expected state.
	RCC_HSEConfig(RCC_HSE_ON);                               // Enable HSE (high speed external clock).

	RCC_ClockSecuritySystemCmd(ENABLE);

	while(RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET ){}    // Wait till HSE is ready.

	RCC_HCLKConfig(RCC_SYSCLK_Div1);                         // HCLK = SYSCLK
	RCC_PCLK2Config(RCC_HCLK_Div1);                          // PCLK2 = HCLK
	RCC_PCLK1Config(RCC_HCLK_Div2);                          // PCLK1 = HCLK/2
	RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);     // PLLCLK = 8MHz * 9 = 72 MHz.

	/* ADCCLK = PCLK2/4 */
	RCC_ADCCLKConfig(RCC_PCLK2_Div4);

	RCC_PLLCmd(ENABLE);                                      // Enable PLL.

	/* Wait till PLL is ready. */
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{
	}

	/* Select PLL as system clock source. */
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	/* Wait till PLL is used as system clock source. */
	while(RCC_GetSYSCLKSource() != 0x08)
	{
	}

	// habilitar clock por periferico
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

	/* Configure HCLK clock as SysTick clock source. */
	//SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
}

void EXTI0_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable GPIOA clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* Configure PA.00 pin as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Enable AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Connect EXTI0 Line to PA.00 pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

	/* Configure EXTI0 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0; // PA0 is connected to EXTI_Line0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; // Triggers on rising edge
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //  todos os bits de prioridade devem ser designados como bits de prioridade de preempção

	/* Enable and set EXTI0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//NVIC_SetPriority(EXTI0_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY);

	/* As funções FreeRTOS que terminam em "FromISR" são seguras para interrupções, mas mesmo essas funções não podem ser chamadas de
	interrupções que tenham prioridade lógica acima da prioridade definida por configMAX_SYSCALL_INTERRUPT_PRIORITY que é definido no
	arquivo de cabeçalho FreeRTOSConfig.h, portanto, qualquer rotina de serviço de interrupção que use uma função RTOS API deve ter sua
	prioridade configurada manualmente 	para um valor numericamente igual ou maior que a configuração configMAX_SYSCALL_INTERRUPT_PRIORITY.
	Isso garante que a prioridade lógica da interrupção seja igual ou menor que a configuração configMAX_SYSCALL_INTERRUPT_PRIORITY */
}

void EXTI_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// EXTI1, linha 1 -> PA1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1); // Interrupt line1

	EXTI_InitStructure.EXTI_Line = EXTI_Line1; // PA1 is connected to EXTI_Line1
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; // Interrupt mode
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // Triggers on falling edge
	EXTI_InitStructure.EXTI_LineCmd = ENABLE; // Enable interrupt
	EXTI_Init(&EXTI_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// EXTI3, linha 3 -> PA3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource3); // Interrupt line3

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	EXTI_InitStructure.EXTI_Line = EXTI_Line3; // PA3 is connected to EXTI_Lin3
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; // Interrupt mode
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // Triggers on falling edge
	EXTI_InitStructure.EXTI_LineCmd = ENABLE; // Enable interrupt
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM2_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /*
     *         TIM_Period * (TIM_Prescaler+1)
     * time = -------------------------------
     *                  72000000
     */


    TIM_DeInit(TIM2);

    /* TIM2 configuration */
    TIM_TimeBaseStructure.TIM_Period = 360;
    TIM_TimeBaseStructure.TIM_Prescaler = 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_ARRPreloadConfig(TIM2, ENABLE);

    TIM_UpdateRequestConfig(TIM2, TIM_UpdateSource_Regular);

    /* Immediate load of TIM2 Precaler value */
    //TIM_PrescalerConfig(TIM2, ((SystemCoreClock/1200) - 1), TIM_PSCReloadMode_Immediate);

    /* Clear TIM2 update pending flag */
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);

    /* TIM2 Interrupt Config */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable TIM2 Update interrupt */
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    /* TIM2 enable counter */
    TIM_Cmd(TIM2, ENABLE);
}

void PWM_TIM3_Config()
{
	uint16_t TimerPeriod = 500;
	uint16_t ChannelPulse = 0;
	GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    //GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

    TIM_DeInit(TIM3);
    TIM_TimeBaseStructure.TIM_Prescaler = 400;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_Cmd(TIM3, ENABLE);

	ChannelPulse = 0; // 0%

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_Pulse = ChannelPulse;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);

	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

void PWM_TIM4_Config(void)
{
	uint16_t TimerPeriod = 500;
	uint16_t ChannelPulse = 0;
	GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_DeInit(TIM4);
    TIM_TimeBaseStructure.TIM_Prescaler = 400;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_Cmd(TIM4, ENABLE);

	ChannelPulse = 250; // 50%

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_Pulse = ChannelPulse;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	//TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	//TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
}

void USART1_Config(void)
{
	GPIO_InitTypeDef GPIO_Structure;
	USART_InitTypeDef USART_Structure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* Usart */
	/* Configure PA.9 (usart1_tx), PA.10 (usart1_rx)  --------------------------*/
	// Usart1 TX
	GPIO_Structure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Structure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Structure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_Structure);

	// Usart1 RX
	GPIO_Structure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Structure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Structure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_Structure);

	USART_DeInit(USART1);
	USART_Structure.USART_BaudRate = 9600;
	USART_Structure.USART_WordLength = USART_WordLength_8b;
	USART_Structure.USART_StopBits = USART_StopBits_1;
	USART_Structure.USART_Parity = USART_Parity_No;
	USART_Structure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Structure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_StructInit(&USART_Structure);
	USART_Init(USART1,&USART_Structure);

	// USART Interrupt Config
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable USARTy Receive and Transmit interrupts */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

	USART_Cmd(USART1, ENABLE);
}

void USART2_Config(void){
	GPIO_InitTypeDef GPIO_Structure;
	USART_InitTypeDef USART_Structure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//------------------------- USART2
	// usart2 tx
	GPIO_Structure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Structure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Structure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_Structure);

	// usart2 rx
	GPIO_Structure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Structure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Structure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_Structure);

	USART_DeInit(USART2);
	USART_Structure.USART_BaudRate = 115200;
	USART_Structure.USART_WordLength = USART_WordLength_8b;
	USART_Structure.USART_StopBits = USART_StopBits_1;
	USART_Structure.USART_Parity = USART_Parity_No;
	USART_Structure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Structure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_StructInit(&USART_Structure);
	USART_Init(USART2,&USART_Structure);

	// USART Interrupt Config
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable USARTy Receive and Transmit interrupts */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_TXE, DISABLE);

	USART_Cmd(USART2, ENABLE);
}

void usart_sendData(USART_TypeDef* usart, char* buffer)
{
	u16 cnt, length, time_out=0;

	length = strlen(buffer);
	for (cnt = 0; cnt < length; cnt++){
		USART_SendData(usart, buffer[cnt]);
		time_out = 0;
		while((USART_GetFlagStatus(usart, USART_FLAG_TXE) == RESET ) && (time_out < 50) ){ // aguarda o dado ser enviado
			_delay(1000);
			time_out++;
		}
	}

}

void _delay(u32 value)
{
	for(; value!=0; value--)
		asm("nop");
}




