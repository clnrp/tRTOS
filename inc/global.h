#ifndef GLOBAL_H_
#define GLOBAL_H_

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#ifndef VAR_GLOBAL // to prevent multiple definition
#define VAR_GLOBAL extern
#else
#define VAR_GLOBAL
#endif

typedef struct
{
	u8   id;
	u16  length;
	u8   buffer[50];
} xMsgQueue1;

VAR_GLOBAL SemaphoreHandle_t xSemaphoreBin1;
VAR_GLOBAL SemaphoreHandle_t xSemaphoreBin2;
VAR_GLOBAL SemaphoreHandle_t xSemaphoreCnt1;
VAR_GLOBAL TaskHandle_t task3Handle;

SemaphoreHandle_t xSemaphoreMut1;

VAR_GLOBAL xMsgQueue1 queueSer2Send,queueSer2Receive;
VAR_GLOBAL xQueueHandle xQueueSer2In;

VAR_GLOBAL int ser2Cnt;
VAR_GLOBAL int eint;

#endif /* GLOBAL_H_ */
