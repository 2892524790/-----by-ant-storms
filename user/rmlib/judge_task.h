#ifndef _JUDGE_TASK_H_
#define _JUDGE_TASK_H_

#include "stm32f4xx.h"                  // Device header
#include "judgement_info.h"
#include "protocol.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "FreeRTOSConfig.h"

 #ifndef __cplusplus /* In C++, 'bool', 'true' and 'false' and keywords */
    #define bool _Bool
    #define true 1
    #define false 0
  #else
    #ifdef __GNUC__
      /* GNU C++ supports direct inclusion of stdbool.h to provide C99
         compatibility by defining _Bool */
      #define _Bool bool
    #endif
  #endif


#define FRAME_HEADER   0xA5   //??

void Judge_task(void *pvParameters);
void Judge_DataVerify(u8 *Buff);

#endif


