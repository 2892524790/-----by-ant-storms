#include "heartbeat_task.h"

#include "led.h"
#include "detect_task.h"

void heartbeat_task(void* pvParameters)
{
    for(;;)

    {

//		if (toe_is_error(errorListLength))
        //	led4_on();
//		else
        //	led4_off();

        osDelay(2);
    }
}




