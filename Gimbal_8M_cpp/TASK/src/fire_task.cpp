#include "Gimbal.hpp"
#include "RC.h"

#ifdef  __cplusplus
extern "C"
{
#endif
    
#include "fire_task.h"
#include "bsp_can.h"
    
#ifdef  __cplusplus
}
#endif

void FIRE_TASK(void const *argument)
{
    while(1)
    {
        vTaskDelay(1);
    }
}