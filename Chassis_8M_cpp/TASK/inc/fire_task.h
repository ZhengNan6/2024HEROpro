#ifndef __FIRE_TASK_H__
#define __FIRE_TASK_H__

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif
    
void fire_task(void const *argument);
    
#ifdef __cplusplus
}
    
#endif

#endif