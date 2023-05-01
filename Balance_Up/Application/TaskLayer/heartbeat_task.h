#ifndef __HEARTBEAT_TASK
#define __HEARTBEAT_TASK

#include "cmsis_os.h"
#include "main.h"

#include "module.h"

extern IWDG_HandleTypeDef hiwdg;

void StartHeartBeatTask(void const * argument);

#endif
