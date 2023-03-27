#ifndef __CONTROL_TASK
#define __CONTROL_TASK

#include "cmsis_os.h"
#include "main.h"
#include "launcher.h"
#include "gimbal.h"
#include "can_protocol.h"
void StartControlTask(void const * argument);

#endif
