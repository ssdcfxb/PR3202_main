#ifndef __CONTROL_TASK
#define __CONTROL_TASK

#include "cmsis_os.h"
#include "main.h"
#include "module.h"

void StartControlTask(void const * argument);
void Start_Super_Task(void const * argument);
#endif


//������û����ʱ��ȥ�Һ���
//bmi2_write_config_file
