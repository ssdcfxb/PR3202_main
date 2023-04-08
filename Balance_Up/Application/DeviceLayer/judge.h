#ifndef __JUDGE_H
#define __JUDGE_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "rp_config.h"

#include "judge_defs.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct 
{
	ext_rfid_status_t rfid_status;
	ext_game_status_t game_status;
	ext_game_robot_status_t game_robot_status;
	ext_power_heat_data_t power_heat_data;
	ext_shoot_data_t shoot_data;
	ext_robot_hurt_t ext_robot_hurt;
	uint8_t	my_color;
	
	int16_t	 offline_cnt;
	int16_t	 offline_max_cnt;
}judge_info_t;

typedef struct judge_struct
{
	judge_info_t      *info;
	void				     (*heart_beat)(struct judge_struct *self);
	dev_work_state_t	work_state;
	dev_id_t			    id;
}judge_t;

extern judge_t judge;

/* Exported functions --------------------------------------------------------*/
void judge_heart_beat(judge_t *jud_sen);

#endif
