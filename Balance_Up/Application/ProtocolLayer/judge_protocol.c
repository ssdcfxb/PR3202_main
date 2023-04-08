 
/* Includes ------------------------------------------------------------------*/
#include "judge_protocol.h"

#include "slave.h"
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void judge_update(judge_t *self, slave_info_t *info)
{
	self->info->game_robot_status.shooter_id1_17mm_cooling_limit = info->shooter_cooling_limit;
	self->info->game_robot_status.shooter_id1_17mm_speed_limit = info->shooter_speed_limit;
	self->info->power_heat_data.shooter_id1_17mm_cooling_heat = info->shooter_cooling_heat;
	self->info->shoot_data.bullet_speed = info->bullet_speed;
	self->info->my_color = info->my_color;
	
	self->info->offline_cnt = 0;
}
