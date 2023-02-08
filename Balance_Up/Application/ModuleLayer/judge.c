#include "judge.h"

judge_info_t judge_info = {
	.offline_max_cnt = 200,
};

judge_t judge = 
{
	.info = &judge_info,
	.heart_beat = judge_heart_beat,
	.work_state = DEV_OFFLINE,
};

void judge_heart_beat(judge_t *jud_sen)
{
	judge_info_t *jud_info = jud_sen->info;

	jud_info->offline_cnt++;
	if(jud_info->offline_cnt > jud_info->offline_max_cnt) {
		jud_info->offline_cnt = jud_info->offline_max_cnt;
		jud_sen->work_state = DEV_OFFLINE;
	} 
	else {
		/* ÀëÏß->ÔÚÏß */
		if(jud_sen->work_state == DEV_OFFLINE)
		{
			jud_sen->work_state = DEV_ONLINE;
		}
	}
}


