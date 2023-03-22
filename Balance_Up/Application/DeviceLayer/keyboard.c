
/* Includes ------------------------------------------------------------------*/
#include "keyboard.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Key_StateUpdate(void);
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
keyboard_t keyboard = {
	.lch_cmd.fric_cmd = lch_reset,
	.lch_cmd.magz_cmd = lch_reset,
	.lch_cmd.shoot_cmd = lch_reset,
	.gim_cmd = gim_reset,
	.gim_mode = gyro,
};
/* Private functions ---------------------------------------------------------*/
void Key_StateUpdate(void)
{
	keyboard.state.last_mouse_btn_l = keyboard.state.mouse_btn_l;
	keyboard.state.mouse_btn_l = rc_sensor.info->mouse_btn_l.status;
	keyboard.state.mouse_btn_r = rc_sensor.info->mouse_btn_r.status;
	keyboard.state.Q = rc_sensor.info->Q.status;
	keyboard.state.W = rc_sensor.info->W.status;
	keyboard.state.E = rc_sensor.info->E.status;
	keyboard.state.R = rc_sensor.info->R.status;
	keyboard.state.A = rc_sensor.info->A.status;
	keyboard.state.S = rc_sensor.info->S.status;
	keyboard.state.D = rc_sensor.info->D.status;
	keyboard.state.F = rc_sensor.info->F.status;
	keyboard.state.G = rc_sensor.info->G.status;
	keyboard.state.Z = rc_sensor.info->Z.status;
	keyboard.state.X = rc_sensor.info->X.status;
	keyboard.state.C = rc_sensor.info->C.status;
	keyboard.state.V = rc_sensor.info->V.status;
	keyboard.state.B = rc_sensor.info->B.status;
	keyboard.state.Shift = rc_sensor.info->Shift.status;
	keyboard.state.Ctrl = rc_sensor.info->Ctrl.status;	
}
/* Exported functions --------------------------------------------------------*/
void Key_StateCheck(void)
{
	Key_StateUpdate();
	
	keyboard.lch_cmd.fric_cmd = lch_reset;
	keyboard.lch_cmd.magz_cmd = lch_reset;
	keyboard.lch_cmd.shoot_cmd = lch_reset;
	/*  Ctrl(优先级顺序):关弹仓 关摩擦轮  */
	if (keyboard.state.Ctrl == down_K)
	{
		if (keyboard.lch_state.magz_state == magz_open)
		{
			keyboard.lch_cmd.magz_cmd = magz_close;
		}
		if (keyboard.lch_state.fric_state == fric_on)
		{
			keyboard.lch_cmd.fric_cmd = fric_off;
		}
	}
	/*  R:开摩擦轮  */
	if (keyboard.state.R == down_K)
	{
		keyboard.lch_cmd.fric_cmd = fric_on;
	}
	/*  G:开弹仓  */
	if (keyboard.state.G == down_K)
	{
		keyboard.lch_cmd.magz_cmd = magz_open;
	}
	/*  B:关弹仓  */
	if (keyboard.state.B == down_K)
	{
		keyboard.lch_cmd.magz_cmd = magz_close;
	}
	
	/*  mouse_btn_l  */
	if (keyboard.state.mouse_btn_l == up_K)
	{
		keyboard.lch_cmd.fric_cmd = fric_on;
	}
	if ((keyboard.state.mouse_btn_l == short_press_K) && (keyboard.lch_state.fric_state == fric_on))
	{
		keyboard.lch_cmd.shoot_cmd = single_shoot;
	}
	if (keyboard.state.mouse_btn_l == long_press_K)
	{
		keyboard.lch_cmd.shoot_cmd = keep_shoot;
	}
	
	
	keyboard.gim_cmd = gim_reset;
	/*  V:掉头  */
	if (keyboard.state.V == down_K)
	{
		keyboard.gim_cmd = gim_turn;
	}
	
	
	keyboard.gim_mode = gyro;
	/*  mouse_btn_r:自瞄  */
	if ((keyboard.state.mouse_btn_l == short_press_K) || (keyboard.state.mouse_btn_l == long_press_K))
	{
		keyboard.gim_mode = vision;
	}
}

