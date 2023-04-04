
/* Includes ------------------------------------------------------------------*/
#include "keyboard.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Key_StateUpdate(void);
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
keyboard_t keyboard;
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
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
