
#ifndef __KEYBOARD_H
#define __KEYBOARD_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "rc_sensor.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/*  ����״̬  */
typedef struct keyboard_state_struct {
  key_board_status_e        last_mouse_btn_l;     //��һ��������
  key_board_status_e        mouse_btn_l;          //������
  key_board_status_e        mouse_btn_r;          //����Ҽ�
  key_board_status_e        Q;                    //����Q
  key_board_status_e        W;                    //����W
  key_board_status_e        E;                    //����E
  key_board_status_e        R;                    //����R
  key_board_status_e        A;                    //����A
  key_board_status_e        S;                    //����S
  key_board_status_e        D;                    //����D
  key_board_status_e        F;                    //����F
  key_board_status_e        G;                    //����G
  key_board_status_e        Z;                    //����Z
  key_board_status_e        X;                    //����X
  key_board_status_e        C;                    //����C
  key_board_status_e        V;                    //����V
  key_board_status_e        B;                    //����B
  key_board_status_e        Shift;                //����Shift
  key_board_status_e        Ctrl;                 //����Ctrl
}keyboard_state_t;

/*  �������ָ��ö��  */
typedef enum {
	lch_reset,  	  // ���ܸ�λ
	fric_on,   		  // ��Ħ����
	fric_off,   		// ��Ħ����
	magz_open,      // ������
	magz_close,     // �ص���
	single_shoot,   // ����
	keep_shoot      // ����
}lch_cmd_e;

/*  �������״̬  */
typedef struct launcher_state_struct {
	lch_cmd_e		fric_state;
	lch_cmd_e		magz_state;
	lch_cmd_e		shoot_state;
}launcher_state_t;

/*  �������ָ��  */
typedef struct launcher_command_struct {
	lch_cmd_e		fric_cmd;
	lch_cmd_e		magz_cmd;
	lch_cmd_e		shoot_cmd;
}lch_cmd_t;

/*  ��ָ̨��  */
typedef enum {
	gim_reset,   		// ���ܸ�λ
	gim_turn,	 	    // ��ͷ
}gim_cmd_e;

/*  ��̨ģʽ  */
typedef enum {
	gyro,	 	    // ������ģʽ
	vision,   	// ����ģʽ
	keep,				// ����
}gim_mode_e;

typedef struct keyboard_struct {
	keyboard_state_t	state;
	launcher_state_t	lch_state;
	lch_cmd_t					lch_cmd;
	gim_cmd_e					gim_cmd;
	gim_mode_e				gim_mode;
}keyboard_t;

extern keyboard_t keyboard;

/* Exported functions --------------------------------------------------------*/
void Key_StateCheck(void);


#endif
