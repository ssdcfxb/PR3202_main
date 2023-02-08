/* Includes ------------------------------------------------------------------*/
#include "imu_sensor.h"
#include "BMI.h"
#include "drv_gpio.h"
#include "drv_tick.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void imu_init(imu_sensor_t *self);
void imu_heart_beat(work_state_t *heart);
extern void imu_update(imu_sensor_t *self);
extern void transform_init(void);
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct bmi2_dev bmi270;
struct bmi2_dev ex_bmi270;

int8_t rs, rs1;

bmi_t bmi_client = {

	.dev = &bmi270,
	.init = &bmi_init,	

};

bmi_t ex_bmi_client = {

	.dev = &ex_bmi270,
	.init = &bmi_init,	

};

imu_info_t imu_info = 
{
	.init_flag = 0,
};

/* Exported variables --------------------------------------------------------*/
imu_sensor_t imu_sensor = {

	.bmi = &bmi_client,
	.ex_bmi = &ex_bmi_client,
	.info = &imu_info,
	.driver.tpye = DR_SPI1,
	.work_state.dev_state = DEV_OFFLINE,
	.id = DEV_ID_IMU,	
	.work_state.offline_max_cnt = 50,	
	
	.init = &imu_init,
	.update = &imu_update,
  .heart_beat = &imu_heart_beat,
};

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void imu_init(struct imu_struct *self)
{
	int8_t rslt;
	uint32_t tickstart = HAL_GetTick();

	self->work_state.dev_state = DEV_OFFLINE;

	self->bmi->read = bmi2_get_regs;
  self->bmi->write = bmi2_set_regs;
	
	if(self->driver.tpye == DR_SPI1 || self->driver.tpye == DR_SPI2 || self->driver.tpye == DR_SPI3){

		self->bmi->drive_type = BMI2_SPI_INTF;
		if(self->driver.tpye == DR_SPI2)
		{
			self->bmi->device_aces = BMI2_INT_ACES;
		}
		else if(self->driver.tpye == DR_SPI1)
		{
			self->bmi->device_aces = BMI2_EXT_ACES;
		}

	}
	else if(self->driver.tpye == DRV_IIC){
	
		self->bmi->drive_type = BMI2_I2C_INTF;
		
	}
	
	rslt = self->bmi->init(self->bmi->dev,self->bmi->drive_type,self->bmi->device_aces);
	rs = rslt;

	while(rslt)
	{
        self->work_state.err_code = IMU_INIT_ERR;
        rslt = self->bmi->init(self->bmi->dev,self->bmi->drive_type,self->bmi->device_aces);
		rs = rslt;
	}	

	
	
//	self->ex_bmi->read = bmi2_get_regs;
//  self->ex_bmi->write = bmi2_set_regs;
//	
//	self->ex_bmi->drive_type = BMI2_SPI_INTF;
//	self->ex_bmi->device_aces = BMI2_EXT_ACES;
//	
//	rslt = self->ex_bmi->init(self->ex_bmi->dev,self->ex_bmi->drive_type,self->ex_bmi->device_aces);
//	rs1 = rslt;
//	
//	while(rslt)
//	{
//	rslt = self->ex_bmi->init(self->ex_bmi->dev,self->ex_bmi->drive_type,self->ex_bmi->device_aces);
//		rs1 = rslt;
//	}	

	self->work_state.dev_state = DEV_ONLINE;
	self->work_state.err_code = IMU_NONE_ERR;
	
	/* Ðý×ª¾ØÕó³õÊ¼»¯ */
	transform_init();
	
	self->info->init_flag = 1;
}


void imu_heart_beat(work_state_t *heart){


	heart->offline_cnt++;
	if(heart->offline_cnt > heart->offline_max_cnt) 
	{
		heart->offline_cnt = heart->offline_max_cnt;
		heart->dev_state = DEV_OFFLINE;
	}
	else 
	{
		if(heart->dev_state == DEV_OFFLINE)
			heart->dev_state = DEV_ONLINE;
	}

}

