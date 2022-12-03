#include "imu_sensor.h"
#include "drv_gpio.h"

void imu_init(imu_sensor_t *self);
void imu_heart_beat(work_state_t *heart);
void imu_update(imu_sensor_t *self, uint8_t *rxBuf);

struct bmi2_dev bmi270;

bmi_t bmi_client = {

	.dev = &bmi270,
	.init = &bmi_init,	

};


imu_sensor_t imu_sensor = {

	.bmi = &bmi_client,
	.driver.tpye = DR_SPI2,
	.work_state.dev_state = DEV_OFFLINE,
	.id = DEV_ID_IMU,	
	.info.init_flag = 0,
	.work_state.offline_max_cnt = 50,	
	
	.init = &imu_init,
	.update = &imu_update,
  .heart_beat = &imu_heart_beat,
};


void imu_init(struct imu_struct *self)
{
	int8_t rslt;
	uint32_t tickstart = HAL_GetTick();

	self->work_state.dev_state = DEV_OFFLINE;

	self->bmi->read = bmi2_get_regs;
  self->bmi->write = bmi2_set_regs;
	
	if(self->driver.tpye == DR_SPI1 || self->driver.tpye == DR_SPI2 || self->driver.tpye == DR_SPI3){

		self->bmi->drive_type = BMI2_SPI_INTF;

	}
	else if(self->driver.tpye == DRV_IIC){
	
		self->bmi->drive_type = BMI2_I2C_INTF;
		
	}
	
	rslt = self->bmi->init(self->bmi->dev,self->bmi->drive_type);
	

	while(rslt)
	{
        self->work_state.err_code = IMU_INIT_ERR;
        rslt = self->bmi->init(self->bmi->dev,self->bmi->drive_type);
	}	

	self->work_state.dev_state = DEV_ONLINE;
	self->work_state.err_code = IMU_NONE_ERR;
	
	self->info.init_flag = 1;
}

void imu_update(struct imu_struct *self, uint8_t *rxBuf)
{
	
	int16_t rx_data[6];
	
	rx_data[0] = (int16_t)rxBuf [0] | ( (int16_t)rxBuf[1] << 8);
	rx_data[1] = (int16_t)rxBuf [2] | ( (int16_t)rxBuf[3] << 8);
	rx_data[2] = (int16_t)rxBuf [4] | ( (int16_t)rxBuf[5] << 8);
	
	rx_data[3] = (int16_t)rxBuf [6] | ( (int16_t)rxBuf[7] << 8);
	rx_data[4] = (int16_t)rxBuf [8] | ( (int16_t)rxBuf[9] << 8);
	rx_data[5] = (int16_t)rxBuf[10] | ( (int16_t)rxBuf[11] << 8);		

	self->info.raw_info.acc_x  = rx_data[0];
	self->info.raw_info.acc_y  = rx_data[1];
	self->info.raw_info.acc_z  = rx_data[2];
	self->info.raw_info.gyro_x = rx_data[3];
	self->info.raw_info.gyro_x = rx_data[4];
	self->info.raw_info.gyro_x = rx_data[5];
	
	
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

int16_t buff[6];
struct bmi2_sens_data imu_data;

int32_t t1,t2,tf;
float r_f;
extern SPI_HandleTypeDef hspi2;
void Get_IMU_Data(void){

		uint8_t len=12,reg;
	
	
		uint8_t da[len + 1],data[12];
		
		t1 = micros();
		
		BMI_CS_LOW();
	
	  reg = 0x0c|0x80;
		HAL_SPI_Transmit(&hspi2, &reg,  1, 1000);
		HAL_SPI_Receive(&hspi2, da, len+1, 1000);
		
		for(uint8_t i=0;i<len+1;i++){
		
			data[i] = da[i+1];
		
		}
		BMI_CS_HIG();
		
	buff[0] = (int16_t)data[0] | ( (int16_t)data[1] << 8);
	buff[1] = (int16_t)data[2] | ( (int16_t)data[3] << 8);
	buff[2] = (int16_t)data[4] | ( (int16_t)data[5] << 8);
	
	buff[3] = (int16_t)data[6] | ( (int16_t)data[7] << 8);
	buff[4] = (int16_t)data[8] | ( (int16_t)data[9] << 8);
	buff[5] = (int16_t)data[10] | ( (int16_t)data[11] << 8);		
		
		t2 = micros();
		tf = t2 - t1;
}

volatile uint32_t start,end,fps;
volatile uint32_t stay_time = 100;
volatile uint32_t out,inn;
volatile int32_t stay;
uint32_t *addl;
uint8_t daa1,daa2;
void BMI_DATA(void)
{
//	uint8_t aaa = 0x2c,bbb = 0xac;
	
//	imu.bmi->write(BMI2_ACC_CONF_ADDR,&aaa,1,imu.bmi->dev);
//	imu.bmi->read(BMI2_ACC_CONF_ADDR,&daa1,1,imu.bmi->dev);


//	imu.bmi->write(BMI2_GYR_CONF_ADDR,&bbb,1,imu.bmi->dev);
//	imu.bmi->read(BMI2_GYR_CONF_ADDR,&daa2,1,imu.bmi->dev);
	

//		inn = micros();
		Get_IMU_Data();



		
}

