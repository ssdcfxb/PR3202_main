#ifndef __BMI_H
#define __BMI_H

#include <stdio.h>
#include "bmi2.h"
#include "bmi270.h"
#include "common.h"
#include "arm_math.h"

//typedef BMI2_INTF_RETURN_TYPE (*bmi2_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
typedef struct bmi_struct {

	struct bmi2_dev *dev;
	
	uint8_t drive_type;
	
	int8_t (*init)(struct bmi2_dev *bmi,uint8_t intf);
	
	int8_t (*read)(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi2_dev *dev);
	int8_t (*write)(uint8_t reg_addr, const uint8_t *data, uint16_t len, struct bmi2_dev *dev);

} bmi_t;


int8_t bmi_init(struct bmi2_dev *bmi2_dev,uint8_t intf);






#endif


