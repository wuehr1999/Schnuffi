/*
 * vl53l0x.h
 *
 *  Created on: May 10, 2021
 *      Author: jonas
 */

#ifndef INC_VL53L0X_H_
#define INC_VL53L0X_H_

#include <stdbool.h>
#include <stdint.h>
#include <stm32f1xx_hal.h>
#include "wire.h"

typedef struct VL53_t
{
	WIRE_t* wire;
	uint8_t address;
	GPIO_TypeDef* xsGPIO;
	uint16_t xsPIN;
	uint8_t lastStatus;
	uint8_t stopVariable;
    uint16_t io_timeout;
    bool did_timeout;
    uint16_t timeout_start_ms;
    uint32_t measurement_timing_budget_us;
}VL53_t;

typedef enum vcselPeriodType { VcselPeriodPreRange, VcselPeriodFinalRange }vcselPeriodType;

bool VL53_Init(VL53_t* tof, WIRE_t* wire, uint8_t address, GPIO_TypeDef* xsGPIO, uint16_t xsPIN);

void VL53_SetTimeout(VL53_t* tof, int timeout);
bool VL53_SetSignalRateLimit(VL53_t* tof, float limit);
bool VL53_SetVcselPulsePeriod(VL53_t* tof, vcselPeriodType type, uint8_t period_pclks);
bool VL53_SetMeasurementTimingBudget(VL53_t* tof, uint32_t budget_us);
bool VL53_TimeoutOccured(VL53_t* tof);
uint16_t VL53_ReadRangeSingle_mm(VL53_t* tof);

#endif /* INC_VL53L0X_H_ */
