/*
 * weightedavg.h
 *
 *  Created on: May 11, 2021
 *      Author: jonas
 */

#ifndef INC_WEIGHTEDAVG_H_
#define INC_WEIGHTEDAVG_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#define min(a, b) (a < b ? a : b)
#define max(a, b) (a < b ? b : a)

typedef struct WEIGHTEDAVG_t
{
	uint32_t* calibMin;
	uint32_t* calibMax;
	uint32_t* weights;
	uint32_t maxVal;
	uint32_t* values;
	uint32_t size;
	uint32_t num, denum;
	uint32_t output;
	bool invert;
}WEIGHTEDAVG_t;

void WEIGHTEDAVG_Init(WEIGHTEDAVG_t* wavg,
		uint32_t size,
		uint32_t* values,
		uint32_t* calibMin,
		uint32_t* calibMax,
		uint32_t maxVal,
		uint32_t* weights);

void WEIGHTEDAVG_Invert(WEIGHTEDAVG_t* wavg, bool invert);

void WEIGHTEDAVG_SetData(WEIGHTEDAVG_t* wavg, uint32_t* data);

void WEIGHTEDAVG_Calibrate(WEIGHTEDAVG_t* wavg);
void WEIGHTEDAVG_AddSamples(WEIGHTEDAVG_t* wavg);
uint32_t WEIGHTEDAVG_Process(WEIGHTEDAVG_t* wavg);

#endif /* INC_WEIGHTEDAVG_H_ */
