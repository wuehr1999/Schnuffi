/*
 * weightedavg.c
 *
 *  Created on: May 11, 2021
 *      Author: jonas
 */
#include "weightedavg.h"

void WEIGHTEDAVG_Init(WEIGHTEDAVG_t* wavg,
		uint32_t size,
		uint32_t* values,
		uint32_t* calibMin,
		uint32_t* calibMax,
		uint32_t maxVal,
		uint32_t* weights)
{
	wavg->size = size;
	wavg->values = values;
	wavg->calibMin = calibMin;
	wavg->calibMax = calibMax;
	wavg->maxVal = maxVal;
	wavg->weights = weights;

	wavg->output = 0;
	wavg->num = 0;
	wavg->denum = 0;
	wavg->invert = false;

	for(uint32_t i = 0; i < size; i++)
	{
		wavg->calibMin[i] = 0xffffffff;
		wavg->calibMax[i] = 0x0;
	}
}

void WEIGHTEDAVG_Invert(WEIGHTEDAVG_t* wavg, bool invert)
{
	wavg->invert = invert;
}

void WEIGHTEDAVG_SetData(WEIGHTEDAVG_t* wavg, uint32_t* data)
{
	wavg->values = data;
}

void WEIGHTEDAVG_Calibrate(WEIGHTEDAVG_t* wavg)
{
	for(uint32_t i = 0; i < wavg->size; i++)
	{
		wavg->calibMin[i] = min(wavg->values[i], wavg->calibMin[i]);
		wavg->calibMax[i] = max(wavg->values[i], wavg->calibMax[i]);
	}
}

void WEIGHTEDAVG_AddSamples(WEIGHTEDAVG_t* wavg)
{
	for(uint32_t i = 0; i < wavg->size; i++)
	{
		wavg->values[i] = min(wavg->values[i], wavg->calibMax[i]);
		wavg->values[i] = max(wavg->values[i], wavg->calibMin[i]);
		wavg->values[i] -= wavg->calibMin[i];
		wavg->values[i] *= wavg->maxVal;
		wavg->values[i] /= (wavg->calibMax[i] - wavg->calibMin[i]);
		if(wavg->invert)
		{
			wavg->num += (wavg->maxVal - wavg->values[i]) * wavg->weights[i];
		}
		else
		{
			wavg->num += wavg->values[i] * wavg->weights[i];
		}
		wavg->denum += wavg->values[i];
	}
}

uint32_t WEIGHTEDAVG_Process(WEIGHTEDAVG_t* wavg)
{
	if(wavg->denum > 0)
	{
		wavg->output = wavg->num / wavg->denum;
	}
	wavg->num = 0;
	wavg->denum = 0;

	return wavg->output;
}

