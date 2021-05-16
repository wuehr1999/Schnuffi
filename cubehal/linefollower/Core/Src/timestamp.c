/*
 * timestamp.c
 *
 *  Created on: May 10, 2021
 *      Author: jonas
 */
#include "timestamp.h"

void Timestamp_Start(Timestamp_t* stmp)
{
	stmp->start = HAL_GetTick();
}

uint32_t Timestamp_Stop(Timestamp_t* stmp)
{
	stmp->stop = HAL_GetTick();
	return stmp->stop - stmp->start;
}

