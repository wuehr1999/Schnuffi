#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <stdio.h>
#include <stm32f1xx_hal.h>

typedef struct Timestamp_t
{
	uint32_t start, stop;
}Timestamp_t;

void Timestamp_Start(Timestamp_t* stmp);
uint32_t Timestamp_Stop(Timestamp_t* stmp);
#endif
