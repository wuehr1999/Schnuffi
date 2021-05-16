/*
 * queue.h
 *
 *  Created on: Apr 10, 2021
 *      Author: jonas
 */

#ifndef INC_QUEUE_H_
#define INC_QUEUE_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct QUEUE_t
{
	int max, rear, front, elements;
	char* buf;
}QUEUE_t;

void QUEUE_Init(QUEUE_t* q, char* buf, int max);
bool QUEUE_IsFull(QUEUE_t* q);
bool QUEUE_IsEmpty(QUEUE_t* q);
void QUEUE_Push(char val, QUEUE_t* q);
char QUEUE_Front(QUEUE_t* q);
char QUEUE_Rear(QUEUE_t* q);
char QUEUE_Pop(QUEUE_t* q);
uint32_t QUEUE_Size(QUEUE_t* q);

#endif /* INC_QUEUE_H_ */
