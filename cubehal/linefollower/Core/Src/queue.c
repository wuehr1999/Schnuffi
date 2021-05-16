/*
 * queue.c
 *
 *  Created on: Apr 10, 2021
 *      Author: jonas
 */
#include "queue.h"

void QUEUE_Init(QUEUE_t* q, char* buf, int max)
{
	q->max = max;
	q->elements = 0;
	q->buf = buf;
	q->rear = q->max - 1;
	q->front = 0;
}
bool QUEUE_IsFull(QUEUE_t* q)
{
	return (q->elements == q->max);
}

bool QUEUE_IsEmpty(QUEUE_t* q)
{
	return (0 == q->elements);
}

void QUEUE_Push(char val, QUEUE_t* q)
{
	q->rear = (q->rear + 1) % q->max;
	q->elements++;
	q->buf[q->rear] = val;
}

char QUEUE_Front(QUEUE_t* q)
{
	return q->buf[q->front];
}

char QUEUE_Rear(QUEUE_t* q)
{
	return q->buf[q->rear];
}

char QUEUE_Pop(QUEUE_t* q)
{
	char c = q->buf[q->front];
	q->front = (q->front + 1) % q->max;
	q->elements--;
	return c;
}

uint32_t QUEUE_Size(QUEUE_t* q)
{
	return q->elements;
}

