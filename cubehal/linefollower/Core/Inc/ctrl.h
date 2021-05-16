/*
 * ctrl.h
 *
 *  Created on: Apr 8, 2021
 *      Author: jonas
 */

#ifndef INC_CTRL_H_
#define INC_CTRL_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct CTRL_t
{
  float Kp, Ki, Kd, T;
  float error, lastError, integral;
  float maxIn, maxOut;
  float output;
}CTRL_t;

void CTRL_Init(float Kp, float Ki, float Kd, float T, float maxIn, float maxOut, CTRL_t* ctrl);
float CTRL_Calculate(float curr, float dest, CTRL_t* ctrl, bool restart);
#endif /* INC_CTRL_H_ */
