/*
 * ctrl.c
 *
 *  Created on: Apr 8, 2021
 *      Author: jonas
 */

#include "ctrl.h"

void CTRL_Init(float Kp, float Ki, float Kd, float T, float maxIn, float maxOut, CTRL_t* ctrl)
{
  ctrl->maxIn = maxIn;
  ctrl->maxOut = maxOut;

  ctrl->Kp = Kp;
  ctrl->Ki = Ki;
  ctrl->Kd = Kd;
  ctrl->T = T;

  CTRL_Calculate(0.0, 0.0, ctrl, true);
}

float CTRL_Calculate(float curr, float dest, CTRL_t* ctrl, bool restart)
{
  if(restart)
  {
    ctrl->error = 0;
    ctrl->lastError = 0;
    ctrl->integral = 0;
  }

  if(dest > ctrl->maxIn)
  {
    dest = ctrl->maxIn;
  }
  else if(dest < -ctrl->maxIn)
  {
    dest = -ctrl->maxIn;
  }

  ctrl->error = dest - curr;

  if(ctrl->output < ctrl->maxOut && ctrl->output > -ctrl->maxOut)
  {
	  ctrl->integral += ctrl->error;
  }
  ctrl->output = ctrl->Kp * ctrl->error + ctrl->Ki * ctrl->integral
		  + ctrl->Kd * (ctrl->error - ctrl->lastError) / ctrl->T;
  ctrl->lastError = ctrl->error;

  return ctrl->output;
}
