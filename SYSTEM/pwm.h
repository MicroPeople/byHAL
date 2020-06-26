#ifndef _PWM_H
#define _PWM_H
#include "sys.h"
extern TIM_HandleTypeDef TIM3_Handler; //定时器句柄

void TIM3_Init(u16 arr, u16 psc);
void TIM14_PWM_Init(u16 arr, u16 psc);
void TIM_SetTIM14Compare1(u32 compare);
#endif
