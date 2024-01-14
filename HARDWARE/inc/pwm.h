#ifndef _pwm_h_
#define _pwm_h_
#include "stm32f10x_tim.h"
#define min_pwm 50
#define max_pwm 1000
#define max(a, b)			(a>b? a:b)
#define min(a, b)			(a<b? a:b)
#define range(x, a, b)		(min(max(x, a), b))
#define CAR_FORWARD() GPIO_SetBits(GPIOB,GPIO_Pin_1);	//电机前进
#define CAR_BACK() GPIO_ResetBits(GPIOB,GPIO_Pin_1);	//电机后退
//判断x是否在[a,b]之间，否则，取最接近的区间
void TIM4_PWM_Init(void);
void PWM_Channel1_Set(u16 val);
void PWM_Channel2_Set(u16 val);
void PWM_Channel3_Set(u16 val);
void PWM_Channel4_Set(u16 val);

#endif
