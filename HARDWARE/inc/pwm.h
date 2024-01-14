#ifndef _pwm_h_
#define _pwm_h_
#include "stm32f10x_tim.h"
#define min_pwm 50
#define max_pwm 1000
#define max(a, b)			(a>b? a:b)
#define min(a, b)			(a<b? a:b)
#define range(x, a, b)		(min(max(x, a), b))
#define CAR_FORWARD() GPIO_SetBits(GPIOB,GPIO_Pin_1);	//���ǰ��
#define CAR_BACK() GPIO_ResetBits(GPIOB,GPIO_Pin_1);	//�������
//�ж�x�Ƿ���[a,b]֮�䣬����ȡ��ӽ�������
void TIM4_PWM_Init(void);
void PWM_Channel1_Set(u16 val);
void PWM_Channel2_Set(u16 val);
void PWM_Channel3_Set(u16 val);
void PWM_Channel4_Set(u16 val);

#endif
