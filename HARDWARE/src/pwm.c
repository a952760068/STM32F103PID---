#include "stm32f10x.h"
#include "pwm.h"
void TIM4_GPIO_Config(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;		//CH1-PB6,CH2-PB7,CH3-PB8,CH4-PB9
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	//倒车控制引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;	
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	CAR_FORWARD();	//默认设置为前进方向
}

void TIM4_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* PWM信号电平跳变值 */
	u16 CCR1_Val = 0;        
	u16 CCR2_Val = 0;
	u16 CCR3_Val = 0;
	u16 CCR4_Val = 0;

/* -----------------------------------------------------------------------
    TIM4 Configuration: generate 4 PWM signals with 4 different duty cycles:
    TIM4CLK = 72 MHz, Prescaler = 72, TIM4 counter clock = 1 MHz
    TIM4 ARR Register = 999 => TIM4 Frequency = TIM4 counter clock/(ARR + 1)
    TIM4 Frequency = 1 KHz.
    TIM4 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 10%
    TIM4 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 10%
    TIM4 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 10%
    TIM4 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 10%
  ----------------------------------------------------------------------- */

  /* Time base configuration */		 
  TIM_TimeBaseStructure.TIM_Period = 499;       //当定时器从0计数到999，即为1000次，为一个定时周期
  TIM_TimeBaseStructure.TIM_Prescaler = 7199;	    //设置预分频：72预分频，即为1MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//设置时钟分频系数：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;	    //配置为PWM模式1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;	   //设置跳变值，当计数器计数到这个值时，电平发生跳变
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;  //当定时器计数值小于CCR1_Val时为高电平

  TIM_OC1Init(TIM4, &TIM_OCInitStructure);	 //使能通道1

  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;	  //设置通道2的电平跳变值，输出另外一个占空比的PWM

  TIM_OC2Init(TIM4, &TIM_OCInitStructure);	  //使能通道2

  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;	//设置通道3的电平跳变值，输出另外一个占空比的PWM

  TIM_OC3Init(TIM4, &TIM_OCInitStructure);	 //使能通道3

  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;	//设置通道4的电平跳变值，输出另外一个占空比的PWM

  TIM_OC4Init(TIM4, &TIM_OCInitStructure);	//使能通道4

  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

//  TIM_ARRPreloadConfig(TIM4, ENABLE);			 // 使能TIM4重载寄存器ARR

  /* TIM4 enable counter */
  TIM_Cmd(TIM4, ENABLE);                   //使能定时器4	
}
void TIM4_PWM_Init(void)
{
	TIM4_GPIO_Config();		//引脚配置
	TIM4_Mode_Config();		//PWM配置
	PWM_Channel1_Set(0);	//配置4通道的PWM，50/1024*5+0.3V=0.5V左右，最低油门为0.5V
	PWM_Channel2_Set(0);
	PWM_Channel3_Set(0);
	PWM_Channel4_Set(0);
	
}
void PWM_Channel1_Set(u16 val)
{
//	val=range(val,0,1023);  //限制PWM输出范围100-1000
	TIM_SetCompare1(TIM4,val);
}
void PWM_Channel2_Set(u16 val)
{
//	val=range(val,0,1023);  //限制PWM输出范围100-1000
	TIM_SetCompare2(TIM4,val);
}
void PWM_Channel3_Set(u16 val)
{
//	val=range(val,0,1023);  //限制PWM输出范围100-1000
	TIM_SetCompare3(TIM4,val);
}
void PWM_Channel4_Set(u16 val)
{
//	val=range(val,0,1023);  //限制PWM输出范围100-1000
	TIM_SetCompare4(TIM4,val);
}

