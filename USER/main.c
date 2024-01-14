#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "string.h"
#include "max6675.h"
#include "pwm.h"
#include "tim3.h"
#include <math.h>
#include "dc_pid.h"
float dat,temprature;
//pid
struct dc_pid pid;
float sv = 40;
u8 time[4] = {0,0,0,0};
u16 pwm_var = 0;
//这是一个温度控制PID延时小程序
 int main(void)
 {	
	delay_init();	    	 //延时函数初始化	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	 
	uart_init(115200);	 //串口初始化为115200
	 
	SPI_MAX6675_Init();		//初始化热电偶转换器
	TIM4_PWM_Init();			//20hz pwm
	TIM3_Int_Init(1999,35999);	//1S
	PWM_Channel1_Set(0);			//初始化了定时器4的4个PWM通道
	PWM_Channel2_Set(0);
	PWM_Channel3_Set(0);
	PWM_Channel4_Set(0);
	Get_temprature(&dat);
	dc_pid_init(&pid,10,0.9,10,0,20);				//初始化PID控制器
	time[0] = 1;
	time[1] = 5;
//	fw_init();			//初始化在线调参协议
	 
	while(1)
	{
			if(!time[0])//1s 采集时间
			{
				time[0] = 1;
				Get_temprature(&dat);
				temprature = dat;
			}
			if(!time[1])//5s PID计算周期
			{
				printf("pid run\r\n");
				time[1] = 5;
				pid.target = sv;	//设定，目标温度值
				if(fabs(temprature-pid.target)>=10)	//设置温度线性区间
				{
					if(temprature > pid.target)
						pwm_var = 0;
					else
						pwm_var = 499;
				}
				else
				{
						dc_pid_calc(&pid,sv,temprature);
						pwm_var = pwm_var + (u16)pid.out;		//增量式PID每次结果需要累加
				}
				if(pwm_var >= 499)pwm_var = 499;
				if(pwm_var <= 0)pwm_var = 0;
				PWM_Channel1_Set(pwm_var);				//输出PWM
				printf("pid:%f,%f,%f,%f\n",pid.target,temprature,pid.out,(float)pwm_var);
			}
	}
}
