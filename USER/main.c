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
//����һ���¶ȿ���PID��ʱС����
 int main(void)
 {	
	delay_init();	    	 //��ʱ������ʼ��	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// �����ж����ȼ�����2
	 
	uart_init(115200);	 //���ڳ�ʼ��Ϊ115200
	 
	SPI_MAX6675_Init();		//��ʼ���ȵ�żת����
	TIM4_PWM_Init();			//20hz pwm
	TIM3_Int_Init(1999,35999);	//1S
	PWM_Channel1_Set(0);			//��ʼ���˶�ʱ��4��4��PWMͨ��
	PWM_Channel2_Set(0);
	PWM_Channel3_Set(0);
	PWM_Channel4_Set(0);
	Get_temprature(&dat);
	dc_pid_init(&pid,10,0.9,10,0,20);				//��ʼ��PID������
	time[0] = 1;
	time[1] = 5;
//	fw_init();			//��ʼ�����ߵ���Э��
	 
	while(1)
	{
			if(!time[0])//1s �ɼ�ʱ��
			{
				time[0] = 1;
				Get_temprature(&dat);
				temprature = dat;
			}
			if(!time[1])//5s PID��������
			{
				printf("pid run\r\n");
				time[1] = 5;
				pid.target = sv;	//�趨��Ŀ���¶�ֵ
				if(fabs(temprature-pid.target)>=10)	//�����¶���������
				{
					if(temprature > pid.target)
						pwm_var = 0;
					else
						pwm_var = 499;
				}
				else
				{
						dc_pid_calc(&pid,sv,temprature);
						pwm_var = pwm_var + (u16)pid.out;		//����ʽPIDÿ�ν����Ҫ�ۼ�
				}
				if(pwm_var >= 499)pwm_var = 499;
				if(pwm_var <= 0)pwm_var = 0;
				PWM_Channel1_Set(pwm_var);				//���PWM
				printf("pid:%f,%f,%f,%f\n",pid.target,temprature,pid.out,(float)pwm_var);
			}
	}
}
