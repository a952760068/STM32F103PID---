#include "fuzzyPID.h"
#include <math.h>
//模糊集合
#define NL   -3
#define NM	 -2
#define NS	 -1
#define ZE	 0
#define PS	 1
#define PM	 2
#define PL	 3

//定义偏差E的范围，因为设置了非线性区间，温度在误差在10°时才开始进行PID调节，这里E的范围为10
#define  MAXE (10)
#define  MINE (-MAXE)
//定义EC的范围，因为温度变化非常缓慢！，每次的EC都非常小，这里可以根据实际需求来调整，
#define  MAXEC (10)
#define  MINEC (-MAXEC)
//定义e,ec的量化因子
#define KE   3/MAXE
#define KEC  3/MAXEC

//定义输出量比例因子
#define  KUP   2.0		//这里只使用了模糊PID的比例增益
#define  KUI   0.0
#define  KUD   0.0

static const float fuzzyRuleKp[7][7]={
	PL,	PL,	PM,	PL,	PS,	PM,	ZE,
	PL,	PM,	PM,	PM,	PS,	PM,	ZE,
	PM,	PS,	PS,	PS,	PS,	PS,	PM,
	PM,	PS,	ZE,	ZE,	ZE,	PS,	PM,
	PS,	PS,	PS,	PS,	PS,	PM,	PM,
	PM,	PM,	PM,	PM,	PL,	PL,	PL,
	PM,	PL,	PL,	PL,	PL,	PL,	PL
};
 
static const float fuzzyRuleKi[7][7]={
	NL,	NL,	NL,	NL,	NM,	NL,	NL,
	NL,	NL,	NM,	NM,	NM,	NL,	NL,
	NM,	NM,	NS,	NS,	NS,	NM,	NM,
	NM,	NS,	ZE,	ZE,	ZE,	NS,	NM,
	NM,	NS,	NS,	NS,	NS,	NM,	NM,
	NM,	NM,	NS,	NM,	NM,	NL,	NL,
	NM,	NL,	NM,	NL,	NL,	NL,	NL
};
 
static const float fuzzyRuleKd[7][7]={
	PS,	PS,	ZE,	ZE,	ZE,	PL,	PL,
	NS,	NS,	NS,	NS,	ZE,	NS,	PM,
	NL,	NL,	NM,	NS,	ZE,	PS,	PM,
	NL,	NM,	NM,	NS,	ZE,	PS,	PM,
	NL,	NM,	NS,	NS,	ZE,	PS,	PS,
	NM,	NS,	NS,	NS,	ZE,	PS,	PS,
	PS,	ZE,	ZE,	ZE,	ZE,	PL,	PL
};
 

void fuzzy(float e,float ec,FUZZY_PID_t *fuzzy_PID)
{
 
     float etemp,ectemp;					
     float eLefttemp,ecLefttemp;			//ec,e，左隶属度
     float eRighttemp ,ecRighttemp;
 
     int eLeftIndex,ecLeftIndex;			//模糊位置标号
     int eRightIndex,ecRightIndex;
		 e = RANGE(e,MINE,MAXE);
     ec = RANGE(ec,MINEC,MAXEC);
		 e = e*KE;
		 ec = ec * KEC;

     etemp = e > 3.0 ? 0.0 : (e < - 3.0 ? 0.0 : (e >= 0.0 ? (e >= 2.0 ? 2.5: (e >= 1.0 ? 1.5 : 0.5)) : (e >= -1.0 ? -0.5 : (e >= -2.0 ? -1.5 : (e >= -3.0 ? -2.5 : 0.0) ))));
     eLeftIndex = (int)((etemp-0.5) + 3);        //[-3,3] -> [0,6]
     eRightIndex = (int)((etemp+0.5) + 3);
     eLefttemp =etemp == 0.0 ? 0.0:((etemp+0.5)-e); 			//
     eRighttemp=etemp == 0.0 ? 0.0:( e-(etemp-0.5));
     ectemp = ec > 3.0 ? 0.0 : (ec < - 3.0 ? 0.0 : (ec >= 0.0 ? (ec >= 2.0 ? 2.5: (ec >= 1.0 ? 1.5 : 0.5)) : (ec >= -1.0 ? -0.5 : (ec >= -2.0 ? -1.5 : (ec >= -3.0 ? -2.5 : 0.0) ))));
     ecLeftIndex = (int)((ectemp-0.5) + 3);        //[-3,3] -> [0,6]
     ecRightIndex = (int)((ectemp+0.5) + 3);

     ecLefttemp =ectemp == 0.0 ? 0.0:((ectemp+0.5)-ec);
     ecRighttemp=ectemp == 0.0 ? 0.0:( ec-(ectemp-0.5));

		
/*************************************反模糊*************************************/
 
 
 
 
	fuzzy_PID->Kp = (eLefttemp * ecLefttemp * fuzzyRuleKp[eLeftIndex][ecLeftIndex]                    
					+ eLefttemp * ecRighttemp * fuzzyRuleKp[eLeftIndex][ecRightIndex]
					+ eRighttemp * ecLefttemp * fuzzyRuleKp[eRightIndex][ecLeftIndex]
					+ eRighttemp * ecRighttemp * fuzzyRuleKp[eRightIndex][ecRightIndex]);
 
	fuzzy_PID->Ki = (eLefttemp * ecLefttemp * fuzzyRuleKi[eLeftIndex][ecLeftIndex]
					+ eLefttemp * ecRighttemp * fuzzyRuleKi[eLeftIndex][ecRightIndex]
					+ eRighttemp * ecLefttemp * fuzzyRuleKi[eRightIndex][ecLeftIndex]
					+ eRighttemp * ecRighttemp * fuzzyRuleKi[eRightIndex][ecRightIndex]);
 
	fuzzy_PID->Kd = (eLefttemp * ecLefttemp *  fuzzyRuleKd[eLeftIndex][ecLeftIndex]
					+ eLefttemp * ecRighttemp * fuzzyRuleKd[eLeftIndex][ecRightIndex]
					+ eRighttemp * ecLefttemp * fuzzyRuleKd[eRightIndex][ecLeftIndex]
					+ eRighttemp * ecRighttemp * fuzzyRuleKd[eRightIndex][ecRightIndex]);
	//对解算出的KP,KI,KD进行量化映射

	fuzzy_PID->Kp = fuzzy_PID->Kp * KUP;
	fuzzy_PID->Ki = fuzzy_PID->Ki * KUI;
  fuzzy_PID->Kd = fuzzy_PID->Kd * KUD;

}
