#include "dc_pid.h"
#include "fuzzyPID.h"
#define max(a, b)			(a>b? a:b)
#define min(a, b)			(a<b? a:b)
#define range(x, a, b)		(min(max(x, a), b))
#define myabs(x)			((x<0)? -x:x)
void dc_pid_init(struct dc_pid *pid,
			dc_t kp,dc_t ki,dc_t kd,
			dc_t out_min,dc_t out_max)
{
	pid->max_limit = out_max;
	pid->min_limit = out_min;
	pid->target = 0;
	pid->feedback = 0;
	pid->out = 0;
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->e_0 = 0;
	pid->e_1 = 0;
	pid->e_2 = 0;
}
/*
增量式PID离散公式：Δu(k)=u(k)-u(k-1) = Kp*Δe(k)+ Ki*e(k)+ Kd*[Δe(k)-Δe(k-1)] ，式中Δe(k)=e(k)-e(k-1)
*/
dc_t dc_pid_calc(struct dc_pid *pid,dc_t target,dc_t feedback)
{
	dc_t ep, ei, ed;
	FUZZY_PID_t fuzzy_pid;
	pid->target = target;
	pid->feedback = feedback;
	pid->e_0 = pid->target - pid->feedback;
	ep = pid->e_0  - pid->e_1;
	ei = pid->e_0;
	ed = pid->e_0 - 2*pid->e_1 + pid->e_2;
	fuzzy(ei,ep,&fuzzy_pid);
	pid->out = (pid->kp + fuzzy_pid.Kp)*ep + (pid->ki + fuzzy_pid.Ki)*ei + (pid->kd + fuzzy_pid.Kd)*ed;
	pid->out = range(pid->out, pid->min_limit, pid->max_limit);
	pid->e_2 = pid->e_1;
	pid->e_1 = pid->e_0;
	return pid->out;
}
