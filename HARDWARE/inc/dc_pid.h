/*
 * This file is part of the DigitalCtrl Library.
 *
 * Copyright (c) 2019,
 *
 * Function: Header file of PID digital algorithm.
 * Created on: 2019-08-30
 */
#ifndef _dc_pid_h_
#define _dc_pid_h_
#ifdef DIGITALCTRL_USING_DOUBLE
typedef double dc_t;
#else
typedef float dc_t;
#endif

#define max(a, b)			(a>b? a:b)
#define min(a, b)			(a<b? a:b)
#define RANGE(x, a, b)		(min(max(x, a), b))

struct dc_pid {
	dc_t max_limit;			//输出限幅
	dc_t min_limit;			//输出限幅
	dc_t target;		//目标输出量
	dc_t feedback;		//实际输出量
	dc_t out;
	dc_t kp;
	dc_t ki;
	dc_t kd;
	dc_t e_0;			//当前误差
	dc_t e_1;			//上一次误差
	dc_t e_2;			//上上次误差
};
void dc_pid_init(struct dc_pid *pid,dc_t kp,dc_t ki,dc_t kd,dc_t out_min,dc_t out_max);
dc_t dc_pid_calc(struct dc_pid *pid,dc_t target,dc_t feedback);
#endif
