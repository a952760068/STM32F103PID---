#ifndef _fuzzyPID_h_
#define _fuzzyPID_h_
#include "dc_pid.h"
typedef struct{
	float Kp;
	float Ki;
	float Kd;
}FUZZY_PID_t;
void fuzzy(float e,float ec,FUZZY_PID_t *fuzzy_PID);			
#endif
