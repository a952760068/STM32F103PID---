#ifndef _max6675_h_
#define _max6675_h_
#include "sys.h"
void SPI_MAX6675_Init(void);
uint8_t Get_temprature(float *temprature);
#endif
