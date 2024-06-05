#ifndef __NTC_H
#define	__NTC_H

#include "main.h"

#define T25 298.15    //电压转温度的公式的采用
#define R25 100
#define B		3950

float temp_Get_R(uint16_t adct);
double myln(double a);
float Get_Kelvin_Temperature(uint16_t t);

#endif
