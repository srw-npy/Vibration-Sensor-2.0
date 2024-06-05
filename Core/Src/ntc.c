#include "ntc.h"

//模拟量转电阻
float temp_Get_R(uint16_t adct)
{
	float v1,v2,r;

	v1 = (float)(adct*3.3)/4096;//将NTC电路上的ADC采样值转换为电压
	v2 = 3.3 - v1;
	r = (v1/v2) * 10;

	return r;
}

//实现ln(x) 的计算
double myln(double a)
{
   int N = 15;//取了前15+1项来估算
   int k,nk;
   double x,xx,y;
   x = (a-1)/(a+1);
   xx = x*x;
   nk = 2*N+1;
   y = 1.0/nk;
   for(k=N;k>0;k--)
   {
     nk = nk - 2;
     y = 1.0/nk+xx*y;

   }
   return 2.0*x*y;
}

float Get_Kelvin_Temperature(uint16_t t)
{
	float N1,N2,N3,N4;
	float Rntc = temp_Get_R(t);
	N1 = (myln(R25)-myln(Rntc))/B;
	N2 = 1/T25 - N1;
	N3 = 1/N2;
	N4 = N3-273.15;//开尔文转摄氏度

	return N4;
}
