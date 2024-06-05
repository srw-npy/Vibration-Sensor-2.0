#include "misc.h"
#include "stdio.h"
#include "time.h"
#include "i2c.h"

extern uint16_t data_frequency;

extern uint32_t rtime_head_8;
extern uint32_t rtime_foot_8;

extern float com_value;
extern float Sen_1;
extern uint16_t adc_buf[2];

extern float Sen_3;
extern int8_t acc_all[6];

// 分离出整数部分（有符号）
char int_separate(float d)
{
	char a = (int)d;

	if((int)d == 0) {
		a = 'o';
	}

	return a;
}

// 分离出小数部分（无符号、小数点后两位）
char dec_separate(float d)
{
	int a = d;
	int b = (d-a)*100.0;
	char ss = b;

	if(b == 0) {
		ss = 'o';
	}

	return ss;
}

// 计算时间戳
void rtime_calculate(uint16_t old_num)
{
	float tim = 1000000 / data_frequency;
	rtime_foot_8 += old_num * tim;

	if(rtime_foot_8 >= 100000000)
	{
		rtime_foot_8 -= 100000000;
		rtime_head_8++;
	}
}

// 通过三轴的z轴数据矫正单轴数据
void Com_Value_Init_DMA(void)
{
	uint8_t flag = 0;
	float accz_1;
	float accz_3;

	printf("The sensor is correcting...... \r\n");

//	while(1)
	while(flag == 0)
	{
		accz_1 = (adc_buf[1] * 3.3/4096 - com_value) / Sen_1;
		accz_3 = acc_all[5] * Sen_3;

		// 上电前，以三轴加速度芯片的z轴加速度accz_3作为标准，
		// 调整补偿量com_value，使传感器上电时的单轴加速度accz_1 = accz_3 ± 0.001g
		if(flag == 0 && (accz_1 - accz_3) > 0.001) {
			com_value += 0.0001;
		}
		else if(flag == 0 && (accz_1 - accz_3) < -0.001) {
			com_value -= 0.0001;
		}
		else {
			flag = 1;
			printf("Calibration is completed \r\n");
		}

		LIS2DH12_ReadAccall(&hi2c1, acc_all);			/* 启动I2C DMA采集 */
		delay_us(100);

		if(flag == 1) {
			printf("corrected information: %.5f  %.5f  %.5f\r\n", accz_3, accz_1, com_value);
			HAL_Delay(500);
		}
	}
}
