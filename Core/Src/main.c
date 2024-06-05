/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "string.h"
#include "lis2dh12.h"
#include "w25qxx.h"
#include "ch395.h"
#include "ntc.h"
#include "misc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t adc_buf[2];
float accz_g, ntc;
float Sen_1 = 0.026;
float com_value = 1.65;

uint8_t IIC_State = 0;
int8_t acc_all[6];
float accx_g, accy_g;
float Sen_3 = 0.032;

const uint8_t TEXT_Buffer[]= {"STM32 SPI TEST!!"};
#define SIZE sizeof(TEXT_Buffer)
uint8_t datatemp[SIZE];
uint32_t flash_size = 16 * 1024 * 1024;

/* 本地网络信息：IP地址、网关地址、子网掩码和MAC地址 */
uint8_t ch395_ipaddr[4]     = {192,168,3,105};
uint8_t ch395_gw_ipaddr[4]  = {192,168,3,1};
uint8_t ch395_ipmask[4]     = {255,255,255,0};
uint8_t ch395_macaddr[6]    = {0x0E,0x0E,0x0E,0x00,0x00,0x00};
uint8_t ch395_sn[]          = {"VSP005"};
/* 远程IP地址设置 */
//uint8_t ch395_des_ipaddr[4] = {116,57,98,230};	// alphamini ip
//uint16_t ch395_des_port = 8888;					// alphamini port
//uint8_t ch395_des_ipaddr[4] = {81,71,132,174};		// piremote ip
//uint16_t ch395_des_port = 14494;					// piremote port
//uint8_t ch395_des_ipaddr[4] = {116,57,98,20};		// wgx ip
//uint16_t ch395_des_port = 8080;						// wgx port
uint8_t ch395_des_ipaddr[4] = {192,168,3,100};		// schoolpi ip
uint16_t ch395_des_port = 1234;						// schoolpi port
static uint8_t socket0_send_buf[7000];
static uint8_t socket0_recv_buf[1024];
ch395_socket cha95_sockct_sta[8];

uint16_t data_frequency = 5000;
uint8_t adc_switch = 1;
uint8_t iic_switch = 1;
uint16_t acc_num = 0;
int16_t pre_accx = 0;
int16_t pre_accy = 0;
int16_t adc_accz_buf_1[1000] = {0};
int16_t adc_accz_buf_2[1000] = {0};
int16_t iic_accx_buf_1[1000] = {0};
int16_t iic_accx_buf_2[1000] = {0};
int16_t iic_accy_buf_1[1000] = {0};
int16_t iic_accy_buf_2[1000] = {0};
char DFH[7] = "dd0101";		// 数据帧头（Data frame header）
char DFF[4] = "end";		// 数据帧尾（Data frame footer）
uint8_t rtim[17];
uint32_t rtime_head_8;
uint32_t rtime_foot_8;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim6)
	{
		int16_t *dataptr_x;
		int16_t *dataptr_y;

		if(iic_switch == 1) {
			dataptr_x = iic_accx_buf_1;
			dataptr_y = iic_accy_buf_1;
		}
		else if(iic_switch == 2) {
			dataptr_x = iic_accx_buf_2;
			dataptr_y = iic_accy_buf_2;
		}
		if(IIC_State == 1) {
			*(dataptr_x + acc_num) = acc_all[1];
			*(dataptr_y + acc_num) = acc_all[3];

			pre_accx = acc_all[1];
			pre_accy = acc_all[3];

			IIC_State = 0;
			LIS2DH12_ReadAccall(&hi2c1, acc_all);
		}
		else {
			*(dataptr_x + acc_num) = pre_accx;
			*(dataptr_y + acc_num) = pre_accy;
		}

		int16_t *dataptr_z;

		if(adc_switch == 1) {
			dataptr_z = adc_accz_buf_1;
		}
		else if(adc_switch == 2) {
			dataptr_z = adc_accz_buf_2;
		}
		*(dataptr_z + acc_num) = adc_buf[1];

		acc_num++;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  // Flash Init
  W25QXX_Init();
  while(W25QXX_ReadID() != W25Q128) {
	  printf("Flash Error!!\r\n");
	  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
	  HAL_Delay(500);
  }

  // ADC Init (ADXL1002 + NTC)
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, (uint32_t)2);

  // Lis2dh12 Init (IIC + DMA)
  LIS2DH12_ReadAcc_Init(&hi2c1);
  LIS2DH12_ReadAccall(&hi2c1, acc_all);
  HAL_Delay(10);

  // Sensor Correct
  Com_Value_Init_DMA();

  // CH395 TCP Init
  printf("ch395_des_ipaddr: %d.%d.%d.%d\r\n", ch395_des_ipaddr[0], ch395_des_ipaddr[1], ch395_des_ipaddr[2], ch395_des_ipaddr[3]);
  printf("ch395_des_port:   %d\r\n", ch395_des_port);
  __HAL_SPI_ENABLE(&hspi1);
  spi1_read_write_byte(0Xff);
  ch395_hardware_init();
  do {
	  ch395q_handler();
  } while(g_ch395q_sta.dhcp_status == DHCP_STA);                                                                       /* 获取DHCP */

  cha95_sockct_sta[0].socket_enable = CH395Q_ENABLE;                                                                  /* 使能socket对 */
  cha95_sockct_sta[0].socket_index = CH395Q_SOCKET_0;                                                                 /* 设置socket对 */
  memcpy(cha95_sockct_sta[0].des_ip, ch395_des_ipaddr, sizeof(cha95_sockct_sta[0].des_ip));                           /* 设置目标IP地址 */
  memcpy(cha95_sockct_sta[0].net_config.ipaddr, ch395_ipaddr, sizeof(cha95_sockct_sta[0].net_config.ipaddr));         /* 设置静态本地IP地址 */
  memcpy(cha95_sockct_sta[0].net_config.gwipaddr, ch395_gw_ipaddr, sizeof(cha95_sockct_sta[0].net_config.gwipaddr));  /* 设置静态网关IP地址 */
  memcpy(cha95_sockct_sta[0].net_config.maskaddr, ch395_ipmask, sizeof(cha95_sockct_sta[0].net_config.maskaddr));     /* 设置静态子网掩码地址 */
  memcpy(cha95_sockct_sta[0].net_config.macaddr, ch395_macaddr, sizeof(cha95_sockct_sta[0].net_config.macaddr));      /* 设置静态MAC地址 */
  cha95_sockct_sta[0].des_port = ch395_des_port;                                                                      /* 目标端口 */
  cha95_sockct_sta[0].sour_port = 8080;                                                                               /* 源端口 */
  cha95_sockct_sta[0].proto = CH395Q_SOCKET_TCP_CLIENT;                                                               /* 设置协议 */
  cha95_sockct_sta[0].send.buf = socket0_send_buf;                                                                    /* 发送数据 */
  cha95_sockct_sta[0].send.size = sizeof(socket0_send_buf);                                                           /* 发送数据大小 */
  cha95_sockct_sta[0].recv.buf =  socket0_recv_buf;                                                                   /* 接收数据缓冲区 */
  cha95_sockct_sta[0].recv.size = sizeof(socket0_recv_buf);                                                           /* 接收数据大小 */
  ch395q_socket_config(&cha95_sockct_sta[0]);                                                                         /* 配置socket参数 */

  if(g_ch395q_sta.dhcp_status == DHCP_DOWN) {
	  printf("ch395_ipaddr:     %d.%d.%d.%d\r\n", ch395_ipaddr[0], ch395_ipaddr[1], ch395_ipaddr[2], ch395_ipaddr[3]);
	  printf("ch395_gw_ipaddr:  %d.%d.%d.%d\r\n", ch395_gw_ipaddr[0], ch395_gw_ipaddr[1], ch395_gw_ipaddr[2], ch395_gw_ipaddr[3]);
	  printf("ch395_ipmask:     %d.%d.%d.%d\r\n", ch395_ipmask[0], ch395_ipmask[1], ch395_ipmask[2], ch395_ipmask[3]);
	  printf("ch395_macaddr:    %02x %02x %02x %02x %02x %02x\r\n", ch395_macaddr[0], ch395_macaddr[1], ch395_macaddr[2],
																   ch395_macaddr[3], ch395_macaddr[4], ch395_macaddr[5]);
  }
  printf("ch395_sn:         %s\r\n", ch395_sn);

  sprintf(socket0_send_buf, "request %s", ch395_sn);
  uint8_t *isFound;
  do {
	  printf("rtim requesting...\r\n");
	  ch395_send_data(0, (uint8_t *)socket0_send_buf, strlen((char *)socket0_send_buf));
	  HAL_Delay(500);
	  ch395q_handler();
	  HAL_Delay(500);
	  isFound = strstr(socket0_recv_buf, "rtim");
  } while(isFound == NULL);
  memcpy(rtim, isFound + 4, 16);
  printf("rtim:[%s]\r\n", rtim);
  sscanf(rtim, "%8d%8d", &rtime_head_8, &rtime_foot_8);

  // TIM6 Tnit
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_Delay(10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  static uint16_t cumulative_num = 0;
	  char *ptr = socket0_send_buf;
	  int16_t *dataptr_z;
	  int16_t *dataptr_x;
	  int16_t *dataptr_y;

	  uint8_t old_adc_switch = adc_switch;
	  uint8_t old_iic_switch = iic_switch;
	  uint16_t old_acc_num = acc_num;

	  (adc_switch == 1)?(adc_switch = 2):(adc_switch = 1);
	  (iic_switch == 1)?(iic_switch = 2):(iic_switch = 1);

	  acc_num = 0;

	  if(old_adc_switch == 1) { dataptr_z = &adc_accz_buf_1[0]; }
	  else if(old_adc_switch == 2) { dataptr_z = &adc_accz_buf_2[0]; }
	  if(old_iic_switch == 1) { dataptr_x = &iic_accx_buf_1[0]; dataptr_y = &iic_accy_buf_1[0]; }
	  else if(old_iic_switch == 2) { dataptr_x = &iic_accx_buf_2[0]; dataptr_y = &iic_accy_buf_2[0]; }

	  cumulative_num += old_acc_num;
	  printf("%d  %08d%08d\r\n", old_acc_num, rtime_head_8, rtime_foot_8);
	  static float test_x, test_y, test_z;
	  if(cumulative_num >= data_frequency) {
		  printf("\r\n");
		  printf(">>> frequency:[%d], cumulative_num:[%d], old_acc_num:[%d], ntc:[%05.1f], rtime:[%08d%08d], ",
				  	  data_frequency, cumulative_num, old_acc_num, ntc, rtime_head_8, rtime_foot_8);
		  printf("    accx:[%.4f], accy:[%.4f], accz:[%.4f]\r\n", test_x, test_y, test_z);
		  cumulative_num = 0;
		  printf("\r\n");
	  }

	  sprintf(ptr, "%s", DFH);

	  sprintf(ptr + 6, "{%08d%08d}", rtime_head_8, rtime_foot_8);
	  rtime_calculate(old_acc_num);

	  ntc = Get_Kelvin_Temperature(adc_buf[0]);
	  if(ntc > 999 || ntc < 0) ntc = 999.9;
	  sprintf(ptr + 6 + 18, "{%05.1f}", ntc);

	  sprintf(ptr + 6 + 18 + 7, "|");

	  for(uint16_t i = 0; i < old_acc_num; i++) {
		  float data_z = (*(dataptr_z + i) * (3.3 / 4096) - com_value) / Sen_1;
		  float data_x = *(dataptr_x + i) * Sen_3;
		  float data_y = *(dataptr_y + i) * Sen_3;
		  test_z = data_z;
		  test_x = data_x;
		  test_y = data_y;
		  sprintf((ptr + 6 + 18 + 7 + 1 + i * 7), "%c%c%c%c%c%c|", int_separate(data_z), dec_separate(data_z),
				  	  	  	  	  	  	  	  	  	  	  	   	   int_separate(data_x), dec_separate(data_x),
																   int_separate(data_y), dec_separate(data_y));
	  }

	  sprintf(ptr + 6 + 18 + 7 + 1 + old_acc_num * 7, "%s", DFF);

	  sprintf(ptr + 6 + 18 + 7 + 1 + old_acc_num * 7 + 3, "\0");

	  ch395_send_data(0, (uint8_t *)socket0_send_buf, strlen((char *)socket0_send_buf));

	  ch395q_handler();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void delay_us(uint32_t udelay)
{
	uint32_t startval, tickn, delays, wait;

	startval = SysTick->VAL;
	tickn = HAL_GetTick();
//	sysc = 100000;  //SystemCoreClock / (1000U / uwTickFreq);
	delays = udelay * 100; //sysc / 1000 * udelay;
	if(delays > startval)
	{
		while(HAL_GetTick() == tickn)
		{

		}
		wait = 100000 + startval - delays;
		while(wait < SysTick->VAL)
		{

		}
	}
	else
	{
		wait = startval - delays;
		while(wait < SysTick->VAL && HAL_GetTick() == tickn)
		{

		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
