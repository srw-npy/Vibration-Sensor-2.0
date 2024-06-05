/*
 * lis2dh12.c
 *
 *  Created on: Oct 10, 2023
 *      Author: 15300
 */
#include "lis2dh12.h"

void LIS2DH12_ReadAcc_Init(I2C_HandleTypeDef *hi2c)
{
	uint8_t ctrlreg1_addr_data[2] = {0x20, 0x9f};
	uint8_t ctrlreg4_addr_data[2] = {0x23, 0x10};

	HAL_I2C_Master_Transmit(hi2c, LIS2DH12_Addr_SD0_SA0_0, ctrlreg1_addr_data, 2, HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(hi2c, LIS2DH12_Addr_SD0_SA0_0, ctrlreg4_addr_data, 2, HAL_MAX_DELAY);
}

void LIS2DH12_ReadWhoAmI_Transmit(I2C_HandleTypeDef *hi2c)
{
	static uint8_t whoami_addr = 0x0F;

	HAL_I2C_Master_Transmit_DMA(hi2c, LIS2DH12_Addr_SD0_SA0_0, &whoami_addr, 1);
}

void LIS2DH12_ReadWhoAmI_Receive(I2C_HandleTypeDef *hi2c, uint8_t *value)
{
	HAL_I2C_Master_Receive_DMA(hi2c, LIS2DH12_Addr_SD0_SA0_0, value, 1);
}

void LIS2DH12_ReadAccx_Transmit(I2C_HandleTypeDef *hi2c)
{
	static uint8_t accx_h_addr = 0x29;

	HAL_I2C_Master_Transmit_DMA(hi2c, LIS2DH12_Addr_SD0_SA0_0, &accx_h_addr, 1);
}

void LIS2DH12_ReadAccx_Receive(I2C_HandleTypeDef *hi2c, uint8_t *value)
{
	HAL_I2C_Master_Receive_DMA(hi2c, LIS2DH12_Addr_SD0_SA0_0, value, 1);
}

void LIS2DH12_ReadAccy_Transmit(I2C_HandleTypeDef *hi2c)
{
	static uint8_t accy_h_addr = 0x2b;

	HAL_I2C_Master_Transmit_DMA(hi2c, LIS2DH12_Addr_SD0_SA0_0, &accy_h_addr, 1);
}

void LIS2DH12_ReadAccy_Receive(I2C_HandleTypeDef *hi2c, uint8_t *value)
{
	HAL_I2C_Master_Receive_DMA(hi2c, LIS2DH12_Addr_SD0_SA0_0, value, 1);
}

void LIS2DH12_ReadAccz_Transmit(I2C_HandleTypeDef *hi2c)
{
	static uint8_t accz_h_addr = 0x2d;

	HAL_I2C_Master_Transmit_DMA(hi2c, LIS2DH12_Addr_SD0_SA0_0, &accz_h_addr, 1);
}

void LIS2DH12_ReadAccz_Receive(I2C_HandleTypeDef *hi2c, uint8_t *value)
{
	HAL_I2C_Master_Receive_DMA(hi2c, LIS2DH12_Addr_SD0_SA0_0, value, 1);
}

void LIS2DH12_ReadAccall(I2C_HandleTypeDef *hi2c, uint8_t *value)
{
	HAL_StatusTypeDef status;
	static uint8_t accall_addr = 0xa8;
//	char message[50] = {0};

	status = HAL_I2C_Mem_Read_DMA(hi2c, LIS2DH12_Addr_SD0_SA0_0, accall_addr, I2C_MEMADD_SIZE_8BIT, value, 6);

//	if(status == HAL_OK)
//	{
//		sprintf(message, "i2c ok!\r\n");
//		HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
//	}
//	else if(status == HAL_ERROR)
//	{
//		sprintf(message, "i2c error!\r\n");
//		HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
//	}
//	else if(status == HAL_BUSY)
//	{
//		sprintf(message, "i2c busy!\r\n");
//		HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
//	}
//	else if(status == HAL_TIMEOUT)
//	{
//		sprintf(message, "i2c timeout!\r\n");
//		HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
//	}
}

