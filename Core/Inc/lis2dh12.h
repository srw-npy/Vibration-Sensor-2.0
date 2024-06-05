/*
 * lis2dh12.h
 *
 *  Created on: Oct 10, 2023
 *      Author: 15300
 */

#ifndef INC_LIS2DH12_H_
#define INC_LIS2DH12_H_

#include "i2c.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"

#define LIS2DH12_Addr_SD0_SA0_0    0x30

void LIS2DH12_ReadAcc_Init(I2C_HandleTypeDef *hi2c);
void LIS2DH12_ReadWhoAmI_Transmit(I2C_HandleTypeDef *hi2c);
void LIS2DH12_ReadWhoAmI_Receive(I2C_HandleTypeDef *hi2c, uint8_t *value);
void LIS2DH12_ReadAccx_Transmit(I2C_HandleTypeDef *hi2c);
void LIS2DH12_ReadAccx_Receive(I2C_HandleTypeDef *hi2c, uint8_t *value);
void LIS2DH12_ReadAccy_Transmit(I2C_HandleTypeDef *hi2c);
void LIS2DH12_ReadAccy_Receive(I2C_HandleTypeDef *hi2c, uint8_t *value);
void LIS2DH12_ReadAccz_Transmit(I2C_HandleTypeDef *hi2c);
void LIS2DH12_ReadAccz_Receive(I2C_HandleTypeDef *hi2c, uint8_t *value);
void LIS2DH12_ReadAccall(I2C_HandleTypeDef *hi2c, uint8_t *value);

#endif /* INC_LIS2DH12_H_ */
