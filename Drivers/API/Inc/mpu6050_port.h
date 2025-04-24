/*
 * mpu6050_driver.h
 *
 *  Created on: Apr 13, 2025
 *      Author: doddy
 */

#ifndef API_INC_MPU6050_PORT_H_
#define API_INC_MPU6050_PORT_H_

#include "stm32f4xx_hal.h"
#include "mpu6050_driver.h"
#include <stdint.h>

extern void Error_Handler(void);

void MPU6050_PortI2C_Init();
void MPU6050_PortI2C_IsReady();
void MPU6050_PortI2C_WriteRegister(uint8_t reg, uint8_t value, uint8_t MAX_SIZE);
void MPU6050_PortI2C_ReadRegister(uint8_t reg, uint8_t* buffer, uint8_t MAX_SIZE, uint8_t length);

#endif /* API_INC_MPU6050_PORT_H_ */
