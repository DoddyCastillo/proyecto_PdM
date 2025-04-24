/*
 * mpu6050_port.c
 *
 *  Created on: Apr 12, 2025
 *      Author: doddy
 */


#include "mpu6050_port.h"
#include "API_uart.h"
#include "stdio.h"
#include "string.h"

static I2C_HandleTypeDef hi2c3;

void MPU6050_PortI2C_Init()
{

	hi2c3.Instance = I2C3;
	hi2c3.Init.ClockSpeed = 100000;
	hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK)
	{
	  Error_Handler();
	}

}


void MPU6050_PortI2C_IsReady()
{
	if (HAL_I2C_IsDeviceReady(&hi2c3, ADDRESS_MPU6050 << 1, 1, HAL_MAX_DELAY) != HAL_OK) Error_Handler();
}


void MPU6050_PortI2C_WriteRegister(uint8_t reg, uint8_t value, uint8_t MAX_SIZE)
{
	if(HAL_I2C_Mem_Write(&hi2c3, ADDRESS_MPU6050 << 1 , reg, MAX_SIZE, &value, MAX_SIZE, HAL_MAX_DELAY) != HAL_OK){
		uartSendString((uint8_t*)"ERROR HANDLER MPU6050 WRITE!\r\n");
		Error_Handler();
	}
}


void MPU6050_PortI2C_ReadRegister(uint8_t reg, uint8_t* buffer, uint8_t MAX_SIZE, uint8_t length)
{
	if(HAL_I2C_Mem_Read(&hi2c3, ADDRESS_MPU6050 << 1 , reg, MAX_SIZE, buffer, length, HAL_MAX_DELAY) != HAL_OK){
		uartSendString((uint8_t*)"ERROR HANDLER MPU6050 READ!\r\n");
		Error_Handler();
	}

}


