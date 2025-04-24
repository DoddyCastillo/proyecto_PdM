/*
 * BMP280_port.c
 *
 *  Created on: Apr 20, 2025
 *      Author: doddy
 */


#include "bmp280_port.h"
#include "API_uart.h"

static SPI_HandleTypeDef hspi2;

void BMP280_SPI_Init(void)
{
	  hspi2.Instance = SPI2;
	  hspi2.Init.Mode = SPI_MODE_MASTER;
	  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	  hspi2.Init.NSS = SPI_NSS_SOFT;
	  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	  hspi2.Init.CRCPolynomial = 10;
	  if (HAL_SPI_Init(&hspi2) != HAL_OK)
	  {
		uartSendString((uint8_t*)"SPI not Init!\r\n");
	    Error_Handler();
	  }
}

void BMP280_SPI_CS_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void BMP280_PortSPI_WriteRegister(uint8_t *valor, uint8_t size)
{
	if(HAL_SPI_Transmit(&hspi2, valor , size, HAL_MAX_DELAY) != HAL_OK)
	{
		uartSendString((uint8_t*)"ERROR HANDLER BMP280 WRITE!\r\n");
		Error_Handler();
	}
}

void BMP280_PortSPI_ReadRegister(uint8_t *valor, uint8_t size)
{
	if(HAL_SPI_Receive(&hspi2, valor , size, HAL_MAX_DELAY) != HAL_OK)
	{
		uartSendString((uint8_t*)"ERROR HANDLER BMP280 READ!\r\n");
		Error_Handler();
	}
}

void BMP280_SPI_CS_Select(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

void BMP280_SPI_CS_Deselect(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}
