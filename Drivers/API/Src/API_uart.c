/*
 * API_uart.c
 *
 *  Created on: Apr 13, 2025
 *      Author: doddy
 */


#include "API_uart.h"
#include <string.h>

#define UART_TIMEOUT 100
#define UART_MAX_SIZE 1024

static UART_HandleTypeDef huart2;

static bool_t checkPointer(const uint8_t *ptr);
static bool_t checkSize(uint16_t size);


bool_t uartInit(void)
{
	  huart2.Instance = USART2;
	  huart2.Init.BaudRate = 115200;
	  huart2.Init.WordLength = UART_WORDLENGTH_8B;
	  huart2.Init.StopBits = UART_STOPBITS_1;
	  huart2.Init.Parity = UART_PARITY_NONE;
	  huart2.Init.Mode = UART_MODE_TX_RX;
	  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

	  if (HAL_UART_Init(&huart2) != HAL_OK)
	  {
		  return false;
	  }

	  else
	  {
		  const char *msg = "Uart inicializada a 115200, 8N1\r\n";

		  HAL_UART_Transmit(&huart2, (uint8_t *) msg , strlen(msg), HAL_MAX_DELAY);

		  return true;
	  }

	  return false;

}

void uartSendString(uint8_t * pstring)
{
    if (!checkPointer(pstring)) return;

    uint16_t length = strlen((char*)pstring);

    if (!checkSize(length)) return;

    HAL_UART_Transmit(&huart2, pstring, length, UART_TIMEOUT);
}

void uartSendStringSize(uint8_t * pstring, uint16_t size)
{
    if (!checkPointer(pstring) || !checkSize(size)) return;

    HAL_UART_Transmit(&huart2, pstring, size, UART_TIMEOUT);
}



static bool_t checkPointer(const uint8_t *ptr)
{
	return (ptr != NULL);
}

static bool_t checkSize(uint16_t size)
{
	return (size > 0 && size <= UART_MAX_SIZE);
}
