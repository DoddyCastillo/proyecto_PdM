/*
 * bmp280_port.h
 *
 *  Created on: Apr 20, 2025
 *      Author: doddy
 */

#ifndef API_INC_BMP280_PORT_H_
#define API_INC_BMP280_PORT_H_

#include "stm32f4xx_hal.h"
#include "stdint.h"

extern void Error_Handler(void);

void BMP280_SPI_Init(void);
void BMP280_SPI_CS_Init(void);
void BMP280_PortSPI_WriteRegister(uint8_t * valor, uint8_t size);
void BMP280_PortSPI_ReadRegister(uint8_t *valor, uint8_t size);
void BMP280_SPI_CS_Select(void);
void BMP280_SPI_CS_Deselect(void);

#endif /* API_INC_BMP280_PORT_H_ */
