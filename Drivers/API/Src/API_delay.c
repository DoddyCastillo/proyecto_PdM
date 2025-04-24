/*
 * API_delay.c
 *
 *  Created on: Mar 20, 2025
 *      Author: doddy
 */

#include <API_delay.h>

/**
  * @brief Inicializa la estructua de retardo con la duración incial especificada
  * @param delay_t puntero a la estructura delay_t
  * @param duration Duración del retardo en milisegundos
  * @retval None
  */

void delayInit( delay_t * delay, tick_t duration ){

	if(delay != NULL && duration > 0 )
	{
		delay->duration = duration;
		delay->running = false;
	}
}

/**
  * @brief Lee el estado del retardo (running) y verficia si el tiempo se ha cumplido
  * @param delay_t puntero a la estructura delay_t
  * @retval true si el retardo ha finalizado, false caso contrario
  */

bool_t delayRead( delay_t * delay ){

	if(delay == NULL) return false;

	if(!delay->running)
	{
		delay->startTime = HAL_GetTick();
		delay->running = true;
	}

	if((HAL_GetTick() - delay->startTime) >= delay->duration)
	{
		delay->running = false;
		return true;
	}

	return false;
}

/**
  * @brief Verifica si el retardo está en ejecución (running).
  * @param delay Puntero a la estructura delay_t.
  * @retval true si el retardo está en ejecución, false caso contrario.
  */

bool_t delayIsRunning(delay_t * delay)
{
	if(delay == NULL) return false;

	return delay->running;
}


/**
 * @brief Modifica la duración de un retardo existente y verifica si el retardo esta en ejecución
 * @param delay puntero a la estructura delay_t
 * @param duration nueva duración del retardo en milisegundos
 * @retval None
 */

void delayWrite( delay_t * delay, tick_t duration ){

    if (delay != NULL && duration > 0) {
        delay->duration = duration;
    }
}
