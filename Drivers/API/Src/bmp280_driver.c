/*
 * BMP280_driver.h
 *
 *  Created on: Apr 20, 2025
 *      Author: doddy
 */

#include "bmp280_driver.h"
#include "bmp280_port.h"
#include "API_uart.h"
#include "math.h"
#include "API_delay.h"

static bmp280_init_state_t bmp280_state = BMP280_INIT_START;
static delay_t bmp280_delay;
static bool_t bmp280_ready = false;

static uint16_t dig_T1, dig_P1;
static int16_t dig_T2, dig_T3;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static int32_t t_fine;

static uint8_t BMP280_ReadRegister(uint8_t reg);
static void BMP280_WriteRegister(uint8_t reg, uint8_t value);
static void BMP280_ReadCalibrationData(void);

/**
 * @brief Lee y almacena los datos de calibración interna del sensor BMP280.
 *
 * Esta función accede a los registros de calibración del BMP280 a través del bus SPI,
 * y guarda los coeficientes necesarios para el cálculo de temperatura y presión corregidas.
 * Estos coeficientes son únicos para cada sensor y deben ser leídos tras el inicio y antes de
 * hacer cualquier conversión de mediciones.
 *
 * @details
 * 1. Se realiza una lectura secuencial de 24 bytes desde la dirección base `BMP280_REG_CALIB_START` (0x88).
 * 2. Se utiliza el protocolo SPI, activando el chip con `BMP280_SPI_CS_Select()` y desactivándolo al final.
 * 3. Los datos se interpretan en formato little-endian y se copian a las variables globales:
 *    - `dig_T1` a `dig_T3` → coeficientes de calibración de temperatura
 *    - `dig_P1` a `dig_P9` → coeficientes de calibración de presión
 *
 * @note
 * - Esta función debe ser llamada solo una vez tras el arranque del sensor.
 * - Es fundamental para el uso de las funciones de compensación de temperatura y presión.
 * - Asegurate de que las funciones `BMP280_PortSPI_WriteRegister` y `BMP280_PortSPI_ReadRegister`
 *   estén correctamente implementadas según el microcontrolador y la biblioteca HAL.
 *
 */

static void BMP280_ReadCalibrationData(void) {
    uint8_t calib_data[24];
    uint8_t reg = BMP280_REG_CALIB_START;
    BMP280_SPI_CS_Select();
    reg |= 0x80; // lectura
    BMP280_PortSPI_WriteRegister(&reg, 1);
    BMP280_PortSPI_ReadRegister(calib_data, 24);
    BMP280_SPI_CS_Deselect();

    dig_T1 = (uint16_t)(calib_data[1] << 8 | calib_data[0]);
    dig_T2 = (int16_t)(calib_data[3] << 8 | calib_data[2]);
    dig_T3 = (int16_t)(calib_data[5] << 8 | calib_data[4]);

    dig_P1 = (uint16_t)(calib_data[7] << 8 | calib_data[6]);
    dig_P2 = (int16_t)(calib_data[9] << 8 | calib_data[8]);
    dig_P3 = (int16_t)(calib_data[11] << 8 | calib_data[10]);
    dig_P4 = (int16_t)(calib_data[13] << 8 | calib_data[12]);
    dig_P5 = (int16_t)(calib_data[15] << 8 | calib_data[14]);
    dig_P6 = (int16_t)(calib_data[17] << 8 | calib_data[16]);
    dig_P7 = (int16_t)(calib_data[19] << 8 | calib_data[18]);
    dig_P8 = (int16_t)(calib_data[21] << 8 | calib_data[20]);
    dig_P9 = (int16_t)(calib_data[23] << 8 | calib_data[22]);
}

/**
 * @brief Lee un único registro del sensor BMP280 utilizando SPI.
 *
 * Esta función realiza una lectura SPI de 8 bits desde el registro especificado del BMP280.
 * Se usa para acceder a configuraciones, resultados de conversión o cualquier registro del sensor.
 *
 * @param reg Dirección del registro a leer (por ejemplo, `BMP280_REG_ID` o `BMP280_REG_TEMP_MSB`).
 *            La función se encarga de establecer el bit de lectura (MSB = 1).
 *
 * @return Valor de 8 bits leído desde el registro solicitado.
 *
 * @details
 * 1. El bit MSB del registro (`reg | 0x80`) se activa para indicar una operación de lectura según
 *    el protocolo SPI del BMP280.
 * 2. Se selecciona el dispositivo activando su línea CS (`BMP280_SPI_CS_Select`).
 * 3. Se escribe la dirección del registro y luego se realiza la lectura de un byte.
 * 4. Finalmente, se desactiva la línea CS (`BMP280_SPI_CS_Deselect`) y se devuelve el valor leído.
 *
 * @note
 * - Esta función es útil para tareas de diagnóstico, lectura de ID del dispositivo, estado o configuración.
 * - Para leer múltiples registros, se recomienda usar funciones de lectura en bloque.
 *
 */

static uint8_t BMP280_ReadRegister(uint8_t reg) {
    uint8_t rx, tx = reg | 0x80; // Set MSB for read
    BMP280_SPI_CS_Select();
    BMP280_PortSPI_WriteRegister(&tx, 1);
    BMP280_PortSPI_ReadRegister(&rx, 1);
    BMP280_SPI_CS_Deselect();
    return rx;
}

/**
 * @brief Escribe un valor de 8 bits en un registro del BMP280 mediante SPI.
 *
 * Esta función permite configurar el sensor BMP280 escribiendo en uno de sus registros de control,
 * configuración o estado. Utiliza el bus SPI para enviar la dirección del registro y el dato.
 *
 * @param reg   Dirección del registro a escribir (por ejemplo, `BMP280_REG_CTRL_MEAS`).
 *              El bit MSB se limpia automáticamente para indicar una operación de escritura.
 * @param value Valor de 8 bits que se desea escribir en el registro.
 *
 * @details
 * 1. Se prepara un arreglo `data[2]` donde:
 *    - `data[0]` contiene la dirección del registro con el bit MSB en 0 (escritura).
 *    - `data[1]` contiene el valor a escribir.
 * 2. Se selecciona el sensor activando el pin CS (`BMP280_SPI_CS_Select`).
 * 3. Se envían ambos bytes mediante `BMP280_PortSPI_WriteRegister`.
 * 4. Se desactiva el pin CS (`BMP280_SPI_CS_Deselect`) para finalizar la transacción SPI.
 *
 * @note
 * - Es fundamental asegurarse de que el sensor esté en modo de reposo o standby
 *   cuando se modifica la configuración, según las especificaciones del datasheet.
 * - El bit MSB debe estar en 0 para escritura (por eso se enmascara con `0x7F`).
 *
 */
static void BMP280_WriteRegister(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg & 0x7F, value};
    BMP280_SPI_CS_Select();
    BMP280_PortSPI_WriteRegister(data, 2);
    BMP280_SPI_CS_Deselect();
}

/**
 * @brief Lee un valor de 8 bits desde un registro del BMP280.
 *
 * Esta función es un alias directo de `BMP280_ReadRegister`, usada para mejorar la legibilidad
 * del código cuando se necesita una lectura simple de 1 byte desde el sensor.
 *
 * @param reg Dirección del registro a leer.
 *
 * @return Valor de 8 bits leído desde el registro.
 *
 * @details
 * Internamente, esta función llama a `BMP280_ReadRegister(reg)`, el cual realiza la transacción
 * SPI correspondiente. Se utiliza comúnmente en etapas de inicialización o verificación de estado.
 *
 * @note
 * - Para mayor control, se puede utilizar directamente `BMP280_ReadRegister()`.
 *
 */
uint8_t BMP280_Read8(uint8_t reg) {
    return BMP280_ReadRegister(reg);
}

/**
 * @brief Escribe un valor de 8 bits en un registro del BMP280.
 *
 * Esta función es un alias directo de `BMP280_WriteRegister`, utilizada para mejorar la legibilidad
 * o mantener compatibilidad con estilos de API donde las funciones se nombran como `Write8`.
 *
 * @param reg   Dirección del registro donde se desea escribir.
 * @param value Valor de 8 bits a escribir en el registro.
 *
 * @details
 * Internamente, esta función llama a `BMP280_WriteRegister(reg, value)`, la cual se encarga
 * de enviar el dato al sensor por SPI. Es útil para configurar parámetros del sensor como el modo de operación,
 * oversampling o filtros.
 */

void BMP280_Write8(uint8_t reg, uint8_t value) {
    BMP280_WriteRegister(reg, value);
}


/**
 * @brief Inicializa el sensor BMP280 a través de la interfaz SPI.
 *
 * Esta función verifica la identidad del sensor, lo resetea, carga los coeficientes de calibración,
 * y configura los registros de operación con parámetros por defecto.
 *
 * @details
 * 1. Espera 100 ms para asegurar que el sensor esté listo tras el encendido.
 * 2. Verifica el ID del sensor (registro `0xD0`) para confirmar que se trata de un BMP280.
 *    Si el valor leído no coincide con `BMP280_CHIP_ID`, se informa por UART y se detiene la ejecución.
 * 3. Se envía un comando de "soft reset" escribiendo `BMP280_RESET_VALUE` en `BMP280_REG_RESET`.
 * 4. Se vuelve a esperar para asegurar la aplicación del reset.
 * 5. Se leen los coeficientes de calibración del sensor con `BMP280_ReadCalibrationData()`.
 * 6. Se configura el registro `CONFIG` con un filtro IIR y tiempo de espera (`tstandby`).
 * 7. Finalmente, se activa el sensor configurando el registro `CTRL_MEAS` con:
 *    - Oversampling x1 para temperatura y presión
 *    - Modo normal de operación (`0x27`)
 *
 */

void BMP280_Init(void) {
    HAL_Delay(100);
    uint8_t id = 0xD0;
    BMP280_SPI_CS_Select();
    id |= 0x80;
    BMP280_PortSPI_WriteRegister(&id, 1);
    BMP280_PortSPI_ReadRegister(&id, 1);
    BMP280_SPI_CS_Deselect();

    if (id != BMP280_CHIP_ID) {
        uartSendString((uint8_t*)"BMP280 NOT FOUND\r\n");
        Error_Handler();
    }

    uint8_t reset_cmd[2] = {BMP280_REG_RESET, BMP280_RESET_VALUE};
    BMP280_PortSPI_WriteRegister(reset_cmd, 2);
    HAL_Delay(100);

    BMP280_ReadCalibrationData();

    uint8_t config[2] = {BMP280_REG_CONFIG, 0xA0};
    BMP280_PortSPI_WriteRegister(config, 2);
    uint8_t ctrl_meas[2] = {BMP280_REG_CTRL_MEAS, 0x27};
    BMP280_PortSPI_WriteRegister(ctrl_meas, 2);
}

/**
 * @brief Lee y calcula la temperatura compensada en grados Celsius desde el BMP280.
 *
 * Esta función realiza la lectura de temperatura cruda desde los registros del BMP280,
 * aplica la compensación utilizando los coeficientes de calibración leídos previamente,
 * y devuelve el valor final en grados Celsius como número de punto flotante.
 *
 * @return Temperatura en °C (float), con dos decimales de precisión aproximada.
 *
 * @details
 * 1. Se leen 3 bytes del sensor (MSB, LSB, XLSB) que conforman el valor ADC de temperatura (`adc_T`).
 * 2. Se arma el valor de 20 bits desplazando y combinando los bytes:
 *    ```
 *    adc_T = (MSB << 12) | (LSB << 4) | (XLSB >> 4)
 *    ```
 * 3. Se aplican las fórmulas de compensación oficiales (int32) que dependen de los coeficientes:
 *    - `dig_T1`, `dig_T2`, `dig_T3`
 * 4. El valor intermedio `t_fine` se guarda globalmente, ya que se utiliza luego para calcular la presión.
 * 5. La temperatura final se devuelve como número flotante dividiendo el resultado entero por 100.0.
 *
 * @note
 * - Es obligatorio haber ejecutado previamente `BMP280_ReadCalibrationData()` para que los coeficientes estén cargados.
 * - El valor `t_fine` debe mantenerse entre mediciones, ya que es requerido por la función de presión.
 *
 */

float BMP280_ReadTemperature(void) {
	BMP280_Write8(BMP280_REG_CTRL_MEAS, 0x27);
    uint8_t raw_data[3];
    uint8_t reg = BMP280_REG_TEMP_MSB | 0x80;
    BMP280_SPI_CS_Select();
    BMP280_PortSPI_WriteRegister(&reg, 1);
    BMP280_PortSPI_ReadRegister(raw_data, 3);
    BMP280_SPI_CS_Deselect();

    int32_t adc_T = (int32_t)(((uint32_t)raw_data[0] << 12) | ((uint32_t)raw_data[1] << 4) | (raw_data[2] >> 4));

    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
                   ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;
    float T = (t_fine * 5 + 128) >> 8;
    return T / 100.0f;
}

/**
 * @brief Lee y calcula la presión atmosférica compensada desde el BMP280.
 *
 * Esta función realiza la lectura cruda de la presión (20 bits) desde el sensor,
 * aplica las fórmulas de compensación oficiales de Bosch y devuelve el valor
 * corregido en hectopascales (hPa o mbar).
 *
 * @return Presión atmosférica en hPa (float). Por ejemplo, 1013.25 hPa para nivel del mar.
 *
 * @details
 * 1. Se leen 3 bytes desde los registros de presión (`MSB`, `LSB`, `XLSB`) y se reconstruye el valor `adc_P` de 20 bits.
 * 2. Se aplican las fórmulas oficiales de compensación usando variables `int64_t` para mantener la precisión.
 * 3. El cálculo requiere que `t_fine` haya sido actualizado previamente llamando a `BMP280_ReadTemperature()`.
 * 4. El resultado se devuelve como número flotante dividiendo entre 25600.0, convirtiendo los pascales en hPa.
 *
 * @note
 * - La función depende de `t_fine`, por lo que **debe llamarse después de `BMP280_ReadTemperature()`**.
 * - Se implementa una verificación para evitar división por cero si `dig_P1` es cero (sensor defectuoso o sin calibración).
 * - Idealmente, se debería leer ambos valores (temperatura y presión) de una misma conversión para máxima coherencia.
 *
 */

float BMP280_ReadPressure(void) {
	BMP280_Write8(BMP280_REG_CTRL_MEAS, 0x27);
    uint8_t raw_data[3];
    uint8_t reg = BMP280_REG_PRESS_MSB | 0x80;
    BMP280_SPI_CS_Select();
    BMP280_PortSPI_WriteRegister(&reg, 1);
    BMP280_PortSPI_ReadRegister(raw_data, 3);
    BMP280_SPI_CS_Deselect();

    int32_t adc_P = (int32_t)(((uint32_t)raw_data[0] << 12) | ((uint32_t)raw_data[1] << 4) | (raw_data[2] >> 4));

    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

    if (var1 == 0) return 0; // Evitar división por cero

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (float)p / 25600.0f;
}

/**
 * @brief Calcula la altitud aproximada (en metros) a partir de la presión atmosférica.
 *
 * Esta función utiliza la presión actual leída desde el BMP280 y la compara con la presión
 * a nivel del mar (`sea_level_hPa`) para estimar la altitud en metros sobre el nivel del mar.
 *
 * @param sea_level_hPa Presión atmosférica a nivel del mar en hPa (típicamente 1013.25).
 *                      Este valor puede variar según la ubicación geográfica y condiciones climáticas.
 *
 * @return Altitud estimada en metros (float).
 *
 * @details
 * 1. Llama a `BMP280_ReadPressure()` para obtener la presión atmosférica actual en hPa.
 * 2. Aplica la **fórmula barométrica estándar**:
 *    ```
 *    Altitud = 44330 × (1 - (P / P0)^0.1903)
 *    ```
 *    donde:
 *    - `P`  = presión medida (hPa)
 *    - `P0` = presión a nivel del mar (hPa)
 * 3. El resultado se devuelve como un número de punto flotante con precisión suficiente para aplicaciones generales.
 *
 * @note
 * - Este cálculo es una estimación y puede verse afectado por condiciones de temperatura, humedad y presión local.
 * - Es útil para aplicaciones como sensores de altura, altímetros, estaciones meteorológicas, etc.
 * - La función depende de `BMP280_ReadPressure()`, el cual **requiere** que se haya actualizado `t_fine` mediante `BMP280_ReadTemperature()` previamente.
 *
 */

float BMP280_ReadAltitude(float sea_level_hPa) {
    float pressure_hPa = BMP280_ReadPressure();
    return 44330.0f * (1.0f - powf(pressure_hPa / sea_level_hPa, 0.1903f));
}

/**
 * @brief Inicializa el sensor BMP280 mediante una máquina de estados finita (FSM).
 *
 * Esta función ejecuta la inicialización del BMP280 en múltiples pasos utilizando delays no bloqueantes.
 * Permite que el sistema continúe operando mientras se completa la secuencia de configuración del sensor.
 * Debe llamarse repetidamente dentro del bucle principal hasta que el sensor esté completamente inicializado.
 *
 * La secuencia incluye:
 * - Tiempo de estabilización tras el encendido.
 * - Lectura y validación del ID del dispositivo.
 * - Reinicio del sensor.
 * - Lectura de datos de calibración.
 * - Configuración de los registros de medición y filtrado.
 *
 * Cambia el estado global `bmp280_ready` a `true` cuando la inicialización finaliza correctamente.
 */

void BMP280_InitStep()
{
	static uint8_t id = 0;

	switch(bmp280_state)
	{
		case BMP280_INIT_START:
            delayInit(&bmp280_delay, 100);  // Delay tras power-on
            bmp280_state = BMP280_WAIT_POWERON;
            break;

        case BMP280_WAIT_POWERON:
            if (delayRead(&bmp280_delay)) {
                bmp280_state = BMP280_READ_ID;
            }
            break;

        case BMP280_READ_ID:
            id = BMP280_REG_ID | 0x80;
            BMP280_SPI_CS_Select();
            BMP280_PortSPI_WriteRegister(&id, 1);
            BMP280_PortSPI_ReadRegister(&id, 1);
            BMP280_SPI_CS_Deselect();
            bmp280_state = BMP280_CHECK_ID;
            break;

        case BMP280_CHECK_ID:
            if (id != BMP280_CHIP_ID) {
                uartSendString((uint8_t*)"BMP280 NOT FOUND\r\n");
                Error_Handler();
            } else {
                bmp280_state = BMP280_RESET_CMD;
            }
            break;

        case BMP280_RESET_CMD: {
            uint8_t reset_cmd[2] = {BMP280_REG_RESET, BMP280_RESET_VALUE};
            BMP280_PortSPI_WriteRegister(reset_cmd, 2);
            delayInit(&bmp280_delay, 100); // Delay tras reset
            bmp280_state = BMP280_WAIT_RESET;
            break;
        }

        case BMP280_WAIT_RESET:
            if (delayRead(&bmp280_delay)) {
                bmp280_state = BMP280_READ_CALIB;
            }
            break;


        case BMP280_READ_CALIB:
            BMP280_ReadCalibrationData();
            bmp280_state = BMP280_CONFIGURE;
            break;


        case BMP280_CONFIGURE: {
            uint8_t config[2] = {BMP280_REG_CONFIG, 0xA0};
            BMP280_PortSPI_WriteRegister(config, 2);
            uint8_t ctrl_meas[2] = {BMP280_REG_CTRL_MEAS, 0x27};
            BMP280_PortSPI_WriteRegister(ctrl_meas, 2);
            bmp280_state = BMP280_INIT_DONE;
            break;
        }

        case BMP280_INIT_DONE:
        	uartSendString((uint8_t*)"BMP280_INIT_DONE\r\n");
            bmp280_ready = true;
            break;

	}
}

/**
 * @brief Indica si el sensor BMP280 ha finalizado su secuencia de inicialización.
 *
 * Esta función devuelve el valor del flag interno `bmp280_ready`, que es establecido a `true`
 * una vez que la máquina de estados `BMP280_InitStep()` ha completado exitosamente todas las etapas
 * de configuración del sensor.
 *
 * @note
 * - Es útil para coordinar la ejecución de lecturas del sensor sólo cuando el BMP280 esté listo.
 * - Se recomienda llamar a esta función en el bucle principal del programa para verificar
 *   si ya es seguro comenzar a utilizar el sensor.
 *
 * @return `true` si la inicialización del BMP280 se ha completado correctamente, `false` en caso contrario.
 */

bool_t BMP280_IsReady(void)
{
    return bmp280_ready;
}
