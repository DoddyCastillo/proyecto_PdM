/*
 * mpu6050_driver.c
 *
 *  Created on: Apr 12, 2025
 *      Author: doddy
 */


#include "mpu6050_driver.h"
#include "mpu6050_port.h"
#include "API_uart.h"
#include "API_delay.h"

static mpu6050_init_state_t mpu_state = MPU6050_INIT_IDLE;
static delay_t mpu_delay;
static bool_t mpu_ready = false;

static Vector3f gyro = {0}, accel = {0};
static Vector3i16 gyroi16 = {0}, acceli16 = {0};

static int16_t MPU6050_RawMeasurementRead(uint8_t address);
// Float Measurements
static float MPU6050_ReadTemperature();
static Vector3f MPU6050_ReadGyroscope();
static Vector3f MPU6050_ReadAccelerometer();
// Int Measurements
static int16_t  MPU6050_ReadTemperatureInt();
static Vector3i16 MPU6050_ReadGyroscopeInt();
static Vector3i16 MPU6050_ReadAccelerometerInt();

/**
 * @brief Lee una medición cruda de 16 bits desde un registro del MPU6050.
 *
 * Esta función realiza la lectura de dos bytes consecutivos desde una dirección
 * del MPU6050 (por ejemplo, registros del acelerómetro, giroscopio o temperatura),
 * y reconstruye el valor como un entero con signo (`int16_t`).
 *
 * @param address Dirección del primer byte del dato de 16 bits a leer (ej., `MPU6050_REG_ACCEL_XOUT_H`).
 *
 * @return Valor crudo de 16 bits con signo (int16_t), combinado de los dos bytes leídos.
 *
 * @details
 * 1. Se lee un bloque de dos bytes desde la dirección especificada usando la función
 *    `MPU6050_PortI2C_ReadRegister()`.
 * 2. Los bytes se combinan en formato big-endian: `buf[0]` es el byte más significativo (MSB),
 *    `buf[1]` el menos significativo (LSB).
 * 3. Se convierte el resultado a `int16_t`, ya que los datos del MPU6050 son firmados.
 *
 * @note
 * - Esta función es una base común para lecturas de aceleración, velocidad angular o temperatura.
 * - Asegurarse de que el sensor esté correctamente inicializado antes de invocar esta función.
 * - Depende de la implementación correcta de `MPU6050_PortI2C_ReadRegister`.
 *
 * @example
 * ```c
 * int16_t ax = MPU6050_RawMeasurementRead(MPU6050_REG_ACCEL_XOUT_H);
 * int16_t gx = MPU6050_RawMeasurementRead(MPU6050_REG_GYRO_XOUT_H);
 * int16_t temp = MPU6050_RawMeasurementRead(MPU6050_REG_TEMP_OUT_H);
 * ```
 */

static int16_t MPU6050_RawMeasurementRead(uint8_t address)
{
	uint8_t buf[LENGTH_DATA];
	MPU6050_PortI2C_ReadRegister(address, buf, MAX_BYTE_REGISTER, LENGTH_DATA);
	return (int16_t)((buf[0] << 8) | buf[1]);
}

// Int Measurements

/**
 * @brief Lee la temperatura interna del MPU6050 y la convierte a °C con escala x100 (centésimas).
 *
 * Esta función accede al registro de temperatura del MPU6050, realiza la conversión
 * según la fórmula del datasheet y devuelve el resultado como un entero con escala x100.
 *
 * @return Temperatura en centésimas de grado Celsius (int16_t).
 *         Por ejemplo, un retorno de `2534` representa 25.34 °C.
 *
 * @details
 * 1. Se obtiene el valor crudo de 16 bits desde el registro `TEMP_OUT_H` usando `MPU6050_RawMeasurementRead()`.
 * 2. Se aplica la fórmula de conversión especificada por el fabricante:
 *    ```
 *    Temp(°C) = (raw / 340) + 36.53
 *    ```
 *    Al multiplicar por 100 para evitar el uso de `float`, se transforma a:
 *    ```
 *    Temp_x100 = ((raw * 100) / 340) + 3653
 *    ```
 * 3. El resultado final es un valor entero, ideal para visualización eficiente en sistemas embebidos.
 *
 * @note
 * - No se utiliza punto flotante, por lo que esta función es adecuada para plataformas sin FPU.
 * - Si necesitás mayor precisión visual, podés convertir este valor a `float` más adelante (dividiendo por 100.0f).
 * - El valor puede fluctuar ligeramente, ya que esta temperatura es la del chip y no del ambiente externo.
 *
 * @example
 * ```c
 * int16_t temp_x100 = MPU6050_ReadTemperatureInt();
 * char buffer[10];
 * FormatIntDecimal(buffer, temp_x100, 2);  // "36.85"
 * LCD_PrintLine(0, buffer);
 * ```
 */

int16_t MPU6050_ReadTemperatureInt()
{
    int16_t raw = MPU6050_RawMeasurementRead(TEMP_OUT_H);
    return ((raw * 100) / 340) + 3653;
}

/**
 * @brief Lee los valores del giroscopio en los tres ejes y los convierte a °/s escalados por 100.
 *
 * Esta función accede a los registros crudos del giroscopio (X, Y, Z) del MPU6050, convierte cada valor
 * a grados por segundo usando la sensibilidad configurada (por defecto ±250 °/s), y los retorna
 * como un vector de enteros (`Vector3i16`) escalados por 100.
 *
 * @return Estructura `Vector3i16` con los valores del giroscopio en X, Y y Z, en centésimas de grado/segundo (°/s × 100).
 *
 * @details
 * 1. El sensor entrega valores crudos de 16 bits por cada eje, desde registros consecutivos:
 *    - `GYRO_XOUT_H`, `GYRO_YOUT_H`, `GYRO_ZOUT_H`
 * 2. Se lee cada eje mediante `MPU6050_RawMeasurementRead()` sumando el desplazamiento de 2 bytes entre registros.
 * 3. Cada valor se escala usando la constante `FS_LSB_GYRO_250` (típicamente `131 LSB/(°/s)` para ±250°/s).
 *    La multiplicación por 100 permite representar el resultado como un entero (evitando `float`).
 *
 * @note
 * - Requiere que el sensor esté configurado previamente con el rango de ±250 °/s para que el factor sea correcto.
 * - Este formato es ideal para visualización, envío por UART o procesamiento sin FPU.
 *
 * @example
 * ```c
 * Vector3i16 gyro = MPU6050_ReadGyroscopeInt();
 * // Por ejemplo: gyro.x = -1234 → -12.34 °/s
 * ```
 */

static Vector3i16 MPU6050_ReadGyroscopeInt()
{
    for (int i = 0; i < 3; i++) {
        int16_t raw_gyro = MPU6050_RawMeasurementRead(GYRO_XOUT_H + 2*i);
        ((int16_t*)&gyroi16)[i] = (raw_gyro * 100) / FS_LSB_GYRO_250;
    }
    return gyroi16;
}

/**
 * @brief Lee los valores del acelerómetro en los tres ejes y los convierte a "g" escalados por 100.
 *
 * Esta función accede a los registros crudos del acelerómetro (X, Y, Z) del MPU6050, convierte cada valor
 * a unidades de aceleración gravitacional (`g`) y los devuelve en una estructura `Vector3i16`.
 * Los resultados están escalados por 100 para evitar el uso de punto flotante.
 *
 * @return Estructura `Vector3i16` con valores de aceleración en X, Y y Z, en centésimas de "g" (g × 100).
 *
 * @details
 * 1. El MPU6050 entrega valores crudos de 16 bits por cada eje desde registros consecutivos:
 *    - `ACCEL_XOUT_H`, `ACCEL_YOUT_H`, `ACCEL_ZOUT_H`
 * 2. Se accede a cada registro utilizando `MPU6050_RawMeasurementRead`, avanzando de 2 en 2 bytes.
 * 3. Cada valor se escala usando la constante `FS_LSB_GYRO_250`, aunque aquí parece haber un **error conceptual**:
 *    ⚠️ **Se debería usar `FS_LSB_ACCEL_2G`** (típicamente 16384 LSB/g) para el acelerómetro, no la constante del giroscopio.
 * 4. La multiplicación por 100 permite obtener una resolución de centésimas de g (`0.01g`), ideal para visualización y procesamiento sin `float`.
 *
 * @note
 * - Asegurarse de que el rango de medición del acelerómetro esté configurado en ±2g si se usa `FS_LSB_ACCEL_2G`.
 * - El uso de `FS_LSB_GYRO_250` en esta función es probablemente un error de copia/pega desde la función de giroscopio.
 *   Debería corregirse a la constante correspondiente del acelerómetro.
 *
 * @example
 * ```c
 * Vector3i16 acc = MPU6050_ReadAccelerometerInt();
 * // Por ejemplo: acc.z = 980 → 9.80 g (aceleración vertical en reposo)
 * ```
 */

static Vector3i16 MPU6050_ReadAccelerometerInt()
{
    for (int i = 0; i < 3; i++) {
        int16_t raw_accel = MPU6050_RawMeasurementRead(ACCEL_XOUT_H + 2*i);
        ((int16_t*)&acceli16)[i] = (raw_accel * 100) / FS_LSB_ACC_250;
    }
    return acceli16;
}

// Float Measurements

/**
 * @brief Lee los valores del acelerómetro en los tres ejes y los convierte a "g" como `float`.
 *
 * Esta función accede a los registros crudos del acelerómetro (X, Y, Z) del MPU6050, los convierte a
 * aceleración en unidades de gravedad ("g") y los devuelve en una estructura `Vector3f` de punto flotante.
 *
 * @return Estructura `Vector3f` con valores de aceleración en X, Y y Z (unidad: g).
 *
 * @details
 * 1. Se lee un valor crudo de 16 bits por cada eje desde los registros:
 *    - `ACCEL_XOUT_H`, `ACCEL_YOUT_H`, `ACCEL_ZOUT_H`
 * 2. Cada lectura se convierte directamente a `float`, dividiendo por el valor de sensibilidad en LSB/g.
 * 3. ⚠️ En esta función se está utilizando `FS_LSB_GYRO_250`, lo cual **no es correcto para el acelerómetro**.
 *    Debe usarse `FS_LSB_ACCEL_2G` (normalmente 16384 LSB/g para ±2g).
 * 4. Los valores finales se almacenan en una estructura `Vector3f` (por ejemplo, con campos `.x`, `.y`, `.z`).
 *
 * @note
 * - Esta función es ideal para procesamiento posterior como filtros digitales, cálculos de ángulo o detección de movimiento.
 * - El uso de punto flotante está optimizado en el STM32F446RE gracias a su unidad FPU.
 *
 * @example
 * ```c
 * Vector3f a = MPU6050_ReadAccelerometer();
 * printf("Accel X: %.2f g\n", a.x);
 * ```
 */

static Vector3f MPU6050_ReadAccelerometer()
{
    for (int i = 0; i < 3; i++) {
        int16_t raw_accel = MPU6050_RawMeasurementRead(ACCEL_XOUT_H + 2*i);
        ((float*)&accel)[i] = raw_accel / FS_LSB_ACC_250;
    }
    return accel;
}

/**
 * @brief Lee la temperatura interna del MPU6050 y la convierte a grados Celsius.
 *
 * Esta función accede al registro de temperatura interna del MPU6050 y aplica la fórmula
 * oficial de conversión a temperatura real en °C utilizando punto flotante.
 *
 * @return Temperatura en grados Celsius como valor `float`.
 *
 * @details
 * 1. Se obtiene el valor crudo de 16 bits desde `TEMP_OUT_H` usando `MPU6050_RawMeasurementRead()`.
 * 2. Se aplica la fórmula oficial del fabricante:
 *    ```
 *    Temp(°C) = (raw / 340.0) + 36.53
 *    ```
 *    Donde `340 LSB/°C` es la sensibilidad y `36.53` es el offset del sensor.
 * 3. El resultado es un valor en punto flotante, preciso y directo para visualización o procesamiento.
 *
 * @note
 * - Este valor representa la temperatura del chip, no del ambiente externo.
 * - Puede ser útil para monitoreo térmico del sistema o compensación en algoritmos más complejos.
 * - La precisión es suficiente para uso general, aunque no se recomienda como sensor ambiental principal.
 *
 * @example
 * ```c
 * float temp = MPU6050_ReadTemperature();
 * printf("Temperatura interna: %.2f °C\n", temp);
 * ```
 */
static float MPU6050_ReadTemperature()
{
	int16_t raw_temprature = MPU6050_RawMeasurementRead(TEMP_OUT_H);
	return (raw_temprature / 340.0f) + 36.53f;
}

/**
 * @brief Lee los valores del giroscopio en los tres ejes y los convierte a °/s como `float`.
 *
 * Esta función obtiene lecturas crudas de 16 bits del giroscopio del MPU6050 para los ejes X, Y y Z,
 * y las convierte a grados por segundo utilizando punto flotante, aprovechando la FPU del STM32F4.
 *
 * @return Estructura `Vector3f` con los valores del giroscopio en X, Y y Z (unidad: °/s).
 *
 * @details
 * 1. Se accede a los registros `GYRO_XOUT_H`, `GYRO_YOUT_H`, `GYRO_ZOUT_H`, leyendo 16 bits por eje.
 * 2. Cada valor crudo se convierte a grados por segundo usando la sensibilidad configurada:
 *    - Para un rango de ±250 °/s, el valor es `131 LSB/(°/s)` → `FS_LSB_GYRO_250 = 131`
 * 3. Se almacena el resultado en una estructura `Vector3f` que contiene los tres ejes como `float`.
 *
 * @note
 * - La constante `FS_LSB_GYRO_250` debe coincidir con la configuración de rango del giroscopio.
 *   Si el rango cambia (por ejemplo, ±500 o ±1000 °/s), debe actualizarse la constante correspondiente.
 * - Esta versión en `float` es ideal para algoritmos como filtros complementarios o de Kalman.
 *
 * @example
 * ```c
 * Vector3f g = MPU6050_ReadGyroscope();
 * printf("Giro X: %.2f °/s, Y: %.2f °/s, Z: %.2f °/s\n", g.x, g.y, g.z);
 * ```
 */
static Vector3f MPU6050_ReadGyroscope()
{
    for (int i = 0; i < 3; i++) {
        int16_t raw_gyro = MPU6050_RawMeasurementRead(GYRO_XOUT_H + 2*i);
        ((float*)&gyro)[i] = raw_gyro / FS_LSB_GYRO_250;
    }
    return gyro;
}


/**
 * @brief Inicializa el sensor MPU6050 mediante interfaz I2C.
 *
 * Esta función configura el MPU6050 para comenzar a tomar mediciones, estableciendo
 * los parámetros clave: encendido del dispositivo, divisor de muestreo, filtro DLPF,
 * y los rangos de medición tanto del giroscopio como del acelerómetro.
 *
 * @details
 * 1. Verifica que el sensor esté conectado mediante `MPU6050_IsAvailable()`.
 *    Si no está disponible, la función finaliza inmediatamente.
 * 2. Escribe los siguientes registros con las configuraciones definidas:
 *    - `PWR_MGMT_1`: activa el sensor (sale de modo sleep).
 *    - `SMPLRT_DIV`: define el divisor de la tasa de muestreo.
 *    - `CONFIG`: configura el DLPF (Digital Low Pass Filter).
 *    - `GYRO_CONFIG`: establece el rango del giroscopio (por defecto ±250 °/s).
 *    - `ACCEL_CONFIG`: establece el rango del acelerómetro (por defecto ±2 g).
 * 3. Aplica un retardo de 100 ms tras encender el sensor para asegurar su estabilidad.
 *
 * @note
 * - Esta función debe llamarse una vez al inicio del sistema antes de comenzar a leer datos.
 * - Las constantes como `CONF_PWR_MGMT`, `FS_GYRO_250` o `FS_ACC_2G` deben estar correctamente definidas
 *   según los valores esperados por el registro correspondiente del MPU6050.
 * - Se puede extender fácilmente para soportar configuración dinámica de rangos o filtros.
 *
 * @example
 * ```c
 * MPU6050_Init();
 * Vector3f acc = MPU6050_ReadAccelerometer();
 * ```
 */


void MPU6050_Init()
{
	if(!MPU6050_IsAvailable()) return;
	MPU6050_PortI2C_WriteRegister(PWR_MGMT_1, CONF_PWR_MGMT, MAX_BYTE_REGISTER);
	HAL_Delay(100);
	MPU6050_PortI2C_WriteRegister(SMPLRT_DIV, CONF_SMPLRT_DIV, MAX_BYTE_REGISTER);
	MPU6050_PortI2C_WriteRegister(CONFIG, CONFIG_DLPF, MAX_BYTE_REGISTER);
	MPU6050_PortI2C_WriteRegister(GYRO_CONFIG, FS_GYRO_250, MAX_BYTE_REGISTER);
	MPU6050_PortI2C_WriteRegister(ACCEL_CONFIG, FS_ACC_2G, MAX_BYTE_REGISTER);
}

/**
 * @brief Verifica la disponibilidad del MPU6050 e inicia su configuración si está presente.
 *
 * Esta función encapsula la llamada a `MPU6050_Init()`, permitiendo una verificación centralizada
 * del sensor MPU6050. Su uso facilita el manejo modular y puede integrarse en rutinas de diagnóstico
 * o de inicialización general del sistema.
 *
 * @details
 * - Internamente llama a `MPU6050_Init()`, la cual se encarga de verificar la disponibilidad del sensor
 *   mediante `MPU6050_IsAvailable()` y, si está presente, configura sus registros.
 * - Esta función puede ser extendida fácilmente para incluir reportes de error, logs, o reinicialización condicional.
 *
 * @note
 * - Aunque actualmente solo llama a `MPU6050_Init()`, su existencia mejora la semántica del código principal.
 *   Por ejemplo: `MPU6050_Check()` puede ser usada como parte de una rutina de diagnóstico o auto-test.
 * - Es buena práctica colocar esta llamada al inicio del `main()` o en el setup de FreeRTOS.
 *
 * @example
 * ```c
 * MPU6050_Check();  // Verifica e inicializa el sensor
 * ```
 */

void MPU6050_Check()
{
	MPU6050_Init();
}

// Int Measurements

/**
 * @brief Obtiene la temperatura interna del MPU6050 en formato entero (°C × 100).
 *
 * Esta función es un alias directo de `MPU6050_ReadTemperatureInt()`, que devuelve la
 * temperatura interna del sensor en centésimas de grado Celsius. Su uso permite mantener
 * una API más legible y coherente.
 *
 * @return Temperatura en centésimas de grado Celsius (por ejemplo, 2534 representa 25.34 °C).
 *
 * @details
 * - Internamente llama a `MPU6050_ReadTemperatureInt()`, que realiza la lectura del registro
 *   de temperatura y aplica la fórmula de conversión sin utilizar punto flotante.
 * - El valor retornado es ideal para visualización, formateo con `FormatIntDecimal`, o envío por UART.
 *
 * @note
 * - No se realiza ningún procesamiento adicional: esta función simplemente reexpone otra función existente.
 * - Puede usarse como parte de una API pública más descriptiva o estandarizada.
 *
 * @example
 * ```c
 * int16_t temp_x100 = MPU6050_GetTemperatureInt();
 * char buffer[10];
 * FormatIntDecimal(buffer, temp_x100, 2);  // Ej: "25.34"
 * LCD_PrintLine(1, buffer);
 * ```
 */

int16_t MPU6050_GetTemperatureInt()
{
	return MPU6050_ReadTemperatureInt();
}

/**
 * @brief Obtiene los valores del giroscopio del MPU6050 en formato entero (°/s × 100).
 *
 * Esta función es un alias directo de `MPU6050_ReadGyroscopeInt()`, que lee los datos del giroscopio
 * en los tres ejes y los devuelve en una estructura `Vector3i16` con los valores escalados en centésimas de grado por segundo.
 *
 * @return Estructura `Vector3i16` con los valores de X, Y, Z del giroscopio (unidad: °/s × 100).
 *
 * @details
 * - Cada componente del vector representa el valor del giroscopio multiplicado por 100 para mantener precisión
 *   sin usar punto flotante.
 * - Es útil en sistemas embebidos donde se prioriza el rendimiento y la eficiencia en RAM/CPU.
 * - Internamente delega toda la lógica de adquisición y escalado a `MPU6050_ReadGyroscopeInt()`.
 *
 * @note
 * - Asegurate de que el MPU6050 haya sido inicializado previamente con `MPU6050_Init()`.
 * - La escala depende de la constante `FS_LSB_GYRO_250`, que debe coincidir con la configuración actual del sensor.
 *
 * @example
 * ```c
 * Vector3i16 gyro = MPU6050_GetGyroscopeInt();
 * printf("Giro X: %d (%.2f °/s)\n", gyro.x, gyro.x / 100.0f);
 * ```
 */

Vector3i16  MPU6050_GetGyroscopeInt()
{
	return MPU6050_ReadGyroscopeInt();
}

/**
 * @brief Obtiene los valores del acelerómetro del MPU6050 en formato entero (g × 100).
 *
 * Esta función retorna los valores del acelerómetro en los tres ejes (X, Y, Z), escalados
 * como centésimas de "g", utilizando la función `MPU6050_ReadAccelerometerInt()`.
 *
 * @return Estructura `Vector3i16` con los valores de aceleración en X, Y y Z, en g × 100.
 *
 * @details
 * - Internamente llama a `MPU6050_ReadAccelerometerInt()`, que se encarga de:
 *   1. Leer los registros crudos del sensor (ACCEL_XOUT_H, etc.).
 *   2. Convertirlos a unidades de "g" multiplicadas por 100.
 * - Este formato es ideal para microcontroladores sin FPU o cuando se desea evitar `float`.
 * - Por ejemplo, un valor de `980` representa una aceleración de 9.80 g.
 *
 * @note
 * - La escala usada debe coincidir con el rango de configuración del acelerómetro (ej. ±2g → 16384 LSB/g).
 * - Asegurate de haber inicializado el sensor con `MPU6050_Init()` antes de llamar esta función.
 *
 * @example
 * ```c
 * Vector3i16 acc = MPU6050_GetAccelerometerInt();
 * printf("Acc X: %d (%.2f g)\n", acc.x, acc.x / 100.0f);
 * ```
 */

Vector3i16  MPU6050_GetAccelerometerInt()
{
	return MPU6050_ReadAccelerometerInt();
}

// Float Measurements

/**
 * @brief Obtiene la temperatura interna del MPU6050 en grados Celsius (`float`).
 *
 * Esta función retorna la temperatura medida internamente por el sensor MPU6050,
 * utilizando precisión en punto flotante. Es un alias directo de `MPU6050_ReadTemperature()`.
 *
 * @return Temperatura en grados Celsius (por ejemplo, `36.53`).
 *
 * @details
 * - Internamente llama a `MPU6050_ReadTemperature()`, que:
 *   1. Lee el valor crudo de 16 bits del registro `TEMP_OUT_H`.
 *   2. Aplica la fórmula de conversión: `(raw / 340.0) + 36.53`.
 * - El valor retornado es útil para monitoreo térmico o compensaciones internas del sistema.
 *
 * @note
 * - Este valor representa la temperatura del chip MPU6050, no necesariamente la temperatura ambiente externa.
 * - Requiere que el sensor haya sido previamente inicializado con `MPU6050_Init()`.
 *
 * @example
 * ```c
 * float temp = MPU6050_GetTemperature();
 * printf("Temperatura interna: %.2f °C\n", temp);
 * ```
 */


float MPU6050_GetTemperature()
{
	return  MPU6050_ReadTemperature();
}


/**
 * @brief Obtiene los valores del giroscopio del MPU6050 como `float` en °/s.
 *
 * Esta función devuelve una estructura `Vector3f` con los valores de velocidad angular
 * en los ejes X, Y y Z, en unidades de grados por segundo (°/s), usando punto flotante.
 * Es un alias directo de `MPU6050_ReadGyroscope()`.
 *
 * @return Estructura `Vector3f` con los valores del giroscopio en °/s (`float`).
 *
 * @details
 * - Internamente llama a `MPU6050_ReadGyroscope()`, que:
 *   1. Lee valores crudos de 16 bits desde los registros `GYRO_XOUT_H`, etc.
 *   2. Los convierte a °/s dividiendo por el factor de sensibilidad (típicamente 131 LSB/(°/s) para ±250 °/s).
 * - Los datos se devuelven como flotantes (`float`), aprovechando la FPU del STM32F4.
 *
 * @note
 * - Asegurate de que el sensor esté correctamente inicializado y configurado con `MPU6050_Init()`.
 * - El valor de sensibilidad (`FS_LSB_GYRO_250`) debe coincidir con el rango activo del sensor.
 *
 * @example
 * ```c
 * Vector3f gyro = MPU6050_GetGyroscope();
 * printf("Giro X: %.2f °/s\n", gyro.x);
 * ```
 */

Vector3f MPU6050_GetGyroscope() {
    return MPU6050_ReadGyroscope();
}

/**
 * @brief Obtiene los valores del acelerómetro del MPU6050 como `float` en unidades de g.
 *
 * Esta función devuelve una estructura `Vector3f` con las lecturas de aceleración en los ejes X, Y y Z,
 * expresadas en unidades de gravedad (g), utilizando punto flotante. Es un alias directo de `MPU6050_ReadAccelerometer()`.
 *
 * @return Estructura `Vector3f` con los valores de aceleración en g (`float`).
 *
 * @details
 * - Internamente llama a `MPU6050_ReadAccelerometer()`, que:
 *   1. Lee los registros crudos (`ACCEL_XOUT_H`, etc.).
 *   2. Convierte los valores a `float` dividiendo por la sensibilidad del acelerómetro (ej. 16384 LSB/g para ±2g).
 * - La salida está en punto flotante, ideal para cálculos posteriores con filtros de actitud, controladores PID, etc.
 *
 * @note
 * - El valor de sensibilidad (`FS_LSB_ACCEL_2G`) debe coincidir con el rango configurado en el sensor.
 * - Asegurate de inicializar el MPU6050 con `MPU6050_Init()` antes de usar esta función.
 *
 * @example
 * ```c
 * Vector3f acc = MPU6050_GetAccelerometer();
 * printf("Accel X: %.2f g\n", acc.x);
 * ```
 */

Vector3f MPU6050_GetAccelerometer()
{
    return MPU6050_ReadAccelerometer();
}

/**
 * @brief Verifica si el sensor MPU6050 está presente y responde correctamente por I2C.
 *
 * Esta función lee el registro de identificación (`WHO_AM_I`) del MPU6050 y compara el valor
 * con la dirección esperada (`ADDRESS_MPU6050`). Devuelve `true` si el sensor está disponible
 * y responde correctamente, o `false` en caso contrario.
 *
 * @return `true` si el sensor fue detectado correctamente, `false` si no hay respuesta o el ID no coincide.
 *
 * @details
 * 1. El registro `WHO_AM_I` (0x75) contiene un valor fijo que identifica al dispositivo (usualmente `0x68`).
 * 2. La función lee este registro usando la interfaz I2C con `MPU6050_PortI2C_ReadRegister()`.
 * 3. Compara el valor recibido con `ADDRESS_MPU6050` (debe estar definido como `0x68` o el valor real del sensor).
 * 4. Si coinciden, devuelve `true`; en caso contrario, `false`.
 *
 * @note
 * - Esta función debe ejecutarse antes de inicializar el sensor para verificar que esté correctamente conectado.
 * - Útil para detección de errores en el bus I2C, fallas de hardware, o control de presencia de múltiples dispositivos.
 *
 * @example
 * ```c
 * if (!MPU6050_IsAvailable()) {
 *     uartSendString((uint8_t*)"MPU6050 no detectado\r\n");
 *     Error_Handler();
 * }
 * ```
 */

bool_t MPU6050_IsAvailable()
{
	uint8_t id_device = 0;
	MPU6050_PortI2C_ReadRegister(WHO_AM_I, &id_device, MAX_BYTE_REGISTER, MAX_BYTE_SEND);
	return id_device == ADDRESS_MPU6050;
}


/**
 * @brief Inicializa el sensor MPU6050 mediante una máquina de estados finita (FSM) y delay no bloqueante.
 *
 * Esta función ejecuta la secuencia de inicialización del sensor MPU6050 paso a paso, permitiendo
 * que el sistema continúe operando mientras el sensor se configura. Cada llamada ejecuta una etapa de la FSM.
 *
 * Etapas de la máquina de estados:
 * - `MPU6050_INIT_IDLE`: Inicia el proceso, cambia al siguiente estado.
 * - `MPU6050_INIT_CHECK`: Verifica si el sensor responde correctamente (WHO_AM_I == 0x68).
 * - `MPU6050_INIT_PWR`: Escribe en `PWR_MGMT_1` para salir del modo sleep.
 * - `MPU6050_INIT_WAIT_PWR`: Espera 100 ms de estabilización usando un delay no bloqueante.
 * - `MPU6050_INIT_CONFIG`: Configura registros internos de muestreo y rangos del sensor.
 * - `MPU6050_INIT_DONE`: Marca que la inicialización ha sido completada.
 *
 * Esta función debe llamarse repetidamente dentro del `while(1)` principal hasta que
 * `MPU6050_IsReady()` retorne `true`.
 *
 * @note
 * - Utiliza `API_delay` para realizar los retardos sin bloquear el sistema.
 * - Usa UART para mensajes de diagnóstico (`MPU6050 DETECTED` o `NOT FOUND`).
 * - En caso de error de detección, se ejecuta `Error_Handler()`.
 */

void MPU6050_InitStep(void)
{
    switch(mpu_state)
    {
        case MPU6050_INIT_IDLE:
            mpu_state = MPU6050_INIT_CHECK;
            break;

        case MPU6050_INIT_CHECK:
            if (!MPU6050_IsAvailable()) {
                uartSendString((uint8_t*)"MPU6050 NOT FOUND\r\n");
                Error_Handler(); // o puedes quedarte en este estado
            } else {
                uartSendString((uint8_t*)"MPU6050 DETECTED\r\n");
                mpu_state = MPU6050_INIT_PWR;
            }
            break;

        case MPU6050_INIT_PWR:
            MPU6050_PortI2C_WriteRegister(PWR_MGMT_1, CONF_PWR_MGMT, MAX_BYTE_REGISTER);
            delayInit(&mpu_delay, 100);  // espera tras encender
            mpu_state = MPU6050_INIT_WAIT_PWR;
            break;

        case MPU6050_INIT_WAIT_PWR:
            if (delayRead(&mpu_delay)) {
                mpu_state = MPU6050_INIT_CONFIG;
            }
            break;

        case MPU6050_INIT_CONFIG:
            MPU6050_PortI2C_WriteRegister(SMPLRT_DIV, CONF_SMPLRT_DIV, MAX_BYTE_REGISTER);
            MPU6050_PortI2C_WriteRegister(CONFIG, CONFIG_DLPF, MAX_BYTE_REGISTER);
            MPU6050_PortI2C_WriteRegister(GYRO_CONFIG, FS_GYRO_250, MAX_BYTE_REGISTER);
            MPU6050_PortI2C_WriteRegister(ACCEL_CONFIG, FS_ACC_2G, MAX_BYTE_REGISTER);
            mpu_state = MPU6050_INIT_DONE;
            break;

        case MPU6050_INIT_DONE:
            mpu_ready = true;
            break;
    }
}

/**
 * @brief Verifica si el sensor MPU6050 ha completado su proceso de inicialización.
 *
 * Esta función devuelve el valor del flag interno `mpu_ready`, que es establecido en `true`
 * al finalizar exitosamente la máquina de estados definida en `MPU6050_InitStep()`.
 *
 * @note
 * - Debe usarse en el bucle principal para determinar si es seguro comenzar a leer datos
 *   del acelerómetro, giroscopio o temperatura.
 * - Si se llama antes de que `MPU6050_InitStep()` complete su secuencia, el resultado será `false`.
 *
 * @return `true` si el sensor está listo para usarse, `false` si la inicialización aún no ha finalizado.
 */

bool_t MPU6050_IsReady(void)
{
    return mpu_ready;
}
