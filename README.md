# Proyecto STM32 con Sensores BMP280 y MPU6050

Este proyecto implementa la comunicación e inicialización de los sensores **BMP280 (presión y temperatura)** y **MPU6050 (acelerómetro y giroscopio)** en una placa con microcontrolador **STM32F446RE**, utilizando **HAL de STM32CubeMX/STM32CubeIDE**.

## Estructura del Proyecto

```
/Drivers
  ├── bmp280_driver.c/h      # Lógica del sensor BMP280
  ├── bmp280_port.c/h        # Abstracción de puerto SPI
  ├── mpu6050_driver.c/h     # Lógica del sensor MPU6050
  ├── mpu6050_port.c/h       # Abstracción de puerto I2C
```

## Inicialización del Proyecto

### 1. Configuración de Periféricos en STM32CubeMX

#### BMP280 - Interfaz SPI
- **SPI2** como maestro.
- Configurar GPIOA Pin 4 como salida (chip select).
- Velocidad recomendada: `SPI_BAUDRATEPRESCALER_2`.

#### MPU6050 - Interfaz I2C
- **I2C3** a 100kHz.
- Dirección de esclavo: `0x68` (AD0 conectado a GND).

### 2. Inclusión de Archivos
Asegúrate de incluir los `.c` y `.h` en tu entorno STM32CubeIDE:
- Agrega `bmp280_driver.c`, `bmp280_port.c`, `mpu6050_driver.c`, `mpu6050_port.c` a los **Sources**.
- Añade los `.h` correspondientes en la carpeta `Inc/` o crea una carpeta `API_INC`.

### 3. Inicialización en `main.c`

```c
BMP280_SPI_Init();
BMP280_SPI_CS_Init();
MPU6050_PortI2C_Init();

while (!BMP280_IsReady()) BMP280_InitStep();
while (!MPU6050_IsReady()) MPU6050_InitStep();
```

## Funcionalidades Implementadas

### BMP280
- Inicialización paso a paso (`BMP280_InitStep`)
- Lectura de:
  - Temperatura (`BMP280_ReadTemperature`)
  - Presión (`BMP280_ReadPressure`)
  - Altitud estimada (`BMP280_ReadAltitude`)

### MPU6050
- Inicialización paso a paso (`MPU6050_InitStep`)
- Lectura de:
  - Temperatura (`MPU6050_GetTemperature`)
  - Acelerómetro (`MPU6050_GetAccelerometer`)
  - Giroscopio (`MPU6050_GetGyroscope`)
- Todas también disponibles en modo entero (`*_Int()`), ideal para plataformas sin FPU.

## Requisitos

- **STM32CubeIDE**
- Biblioteca HAL para STM32F4
- Comunicación UART para mensajes de diagnóstico (opcional)

## Créditos

Autor: **doddy**  
Fecha de creación: abril 2025  
Sensores utilizados: **Bosch BMP280** y **InvenSense MPU6050**
