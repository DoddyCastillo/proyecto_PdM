/*
 * mpu6050_driver.h
 *
 *  Created on: Apr 12, 2025
 *      Author: doddy
 */

#ifndef API_INC_MPU6050_DRIVER_H_
#define API_INC_MPU6050_DRIVER_H_

#include "stdbool.h"
#include "stdint.h"
typedef bool bool_t;

// Length data register Measurements
#define LENGTH_DATA 2

// Who AM I register
#define WHO_AM_I 0X75

#define MAX_BYTE_REGISTER 1
#define MAX_BYTE_SEND     1

// MPU6050 I2C address (AD=0)
#define ADDRESS_MPU6050  0x68

// Gyroscope Configuration
#define GYRO_CONFIG      0x1B

#define FS_GYRO_250      0x00
#define FS_GYRO_500      0x08
#define FS_GYRO_1000     0x10
#define FS_GYRO_2000     0x18

// Accelerometer Configuration

#define ACCEL_CONFIG     0x1C

#define FS_ACC_2G        0x00
#define FS_ACC_4G        0x08
#define FS_ACC_8G        0x10
#define FS_ACC_16G       0x18

// Sensitivity GYRO constants
#define FS_LSB_GYRO_250  131.0f
#define FS_LSB_GYRO_500  65.5f
#define FS_LSB_GYRO_1000 32.8f
#define FS_LSB_GYRO_2000 16.4f

// Sensitivity ACC constans
#define FS_LSB_ACC_250   16384.0f
#define FS_LSB_ACC_500   8192.0f
#define FS_LSB_ACC_1000  4096.0f
#define FS_LSB_ACC_2000  2048.0f

//Sample Rate
#define SMPLRT_DIV       0x19
#define CONF_SMPLRT_DIV  0x00

// Power Management
#define PWR_MGMT_1       0x6B
#define CONF_PWR_MGMT    0x00

// Configuration
#define CONFIG           0x1A
#define CONFIG_DLPF      0x03

// Temperature Measurements
#define TEMP_OUT_H       0x41

// Gyroscope Measurements
#define GYRO_XOUT_H      0x43

// Accelerometer Measurements
#define ACCEL_XOUT_H     0x3B

// Vector structure

typedef struct {
    float x;
    float y;
    float z;
} Vector3f;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;

}Vector3i16;

typedef enum {
    MPU6050_INIT_IDLE,
    MPU6050_INIT_CHECK,
    MPU6050_INIT_PWR,
    MPU6050_INIT_WAIT_PWR,
    MPU6050_INIT_CONFIG,
    MPU6050_INIT_DONE
} mpu6050_init_state_t;

#define ADDRESS_MPU6050 0x68

// MFS
void MPU6050_InitStep();
bool_t MPU6050_IsReady(void);

//Normal
void  MPU6050_Init();
void  MPU6050_Check();
// Float Measurements
float MPU6050_GetTemperature();
Vector3f  MPU6050_GetGyroscope();
Vector3f  MPU6050_GetAccelerometer();

// Int Measurements
int16_t MPU6050_GetTemperatureInt();
Vector3i16  MPU6050_GetGyroscopeInt();
Vector3i16  MPU6050_GetAccelerometerInt();

bool_t MPU6050_IsAvailable();


#endif /* API_INC_MPU6050_DRIVER_H_ */
