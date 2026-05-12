#ifndef SENSOR_MPU6050_H
#define SENSOR_MPU6050_H

#include <stdint.h>

/**
 * @brief Initialize the MPU6050 DMP and start its reading task.
 * 
 * @return 0 on success, non-zero on failure.
 */
int mpu6050_controller_init(void);

/**
 * @brief Get the latest cached DMP sensor data
 * 
 * @param pitch Pointer to store pitch (degrees)
 * @param roll  Pointer to store roll (degrees)
 * @param yaw   Pointer to store yaw (degrees)
 */
void mpu6050_get_euler_angles(float *pitch, float *roll, float *yaw);

/**
 * @brief Get the latest cached accelerometer and gyroscope data from DMP read
 *
 * @param accel_g_out  Output accel in g (array size 3) or NULL
 * @param gyro_dps_out Output gyro in dps (array size 3) or NULL
 */
void mpu6050_get_accel_gyro(float accel_g_out[3], float gyro_dps_out[3]);

#endif // SENSOR_MPU6050_H
