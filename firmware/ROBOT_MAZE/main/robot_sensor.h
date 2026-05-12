#ifndef ROBOT_SENSOR_H
#define ROBOT_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the MPU6050 sensor and its reading task.
 */
void robot_sensor_init(void);

/**
 * @brief Get the latest yaw angle in degrees.
 * @return Latest yaw angle.
 */
float robot_sensor_get_yaw(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_SENSOR_H */