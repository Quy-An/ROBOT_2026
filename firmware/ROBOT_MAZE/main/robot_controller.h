#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief Initialize the robot control task
 */
void robot_controller_task_start(void);

/**
 * @brief Update the target velocity for the robot
 * 
 * @param vx Linear velocity in x-axis (m/s)
 * @param wz Angular velocity around z-axis (rad/s)
 */
void robot_controller_set_velocity(float vx, float wz);

#endif // ROBOT_CONTROLLER_H
