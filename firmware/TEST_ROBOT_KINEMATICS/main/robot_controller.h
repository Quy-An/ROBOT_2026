#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MAX_RAD_S 27.0f         // 27.23f

typedef struct {
    float vx;    // Vận tốc dài trục x (m/s)
    float wz;    // Vận tốc góc quanh trục z (rad/s)
} robot_velocity_t;

typedef struct {
    float left;  // rad/s (Dương là CCW)
    float right; // rad/s (Âm là CW - theo yêu cầu của bạn)
} wheel_velocity_t;

// Khởi tạo các hằng số tính toán sẵn
void kinematics_init(void);

// Động học thuận: Wheel -> Robot
robot_velocity_t forward_kinematics(wheel_velocity_t wheel);

// Động học nghịch: Robot -> Wheel
wheel_velocity_t inverse_kinematics(robot_velocity_t robot);

void move_robot(float vx, float wz);

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

void robot_drive_straight(float vx);
void robot_turn_relative(float delta_yaw_deg);

#endif // ROBOT_CONTROLLER_H
