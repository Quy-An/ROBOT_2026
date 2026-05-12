#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

#define MAX_RAD_S 25.0f         // 27.23f

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

#endif