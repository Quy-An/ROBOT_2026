#ifndef PID_CONTROL_H
#define PID_CONTROL_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float error_prev;
    float integral;
    float output_limit;
    float integral_limit;
} pid_const_t;

/**
 * @brief Khởi tạo tham số PID
 */
void pid_init(pid_const_t *pid, float kp, float ki, float kd, float limit);

/**
 * @brief Tính toán đầu ra PID
 */
float pid_compute(pid_const_t *pid, float setpoint, float measure, float dt);

/**
 * @brief Reset bộ tích phân (tránh vọt lố khi chuyển trạng thái)
 */
void pid_reset(pid_const_t *pid);

/**
 * @brief Chuẩn hóa góc về khoảng [-180, 180] để tính sai số góc chính xác
 */
float normalize_angle(float angle);

/**
 * @brief Chuẩn hóa góc về khoảng [-PI, PI] để tính toán sai số góc chính xác
 */
float normalize_angle_rad(float angle_rad);

#endif