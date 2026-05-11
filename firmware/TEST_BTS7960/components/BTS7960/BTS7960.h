#ifndef BTS7960_H
#define BTS7960_H

#include <stdint.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Khởi tạo BTS7960 (PWM, GPIO)
 */
void BTS7960_init(void);

/**
 * @brief Điều khiển động cơ trái
 * @param pwm Giá trị PWM (-100 ~ 100), âm là quay ngược, dương là xuôi, 0 là dừng
 */
void BTS7960_set_left(int pwm);

/**
 * @brief Điều khiển động cơ phải
 * @param pwm Giá trị PWM (-100 ~ 100), âm là quay ngược, dương là xuôi, 0 là dừng
 */
void BTS7960_set_right(int pwm);

/**
 * @brief Dừng cả 2 động cơ
 */
void BTS7960_stop(void);

#ifdef __cplusplus
}
#endif

#endif // BTS7960_H
